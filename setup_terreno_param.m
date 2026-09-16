function setup_terreno_param(modo, mdl)
%SETUP_TERRENO_PARAM  Rende parametrica la posizione degli ostacoli.
%
%   setup_terreno_param              DRY RUN: mostra cosa farebbe
%   setup_terreno_param applica      modifica il modello e salva
%   setup_terreno_param ripristina   rimette floor_off ovunque
%
% COSA FA, E PERCHE' UNA VOLTA SOLA
%   I sette Rigid Transform che posizionano gli ostacoli (RT10..RT16) hanno
%   tutti la stessa traslazione, floor_off: gli ostacoli stanno dove li mette
%   la loro mesh e non si possono spostare da script. Qui ciascuno passa a
%   leggere  terreno.off(:,k)  - una variabile del workspace.
%
%   Da quel momento il .slx NON SI TOCCA PIU'. Il terreno di ogni task si
%   sceglie con applica_terreno('T5') prima di simulare. Il modello e' un
%   file binario che git non sa fondere: ogni sua modifica e' un conflitto
%   potenziale e blocca chi dei due non ce l'ha in mano. Questa e' l'ultima.
%
% COME SI SPEGNE UN OSTACOLO
%   Non si toglie dal modello: lo si parcheggia sotto il pavimento, dove non
%   tocca nessuno. Nessun blocco da aggiungere o commentare, e il costo in
%   simulazione di un contatto che non avviene mai e' trascurabile.
%
% SICUREZZA
%   - dry run di default
%   - copia di backup del .slx prima di modificare
%   - verifica che ogni RT sia davvero collegato al suo File Solid: se
%     l'associazione non torna, non tocca niente
%
% PRIMA
%   Serve il blocco cfg.terreno in phantomx_config e il default in init_gait,
%   altrimenti dopo la modifica il modello non compila piu' (cerca una
%   variabile 'terreno' che nessuno definisce).
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(modo), modo = 'dryrun'; end
if nargin < 2 || isempty(mdl),  mdl  = 'phantomx_sim_zero'; end
modo = lower(strtrim(char(modo)));
if ~ismember(modo, {'dryrun','applica','ripristina'})
    error('Modo non riconosciuto: %s\nUsa: dryrun | applica | ripristina', modo);
end

%% ---- associazione RT -> File Solid, verificata ----
% RT10 muove File Solid1, RT11 File Solid2, ... RT16 File Solid7.
rtNum = 10:16;
nProp = numel(rtNum);

giaAperto = bdIsLoaded(mdl);
if ~giaAperto, load_system(mdl); end

blocchi = cell(1,nProp);  attuale = cell(1,nProp);  solido = cell(1,nProp);
for k = 1:nProp
    b = sprintf('%s/Rigid\nTransform%d', mdl, rtNum(k));
    try
        attuale{k} = get_param(b, 'TranslationCartesianOffset');
    catch
        fprintf(2,'\n  Non trovo il blocco Rigid Transform%d. Mi fermo.\n\n', rtNum(k));
        if ~giaAperto, close_system(mdl,0); end
        return
    end
    blocchi{k} = b;
    solido{k}  = collegato(b);
end

%% ---- verifica dell'associazione ----
atteso = arrayfun(@(k) sprintf('File Solid%d', k), 1:nProp, 'UniformOutput', false);
ok = true;
for k = 1:nProp
    if ~strcmp(solido{k}, atteso{k}), ok = false; end
end

fprintf('\n============ TERRENO PARAMETRICO ============\n');
fprintf('  modello: %s     modo: %s\n\n', mdl, upper(modo));
fprintf('%-22s %-16s %-24s %s\n', 'blocco', 'collegato a', 'ora', 'diventa');
for k = 1:nProp
    if strcmp(modo,'ripristina')
        nuovo = 'floor_off';
    else
        nuovo = sprintf('terreno.off(:,%d)', k);
    end
    fprintf('Rigid Transform%-7d %-16s %-24s %s\n', ...
            rtNum(k), solido{k}, attuale{k}, nuovo);
end

if ~ok
    fprintf(2,['\n  L''associazione non e'' quella attesa (RT10->File Solid1 ...\n' ...
               '  RT16->File Solid7). Non tocco niente: verifica il cablaggio\n' ...
               '  prima di procedere.\n\n']);
    if ~giaAperto, close_system(mdl,0); end
    return
end

if strcmp(modo,'dryrun')
    fprintf(['\n--------------------------------------------------------\n' ...
             'DRY RUN: non ho modificato nulla.\n' ...
             '  setup_terreno_param applica      scrive e salva il modello\n' ...
             '  setup_terreno_param ripristina   torna a floor_off\n' ...
             '--------------------------------------------------------\n\n']);
    if ~giaAperto, close_system(mdl,0); end
    return
end

%% ---- backup ----
slx = which([mdl '.slx']);
if isempty(slx), slx = [mdl '.slx']; end
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
bk = fullfile(fileparts(slx), sprintf('%s_backup_%s.slx', mdl, stamp));
try
    copyfile(slx, bk);
    fprintf('\nBackup: %s\n', bk);
catch ME
    fprintf(2,'\nBackup fallito (%s). Mi fermo.\n\n', ME.message);
    if ~giaAperto, close_system(mdl,0); end
    return
end

%% ---- scrittura ----
n = 0;
for k = 1:nProp
    if strcmp(modo,'ripristina')
        nuovo = 'floor_off';
    else
        nuovo = sprintf('terreno.off(:,%d)', k);
    end
    try
        set_param(blocchi{k}, 'TranslationCartesianOffset', nuovo);
        n = n + 1;
    catch ME
        fprintf(2,'  FALLITO Rigid Transform%d : %s\n', rtNum(k), ME.message);
    end
end

save_system(mdl);
fprintf('Aggiornati %d blocchi su %d. Modello salvato.\n', n, nProp);
if ~giaAperto, close_system(mdl,0); end

fprintf(['\nAdesso:\n' ...
         '   init_gait\n' ...
         '   applica_terreno(''T1'')\n' ...
         '   out = sim(''%s'',''StopTime'',''10'');\n\n' ...
         'E questo e'' l''ULTIMO commit sul .slx per il terreno: da qui in poi\n' ...
         'i task si scelgono da script.\n\n'], mdl);

end

%% ================================================================
function nome = collegato(blocco)
%COLLEGATO  Nome del primo File Solid attaccato a questo blocco.
nome = '(sconosciuto)';
try
    pc = get_param(blocco, 'PortConnectivity');
catch
    return
end
for p = 1:numel(pc)
    h = [pc(p).SrcBlock, pc(p).DstBlock];
    h = h(h > 0);
    for j = 1:numel(h)
        n = strrep(get_param(h(j),'Name'), newline, ' ');
        if startsWith(n, 'File Solid'), nome = n; return; end
    end
end
end