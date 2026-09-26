function setup_modello(modo, mdl)
%SETUP_MODELLO  Le modifiche strutturali al .slx, tutte in una volta.
%
%   setup_modello              DRY RUN: mostra cosa farebbe
%   setup_modello applica      modifica il modello e salva
%
% PERCHE' TUTTE INSIEME
%   Il .slx e' binario: git non sa fonderlo, quindi ogni sua modifica e' un
%   conflitto potenziale e blocca chi dei due non ce l'ha in mano. Meglio un
%   solo giro con tutto dentro che quattro giri separati.
%
% COSA FA
%   1. Ancora al World i Rigid Transform del pavimento liscio (RT9) e della
%      rampa (RT7). Quando i tre Brick sono stati commentati e' sparita anche
%      la linea che li teneva attaccati al mondo: riattivarli non bastava,
%      il pavimento cadeva come un corpo libero trascinando giu' il robot.
%
%   2. Riattiva i To Workspace Fleg e Fsum, oggi commentati. Senza le forze
%      di contatto tutta la famiglia D delle metriche e' cieca: niente
%      picco normalizzato, niente dispersione del carico fra le zampe.
%
%   3. Porta la soglia di coppia della retroazione da un literal (0.5 nel
%      blocco Constant28) alla variabile c2_soglia. Due motivi: e' un
%      parametro da giustificare in relazione, e soprattutto diventa
%      L'INTERRUTTORE fra C1 e C2 (vedi sotto).
%
% L'INTERRUTTORE C1/C2, SENZA AGGIUNGERE BLOCCHI
%   La retroazione rileva il contatto quando |tau| supera la soglia. Con
%   soglia infinita il confronto e' sempre falso, la zampa non viene mai
%   bloccata e il comportamento torna a essere quello in anello aperto.
%   Quindi:
%       cfg.c2.attiva = true   -> c2_soglia = cfg.c2.soglia_tau   (C2)
%       cfg.c2.attiva = false  -> c2_soglia = inf                 (C1)
%   Nessuno Switch da inserire, nessuna linea da ridisegnare: e' una
%   modifica che non puo' rompere il modello.
%
% COSA NON FA, E VA FATTO A MANO
%   Il contatto verso la rampa e verso il terreno imperfetto. Oggi solo
%   l'etichetta 'pavimento' e le sette 'ostacolo*' hanno blocchi di contatto
%   nei sei piedi; 'rampa' ha l'etichetta ma nessun contatto. Aggiungerlo
%   significa una porta e uno Spatial Contact Force in ciascuno dei sei
%   Subsystem-piede: e' lavoro da fare nello schema, non da script, e senza
%   di esso T4 e T6 misurano un robot che attraversa il terreno.
%
% SICUREZZA
%   - dry run di default
%   - copia di backup del .slx prima di modificare
%   - ogni intervento e' verificato prima: se una precondizione non torna,
%     quell'intervento viene saltato e gli altri proseguono
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(modo), modo = 'dryrun'; end
if nargin < 2 || isempty(mdl),  mdl  = 'phantomx_sim_zero'; end
modo = lower(strtrim(char(modo)));
if ~ismember(modo, {'dryrun','applica'})
    error('Modo non riconosciuto: %s\nUsa: dryrun | applica', modo);
end

giaAperto = bdIsLoaded(mdl);
if ~giaAperto, load_system(mdl); end

fprintf('\n=============== SETUP MODELLO ===============\n');
fprintf('  modello: %s     modo: %s\n', mdl, upper(modo));

azioni = {};   % {descrizione, funzione}

%% ---- 1. ancoraggio al World ----
for spec = { {'Rigid Transform9','pavimento liscio (Brick Solid3)'}, ...
             {'Rigid Transform7','rampa (Brick Solid1)'} }
    nomeRT = spec{1}{1};  descr = spec{1}{2};
    blocco = [mdl '/' strrep(nomeRT,'Rigid Transform',sprintf('Rigid\nTransform'))];
    if ~esiste(blocco)
        fprintf(2,'  [1] %s non trovato, salto.\n', nomeRT);  continue
    end
    if ancorato(blocco)
        fprintf('  [1] %-18s gia'' ancorato al World.\n', nomeRT);
    else
        fprintf('  [1] %-18s DA ANCORARE al World   (%s)\n', nomeRT, descr);
        azioni{end+1} = {sprintf('ancora %s al World', nomeRT), ...
                         @() ancora(mdl, blocco)};                    %#ok<AGROW>
    end
end

%% ---- 2. To Workspace delle forze ----
for v = {'Fleg','Fsum'}
    b = trovaToWorkspace(mdl, v{1});
    if isempty(b)
        fprintf(2,'  [2] To Workspace %-5s non trovato, salto.\n', v{1});  continue
    end
    if strcmpi(get_param(b,'Commented'),'on')
        fprintf('  [2] %-5s COMMENTATO, da riattivare\n', v{1});
        azioni{end+1} = {sprintf('riattiva %s', v{1}), ...
                         @() set_param(b,'Commented','off')};         %#ok<AGROW>
    else
        fprintf('  [2] %-5s gia'' attivo.\n', v{1});
    end
end

%% ---- 3. soglia della retroazione ----
bc = [mdl '/Constant28'];
if ~esiste(bc)
    fprintf(2,'  [3] Constant28 non trovato: la soglia sta altrove, salto.\n');
else
    val = get_param(bc, 'Value');
    if strcmp(val, 'c2_soglia')
        fprintf('  [3] soglia gia'' parametrica (c2_soglia).\n');
    else
        fprintf('  [3] soglia di coppia: %-10s -> c2_soglia\n', val);
        fprintf('      VERIFICA che Constant28 sia davvero la soglia dei sei\n');
        fprintf('      Relational Operator ">" e non un altro parametro.\n');
        azioni{end+1} = {'soglia -> c2_soglia', ...
                         @() set_param(bc,'Value','c2_soglia')};      %#ok<AGROW>
    end
end

%% ---- esecuzione ----
if isempty(azioni)
    fprintf('\n  Niente da fare: il modello e'' gia'' a posto.\n');
    fprintf('=============================================\n\n');
    if ~giaAperto, close_system(mdl,0); end
    return
end

if strcmp(modo,'dryrun')
    fprintf(['\n--------------------------------------------------------\n' ...
             'DRY RUN: %d interventi previsti, non ho modificato nulla.\n' ...
             '  setup_modello applica\n' ...
             '--------------------------------------------------------\n\n'], numel(azioni));
    if ~giaAperto, close_system(mdl,0); end
    return
end

slx = which([mdl '.slx']);  if isempty(slx), slx = [mdl '.slx']; end
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

fprintf('\n');
n = 0;
for k = 1:numel(azioni)
    try
        azioni{k}{2}();
        fprintf('  fatto: %s\n', azioni{k}{1});
        n = n + 1;
    catch ME
        fprintf(2,'  FALLITO: %s  (%s)\n', azioni{k}{1}, ME.message);
    end
end

save_system(mdl);
fprintf('\n%d interventi su %d. Modello salvato.\n', n, numel(azioni));
if ~giaAperto, close_system(mdl,0); end

fprintf(['\nAdesso servono in phantomx_config:\n' ...
         '    cfg.c2.attiva     = true;   %% false = C1, anello aperto\n' ...
         '    cfg.c2.soglia_tau = 0.5;    %% [N*m]\n' ...
         'e in init_gait, prima delle stampe:\n' ...
         '    if cfg.c2.attiva, c2_soglia = cfg.c2.soglia_tau; else, c2_soglia = inf; end\n\n' ...
         'Poi:  init_gait ; applica_terreno(''T1'') ; sim(''%s'',''StopTime'',''5'')\n\n'], mdl);

end

%% ================================================================
function t = esiste(b)
t = true;
try, get_param(b,'Name'); catch, t = false; end
end

function t = ancorato(blocco)
%ANCORATO  Il lato B del Rigid Transform arriva al World?
t = false;
try
    pc = get_param(blocco, 'PortConnectivity');
catch
    return
end
for p = 1:numel(pc)
    h = [pc(p).SrcBlock, pc(p).DstBlock];  h = h(h > 0);
    for j = 1:numel(h)
        if strcmp(strrep(get_param(h(j),'Name'), newline, ' '), 'World'), t = true; return; end
    end
end
end

function ancora(mdl, blocco)
%ANCORA  Collega il lato B del Rigid Transform al World.
set_param(blocco, 'Commented', 'off');
corto = strrep(blocco, [mdl '/'], '');
add_line(mdl, 'World/RConn1', [corto '/RConn1'], 'autorouting', 'on');
end

function b = trovaToWorkspace(mdl, nomeVar)
b = '';
l = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                'BlockType','ToWorkspace');
for k = 1:numel(l)
    if strcmp(get_param(l{k},'VariableName'), nomeVar), b = l{k}; return; end
end
end