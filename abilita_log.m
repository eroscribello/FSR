function catena = abilita_log(stato, verbose, mdl)
% Accende i To Workspace delle forze e la catena che li alimenta.
%
%   abilita_log              DRY RUN: elenca cosa accenderebbe, non tocca nulla
%   abilita_log('on')        accende
%   abilita_log('off')       rispegne solo i To Workspace, non la catena
%

if nargin < 1 || isempty(stato),   stato = 'dryrun'; end
if nargin < 2 || isempty(verbose), verbose = true;   end
if nargin < 3 || isempty(mdl),     mdl = 'phantomx_sim_zero'; end

stato = lower(strtrim(char(stato)));
if ~ismember(stato, {'dryrun','on','off'})
    error('abilita_log:stato','Usa ''dryrun'', ''on'' oppure ''off''.');
end

if ~bdIsLoaded(mdl), load_system(mdl); end

VARIABILI = {'Fleg','Fsum'};

%% ---- i To Workspace bersaglio ----
tutti = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                    'IncludeCommented','on', 'BlockType','ToWorkspace');
bersagli = {};
for iv = 1:numel(VARIABILI)
    trovato = false;
    for k = 1:numel(tutti)
        if strcmp(get_param(tutti{k},'VariableName'), VARIABILI{iv})
            bersagli{end+1} = tutti{k};                               %#ok<AGROW>
            trovato = true;
        end
    end
    if ~trovato && verbose
        fprintf(2,'  To Workspace %s non trovato nel modello.\n', VARIABILI{iv});
    end
end

%% ---- spegnimento: solo i bersagli ----
if strcmp(stato,'off')
    for k = 1:numel(bersagli)
        set_param(bersagli{k}, 'Commented', 'on');
    end
    catena = bersagli;
    if verbose
        fprintf('\n--- LOG DELLE FORZE: OFF ---\n');
        fprintf('  spenti %d To Workspace (la catena a monte resta come e'')\n', numel(bersagli));
        fprintf('  (il modello NON e'' stato salvato)\n\n');
    end
    return
end

%% ---- risalita della catena ----
catena  = {};
visti   = [];
fronte  = bersagli;

for giro = 1:50                   
    nuovo = {};
    for i = 1:numel(fronte)
        b = fronte{i};
        h = getSimulinkBlockHandle(b);
        if h <= 0 || any(visti == h), continue; end
        visti(end+1) = h;                                             %#ok<AGROW>

        sr = sorgenti(b, mdl);
        for q = 1:numel(sr)
            nomeSrc = sr{q};
            if isempty(nomeSrc) || ~(ischar(nomeSrc) || isstring(nomeSrc))
                continue
            end
            nomeSrc = char(nomeSrc);
            hs = getSimulinkBlockHandle(nomeSrc);
            if hs <= 0 || any(visti == hs), continue; end
            c = '';
            try
                c = get_param(nomeSrc, 'Commented');
            catch
                continue
            end
            if strcmp(c, 'off')

                continue
            end
            catena{end+1} = nomeSrc;                                  %#ok<AGROW>
            nuovo{end+1}  = nomeSrc;                                  %#ok<AGROW>
        end
    end
    fronte = nuovo;
    if isempty(fronte), break; end
end

% i bersagli stessi, se commentati
for k = 1:numel(bersagli)
    if ~strcmp(get_param(bersagli{k},'Commented'),'off')
        catena{end+1} = bersagli{k};                                  %#ok<AGROW>
    end
end

if ~isempty(catena)
    catena = reshape(unique(catena), 1, []);
end

%% ---- riepilogo ----
if verbose
    fprintf('\n--- LOG DELLE FORZE: %s ---\n', upper(stato));
    if isempty(catena)
        fprintf('  Niente da accendere: la catena di Fleg e Fsum e'' gia'' attiva.\n');
    else
        fprintf('  da riaccendere, %d blocchi:\n', numel(catena));
        for k = 1:numel(catena)
            nm = strrep(get_param(catena{k},'Name'), newline, ' ');
            bt = get_param(catena{k},'BlockType');
            fprintf('    %-26s %s\n', nm, bt);
        end
    end
end

%% ---- applicazione ----
if strcmp(stato,'dryrun')
    if verbose
        fprintf(['\n  DRY RUN: non ho modificato nulla.\n' ...
                 '    abilita_log on\n' ...
                 '  Controlla l''elenco qui sopra: se contiene blocchi che non\n' ...
                 '  c''entrano con le forze di contatto, dimmelo prima di\n' ...
                 '  accenderli.\n\n']);
    end
    return
end

n = 0;
for k = 1:numel(catena)
    try
        set_param(catena{k}, 'Commented', 'off');
        n = n + 1;
    catch ME
        fprintf(2,'  FALLITO %s: %s\n', ...
                strrep(get_param(catena{k},'Name'), newline, ' '), ME.message);
    end
end

%% ---- logging dei blocchi di contatto ----
nContatti = 0;
try
    bc = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                     'IncludeCommented','on', 'RegExp','on', ...
                     'SourceType','Spatial\s*Contact\s*Force');
catch
    bc = {};
end
if isempty(bc)
    tuttiB = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                         'IncludeCommented','on', 'Type','block');
    bc = {};
    for k = 1:numel(tuttiB)
        n = strrep(get_param(tuttiB{k},'Name'), newline, ' ');
        if startsWith(n, 'Spatial Contact Force'), bc{end+1} = tuttiB{k}; end %#ok<AGROW>
    end
end

for k = 1:numel(bc)
    try
        if ~strcmpi(get_param(bc{k},'LogSimulationData'), 'on')
            set_param(bc{k}, 'LogSimulationData', 'on');
            nContatti = nContatti + 1;
        end
    catch
    end
end
if verbose
    fprintf('  contatti: %d blocchi Spatial Contact Force su %d portati a LogSimulationData = on\n', ...
            nContatti, numel(bc));
    if numel(bc) == 0
        fprintf(2,'  nessun blocco di contatto trovato: le forze resteranno NaN\n');
    end
end
try
    if ~strcmpi(get_param(mdl,'SimscapeLogType'),'all')
        set_param(mdl,'SimscapeLogType','all');
        if verbose, fprintf('  SimscapeLogType -> all\n'); end
    end
    if ~strcmpi(get_param(mdl,'SimscapeLogLimitData'),'off')
        set_param(mdl,'SimscapeLogLimitData','off');
        if verbose, fprintf('  SimscapeLogLimitData -> off\n'); end
    end
catch
end

%% ---- formato dei To Workspace ----
for k = 1:numel(bersagli)
    v = get_param(bersagli{k}, 'VariableName');
    try
        prima = get_param(bersagli{k}, 'SaveFormat');
        if ~strcmpi(prima, 'Timeseries')
            set_param(bersagli{k}, 'SaveFormat', 'Timeseries');
            if verbose
                fprintf('  %s: formato %s -> Timeseries\n', v, prima);
            end
        end
    catch ME
        fprintf(2,'  %s: formato non impostato (%s)\n', v, ME.message);
    end
    desiderati = { 'SampleTime',      '-1'   % -1 = eredita il passo del segnale
                   'MaxDataPoints',   'inf'
                   'LimitDataPoints', 'off'
                   'Decimation',      '1'   };
    for ip = 1:size(desiderati,1)
        par = desiderati{ip,1};
        val = desiderati{ip,2};
        try
            prima = get_param(bersagli{k}, par);
        catch
            continue    % il parametro non esiste in questa release
        end
        if ~strcmpi(strtrim(char(prima)), val)
            try
                set_param(bersagli{k}, par, val);
                if verbose
                    fprintf('  %s: %-15s %s -> %s\n', v, par, strtrim(char(prima)), val);
                end
            catch ME
                fprintf(2,'  %s: %s non impostato (%s)\n', v, par, ME.message);
            end
        end
    end
end

if verbose
    fprintf('\n  riaccesi %d blocchi su %d della catena Fleg/Fsum.\n', n, numel(catena));
    fprintf('  (il modello NON e'' stato salvato: va rifatto a ogni sessione)\n');
    fprintf(['\n  VERIFICA - le forze vanno cercate nel LOG, non in Fleg:\n' ...
             '     init_gait ; applica_terreno(''T1'')\n' ...
             '     out = sim(''%s'',''StopTime'',''3'');\n' ...
             '     T = cerca_log(out, ''contact'');\n' ...
             '  Devono comparire i blocchi di contatto con le loro variabili.\n' ...
             '  Fleg resta inservibile finche'' nessuno pubblica le etichette\n' ...
             '  Force_sens_*, e non serve: il log e'' una sorgente migliore,\n' ...
             '  perche'' porta le componenti e non il modulo.\n\n'], mdl);
end

end

%% ================================================================
function src = sorgenti(blocco, mdl)
%SORGENTI  I blocchi che alimentano questo, attraversando anche From/Goto.
src = {};
padre = get_param(blocco, 'Parent');

% --- sorgenti dirette ---
try
    pc = get_param(blocco, 'PortConnectivity');
catch
    return
end
for p = 1:numel(pc)
    h = pc(p).SrcBlock;
    if h > 0
        src{end+1} = getfullname(h);                                  %#ok<AGROW>
    end
end

% --- se questo blocco e' un From, la sorgente vera e' il suo Goto ---
bt = '';  try bt = get_param(blocco,'BlockType'); catch, end
if strcmp(bt,'From')
    try
        tag = get_param(blocco,'GotoTag');
        g = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                        'IncludeCommented','on', 'BlockType','Goto', ...
                        'GotoTag', tag);
        for k = 1:numel(g), src{end+1} = g{k}; end                    %#ok<AGROW>
    catch
    end
end

% --- se e' un Inport di sottosistema, la sorgente sta al livello di sopra ---
if strcmp(bt,'Inport') && ~strcmp(padre, mdl)
    try
        n  = str2double(get_param(blocco,'Port'));
        pc2 = get_param(padre, 'PortConnectivity');
        for p = 1:numel(pc2)
            t = pc2(p).Type;
            if ischar(t) && strcmp(t, num2str(n)) && pc2(p).SrcBlock > 0
                src{end+1} = getfullname(pc2(p).SrcBlock);            %#ok<AGROW>
            end
        end
    catch
    end
end

if ~isempty(src)
    src = reshape(unique(src), 1, []);
end
end
