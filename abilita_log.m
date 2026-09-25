function catena = abilita_log(stato, verbose, mdl)
%ABILITA_LOG  Accende i To Workspace delle forze E la catena che li alimenta.
%
%   abilita_log              DRY RUN: elenca cosa accenderebbe, non tocca nulla
%   abilita_log('on')        accende
%   abilita_log('off')       rispegne solo i To Workspace, non la catena
%
% PERCHE' SERVE
%   Fleg e Fsum sono commentati nel modello. adatta_simscape costruisce
%   run.Fc e run.contact SOLO da Fleg (riga 262): senza quel segnale restano
%   NaN tutte le metriche che dipendono dal contatto -
%       appoggio_medio, disp_carico, Fz_max_norm, slip_tot, slip_per_passo
%   - cioe' tutta la famiglia D piu' il criterio di taratura_T2, che sceglie
%   proprio su appoggio_medio e disp_carico.
%
% [CORRETTO] SCOMMENTARE IL To Workspace NON BASTA
%   La prima versione toccava solo i due blocchi To Workspace, e sembrava
%   riuscire: erano gia' attivi (Commented = off). Il segnale non arrivava
%   perche' erano commentate le loro SORGENTI - Mux35 per Fleg, Add1 per
%   Fsum. Un blocco attivo alimentato da un blocco commentato non produce
%   nulla, e niente lo segnala.
%
%   Adesso la funzione RISALE la catena: dal To Workspace va indietro di
%   sorgente in sorgente, e si ferma appena incontra un blocco attivo -
%   oltre non serve andare, perche' un blocco attivo il suo segnale lo
%   produce. Cosi' l'insieme da riaccendere e' il minimo necessario.
%
%   La risalita attraversa anche le coppie From/Goto, che in questo modello
%   sono molte: arrivata a un From, salta al Goto con la stessa etichetta.
%
% DRY RUN DI DEFAULT
%   Riaccendere una catena tocca il modello in punti che nessuno ha scelto a
%   mano: prima si guarda l'elenco. E' la stessa prudenza di setup_modello e
%   fix_mesh_paths.
%
% E I FLAG c_* NON SONO UN'ALTERNATIVA
%   I sei c_lf..c_rr sono flag di contatto veri, ma nascono dal confronto
%   |tau_misurata - tau_attesa| > c2_soglia, con tau_attesa prodotta dal
%   blocco Inverse Dynamics.
%   [CORRETTO 25/9] QUI C'ERA SCRITTO  |tau| > c2_soglia,  SENZA LA
%   DIFFERENZA. La regola vera e' stata letta nel modello risalendo il
%   collegamento del Constant c2_soglia: Inverse Dynamics -> Reshape ->
%   Subtract (segni "+-") -> Abs -> Demux -> Mux -> Relational Operator, sei
%   volte, una per zampa. E' la forma del paper (Arrigoni et al. par. 5), non
%   una soglia di coppia secca. La differenza non e' accademica: significa
%   che il flag dipende da quanto e' accurato il modello dello stimatore, ed
%   e' il motivo per cui dal 22 al 25 settembre non ha funzionato (vedi
%   allinea_stimatore e docs/piano_confronto.md sezione 11).
%   In C1 la soglia e' infinita, quindi in anello aperto
%   valgono ZERO per costruzione: usarli come contatto darebbe
%   appoggio_medio = 0 su ogni run C1, che e' peggio di un NaN perche'
%   sembra un dato. In C1 l'unica sorgente di contatto e' la forza.
%
% NIENTE SALVATAGGIO
%   set_param senza save_system, come applica_terreno: il .slx sul disco non
%   cambia, quindi nessun conflitto git e nessun bisogno che il modello sia
%   libero. Va rilanciata a ogni sessione di MATLAB.
%
% Progetto FSR PhantomX - A. Russo

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

% Indicizzazione ESPLICITA, non "for s = cellArray": un for su un cell
% itera sulle COLONNE, quindi si comporta in modo diverso a seconda che il
% cell sia riga o colonna - e unique() restituisce una colonna. Era la causa
% dell'"Index exceeds array bounds".
for giro = 1:50                     % la catena e' corta; il limite e' una rete
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
                % blocco attivo: il suo segnale lo produce, la risalita
                % finisce qui
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
% unique su un cell restituisce una COLONNA: la riporto a riga, cosi' chi
% scorre il risultato non dipende dall'orientamento
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
% QUESTA E' LA SORGENTE VERA DELLE FORZE.
% Il ramo Fleg/Fsum del modello e' un abbozzo mai finito: i dodici From
% cercano le etichette Force_sens_lf..Force_sens_rr e nel modello NON esiste
% alcun Goto che le produca. Un From senza Goto viene risolto come costante,
% con tempo di campionamento infinito: da qui l'unico campione che Fleg
% restituiva, e l'errore muto di interp1.
%
% Le forze pero' esistono: ogni Spatial Contact Force ha un interruttore di
% logging, LogSimulationData, che nel modello e' spento. Accendendolo il
% blocco compare nel simlog con le sue variabili interne - forza normale,
% attrito, penetrazione - senza aggiungere NESSUNA porta e senza toccare lo
% schema. E' un set_param, come applica_terreno.
nContatti = 0;
try
    bc = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                     'IncludeCommented','on', 'RegExp','on', ...
                     'SourceType','Spatial\s*Contact\s*Force');
catch
    bc = {};
end
if isempty(bc)
    % ripiego sul nome, se SourceType non e' interrogabile
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
        % il parametro non esiste in questa release del blocco
    end
end
if verbose
    fprintf('  contatti: %d blocchi Spatial Contact Force su %d portati a LogSimulationData = on\n', ...
            nContatti, numel(bc));
    if numel(bc) == 0
        fprintf(2,'  nessun blocco di contatto trovato: le forze resteranno NaN\n');
    end
end

% Il log deve poter contenere tutto: con LogLimitData acceso tiene solo gli
% ultimi N punti e la run sembra iniziare a meta', in silenzio.
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
% Un To Workspace in formato 'Array' NON porta con se' il tempo, e se ha un
% suo SampleTime o una Decimation la sua lunghezza non e' quella di tout.
% Conseguenza misurata: adatta_simscape si fermava su
%   interp1: "X and V must be of the same length"
% senza dire quale segnale. In formato Timeseries il tempo viaggia col dato e
% il problema non puo' presentarsi.
%
% Si toglie anche il limite sui punti: un To Workspace con
% "Limit data points to last N" tiene gli ULTIMI N campioni e la run sembra
% iniziare a meta', in silenzio. E' lo stesso inganno di
% SimscapeLogLimitData, che ci e' gia' costato il "regime 5.0 s".
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
    % I quattro parametri che possono far loggare MENO di quello che serve,
    % ognuno in silenzio. Misurato su questo modello: Fleg restituiva UN
    % SOLO campione (Time [1x1], Data [1x6]) e adatta_simscape moriva su
    % interp1 senza poter dire perche'.
    %   SampleTime      se e' un numero >= durata della run, si logga solo t=0
    %   MaxDataPoints   un massimo basso taglia la serie
    %   LimitDataPoints tiene gli ULTIMI N punti: la run sembra iniziare a meta'
    %   Decimation      logga un campione su N
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
bt = '';  try, bt = get_param(blocco,'BlockType'); catch, end
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
