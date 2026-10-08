function info = log_zmp(stato, mdl)
%LOG_ZMP  Porta le uscite del calcolo ZMP nel log, senza toccare il modello.
%
%   log_zmp            DRY RUN: dice cosa farebbe, non cambia niente
%   log_zmp('on')      accende il logging delle quattro uscite
%   log_zmp('off')     lo rispegne
%
% PERCHE' NON SI USA LO SCOPE
%   Oggi x_zmp, y_zmp, margin e is_stable finiscono su XY Graph e Scope, che
%   non esportano niente nel workspace. Lo Scope avrebbe un suo salvataggio
%   ('SaveToWorkspace'/'DataLogging'), ma nome e formato dei parametri sono
%   cambiati fra le versioni di MATLAB. Il logging della PORTA funziona
%   uguale ovunque e non dipende da come e' configurato lo Scope.
%
% DOVE FINISCONO I DATI
%   In out.logsout, con i nomi zmp_x, zmp_y, zmp_margine, zmp_stabile.
%   Li rilegge stato_zmp.
%
% DIMENSIONI FISSATE (in memoria)
%   Tutti gli ingressi e le uscite del blocco hanno size -1 (ereditata).
%   Con il logging acceso sulle uscite, la propagazione delle dimensioni
%   all'indietro arriva fino alla Gain 3x3 aggiunta in Subsystem..Subsystem5
%   (forze f_* = Force_lf..Force_rr) e la risolve a scalare: errore
%   "Gain ... dims 1". 'on' fissa quindi f_* a [3 1] e le quattro uscite a 1,
%   che sono le dimensioni che il codice usa davvero (forces(3,i), uscite
%   scalari). 'off' le rimette a -1.
%
% GAIN DELLA RAMPA (causa vera dell'errore "Subsystem5/Gain ... dims [1]")
%   In ogni piede la forza della rampa passa da PS-Simulink Converter75 a
%   una Gain 3x3 (rotazione). Nei task senza rampa applica_terreno commenta
%   Contact_rampa: il convertitore resta scollegato, la sua dimensione non e'
%   definita e, con il logging acceso, Simulink la risolve a 1 -> errore.
%   La forza della rampa li' vale zero per costruzione, quindi 'on' mette le
%   sei Gain a 0 (elemento per elemento): f_* = forza del pavimento, esatta.
%   VALE SOLO PER TASK SENZA RAMPA (T1, T2, T3, T5, T6, T7). 'off' rimette
%   le Gain originali.
%
% NON SALVA IL MODELLO: solo set_param in memoria, come commenta_carico.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(stato), stato = 'dryrun'; end
if nargin < 2 || isempty(mdl),   mdl   = 'phantomx_sim_zero'; end
stato = lower(strtrim(char(stato)));
if ~ismember(stato, {'dryrun','on','off'})
    error('log_zmp:stato', 'Usa ''dryrun'', ''on'' oppure ''off''.');
end
if ~bdIsLoaded(mdl), load_system(mdl); end

blocco = [mdl '/MATLAB Function4'];
NOMI   = {'zmp_x', 'zmp_y', 'zmp_margine', 'zmp_stabile'};

if getSimulinkBlockHandle(blocco) == -1
    error('log_zmp:blocco', ...
        ['Non trovo "%s".\nIl calcolo ZMP sta solo in phantomx_sim_zero: in\n' ...
         'phantomx_sim_attitude non c''e'', quindi per C3 e C3P questa misura\n' ...
         'non esiste.'], blocco);
end

ph = get_param(blocco, 'PortHandles');
if numel(ph.Outport) ~= numel(NOMI)
    error('log_zmp:porte', ...
        ['"%s" ha %d uscite invece di %d.\nIn phantomx_sim_attitude esiste un ' ...
         'blocco con lo stesso nome che calcola il delta_z dell''assetto: ' ...
         'controlla di essere sul modello giusto.'], ...
         blocco, numel(ph.Outport), numel(NOMI));
end

info = struct('mdl', mdl, 'blocco', blocco, 'stato', stato, 'nomi', {NOMI});

if strcmp(stato, 'dryrun')
    fprintf('\n  [log_zmp] DRY RUN su %s\n', blocco);
    for k = 1:numel(NOMI)
        fprintf('     uscita %d -> %-14s   logging ora: %s\n', k, NOMI{k}, ...
                get_param(ph.Outport(k), 'DataLogging'));
    end
    fprintf('     SignalLogging del modello: %s (%s)\n', ...
            get_param(mdl, 'SignalLogging'), get_param(mdl, 'SignalLoggingName'));
    D = dati_chart(blocco);
    for k = 1:numel(D)
        fprintf('     %-10s size %s\n', D(k).Name, D(k).Props.Array.Size);
    end
    fprintf('\n');
    return
end

switch stato
    case 'on'
        set_param(mdl, 'SignalLogging', 'on');
        fissa_dimensioni(blocco, true);
        gain_rampa(mdl, true);
        for k = 1:numel(NOMI)
            set_param(ph.Outport(k), 'DataLogging', 'on', ...
                      'DataLoggingNameMode', 'Custom', 'DataLoggingName', NOMI{k});
        end
        fprintf('  [log_zmp] logging acceso: %s\n', strjoin(NOMI, ', '));
        fprintf('            dopo la run:  stato_zmp(<out>, ''T2'', ''C2'')\n');
    case 'off'
        for k = 1:numel(NOMI)
            set_param(ph.Outport(k), 'DataLogging', 'off');
        end
        fissa_dimensioni(blocco, false);
        gain_rampa(mdl, false);
        fprintf('  [log_zmp] logging spento su %s\n', blocco);
end
end

%% ================= helper =================
function D = dati_chart(blocco)
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blocco);
if isempty(ch)
    error('log_zmp:chart', 'Non trovo il codice MATLAB di "%s".', blocco);
end
D = find(ch, '-isa', 'Stateflow.Data');
end

function fissa_dimensioni(blocco, accendi)
%FISSA_DIMENSIONI  f_* a [3 1] e uscite a 1 (accendi), oppure tutto a -1.
D = dati_chart(blocco);
for k = 1:numel(D)
    nome = D(k).Name;
    if ~accendi
        nuova = '-1';
    elseif startsWith(nome, 'f_')
        nuova = '[3 1]';
    elseif strcmp(D(k).Scope, 'Output')
        nuova = '1';
    else
        continue                       % c_* restano come sono
    end
    D(k).Props.Array.Size = nuova;
end
if accendi
    fprintf('  [log_zmp] dimensioni fissate: f_* [3 1], uscite 1\n');
else
    fprintf('  [log_zmp] dimensioni rimesse a ereditate (-1)\n');
end
end

function gain_rampa(mdl, azzera)
%GAIN_RAMPA  Azzera (o ripristina) la Gain di rotazione della rampa nei 6 piedi.
PIEDI = {'Subsystem','Subsystem1','Subsystem2','Subsystem3','Subsystem4','Subsystem5'};
if azzera && evalin('base', 'exist(''TERRENO_ATTIVO'',''var'')') && ...
        ismember(evalin('base', 'TERRENO_ATTIVO'), {'T4','T4D'})
    warning('log_zmp:rampa', ['Terreno attivo ora: %s (con rampa). Va bene solo ' ...
          'se la run che segue e'' su un task senza rampa (es. script_T2).'], ...
          evalin('base', 'TERRENO_ATTIVO'));
end
n = 0;
for p = 1:numel(PIEDI)
    g = [mdl '/' PIEDI{p} '/Gain'];
    if getSimulinkBlockHandle(g) == -1, continue; end
    ud = get_param(g, 'UserData');
    if azzera
        if ~(isstruct(ud) && isfield(ud, 'gain_orig'))
            set_param(g, 'UserData', struct('gain_orig', get_param(g, 'Gain'), ...
                      'molt_orig', get_param(g, 'Multiplication')));
        end
        set_param(g, 'Gain', '0', 'Multiplication', 'Element-wise(K.*u)');
        n = n + 1;
    elseif isstruct(ud) && isfield(ud, 'gain_orig')
        set_param(g, 'Gain', ud.gain_orig, 'Multiplication', ud.molt_orig);
        set_param(g, 'UserData', []);
        n = n + 1;
    end
end
if azzera
    fprintf('  [log_zmp] Gain rampa azzerate in %d piedi (solo task senza rampa)\n', n);
else
    fprintf('  [log_zmp] Gain rampa ripristinate in %d piedi\n', n);
end
end
