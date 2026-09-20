function info = applica_terreno(task, verbose, mdl)
%APPLICA_TERRENO Configura gli elementi del terreno per la simulazione.
%
% USO:
%   applica_terreno('T1')    -> Piano liscio
%   applica_terreno('T4')    -> Piano liscio + Rampa
%   applica_terreno('T5')    -> Piano liscio + Ostacolo 1
%   applica_terreno('T6')    -> Piano liscio + Tutti gli ostacoli (1..7)
%   applica_terreno('TUTTO') -> Tutti gli elementi attivi

if nargin < 1 || isempty(task),    task = 'T1'; end
if nargin < 2 || isempty(verbose), verbose = true; end
if nargin < 3 || isempty(mdl),     mdl = 'phantomx_sim_zero'; end

task = upper(strtrim(char(task)));
if ~bdIsLoaded(mdl), load_system(mdl); end

cfg = phantomx_config();

% Elenco esatto dei 6 subsystem dei piedi
PIEDI = {'Subsystem', 'Subsystem1', 'Subsystem2', 'Subsystem3', 'Subsystem4', 'Subsystem5'};

%% 1. Catalogo elementi terreno (Nome -> [Solido, RigidTransform])
cat = struct();
cat.pavimento = struct('solido', 'Solid_Pavimento', 'rt', 'Rigid Transform_Pavimento');
cat.rampa     = struct('solido', 'Solid_Rampa',     'rt', 'Rigid Transform_Rampa');

% Inserimento dinamico per i 7 ostacoli
for k = 1:7
    tag = sprintf('ostacolo%d', k);
    cat.(tag) = struct('solido', sprintf('Solid_Ostacolo%d', k), ...
                       'rt',     sprintf('Rigid Transform_Ostacolo%d', k));
end

tutti_gli_elementi = fieldnames(cat);

%% 2. Selezione elementi attivi per Task
switch task
    case {'T1','T2','T3','T7'}, elementi_attivi = {'pavimento'};
    case 'T4',                  elementi_attivi = {'pavimento', 'rampa'};
    case 'T5',                  elementi_attivi = {'pavimento', 'ostacolo1'};
    case 'T6',                  elementi_attivi = [{'pavimento'}, arrayfun(@(k) sprintf('ostacolo%d',k), 1:7, 'UniformOutput',false)];
    case 'TUTTO',               elementi_attivi = tutti_gli_elementi.';
    otherwise
        error('applica_terreno:task', 'Task ''%s'' non previsto.', task);
end

%% 3. Normalizzazione: Garantire Rigid Transform SEMPRE attivi
for i = 1:numel(tutti_gli_elementi)
    elem = tutti_gli_elementi{i};
    try
        set_param([mdl '/' cat.(elem).rt], 'Commented', 'off');
    catch
        % Ignora se il nome del Rigid Transform segue una numerazione generica
    end
end

%% 4. Quota ostacoli e posizionamento rampa
lisciOn = ismember('pavimento', elementi_attivi);
ost_dz  = ternario(lisciOn, getfield_default(cfg, 'terreno.ost_dz', 0.025), 0);
assignin('base', 'ost_dz', ost_dz);

% Offset quota per gli ostacoli
for k = 1:7
    elem = sprintf('ostacolo%d', k);
    try
        rt_path = [mdl '/' cat.(elem).rt];
        if ost_dz == 0
            set_param(rt_path, 'TranslationCartesianOffset', 'floor_off');
        else
            set_param(rt_path, 'TranslationCartesianOffset', 'floor_off(:).'' + [0 0 ost_dz]');
        end
    catch
    end
end

% Posa della Rampa (se attiva)
if ismember('rampa', elementi_attivi) && isfield(cfg, 'terreno')
    try
        rt_rampa = [mdl '/' cat.rampa.rt];
        set_param(rt_rampa, 'TranslationCartesianOffset', mat2str(cfg.terreno.rampa_pos));
        set_param(rt_rampa, 'RotationMethod',          'StandardAxis');
        set_param(rt_rampa, 'RotationStandardAxis',    '+Y');
        set_param(rt_rampa, 'RotationAngle',           num2str(cfg.terreno.rampa_gradi));
    catch ME
        if verbose, warning('Impossibile impostare la posa della rampa: %s', ME.message); end
    end
end

%% 5. Attivazione / Disattivazione Solidi e Contatti
for i = 1:numel(tutti_gli_elementi)
    elem = tutti_gli_elementi{i};
    is_attivo = ismember(elem, elementi_attivi);
    stato_commento = ternario(is_attivo, 'off', 'on'); % 'off' = blocco attivo!
    
    % A. Solido nel Terreno
    try
        set_param([mdl '/' cat.(elem).solido], 'Commented', stato_commento);
    catch
        if verbose, warning('Solido non trovato: %s', cat.(elem).solido); end
    end
    
    % B. Contatto dedicato in ciascuno dei 6 piedi
    for p = 1:numel(PIEDI)
        blocco_contatto = sprintf('%s/%s/Contact_%s', mdl, PIEDI{p}, elem);
        try
            set_param(blocco_contatto, 'Commented', stato_commento);
        catch
            if verbose
                warning('Contatto non trovato: %s', blocco_contatto);
            end
        end
    end
end

%% 6. Salvataggio stato nel Base Workspace
assignin('base', 'TERRENO_ATTIVO', task);
info = struct('task', task, 'attivi', {elementi_attivi}, 'modello', mdl);

if verbose
    fprintf('\n--- TERRENO APPLICATO: %s ---\n', task);
    fprintf('  Piedi aggiornati: %s\n', strjoin(PIEDI, ', '));
    fprintf('  Elementi attivi: %s\n', strjoin(elementi_attivi, ', '));
    fprintf('  (Il modello .slx NON e'' stato salvato su disco)\n\n');
end

end

%% ==================== FUNZIONI AUSILIARIE ====================
function v = ternario(cond, a, b)
    if cond, v = a; else, v = b; end
end

function val = getfield_default(s, fieldpath, default_val)
    try
        val = eval(['s.' fieldpath]);
    catch
        val = default_val;
    end
end