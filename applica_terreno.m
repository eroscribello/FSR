function info = applica_terreno(task, verbose, mdl, opt)
% APPLICA_TERRENO Configura gli elementi del terreno per la simulazione.
%
% USO:
%   applica_terreno('T1')    -> Piano liscio
%   applica_terreno('T4')    -> Piano liscio + Rampa (salita)
%   applica_terreno('T4D')   -> Piano liscio + Dosso (salita, cima, discesa)
%   applica_terreno('T5')    -> Piano liscio + Ostacolo 1
%   applica_terreno('T6')    -> Piano liscio + Tutti gli ostacoli (1..7)
%
% OPZIONI (quarto argomento)
%   .rampa_gradi   inclinazione della rampa di T4 [gradi]. Default da cfg.
%
% RAMPA E DOSSO USANO LO STESSO SOLIDO (Solid_Rampa)
%   T4  : il cubo 8 x 8 x 0.1 del pavimento, inclinato.
%   T4D : un prisma a dosso, scritto come STL da dosso_profilo e caricato nel
%         solido cambiandone il file. Il contatto dei piedi e' collegato al
%         solido, quindi segue la geometria senza toccare il .slx.
%   Ogni chiamata rimette il file giusto: T4 dopo T4D torna al cubo.
if nargin < 1 || isempty(task),    task = 'T1'; end
if nargin < 2 || isempty(verbose), verbose = true; end
if nargin < 3 || isempty(mdl),     mdl = 'phantomx_sim_zero'; end
if nargin < 4, opt = struct(); end

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

mancanti = {};
for i = 1:numel(tutti_gli_elementi)
    elem = tutti_gli_elementi{i};
    cat.(elem).rt_path = rt_collegato(mdl, cat.(elem).solido);
    if isempty(cat.(elem).rt_path)
        if getSimulinkBlockHandle([mdl '/' cat.(elem).rt]) > 0
            cat.(elem).rt_path = [mdl '/' cat.(elem).rt];
        end
    end
end

%% 2. Selezione elementi attivi per Task
switch task
    case {'T1','T2','T3','T7'}, elementi_attivi = {'pavimento'};
    case {'T4','T4D'},          elementi_attivi = {'pavimento', 'rampa'};
    case 'T5',                  elementi_attivi = {'pavimento', 'ostacolo1'};
    case 'T6',                  elementi_attivi = [{'pavimento'}, arrayfun(@(k) sprintf('ostacolo%d',k), 1:7, 'UniformOutput',false)];
    case 'TUTTO',               elementi_attivi = tutti_gli_elementi.';
    otherwise
        error('applica_terreno:task', 'Task ''%s'' non previsto.', task);
end

%% 3. Normalizzazione: Garantire Rigid Transform sempre attivi
for i = 1:numel(tutti_gli_elementi)
    elem = tutti_gli_elementi{i};
    if isempty(cat.(elem).rt_path), continue; end
    set_param(cat.(elem).rt_path, 'Commented', 'off');
end

%% 4. Quota ostacoli e posizionamento rampa
lisciOn = ismember('pavimento', elementi_attivi);
ost_dz  = ternario(lisciOn, getfield_default(cfg, 'terreno.ost_dz', 0.025), 0);
assignin('base', 'ost_dz', ost_dz);

ost_dx = zeros(1,7);
if isfield(cfg,'terreno') && isfield(cfg.terreno,'ost_dx') && isfield(cfg.terreno.ost_dx, task)
    ost_dx(1:numel(cfg.terreno.ost_dx.(task))) = cfg.terreno.ost_dx.(task);
end
for k = 1:7
    elem = sprintf('ostacolo%d', k);
    rt_path = cat.(elem).rt_path;
    if isempty(rt_path)
        mancanti{end+1} = sprintf('Rigid Transform di %s', cat.(elem).solido); 
        continue
    end
    try
        if ost_dz == 0 && ost_dx(k) == 0
            set_param(rt_path, 'TranslationCartesianOffset', 'floor_off');
        elseif ost_dx(k) == 0
            set_param(rt_path, 'TranslationCartesianOffset', 'floor_off(:).'' + [0 0 ost_dz]');
        else
            set_param(rt_path, 'TranslationCartesianOffset', ...
                sprintf('floor_off(:).'' + [%.6g 0 ost_dz]', ost_dx(k)));
        end
    catch ME
        mancanti{end+1} = sprintf('%s (%s): %s', cat.(elem).solido, rt_path, ME.message);
    end
end
if verbose
    if any(ost_dx), fprintf('  ostacoli spostati in x: %s m\n', mat2str(ost_dx)); end
    for i = 1:numel(elementi_attivi)
        e = elementi_attivi{i};
        fprintf('  %-18s -> %s\n', cat.(e).solido, ternario(isempty(cat.(e).rt_path), ...
                '(nessun Rigid Transform collegato)', strrep(cat.(e).rt_path, newline, ' ')));
    end
end

if ismember('rampa', elementi_attivi) && isfield(cfg, 'terreno')
    rt_rampa  = cat.rampa.rt_path;
    sol_rampa = [mdl '/' cat.rampa.solido];
    if isempty(rt_rampa)
        mancanti{end+1} = sprintf('Rigid Transform di %s', cat.rampa.solido);   
    else
        try
            if strcmp(task, 'T4D')
                % dosso scritto direttamente in coordinate mondo
                G   = dosso_profilo(cfg.terreno.dosso, cfg.floor_top);
                stl = scrivi_dosso(G, cfg.terreno.dosso.larghezza);
                set_param(sol_rampa, 'ExtGeomFileName', stl);
                set_param(rt_rampa, 'TranslationCartesianOffset', '[0 0 0]');
                set_param(rt_rampa, 'RotationMethod', 'None');
                if verbose
                    fprintf('  dosso: salita %.3f-%.3f m, cima fino a %.3f, discesa fino a %.3f (H %.0f mm)\n', ...
                            G.x(1), G.x(2), G.x(3), G.x(4), 1e3*G.H);
                end
            else
                gradi = cfg.terreno.rampa_gradi;
                if isfield(opt,'rampa_gradi') && ~isempty(opt.rampa_gradi)
                    gradi = opt.rampa_gradi;
                end
                t_blocco = posa_rampa(gradi, cfg.terreno.rampa_x_inizio, cfg.floor_top);
                set_param(sol_rampa, 'ExtGeomFileName', cfg.terreno.rampa_stl);
                set_param(rt_rampa, 'TranslationCartesianOffset', mat2str(t_blocco, 6));
                set_param(rt_rampa, 'RotationMethod',          'StandardAxis');
                set_param(rt_rampa, 'RotationStandardAxis',    '+Y');
                set_param(rt_rampa, 'RotationAngle',           num2str(gradi));
                if verbose
                    fprintf('  rampa: %g gradi, esce dal pavimento a x = %.3f m (offset blocco %s)\n', ...
                            gradi, cfg.terreno.rampa_x_inizio, mat2str(t_blocco, 4));
                end
            end
        catch ME
            mancanti{end+1} = sprintf('%s (%s): %s', cat.rampa.solido, rt_rampa, ME.message); 
        end
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

if ~isempty(mancanti)
    attivi_mancanti = false;
    for i = 1:numel(elementi_attivi)
        attivi_mancanti = attivi_mancanti || ...
            any(contains(mancanti, cat.(elementi_attivi{i}).solido));
    end
    msg = sprintf('applica_terreno(%s): non applicato a\n  %s', task, strjoin(mancanti, '\n  '));
    if attivi_mancanti && (any(ost_dx) || ismember('rampa', elementi_attivi))
        error('applica_terreno:mancanti', '%s', msg);
    else
        warning('applica_terreno:mancanti', '%s', msg);
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

function rt = rt_collegato(mdl, solido)
%RT_COLLEGATO  Il Rigid Transform collegato direttamente al solido, o ''.
rt  = '';
sol = [mdl '/' solido];
if getSimulinkBlockHandle(sol) < 0, return; end
hs = get_param(sol, 'Handle');
ph = get_param(sol, 'PortHandles');
for p = [ph.LConn ph.RConn]
    l = get_param(p, 'Line');
    if l <= 0, continue; end
    h = [get_param(l,'SrcBlockHandle'), reshape(get_param(l,'DstBlockHandle'),1,[])];
    for b = h(h > 0 & h ~= hs)
        ref = regexprep(get_param(b, 'ReferenceBlock'), '\s+', ' ');
        nm  = regexprep(get_param(b, 'Name'),           '\s+', ' ');
        if contains(ref, 'Rigid Transform') || startsWith(nm, 'Rigid Transform')
            rt = getfullname(b);
            return
        end
    end
end
end

function t = posa_rampa(gradi, x_e, z_e)
%POSA_RAMPA  Offset del Rigid Transform perche' la rampa esca dal pavimento in x_e.
a  = deg2rad(gradi);
Ry = @(q) [cos(q) 0 sin(q); 0 1 0; -sin(q) 0 cos(q)];
p  = [x_e; 0; z_e] - Ry(-a) * [-0.5; 0; 0.05];
t  = (-Ry(a) * p).';
end

function f = scrivi_dosso(G, larghezza)
%SCRIVI_DOSSO  STL del prisma a dosso, estruso in y su +-larghezza/2.
P = G.P;  n = size(P,1);  w = larghezza/2;
V = [P(:,1) -w*ones(n,1) P(:,2);  P(:,1) w*ones(n,1) P(:,2)];
F = zeros(0,3);
for k = 2:n-1                                   % le due facce laterali
    F(end+1,:) = [1 k k+1];                     %#ok<AGROW>
    F(end+1,:) = [n+1 n+k n+k+1];               %#ok<AGROW>
end
for k = 1:n                                     % il contorno
    j = mod(k,n) + 1;
    F(end+1,:) = [k j n+j];                     %#ok<AGROW>
    F(end+1,:) = [k n+j n+k];                   %#ok<AGROW>
end
c = mean(V,1);                                  % normali verso l'esterno
for i = 1:size(F,1)
    a = V(F(i,1),:);  b = V(F(i,2),:);  e = V(F(i,3),:);
    if dot(cross(b-a, e-a), (a+b+e)/3 - c) < 0, F(i,[2 3]) = F(i,[3 2]); end
end
radice = fileparts(which('applica_terreno'));
f = fullfile(radice, 'simscape', 'props', 'T4D_dosso.stl');
stlwrite(triangulation(F, V), f);
end

