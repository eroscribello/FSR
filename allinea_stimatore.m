function info = allinea_stimatore(mdl, verbose, modo)
%ALLINEA_STIMATORE  Mette nel blocco Inverse Dynamics le stesse inerzie del robot.
%
%   allinea_stimatore                          corregge phantomx_sim_zero
%   allinea_stimatore(mdl, true)               con riepilogo
%   allinea_stimatore(mdl, true, 'controlla')  NON scrive: dice solo come sta
%
%   Corregge le inerzie dei 25 Solid di Simscape -
%   quelle dell'URDF sono ~1000 volte troppo grandi - ma NON tocca il
%   rigidBodyTree. 
%
%   Riscrive massa e inerzia dei corpi di robotModel con gli stessi cfg.I_*
%   che applica_inerzie mette nei Solid, e forza la maschera del sottosistema
%   Inverse Dynamics a rivalutare, cosi' TreeStruct viene ricostruito.
%
%
if nargin < 1 || isempty(mdl),     mdl = 'phantomx_sim_zero'; end
if nargin < 2 || isempty(verbose), verbose = false; end
if nargin < 3 || isempty(modo),    modo = 'correggi'; end

cfg = phantomx_config();
if ~bdIsLoaded(mdl), load_system(mdl); end

%% ---- il sottosistema mascherato ----
blk = as_trova_blocco(mdl);

%% ---- l'albero ----
if ~evalin('base', 'exist(''robotModel'',''var'')')
    fprintf('  robotModel non c''e'': lo costruisco con Setup_robot_object...\n');
    evalin('base', 'Setup_robot_object');
end
rm = evalin('base', 'robotModel');

%% ---- quali corpi, e con quale inerzia ----
B = struct('nome',{}, 'I_ora',{}, 'I_giusta',{});
saltati = {};
for k = 1:numel(rm.Bodies)
    nome = rm.Bodies{k}.Name;
    if     startsWith(nome,'MP_BODY'), In = cfg.I_body;
    elseif startsWith(nome,'c1_'),     In = cfg.I_c1;
    elseif startsWith(nome,'c2_'),     In = cfg.I_c2;
    elseif startsWith(nome,'thigh_'),  In = cfg.I_thigh;
    elseif startsWith(nome,'tibia_'),  In = cfg.I_tibia;
    else,  saltati{end+1} = nome; continue                         %#ok<AGROW>
    end
    B(end+1) = struct('nome', nome, ...                            %#ok<AGROW>
                      'I_ora', rm.Bodies{k}.Inertia, ...
                      'I_giusta', [In(1) In(2) In(3) 0 0 0]);
end

if numel(B) ~= 25
    error('allinea_stimatore:corpi', ...
        ['Nell''albero ho riconosciuto %d corpi del robot, ne attendevo 25.\n' ...
         'Non riconosciuti: %s\n' ...
         'L''URDF e'' cambiato: controlla la mappa dei nomi prima di correggere.'], ...
        numel(B), strjoin(saltati, ', '));
end

gia = false(1, numel(B));
for k = 1:numel(B)
    gia(k) = as_uguali(B(k).I_ora, B(k).I_giusta);
end

info = struct('modello', mdl, 'blocco', blk, 'corpi', numel(B), ...
              'gia_allineati', sum(gia), 'allineati', 0, 'modo', modo, ...
              'non_riconosciuti', {saltati});

%% ---- solo controllo ----
if strcmp(modo,'controlla')
    fprintf('\n--- STIMATORE (solo controllo): %d corpi, %d gia'' allineati ---\n', ...
            numel(B), sum(gia));
    fprintf('  %-12s %-38s %s\n', 'corpo', 'nell''albero', 'come nel robot');
    for k = [1 2 numel(B)]
        fprintf('  %-12s %-38s %s\n', B(k).nome, mat2str(B(k).I_ora,4), ...
                mat2str(B(k).I_giusta,4));
    end
    if sum(gia) == numel(B)
        fprintf('  stimatore e robot sono allineati.\n\n');
    else
        fprintf(2,'  DISALLINEATI: il flag di contatto di C2 non e'' affidabile.\n\n');
    end
    return
end

%% ---- correzione dell'albero ----
for k = 1:numel(B)
    b = getBody(rm, B(k).nome);
    b.Inertia = B(k).I_giusta;
    if ~as_uguali(getBody(rm, B(k).nome).Inertia, B(k).I_giusta)
        nuovo = copy(getBody(rm, B(k).nome));
        nuovo.Inertia = B(k).I_giusta;
        replaceBody(rm, B(k).nome, nuovo);
    end
end

male = {};
for k = 1:numel(B)
    if ~as_uguali(getBody(rm, B(k).nome).Inertia, B(k).I_giusta)
        male{end+1} = B(k).nome;                                   %#ok<AGROW>
    end
end
if ~isempty(male)
    error('allinea_stimatore:scrittura', ...
        ['Non sono riuscito a scrivere l''inerzia di %d corpi (%s).\n' ...
         'Senza questo l''allineamento non c''e'': mi fermo invece di dire\n' ...
         'che ho corretto.'], numel(male), strjoin(male(1:min(3,end)), ', '));
end

assignin('base', 'robotModel', rm);
info.allineati = numel(B);

%% ---- farlo arrivare nella maschera ----
par = as_nome_parametro(blk);
set_param(blk, par, get_param(blk, par));

%% ---- la verifica che conta: cosa e' arrivato al blocco ----
try
    w = get_param(blk, 'MaskWSVariables');
    j = find(strcmp({w.Name}, 'RigidBodyTree'), 1);
    if isempty(j)
        error('allinea_stimatore:maschera', ...
            'Nel mask workspace non c''e'' RigidBodyTree: non posso verificare.');
    end
    rmb = w(j).Value;
    fuori = {};
    for k = 1:numel(B)
        if ~as_uguali(getBody(rmb, B(k).nome).Inertia, B(k).I_giusta)
            fuori{end+1} = B(k).nome;                              %#ok<AGROW>
        end
    end
    if ~isempty(fuori)
        error('allinea_stimatore:nonArrivato', ...
            ['L''albero nel workspace e'' corretto ma il blocco usa ancora\n' ...
             'quello vecchio (%d corpi fuori, es. %s). La maschera non ha\n' ...
             'rivalutato: apri il blocco Inverse Dynamics, riscrivi robotModel\n' ...
             'nel campo Rigid body tree e dai OK, poi rilancia questa funzione\n' ...
             'in modo ''controlla''.'], numel(fuori), fuori{1});
    end
    info.verificato = true;
catch err
    if strcmp(err.identifier,'allinea_stimatore:nonArrivato'), rethrow(err); end
    fprintf(2,'  Non ho potuto verificare il mask workspace (%s).\n', err.message);
    fprintf(2,'  L''albero nel base workspace E'' corretto, ma non posso dire\n');
    fprintf(2,'  che sia arrivato al blocco.\n');
    info.verificato = false;
end

if verbose
    fprintf('\n--- STIMATORE ALLINEATO: %d corpi ---\n', numel(B));
    for k = [1 2 numel(B)]
        fprintf('  %-12s %-38s -> %s\n', B(k).nome, mat2str(B(k).I_ora,4), ...
                mat2str(B(k).I_giusta,4));
    end
    fprintf('  blocco: %s\n', strrep(blk,[mdl '/'],''));
    fprintf('  (in memoria: il .slx NON e'' stato salvato)\n\n');
end
end

%% ========================= helper =========================
function blk = as_trova_blocco(mdl)
cand = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                   'BlockType','SubSystem');
blk = '';
for k = 1:numel(cand)
    m = Simulink.Mask.get(cand{k});
    if isempty(m), continue; end
    if any(strcmp({m.Parameters.Name}, 'RigidBodyTree'))
        blk = cand{k};
        return
    end
end
if isempty(blk)
    error('allinea_stimatore:blocco', ...
        ['In %s non c''e'' nessun sottosistema con un parametro di maschera\n' ...
         'RigidBodyTree. O il blocco Inverse Dynamics non c''e'' piu'', o e''\n' ...
         'stato sostituito: guarda prima di correggere.'], mdl);
end
end

function par = as_nome_parametro(blk)
m = Simulink.Mask.get(blk);
par = 'RigidBodyTree';
if ~any(strcmp({m.Parameters.Name}, par))
    par = m.Parameters(1).Name;
end
end

function tf = as_uguali(a, b)
a = double(a(:));  b = double(b(:));
tf = numel(a) == numel(b) && all(abs(a - b) <= 1e-12 + 1e-9*abs(b));
end
