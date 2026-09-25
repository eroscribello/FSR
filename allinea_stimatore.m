function info = allinea_stimatore(mdl, verbose, modo)
%ALLINEA_STIMATORE  Mette nel blocco Inverse Dynamics le stesse inerzie del robot.
%
%   allinea_stimatore                          corregge phantomx_sim_zero
%   allinea_stimatore(mdl, true)               con riepilogo
%   allinea_stimatore(mdl, true, 'controlla')  NON scrive: dice solo come sta
%
% PERCHE' ESISTE
%   [MISURATO 25/9] Il flag di contatto di C2 non e' una soglia sulla coppia:
%   il modello confronta  |tau_misurata - tau_attesa| > cfg.c2.soglia_tau,
%   e tau_attesa la calcola il blocco Inverse Dynamics, cioe' un
%   rigidBodyTree costruito da Setup_robot_object con importrobot dall'URDF.
%
%   Dal 22/9 applica_inerzie corregge le inerzie dei 25 Solid di Simscape -
%   quelle dell'URDF sono ~1000 volte troppo grandi - ma NON tocca il
%   rigidBodyTree. Da quel giorno il robot simulato e il modello che ne
%   prevede la coppia sono due robot diversi, e la differenza fra coppia
%   misurata e coppia attesa e' grande sempre, con o senza contatto.
%
%   Misura, stessa run T2 C2 di 10 s (results/diagnostica/valida_soglia.csv):
%
%                                   disallineati   allineati
%     flag alto con la zampa in aria     95.1%        8.3%
%     errore in appoggio                  7.0%        0.9%
%     errore di un flag SEMPRE alto       7.0%        8.7%
%     voli senza reset di z_ext          20 / 60      1 / 60
%
%   Nella colonna di sinistra il flag non fa meglio di una costante: non
%   porta informazione. E in ricerca_terreno il ramo che azzera z_ext e
%   z_hold sta dentro  if c(i) == 0, quindi con il flag incollato a 1 il
%   comando degenera in  min(z, z_hold(t_reset)), una costante.
%   CONSEGUENZA: dal 22/9 C2 non cerca il terreno. Le righe C2 delle tabelle
%   prodotte dopo quella data non descrivono C2.
%
% COSA FA
%   Riscrive massa e inerzia dei corpi di robotModel con gli stessi cfg.I_*
%   che applica_inerzie mette nei Solid, e forza la maschera del sottosistema
%   Inverse Dynamics a rivalutare, cosi' TreeStruct viene ricostruito.
%   Poi VERIFICA di averlo fatto: rilegge il mask workspace e confronta. Una
%   correzione che non attecchisce in silenzio sarebbe peggio di nessuna
%   correzione - e' esattamente l'errore che stiamo riparando.
%
% COSA NON FA
%   Nessun save_system. Come applica_terreno e applica_inerzie, scrive solo
%   in memoria: il .slx del collega resta quello committato.
%
% DOVE VA CHIAMATA
%   Subito dopo applica_inerzie, in ogni script di campagna:
%       applica_terreno('T5', false, mdl);
%       applica_inerzie(mdl);
%       allinea_stimatore(mdl);     % <-- le due copie devono restare pari
%
%   Le due funzioni vanno sempre insieme: correggerne una sola e' come essere
%   rimasti al 22/9. Se un giorno le inerzie verranno corrette direttamente
%   nel .slx, va corretto anche l'URDF, o questa resta necessaria.
%
% ATTENZIONE A clear all
%   init_gait ricostruisce robotModel solo se non esiste
%   (if ~exist('robotModel','var')). Dopo un clear all l'albero torna quello
%   dell'URDF: per questo la chiamata sta negli script, non una volta sola.
%
% Progetto FSR PhantomX - A. Russo

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
% Stessa mappa di applica_inerzie, sugli stessi prefissi di nome. Se cambia
% li', deve cambiare qui: sono la stessa decisione scritta due volte, e
% questa riga di commento e' l'unico legame che le tiene insieme.
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

% La stessa guardia di applica_inerzie, per lo stesso motivo: allinearne 24
% su 25 lascerebbe un disallineamento piu' difficile da vedere di questo.
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
% getBody restituisce un riferimento nelle versioni recenti e una copia in
% quelle vecchie. Invece di fidarsi, si scrive e si rilegge: se non ha
% attecchito si ripiega su replaceBody, che e' la strada documentata.
for k = 1:numel(B)
    b = getBody(rm, B(k).nome);
    b.Inertia = B(k).I_giusta;
    if ~as_uguali(getBody(rm, B(k).nome).Inertia, B(k).I_giusta)
        nuovo = copy(getBody(rm, B(k).nome));
        nuovo.Inertia = B(k).I_giusta;
        replaceBody(rm, B(k).nome, nuovo);
    end
end

% controllo sull'oggetto, prima ancora di portarlo nel modello
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
% Il parametro di maschera vale la stringa 'robotModel': riassegnandolo, la
% MaskInitialization rivaluta e TreeStruct viene ricostruito dall'albero
% corretto. E' un set_param su un parametro di maschera, non un save.
par = as_nome_parametro(blk);
set_param(blk, par, get_param(blk, par));

%% ---- la verifica che conta: cosa e' arrivato al blocco ----
% Tutto il resto puo' andare bene e questa fallire lo stesso: se TreeStruct
% non si e' ricostruito, il blocco continua a usare le inerzie dell'URDF e
% noi crederemmo di aver risolto. E' il modo tipico in cui un problema come
% questo sopravvive alla sua correzione.
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
%AS_TROVA_BLOCCO  Il sottosistema mascherato che porta il rigidBodyTree.
%   Si cerca per PARAMETRO, non per nome: un blocco rinominato non deve far
%   fallire in silenzio una funzione che serve proprio a evitare i silenzi.
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
