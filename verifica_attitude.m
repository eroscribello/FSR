function info = verifica_attitude(mdl_c3, mdl_rif)
% Verifica se il modello di C3 regge la catena di misura della campagna

if nargin < 1 || isempty(mdl_c3),  mdl_c3  = 'phantomx_sim_attitude'; end
if nargin < 2 || isempty(mdl_rif), mdl_rif = 'phantomx_sim_zero';     end

M = {mdl_c3, mdl_rif};
aperti_prima = false(1,2);
for k = 1:2
    aperti_prima(k) = bdIsLoaded(M{k});
    if ~aperti_prima(k)
        try
            load_system(M{k});
        catch e
            error('verifica_attitude:carica', ...
                'Non riesco a caricare %s:\n  %s', M{k}, e.message);
        end
    end
end

fprintf('\n===============================================================\n');
fprintf('  VERIFICA ATTITUDE - %s  contro  %s\n', mdl_c3, mdl_rif);
fprintf('===============================================================\n');

R = struct('punto', {}, 'c3', {}, 'rif', {}, 'esito', {}, 'nota', {});

%% ---- 1. log di Simscape ----
v = cell(1,2);
for k = 1:2
    try v{k} = get_param(M{k}, 'SimscapeLogType'); catch, v{k} = '<assente>'; end
end
R(end+1) = va_riga('1. SimscapeLogType', v{1}, v{2}, ...
    strcmp(v{1},'all'), ...
    'se non e'' ''all'' il simlog e'' vuoto: non esce NESSUNA metrica');

%% ---- 2. To Workspace ----
tw = cell(1,2);
for k = 1:2, tw{k} = va_tows(M{k}); end
serve = {'torque_sens', 'Fleg'};
manca = setdiff(serve, tw{1});
R(end+1) = va_riga('2. To Workspace', va_lista(tw{1}), va_lista(tw{2}), ...
    isempty(manca), ...
    va_testo_manca(manca));

%% ---- 3. Solid con inerzia ----
ns = nan(1,2); msg3 = '';
for k = 1:2
    try
        i3 = applica_inerzie(M{k}, false, 'controlla');
        ns(k) = i3.solidi;
    catch e
        msg3 = e.message;
    end
end
R(end+1) = va_riga('3. Solid con inerzia', va_num(ns(1)), va_num(ns(2)), ...
    ~isnan(ns(1)) && ns(1) == ns(2), ...
    va_primo(msg3, 'un numero diverso = robot diverso: il confronto non regge'));

%% ---- 4. blocco Inverse Dynamics (maschera RigidBodyTree) ----
has = false(1,2); nb = zeros(1,2);
for k = 1:2
    nb(k) = va_conta_rbt(M{k});
    has(k) = nb(k) > 0;
end
if has(1)
    nota4 = 'C3 USA il flag di contatto: allinea_stimatore va chiamato SEMPRE';
else
    nota4 = 'C3 NON usa il flag di contatto: allinea_stimatore non serve, e OVERRIDE_C2 = false';
end
R(end+1) = va_riga('4. Inverse Dynamics', va_si(has(1)), va_si(has(2)), ...
    true, nota4);      % nessuno dei due esiti e' un errore: e' informazione

%% ---- 4-bis. allinea_stimatore riesce davvero a lavorarci ----
if has(1)
    ok4b = false; nota4b = '';
    try
        i4 = allinea_stimatore(mdl_c3, false, 'controlla');
        ok4b = true;
        nota4b = sprintf('%d corpi, %d gia'' allineati', i4.corpi, i4.gia_allineati);
    catch e
        nota4b = e.message;
    end
    R(end+1) = va_riga('4b. allinea_stimatore', va_si(ok4b), '-', ok4b, nota4b);
end

%% ---- 5. chi legge c2_par ----
u = cell(1,2);
for k = 1:2, u{k} = va_variabili(M{k}); end
usa_c2 = any(strcmp(u{1}, 'c2_par'));
if usa_c2
    nota5 = 'OVERRIDE_C2 ha effetto su C3: da decidere se tiene o sostituisce la ricerca del terreno';
else
    nota5 = 'C3 ignora OVERRIDE_C2: la ricerca del terreno non e'' nel suo percorso';
end
R(end+1) = va_riga('5. usa c2_par', va_si(usa_c2), ...
    va_si(any(strcmp(u{2}, 'c2_par'))), true, nota5);

%% ---- 6. InitFcn ----
f = cell(1,2);
for k = 1:2
    try f{k} = get_param(M{k}, 'InitFcn'); catch, f{k} = ''; end
end
ha_ig = contains(f{1}, 'init_gait');
R(end+1) = va_riga('6. InitFcn chiama init_gait', va_si(ha_ig), ...
    va_si(contains(f{2}, 'init_gait')), ha_ig, ...
    'senza init_gait gli OVERRIDE_* non arrivano al modello');

%% ---- 7. giunti rotoidali (controllo di sanita') ----
nr = zeros(1,2);
for k = 1:2
    nr(k) = numel(find_system(M{k}, 'LookUnderMasks','all', 'FollowLinks','on', ...
                  'ReferenceBlock', 'sm_lib/Joints/Revolute Joint'));
end
if all(nr == 0)
    nota7 = 'ReferenceBlock non riconosciuto in questa versione: controllo non informativo';
else
    nota7 = 'un numero diverso = meccanismo diverso';
end
R(end+1) = va_riga('7. Revolute Joint', va_num(nr(1)), va_num(nr(2)), ...
    nr(1) == nr(2), nota7);

%% ================= stampa =================
fprintf('\n  %-28s %-22s %-22s %s\n', 'punto', mdl_c3, mdl_rif, '');
fprintf('  %s\n', repmat('-', 1, 80));
for k = 1:numel(R)
    fprintf('  %-28s %-22s %-22s %s\n', R(k).punto, ...
            va_taglia(R(k).c3, 22), va_taglia(R(k).rif, 22), R(k).esito);
end

fprintf('\n  NOTE\n');
for k = 1:numel(R)
    if ~isempty(R(k).nota)
        fprintf('    %-28s %s\n', R(k).punto, R(k).nota);
    end
end

%% ================= verdetto =================
blocca = {};
for k = 1:numel(R)
    if strcmp(R(k).esito, 'NO'), blocca{end+1} = R(k).punto; end %#ok<AGROW>
end

fprintf('\n  ---------------------------------------------------------------\n');
if isempty(blocca)
    fprintf('  LA CATENA REGGE.\n');
    fprintf('  Gli script dei task funzionano su %s cambiando\n', mdl_c3);
    fprintf('  il nome del modello nella riga in testa. adatta_simscape non\n');
    fprintf('  va toccato: legge l''impianto Simscape, non il controllore.\n');
else
    fprintf('  DA SISTEMARE PRIMA DI LANCIARE:\n');
    for k = 1:numel(blocca), fprintf('    - %s\n', blocca{k}); end
end
fprintf('\n  RESTA COMUNQUE DA FARE, qualunque sia l''esito qui sopra:\n');
fprintf('    - il nome del controllore e'' binario (C1/C2): lanciando C3\n');
fprintf('      cosi'' com''e'' SOVRASCRIVI results/T*_C2.csv\n');
fprintf('    - decidere OVERRIDE_C2 per C3 (vedi punto 5)\n');
fprintf('  ---------------------------------------------------------------\n\n');

info = struct('modello', mdl_c3, 'riferimento', mdl_rif, ...
              'punti', {R}, 'bloccanti', {blocca}, ...
              'usa_flag_contatto', has(1), 'usa_c2_par', usa_c2);

%% ---- chiudi solo quello che abbiamo aperto noi ----
for k = 1:2
    if ~aperti_prima(k) && bdIsLoaded(M{k})
        if strcmp(get_param(M{k}, 'Dirty'), 'on')
            warning('verifica_attitude:sporco', ...
                ['%s risulta modificato e resta aperto: NON salvarlo.\n' ...
                 'Chiudilo con  bdclose(''%s'')  scartando le modifiche.'], ...
                M{k}, M{k});
        else
            bdclose(M{k});
        end
    end
end
end

%% ================= helper =================
function r = va_riga(punto, c3, rif, ok, nota)
if ok, e = 'ok'; else, e = 'NO'; end
r = struct('punto', punto, 'c3', c3, 'rif', rif, 'esito', e, 'nota', nota);
end

function n = va_tows(mdl)
%VA_TOWS  Nomi delle variabili scritte dai To Workspace.
b = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                'BlockType','ToWorkspace');
n = {};
for k = 1:numel(b)
    try n{end+1} = get_param(b{k}, 'VariableName'); end %#ok<AGROW,TRYNC>
end
n = unique(n);
end

function u = va_variabili(mdl)
%VA_VARIABILI  Variabili del workspace usate dal modello.
u = {};
try
    v = Simulink.findVars(mdl, 'SearchMethod', 'cached');
    if isempty(v), v = Simulink.findVars(mdl); end
    u = {v.Name};
catch
    b = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Type','block');
    for k = 1:numel(b)
        for p = {'Value','Gain','Table'}
            try
                s = get_param(b{k}, p{1});
                if ischar(s) && ~isempty(s), u{end+1} = s; end %#ok<AGROW>
            catch
            end
        end
    end
    u = u(contains(u, 'c2_par'));
    if ~isempty(u), u = {'c2_par'}; end
end
u = unique(u);
end

function n = va_conta_rbt(mdl)
%VA_CONTA_RBT  Sottosistemi con parametro di maschera RigidBodyTree.
c = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                'BlockType','SubSystem');
n = 0;
for k = 1:numel(c)
    try
        m = Simulink.Mask.get(c{k});
        if isempty(m), continue; end
        if any(strcmp({m.Parameters.Name}, 'RigidBodyTree')), n = n + 1; end
    catch
    end
end
end

function s = va_lista(c)
if isempty(c), s = '<nessuno>'; else, s = strjoin(c, ' '); end
end

function s = va_num(x)
if isnan(x), s = '<errore>'; else, s = sprintf('%d', x); end
end

function s = va_si(b)
if b, s = 'si'; else, s = 'no'; end
end

function s = va_taglia(s, n)
s = char(s);
if numel(s) > n, s = [s(1:n-1) '~']; end
end

function s = va_testo_manca(manca)
if isempty(manca)
    s = '';
else
    s = sprintf('MANCA %s: niente coppia/contatto, salta anche la fattibilita''', ...
                strjoin(manca, ', '));
end
end

function s = va_primo(a, b)
if isempty(a), s = b; else, s = a; end
end
