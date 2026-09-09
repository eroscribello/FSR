function fix_mesh_paths(modo, modello)
%FIX_MESH_PATHS  Sistema i percorsi delle mesh nei blocchi File Solid.
%
%   fix_mesh_paths                  DRY RUN: mostra cosa farebbe, non tocca nulla
%   fix_mesh_paths assoluti         percorsi -> assoluti per QUESTA macchina
%   fix_mesh_paths repo             percorsi -> relativi alla RADICE del repo
%                                   phantomx_description-master/meshes/<file>.STL
%                                   E' la forma da committare: uguale per tutti.
%   fix_mesh_paths relativi         percorsi -> forma corta  meshes/<file>.STL
%                                   (funziona solo lanciando da dentro il pacchetto)
%
%   fix_mesh_paths assoluti miomodello    su un modello diverso
%
% PERCHE' SERVE
%   Simscape risolve i percorsi relativi rispetto al CURRENT FOLDER di MATLAB,
%   non alla posizione del .slx. Quindi il modello compila solo se lanci da una
%   cartella precisa, e quale sia dipende da come sono scritti i percorsi.
%
% QUALE MODALITA' USARE
%   repo      -> in un repository CONDIVISO. I percorsi sono uguali per tutti,
%                il .slx non cambia da macchina a macchina e non genera
%                conflitti git. Si lancia MATLAB dalla radice del repo, che e'
%                gia' quello che chiede startup_phantomx.
%   assoluti  -> solo per un test locale su una macchina sola. NON committare
%                un .slx con percorsi assoluti: sulla macchina dell'altro non
%                funziona, e il .slx e' binario quindi git non sa fonderlo.
%
% NON SCAMBIARE LE MESH _l CON LE _r
%   L'URDF usa thigh_l.STL / tibia_l.STL per tutte e sei le zampe: i frame
%   dei link destri sono gia' ruotati, quindi la mesh sinistra e' quella
%   giusta anche a destra. Le _r presenti in meshes/ sono di un'altra
%   convenzione e scambiarle scompone le zampe destre. Provato, si rompe.
%
% SICUREZZA
%   - dry run di default
%   - copia di backup del .slx prima di ogni modifica
%   - non tocca niente se anche una sola mesh di destinazione non esiste
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(modo),    modo = 'dryrun';               end
if nargin < 2 || isempty(modello), modello = 'phantomx_sim_zero'; end

modo = lower(strtrim(char(modo)));
if ~ismember(modo, {'dryrun','assoluti','relativi','repo'})
    error('Modo non riconosciuto: %s\nUsa: dryrun | repo | assoluti | relativi', modo);
end
[~, modello] = fileparts(modello);

%% ---- dove stanno le mesh ----
meshDir = trova_meshes();
if isempty(meshDir)
    fprintf(2,['\nNon trovo la cartella meshes (cerco body.STL a partire da\n' ...
               '%s e risalendo di 3 livelli).\n' ...
               'Lanciami da dentro o accanto a phantomx_description-master.\n\n'], pwd);
    return
end
fprintf('\nCartella mesh: %s\n', meshDir);

%% ---- carica il modello ----
giaAperto = bdIsLoaded(modello);
if ~giaAperto
    fprintf('Carico %s ...\n', modello);
    load_system(modello);
end

blocchi = find_system(modello, 'LookUnderMasks','all', 'FollowLinks','on', ...
                      'MatchFilter', @Simulink.match.allVariants, 'Type','Block');

blk = {}; att = {};
for k = 1:numel(blocchi)
    try
        v = get_param(blocchi{k}, 'ExtGeomFileName');
    catch
        continue
    end
    if isempty(v), continue; end
    blk{end+1} = blocchi{k};                                        %#ok<AGROW>
    att{end+1} = strtrim(char(v));                                  %#ok<AGROW>
end

if isempty(blk)
    fprintf(2,'Nessun blocco File Solid trovato.\n');
    if ~giaAperto, close_system(modello,0); end
    return
end

%% ---- calcola i nuovi percorsi ----
nuovo = att;
for k = 1:numel(blk)
    [~, nm, ex] = fileparts(att{k});
    file = [nm ex];

    switch modo
        case 'assoluti'
            nuovo{k} = fullfile(meshDir, file);

        case 'relativi'
            nuovo{k} = ['meshes/' file];

        case 'repo'
            nuovo{k} = ['phantomx_description-master/meshes/' file];

        otherwise   % dryrun: mostra cosa farebbe 'repo'
            nuovo{k} = ['phantomx_description-master/meshes/' file];
    end
end

%% ---- verifica che tutte le destinazioni esistano ----
mancanti = {};
for k = 1:numel(blk)
    p = nuovo{k};
    if isempty(regexp(p, '^([A-Za-z]:[\\/]|[\\/])', 'once'))
        if startsWith(p, 'phantomx_description-master')
            p = fullfile(meshDir, '..', '..', p);   % relativo alla radice del repo
        else
            p = fullfile(meshDir, '..', p);         % relativo al pacchetto
        end
    end
    if ~isfile(p)
        mancanti{end+1} = nuovo{k};                                 %#ok<AGROW>
    end
end

%% ---- report ----
cambia = ~strcmp(att, nuovo);
fprintf('\nModo: %s\n', upper(modo));
fprintf('Blocchi File Solid: %d   da modificare: %d\n\n', numel(blk), sum(cambia));

for k = find(cambia)
    corto = strrep(blk{k}, [modello '/'], '');
    fprintf('  %-22s\n      da : %s\n      a  : %s\n', corto, att{k}, nuovo{k});
end
if ~any(cambia)
    fprintf('  Niente da cambiare: i percorsi sono gia'' nella forma richiesta.\n');
end

if ~isempty(mancanti)
    fprintf(2,'\n%d percorsi di destinazione NON esistono. Non tocco niente.\n', numel(mancanti));
    fprintf(2,'  %s\n', mancanti{:});
    if ~giaAperto, close_system(modello,0); end
    return
end

if strcmp(modo,'dryrun')
    fprintf(['\n--------------------------------------------------------\n' ...
             'DRY RUN: non ho modificato nulla.\n' ...
             '  fix_mesh_paths repo        forma da committare (consigliata)\n' ...
             '  fix_mesh_paths assoluti    solo per test locali, NON committare\n' ...
             '  fix_mesh_paths relativi    forma corta, richiede cd nel pacchetto\n' ...
             '--------------------------------------------------------\n\n']);
    if ~giaAperto, close_system(modello,0); end
    return
end

if ~any(cambia)
    if ~giaAperto, close_system(modello,0); end
    fprintf('\n');
    return
end

%% ---- backup ----
slx = which([modello '.slx']);
if isempty(slx), slx = [modello '.slx']; end
stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
bk = fullfile(fileparts(slx), sprintf('%s_backup_%s.slx', modello, stamp));
try
    copyfile(slx, bk);
    fprintf('\nBackup: %s\n', bk);
catch ME
    fprintf(2,'\nBackup fallito (%s). Mi fermo.\n\n', ME.message);
    if ~giaAperto, close_system(modello,0); end
    return
end

%% ---- scrittura ----
nOk = 0;
for k = find(cambia)
    try
        set_param(blk{k}, 'ExtGeomFileName', nuovo{k});
        nOk = nOk + 1;
    catch ME
        fprintf(2,'  FALLITO %s : %s\n', blk{k}, ME.message);
    end
end

save_system(modello);
fprintf('Aggiornati %d blocchi su %d. Modello salvato.\n', nOk, sum(cambia));

if ~giaAperto, close_system(modello,0); end

switch modo
    case 'repo'
        fprintf(['\nVerifica: METTITI NELLA RADICE del repository e lancia\n' ...
                 '    audit_mesh(''%s'')\n' ...
                 'Devono risultare tutte Esiste = true. Da un''altra cartella no,\n' ...
                 'ed e'' corretto: il modello va lanciato dalla radice, come fa\n' ...
                 'startup_phantomx. Questa e'' la forma da committare.\n\n'], modello);
    case 'assoluti'
        fprintf(['\nVerifica da una cartella QUALSIASI:\n' ...
                 '    cd(tempdir); audit_mesh(''%s'')\n' ...
                 'Se Esiste e'' true su tutte le righe, la dipendenza dal current\n' ...
                 'folder e'' sparita.\n' ...
                 'NON committare il .slx in questo stato: i percorsi valgono solo\n' ...
                 'su questa macchina. Prima del commit: fix_mesh_paths repo\n\n'], modello);
    otherwise
        fprintf(['\nVerifica mettendoti dentro phantomx_description-master:\n' ...
                 '    audit_mesh(''%s'')\n\n'], modello);
end

end

%% ==================== helper ====================
function d = trova_meshes()
d = '';
base = pwd;
for k = 0:3
    if isempty(base) || ~isfolder(base), break; end
    % scorciatoia: la cartella si chiama quasi sempre 'meshes'
    cand = fullfile(base, 'phantomx_description-master', 'meshes');
    if isfile(fullfile(cand,'body.STL')), d = cand; return; end
    cand = fullfile(base, 'meshes');
    if isfile(fullfile(cand,'body.STL')), d = cand; return; end
    % ricerca ricorsiva, piu' lenta
    hit = dir(fullfile(base, '**', 'body.STL'));
    if ~isempty(hit), d = hit(1).folder; return; end
    base = fileparts(base);
end
end