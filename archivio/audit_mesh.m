function T = audit_mesh(modello, cartellaBase)
%AUDIT_MESH  Elenca le mesh referenziate dai blocchi File Solid del modello.
%
%   T = audit_mesh
%   T = audit_mesh('phantomx_sim_zero')
%   T = audit_mesh('phantomx_sim_zero', 'C:\...\altra_cartella')
%
% Il secondo argomento e' la cartella rispetto a cui risolvere i percorsi
% relativi: serve a rispondere alla domanda "il modello compilerebbe se
% lanciato DA LI'?". Se lo ometti usa la cartella corrente.
%
% v2 - legge SOLO il parametro ExtGeomFileName.
%      (la v1 leggeva anche MaskValueString, che e' la maschera serializzata
%       e contiene il percorso dentro una stringa lunga: 25 falsi "file non
%       trovati". Corretto.)
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(modello),      modello = 'phantomx_sim_zero'; end
if nargin < 2 || isempty(cartellaBase), cartellaBase = pwd;            end

[~, modello] = fileparts(modello);

giaAperto = bdIsLoaded(modello);
if ~giaAperto
    fprintf('Carico %s ...\n', modello);
    load_system(modello);
end

blocchi = find_system(modello, 'LookUnderMasks','all', 'FollowLinks','on', ...
                      'MatchFilter', @Simulink.match.allVariants, 'Type','Block');

blocco = {}; percorso = {};
for k = 1:numel(blocchi)
    try
        v = get_param(blocchi{k}, 'ExtGeomFileName');
    catch
        continue                        % il blocco non ha questo parametro
    end
    if isempty(v), continue; end
    blocco{end+1}   = blocchi{k};                                   %#ok<AGROW>
    percorso{end+1} = strtrim(char(v));                             %#ok<AGROW>
end

if ~giaAperto, close_system(modello, 0); end

if isempty(blocco)
    fprintf(2,'\nNessun blocco File Solid con geometria esterna in %s.\n\n', modello);
    T = table(); return
end

n        = numel(blocco);
assoluto = false(n,1);
esiste   = false(n,1);
risolto  = cell(n,1);

for k = 1:n
    p = percorso{k};
    assoluto(k) = ~isempty(regexp(p, '^([A-Za-z]:[\\/]|[\\/])', 'once'));
    if assoluto(k)
        risolto{k} = p;
    else
        risolto{k} = fullfile(cartellaBase, p);
    end
    esiste(k) = isfile(risolto{k});
end

% nome corto della sola mesh, per leggere la tabella senza impazzire
soloFile = cell(n,1);
for k = 1:n
    [~,nm,ex] = fileparts(percorso{k});
    soloFile{k} = [nm ex];
end

% nome corto del blocco
bloccoCorto = strrep(blocco(:), [modello '/'], '');

T = table(bloccoCorto, soloFile, percorso(:), risolto, esiste, assoluto, ...
    'VariableNames', {'Blocco','Mesh','Percorso','Risolto','Esiste','Assoluto'});

%% ---------------- report ----------------
fprintf('\n=== MESH DI %s ===\n', upper(modello));
fprintf('Blocchi File Solid : %d\n', n);
fprintf('Risolti rispetto a : %s\n\n', cartellaBase);

disp(T(:, {'Blocco','Mesh','Esiste','Assoluto'}));

fprintf('\n--- SINTESI ---\n');

if all(esiste)
    fprintf('  Tutte le %d mesh sono raggiungibili da questa cartella.\n', n);
else
    fprintf(2,'  %d mesh NON raggiungibili da qui:\n', sum(~esiste));
    for k = find(~esiste)'
        fprintf(2,'    %-22s  %s\n', T.Blocco{k}, T.Percorso{k});
    end
end

nRel = sum(~assoluto);
if nRel > 0
    fprintf(2,['\n  %d percorsi RELATIVI.\n' ...
               '  Simscape li risolve rispetto al CURRENT FOLDER di MATLAB,\n' ...
               '  non rispetto alla posizione del .slx: il modello compila solo\n' ...
               '  se lanci da una cartella precisa. Per togliere la dipendenza:\n' ...
               '      fix_mesh_paths           %% dry run\n' ...
               '      fix_mesh_paths assoluti\n'], nRel);
else
    fprintf('\n  Tutti i percorsi sono assoluti: nessuna dipendenza dal current folder.\n');
    fprintf('  (ma sono legati a QUESTA macchina: chi clona il repo rilancia fix_mesh_paths)\n');
end

% mesh usate piu' volte / possibili scambi destra-sinistra
fprintf('\n  Mesh distinte usate:\n');
[u,~,ic] = unique(soloFile);
for k = 1:numel(u)
    fprintf('    %-16s x%d\n', u{k}, sum(ic==k));
end

dx = find(contains(lower(T.Blocco),{'_rf','_rm','_rr'}) & ...
          contains(lower(T.Mesh), '_l.'));
if ~isempty(dx)
    fprintf(['\n  NOTA: %d blocchi di zampa destra usano la mesh _l. E'' CORRETTO.\n' ...
             '  L''URDF di questo robot referenzia thigh_l.STL e tibia_l.STL per\n' ...
             '  tutte e sei le zampe, mai le versioni _r. I frame dei link destri\n' ...
             '  sono gia'' ruotati (j_c1_* con rpy 0 4.7123 <yaw>, yaw a passi di 45),\n' ...
             '  quindi la mesh sinistra si orienta bene anche a destra.\n' ...
             '  Le mesh _r esistono in meshes/ ma appartengono a un''altra\n' ...
             '  convenzione: sostituirle applica la rotazione due volte e la zampa\n' ...
             '  si scompone. NON scambiarle.\n'], numel(dx));
end

fprintf('\n');
end