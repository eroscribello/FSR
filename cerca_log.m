function T = cerca_log(out, filtro, solo_serie)
% Cerca un nodo nell'albero del log di Simscape.
%
%   T = cerca_log(out)                     tutto l'albero
%   T = cerca_log(out, 'contact')          solo i percorsi che contengono
%                                          'contact' (maiuscole ignorate)
%   T = cerca_log(out, 'force|contact')    espressione regolare
%   T = cerca_log(out, 'contact', false)   anche i nodi senza serie
%
% USO
%   init_gait ; applica_terreno('T1')
%   out = sim('phantomx_sim_zero','StopTime','3');
%   T = cerca_log(out, 'contact')

if nargin < 2, filtro = ''; end
if nargin < 3 || isempty(solo_serie), solo_serie = true; end

%% ---- il log ----
simlog = [];
for nome = {'simlog','simlog_phantomx_sim_zero'}
    if isprop(out, nome{1}) || (isstruct(out) && isfield(out, nome{1}))
        simlog = out.(nome{1});  break
    end
end
if isempty(simlog)
    try
        p = properties(out);
        for k = 1:numel(p)
            v = out.(p{k});
            if isa(v, 'simscape.logging.Node'), simlog = v; break; end
        end
    catch
    end
end
if isempty(simlog)
    error('cerca_log:noLog', ...
        ['Nessun log di Simscape in questa uscita. Serve\n' ...
         '    set_param(mdl,''SimscapeLogType'',''all'')\n' ...
         'e SimscapeLogLimitData a off.']);
end

%% ---- appiattimento ----
L = raccogli(simlog, '', struct('percorso',{},'nome',{},'nodo',{}));

%% ---- filtro e misura ----
righe = {};
for k = 1:numel(L)
    if ~isempty(filtro) && isempty(regexpi(L(k).percorso, filtro, 'once'))
        continue
    end

    haSerie = false;  n = 0;  un = '';  mn = NaN;  mx = NaN;
    try
        s = L(k).nodo.series;
        haSerie = true;
        v = s.values;
        n = numel(s.time);
        try un = char(s.unit); catch, un = ''; end
        mn = scalare(min(v(:)));
        mx = scalare(max(v(:)));
    catch
    end

    if solo_serie && ~haSerie, continue; end

    righe(end+1,:) = {L(k).percorso, L(k).nome, haSerie, n, un, mn, mx}; %#ok<AGROW>
end

if isempty(righe)
    fprintf(2,'\nNessun nodo corrisponde a ''%s''.\n', filtro);
    fprintf(2,'Prova senza filtro, oppure con un pezzo di nome piu'' corto.\n');
    fprintf(2,'Nodi totali nell''albero: %d\n\n', numel(L));
    T = table();
    return
end

T = table(string(righe(:,1)), string(righe(:,2)), ...
          logical(cell2mat(righe(:,3))), cell2mat(righe(:,4)), ...
          string(righe(:,5)), cell2mat(righe(:,6)), cell2mat(righe(:,7)), ...
          'VariableNames', ...
          {'percorso','nome','serie','campioni','unita','minimo','massimo'});

%% ---- riepilogo ----
fprintf('\n=== LOG DI SIMSCAPE: %d nodi, %d corrispondono ===\n', numel(L), height(T));
esc = T.massimo - T.minimo;
costanti = T.serie & esc == 0;
if any(costanti)
    fprintf(2,'  %d segnali COSTANTI (massimo = minimo): non portano informazione\n', nnz(costanti));
end
fprintf('  %d segnali con variazione\n', nnz(T.serie & esc > 0));

%% ---- i rami di primo livello ----
rami = regexprep(T.percorso, '^\.([^.]+)\..*$', '$1');
rami = regexprep(rami, '^\.', '');
[u, ~, ix] = unique(rami);
fprintf('\n  --- rami di primo livello ---\n');
for k = 1:numel(u)
    fprintf('    %-28s %3d nodi\n', u(k), nnz(ix == k));
end
fprintf('\n');

disp(T)

end

%% ================================================================
function x = scalare(v)
if isempty(v) || ~isnumeric(v), x = NaN; else, x = double(v(1)); end
end

%% ================================================================
function L = raccogli(nodo, percorso, L)
%RACCOGLI  Appiattisce l'albero del log. Stessa logica di adatta_simscape.
try ids = nodo.childIds; catch, return; end
for k = 1:numel(ids)
    id = ids{k};
    try c = nodo.(id); catch, continue; end
    p = [percorso '.' id];
    L(end+1) = struct('percorso',p, 'nome',id, 'nodo',c);   %#ok<AGROW>
    L = raccogli(c, p, L);
end
end
