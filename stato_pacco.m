function R = stato_pacco(out, task, cfg)
% Verifica se il pacco e' rimasto sul robot
%   script_T4;  stato_pacco(t4_out, 'T4')      % nella stessa sessione, con C3P
%

if nargin < 3 || isempty(cfg), cfg = phantomx_config(); end
task = upper(char(task));

%% ---- il nodo del giunto nel log ----
lg = [];
for nome = {'simlog','simlog_phantomx_sim_attitude'}
    try lg = out.(nome{1}); break; catch, end
end
if isempty(lg)
    error('stato_pacco:log', 'Nessun log di Simscape nell''uscita della simulazione.');
end
ids = lg.childIds;
k = find(~cellfun(@isempty, regexp(ids, '6_DOF_Joint1$', 'once')), 1);
if isempty(k)
    error('stato_pacco:giunto', ['Nel log non c''e'' 6-DOF Joint1. Nodi di primo ' ...
        'livello: %s.\nIl carico e'' stato commentato, o il blocco rinominato.'], ...
        strjoin(ids, ', '));
end
g = lg.(ids{k});

P = [];
for c = {'Px','Py','Pz'}
    s = g.(c{1}).p.series;
    t = s.time;   t = t(:);        % s.time(:) e' letto come chiamata di metodo
    v = s.values('m');
    P = [P, v(:)];                                                %#ok<AGROW>
end

%% ---- spostamento dal riferimento ----
ref = t >= 1.5 & t <= 2.0;
if ~any(ref)
    error('stato_pacco:corta', 'La run e'' piu'' corta di 2 s: niente riferimento.');
end
P0 = median(P(ref,:), 1);
d  = vecnorm(P - P0, 2, 2);
reg = t >= 2*cfg.T;

R = struct();
R.task        = string(task);
R.controller  = "C3P";
R.spost_max   = max(d(reg));
R.spost_fin   = d(end);
[~, im]       = max(d .* reg);
R.t_max       = t(im);
R.dz_max      = max(abs(P(reg,3) - P0(3)));
if R.spost_max < 0.02,     R.esito = "fermo";
elseif R.spost_max < 0.05, R.esito = "scivola, resta sul vassoio";
else,                      R.esito = "al bordo o caduto";
end
R.data = string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm'));

fprintf('\n=============== PACCO - %s, C3P ===============\n', task);
fprintf('  spostamento dal riferimento   max %.1f mm (t = %.2f s)   finale %.1f mm\n', ...
        1e3*R.spost_max, R.t_max, 1e3*R.spost_fin);
fprintf('  di cui in Pz                  max %.1f mm\n', 1e3*R.dz_max);
if R.esito == "al bordo o caduto"
    fprintf(2, '  -> %s: guardare l''animazione prima di dichiarare il task.\n', R.esito);
else
    fprintf('  -> %s\n', R.esito);
end

%% ---- CSV ----
dest = fullfile('results', 'diagnostica');
if ~isfolder(dest), mkdir(dest); end
f = fullfile(dest, 'pacco_C3P.csv');
T = struct2table(R);
if isfile(f)
    o = detectImportOptions(f, 'TextType', 'string');
    o = setvartype(o, {'task','controller','esito','data'}, 'string');
    V = readtable(f, o);
    V = V(V.task ~= R.task, T.Properties.VariableNames);
    T = [V; T];
end
writetable(sortrows(T, 'task'), f);
fprintf('  scritto  %s\n\n', f);
end
