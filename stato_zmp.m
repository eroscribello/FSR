function R = stato_zmp(out, task, ctrl, cfg)
%STATO_ZMP  Legge il margine ZMP dal log e separa i campioni non calcolabili.
%
%   log_zmp('on');  script_T2;  stato_zmp(t2_out, 'T2', 'C2')
%
% LO ZERO NON E' UN MARGINE
%   La funzione nel modello calcola il margine solo se almeno tre zampe
%   superano Fmin = 2 N; altrimenti esce dal ramo 'else' con
%   x_zmp = y_zmp = margin = 0. Quello zero e' una SENTINELLA che vuol dire
%   "non calcolabile", non "centro di pressione esattamente sul bordo".
%   Disegnarlo come 0 fa sembrare il robot al limite proprio dove la misura
%   non c'e'. Qui diventa NaN, e la frazione di tempo non calcolabile viene
%   riportata: e' essa stessa un risultato.
%
% ATTENZIONE A is_stable
%   Vale 0 sia quando il margine e' negativo sia quando non e' calcolabile:
%   confonde "instabile" con "non lo so". Si logga per completezza ma non va
%   usato come esito.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 3 || isempty(ctrl), ctrl = 'C2'; end
if nargin < 4 || isempty(cfg),  cfg  = phantomx_config(); end
task = upper(char(task));  ctrl = upper(char(ctrl));

%% ---- il log ----
L = [];
for nome = {'logsout', 'yout'}
    try L = out.(nome{1}); break; catch, end
end
if isempty(L)
    error('stato_zmp:log', ...
        ['Nessun logsout nell''uscita della simulazione.\n' ...
         'Il logging delle porte va acceso PRIMA della run:  log_zmp(''on'')']);
end

NOMI = {'zmp_x','zmp_y','zmp_margine','zmp_stabile'};
S = struct();
for k = 1:numel(NOMI)
    try
        S.(NOMI{k}) = L.getElement(NOMI{k}).Values;
    catch
        error('stato_zmp:segnale', ...
            ['Nel log manca "%s".\nControlla con  log_zmp  che il logging sia ' ...
             'acceso, e che la run sia successiva.'], NOMI{k});
    end
end

t = S.zmp_margine.Time(:);
x = double(S.zmp_x.Data(:));
y = double(S.zmp_y.Data(:));
m = double(S.zmp_margine.Data(:));

%% ---- separazione dei campioni non calcolabili ----
% Il ramo 'else' azzera TUTTE E TRE le uscite insieme: e' la firma che le
% distingue da un margine genuinamente nullo, che con x e y esattamente a
% zero non capita.
nc = (m == 0) & (x == 0) & (y == 0);
m(nc) = NaN;

reg = t >= 2*cfg.T;                       % stesso avvio di regime delle metriche
R = struct();
R.task        = string(task);
R.controller  = string(ctrl);
R.frac_nc     = sum(nc & reg) / sum(reg);
R.eventi_nc   = sum(diff([false; nc(reg)]) == 1);
R.margine_min = min(m(reg), [], 'omitnan');
R.margine_med = median(m(reg), 'omitnan');
R.margine_max = max(m(reg), [], 'omitnan');
R.frac_neg    = sum(m(reg) < 0) / sum(~isnan(m(reg)));
R.geom_tripode = zmp_geom(cfg);
R.data = string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm'));

fprintf('\n=============== ZMP - %s, %s ===============\n', task, ctrl);
fprintf('  margine valido      min %.3f   mediana %.3f   max %.3f  m\n', ...
        R.margine_min, R.margine_med, R.margine_max);
fprintf('  previsione geometrica del tripode          %.3f  m\n', R.geom_tripode);
fprintf('  campioni negativi                          %.2f %%\n', 100*R.frac_neg);
fprintf('  NON calcolabile (meno di 3 zampe sopra Fmin)  %.1f %% del tempo, %d episodi\n', ...
        100*R.frac_nc, R.eventi_nc);
if R.frac_nc > 0.02
    fprintf(2, ['  La soglia Fmin = 2 N e'' il 39%% del carico nominale di una zampa\n' ...
                '  (15.55 N su tre): in questi istanti il poligono non ha tre vertici.\n']);
end

%% ---- CSV ----
dest = fullfile('results', 'diagnostica');
if ~isfolder(dest), mkdir(dest); end
f = fullfile(dest, sprintf('zmp_%s_%s.csv', ctrl, task));
writetable(table(t, x, y, m, double(nc), 'VariableNames', ...
                 {'t','x_zmp','y_zmp','margine','non_calcolabile'}), f);
fprintf('  scritto  %s\n\n', f);
end

% =====================================================================
function d = zmp_geom(cfg)
%ZMP_GEOM  Distanza dal centro del corpo al lato piu' vicino del tripode.
% Ricavata dalla sola geometria, serve da riscontro al margine misurato.
p = zeros(2,6);
for i = 1:6
    p(:,i) = [cfg.p_hip(1,i) + cfg.r_offset*cos(cfg.alpha(i));
              cfg.p_hip(2,i) + cfg.r_offset*sin(cfg.alpha(i))];
end
d = Inf;
for fase = [0 0.5]
    tri = find(abs(cfg.phase(:)' - fase) < 1e-9);
    if numel(tri) ~= 3, continue; end
    for k = 1:3
        a = p(:, tri(k));  b = p(:, tri(mod(k,3)+1));
        e = b - a;
        d = min(d, abs(e(1)*(0-a(2)) - e(2)*(0-a(1))) / norm(e));
    end
end
end
