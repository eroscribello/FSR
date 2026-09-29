function info = diagnosi_appoggio(r, cfg, v_app)
%DIAGNOSI_APPOGGIO  Perche' appoggio_cinematico non trova nessun appoggio.
%
%   diagnosi_appoggio(t4_run)
%   info = diagnosi_appoggio(r, cfg, 0.05)
%
% LA DOMANDA
%   [29/9] script_T4 su C3 si e' fermato con "Index exceeds array bounds" in
%   t4_salita: zs_ord = sort(zf(fermo)) era VUOTO, cioe' appoggio_cinematico
%   non ha riconosciuto nemmeno un appoggio in 20 s. Ma l'animazione mostra
%   un tripode regolare.
%
%   Il criterio e' due condizioni insieme:
%       |v| del piede nel mondo  <  v_app            (0.05 m/s)
%       per almeno  0.25 * T_stance  di fila         (0.125 s = 25 campioni)
%   Puo' fallire per la prima, per la seconda, o per entrambe, e le tre cose
%   vogliono dire cose diverse. Questa funzione dice quale.
%
% SOLO LETTURA. Non simula, non tocca il modello: legge una run gia' fatta,
% quella che e' rimasta nel workspace dopo l'errore.
%
% I TRE ESITI, dichiarati qui prima di guardare i numeri
%
%   A. tratti LUNGHI, velocita' appena sopra la soglia
%      Il piede striscia lentamente invece di stare fermo. Non e' un difetto
%      del controllore: e' v_app troppo stretta per questo task. Si alza, e
%      si DICHIARA di averla alzata.
%
%   B. tratti CORTI, velocita' che attraversa la soglia avanti e indietro
%      Chattering: il piede vibra. L'andatura puo' sembrare corretta
%      nell'animazione e il contatto essere frammentato sotto i 0.125 s.
%      Il problema e' nell'anello, non nella misura. Candidato: il PID
%      d'assetto, discreto a 1 ms, in presa diretta sulla quota del piede.
%
%   C. nessun tratto, velocita' sempre alta
%      Il piede non si appoggia mai davvero. Va riguardata l'animazione al
%      rallentatore prima di toccare qualunque soglia.
%
% COME SI CHIUDE LA QUESTIONE
%   Lo stesso grafico su C2, stesso task:
%       OVERRIDE_CTRL = 'C2'; script_T4; clear OVERRIDE_CTRL
%       diagnosi_appoggio(t4_run)
%   Stessa rampa, stesso impianto, unica differenza l'anello d'assetto.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(r)
    if evalin('base', 'exist(''t4_run'',''var'')')
        r = evalin('base', 't4_run');
        fprintf('  (nessun argomento: uso t4_run dal base workspace)\n');
    else
        error('diagnosi_appoggio:run', ...
            'Serve una run. Dopo un errore in script_T4 e'' rimasta in t4_run.');
    end
end
if nargin < 2 || isempty(cfg),   cfg   = phantomx_config(); end
if nargin < 3 || isempty(v_app), v_app = 0.05; end

if ~isfield(r,'pf') || isempty(r.pf)
    error('diagnosi_appoggio:pf', 'Serve la posizione dei piedi (run.pf).');
end

t    = r.t(:);
dt   = median(diff(t));
dmin = round(0.25 * cfg.T_stance / dt);
nomi = cfg.legNamesCAN;

%% ---- velocita' di ogni piede ----
V = zeros(numel(t), 6);
for i = 1:6
    c = 3*(i-1) + (1:3);
    V(:,i) = vecnorm(gradient(r.pf(:,c).', dt).', 2, 2);
end

fprintf('\n=============================================================\n');
fprintf('  DIAGNOSI APPOGGIO - %s\n', da_meta(r));
fprintf('  criterio: |v| < %.3f m/s per almeno %d campioni (%.3f s)\n', ...
        v_app, dmin, dmin*dt);
fprintf('=============================================================\n\n');

%% ---- per zampa: quanto sta sotto soglia, e per quanto di fila ----
fprintf('  %-5s %10s %12s %12s %10s\n', ...
        'zampa', 'sotto v_app', 'tratto max', 'serve', 'mediana |v|');
fprintf('  %s\n', repmat('-', 1, 56));
L = zeros(1,6);
for i = 1:6
    g = V(:,i) < v_app;
    L(i) = da_tratto_max(g);
    fprintf('  %-5s %9.1f%% %9d cmp %9d cmp %9.3f\n', ...
            nomi{i}, 100*mean(g), L(i), dmin, median(V(:,i)));
end

%% ---- quale soglia servirebbe ----
% Non per adottarla: per capire di quanto si sbaglia. Una soglia che deve
% raddoppiare e' un'altra cosa da una che deve decuplicare.
prove = v_app * [1 1.5 2 3 5 10 20];
fprintf('\n  soglia necessaria perche'' OGNI zampa abbia un tratto valido:\n');
fprintf('  %10s %s\n', 'v_app', 'zampe con un tratto >= dmin');
serve = NaN;
for k = 1:numel(prove)
    n = 0;
    for i = 1:6
        if da_tratto_max(V(:,i) < prove(k)) >= dmin, n = n + 1; end
    end
    fprintf('  %9.3f  %d su 6\n', prove(k), n);
    if n == 6 && isnan(serve), serve = prove(k); end
end

%% ---- verdetto ----
fprintf('\n  -----------------------------------------------------------\n');
sotto = mean(V(:) < v_app);
if all(L >= dmin)
    esito = 'nessun problema';
    fprintf('  Tutte le zampe hanno un tratto valido: se t4_salita si ferma\n');
    fprintf('  lo stesso, il problema non sta qui.\n');
elseif sotto < 0.05
    esito = 'C - il piede non si ferma mai';
    fprintf('  C. Solo il %.1f%% dei campioni sta sotto soglia, su qualunque\n', 100*sotto);
    fprintf('  zampa. Il piede non si appoggia. Guarda l''animazione al\n');
    fprintf('  rallentatore PRIMA di toccare le soglie.\n');
elseif ~isnan(serve) && serve <= 2*v_app
    esito = 'A - il piede striscia';
    fprintf('  A. Con v_app = %.3f (%.1fx) tutte le zampe rientrano: il piede\n', ...
            serve, serve/v_app);
    fprintf('  striscia lentamente invece di stare fermo. E'' la soglia a\n');
    fprintf('  essere stretta per questo task. Se la alzi, DICHIARALO.\n');
else
    esito = 'B - chattering';
    fprintf('  B. Il %.1f%% dei campioni sta sotto soglia, ma il tratto piu''\n', 100*sotto);
    fprintf('  lungo e'' di %d campioni contro i %d richiesti: la velocita''\n', max(L), dmin);
    fprintf('  attraversa la soglia avanti e indietro. E'' chattering, e\n');
    fprintf('  l''andatura puo'' sembrare corretta nell''animazione lo stesso.\n');
    fprintf('  Il problema e'' nell''anello, non nella misura.\n');
end
fprintf('  -----------------------------------------------------------\n\n');

%% ---- grafico: due cicli al centro della run ----
tc = t(1) + 0.5*(t(end) - t(1));
fin = t >= tc - cfg.T & t <= tc + cfg.T;

figure;
subplot(2,1,1); hold on; grid on
plot(t(fin), 1e3*r.pf(fin, 3:3:18));
ylabel('z piedi [mm]');
title(sprintf('%s - due cicli al centro della run', da_meta(r)));
legend(nomi, 'Location','eastoutside');

subplot(2,1,2); hold on; grid on
plot(t(fin), V(fin,:));
yline(v_app, 'k--', 'v_{app}');
xlabel('t [s]'); ylabel('|v| piede [m/s]');
legend(nomi, 'Location','eastoutside');

info = struct('esito', esito, 'v_app', v_app, 'dmin', dmin, ...
              'tratto_max', L, 'frazione_sotto', sotto, ...
              'v_app_necessaria', serve, 'V', V);
end

%% ================= helper =================
function n = da_tratto_max(g)
%DA_TRATTO_MAX  Lunghezza del piu' lungo tratto consecutivo di true.
g = g(:);
if ~any(g), n = 0; return; end
ini = find(diff([false; g]) ==  1);
fin = find(diff([g; false]) == -1);
n = max(fin - ini + 1);
end

function s = da_meta(r)
%DA_META  Etichetta leggibile della run, se c'e'.
s = 'run';
if isfield(r, 'meta') && isstruct(r.meta)
    c = ''; k = '';
    if isfield(r.meta, 'controller'), c = char(string(r.meta.controller)); end
    if isfield(r.meta, 'task'),       k = char(string(r.meta.task));       end
    if ~isempty(c) || ~isempty(k), s = strtrim([k ' ' c]); end
end
end
