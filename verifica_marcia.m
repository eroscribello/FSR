function verifica_marcia(t_stop)
%VERIFICA_MARCIA  Il robot va avanti, e alla velocita' giusta?
%
%   verifica_marcia          10 s
%   verifica_marcia(20)      durata a scelta
%
% PERCHE'
%   Il segno di theta non e' mai stato verificato: la profondita' del piede e'
%   simmetrica in theta, quindi il test dei segni non poteva distinguerlo
%   ([+1 ...] e [-1 ...] davano numeri identici). Se fosse invertito il robot
%   camminerebbe all'indietro o di traverso, e nessuna delle verifiche fatte
%   finora se ne accorgerebbe.
%
%   Qui si misura il moto del corpo: direzione, velocita', deriva laterale,
%   scorrimento rispetto al passo nominale.
%
% COSA MISURA (famiglia A delle metriche, piu' un pezzo della B)
%   - distanza percorsa e direzione
%   - velocita' media a regime, confrontata con v_nom = S / T_stance
%   - deriva laterale rispetto all'avanzamento
%   - oscillazione verticale del corpo
%   - scorrimento: distanza reale / distanza attesa
%
% Non modifica il modello: usa il log di Simscape.
%
% PRIMA
%   startup_phantomx ; init_gait
%
% Progetto FSR PhantomX

if nargin < 1, t_stop = 10; end

mdl = 'phantomx_sim_zero';
if ~bdIsLoaded(mdl), load_system(mdl); end
cfg = phantomx_config();

logPrima = get_param(mdl,'SimscapeLogType');
logNome  = get_param(mdl,'SimscapeLogName');
set_param(mdl,'SimscapeLogType','all');
ripristina = onCleanup(@() set_param(mdl,'SimscapeLogType',logPrima)); %#ok<NASGU>

fprintf('\n========= VERIFICA MARCIA =========\n');
fprintf('  simulo %.0f s...\n', t_stop);
evalc('out = sim(mdl, ''StopTime'', num2str(t_stop));');

if isprop(out,logNome) || isfield(out,logNome)
    simlog = out.(logNome);
else
    fprintf(2,'  Log di Simscape assente.\n\n'); return
end

%% ---- posizione del corpo ----
[t, x] = serie(simlog, 'Px');
[~, y] = serie(simlog, 'Py');
[~, z] = serie(simlog, 'Pz');

if isempty(x) || isempty(y)
    fprintf(2,'  Non trovo Px/Py nel log: il giunto del corpo potrebbe avere\n');
    fprintf(2,'  primitive con nomi diversi. Guarda l''albero con  simlog.print\n\n');
    return
end

% allinea le tre serie su una base comune
tu = linspace(max([t(1) 0]), t(end), 4000).';
xu = interp1(t, x, tu);  yu = interp1(t, y, tu);  zu = interp1(t, z, tu);

%% ---- regime: scarta il primo secondo ----
i0 = find(tu >= 1, 1);  if isempty(i0), i0 = 1; end
tr = tu(i0:end);  xr = xu(i0:end);  yr = yu(i0:end);  zr = zu(i0:end);

dx = xr(end) - xr(1);
dy = yr(end) - yr(1);
dt = tr(end) - tr(1);

dist   = hypot(dx, dy);
dirz   = atan2(dy, dx);
v_med  = dist / dt;
v_x    = dx / dt;

fprintf('\n--- MOTO DEL CORPO (regime, %.1f s) ---\n', dt);
fprintf('  avanzamento  x   %+8.4f m\n', dx);
fprintf('  deriva       y   %+8.4f m   (%.1f%% dell''avanzamento)\n', ...
        dy, 100*abs(dy)/max(abs(dx),eps));
fprintf('  distanza         %8.4f m\n', dist);
fprintf('  direzione        %+8.1f deg   (0 = avanti)\n', rad2deg(dirz));
fprintf('  velocità media   %8.4f m/s\n', v_med);
fprintf('  componente x     %+8.4f m/s\n', v_x);
fprintf('  v_nom (S/T_st)   %8.4f m/s\n', cfg.v_nom);

%% ---- scorrimento ----
att = cfg.v_nom * dt;
fprintf('\n--- SCORRIMENTO ---\n');
fprintf('  distanza attesa  %8.4f m\n', att);
fprintf('  distanza reale   %8.4f m\n', abs(dx));
fprintf('  rapporto         %8.3f     (1 = nessuno scorrimento)\n', abs(dx)/att);

%% ---- oscillazione verticale ----
fprintf('\n--- QUOTA DEL CORPO ---\n');
fprintf('  media            %8.4f m\n', mean(zr));
fprintf('  escursione       %8.2f mm\n', 1000*(max(zr)-min(zr)));

%% ---- verdetto ----
fprintf('\n--- VERDETTO ---\n');
ok = true;

if v_x > 0.5*cfg.v_nom
    fprintf('  DIREZIONE OK: il robot va avanti.\n');
elseif v_x < -0.5*cfg.v_nom
    fprintf(2,'  VA ALL''INDIETRO: il segno di theta e'' invertito in inv_kyn.\n');
    ok = false;
else
    fprintf(2,'  NON AVANZA (%.4f m/s contro %.4f attesi): scorre sul posto.\n', ...
            v_x, cfg.v_nom);
    ok = false;
end

if abs(dy) > 0.15*abs(dx)
    fprintf(2,'  DERIVA LATERALE del %.0f%%: asimmetria fra destra e sinistra,\n', ...
            100*abs(dy)/max(abs(dx),eps));
    fprintf(2,'  probabile errore su side o alpha di una zampa.\n');
    ok = false;
end

r = abs(dx)/att;
if r < 0.85
    fprintf(2,'  SCORRIMENTO del %.0f%%: i piedi strisciano in appoggio.\n', 100*(1-r));
elseif r > 1.15
    fprintf(2,'  Avanza PIU'' del previsto (%.0f%%): v_nom non descrive\n', 100*(r-1));
    fprintf(2,'  l''andatura reale, controlla beta_stance e duty.\n');
else
    fprintf('  SCORRIMENTO trascurabile (%.0f%% della distanza attesa).\n', 100*r);
end

if ok
    fprintf('\n  Il punto 1 e'' chiuso: si puo'' passare ad adatta_simscape.\n');
end
fprintf('\n===================================\n\n');
end

%% ================================================================
function [t, v] = serie(simlog, nome)
n = raccogli(simlog, nome, {});
t = []; v = [];
for k = 1:numel(n)
    try
        s = n{k}.p.series;  vk = s.values;  tk = s.time;
    catch
        continue
    end
    if isempty(v) || (max(vk)-min(vk)) > (max(v)-min(v)), v = vk; t = tk; end
end
end

function nodi = raccogli(nodo, nome, nodi)
try, ids = nodo.childIds; catch, return; end
for k = 1:numel(ids)
    try, c = nodo.(ids{k}); catch, continue; end
    if strcmp(ids{k}, nome), nodi{end+1} = c; end          %#ok<AGROW>
    nodi = raccogli(c, nome, nodi);
end
end