%% script_T6.m - T6: terreno con ostacoli  [impianto SIMSCAPE]
%
% Il robot incontra prima una scalinata composta da 3 gradini in salita e 2
% in discesa, poi un gradino solo sul lato sinistro del corpo e due sul
% lato destro. 
%
t6_cfg       = phantomx_config();

t6_ctrl      = 'C2';       % 'C1' | 'C2' | 'C3'
t6_dur       = 32;          % [s] Durata fissa della simulazione
% 32 e non 25: l'ultimo ostacolo finisce a x = 3.388 m e la zampa posteriore
% lo lascia quando il corpo e' a 3.388 + 0.224 = 3.612 m. Alle velocita' di
% regime misurate (0.116 / 0.125 / 0.119 m/s) ci arrivano a 31.1 / 29.0 /
% 30.3 s. A 25 s la corsa finiva con il robot ancora sul sesto e settimo
% ostacolo, e "non ha superato il percorso" era un artefatto della durata.
t6_xbordo    = 4 - 0.05;    % [m] il pavimento e' il cubo 8x8: x in [-4, 4]
t6_soglia_su = 0.015;       % [m] appoggio piu' alto del piano = sull'ostacolo
t6_v_appoggio = 0.05;       % [m/s] piede piu' lento di cosi' = fermo, in appoggio

if exist('OVERRIDE_CTRL','var') && ~isempty(OVERRIDE_CTRL)
    t6_ctrl = OVERRIDE_CTRL;
    fprintf(2, '  [OVERRIDE_CTRL] controllore forzato a %s\n', t6_ctrl);
end
[t6_mdl, t6_c2, t6_info] = scegli_controllore(t6_ctrl);

applica_terreno('T6', false, t6_mdl);
applica_inerzie(t6_mdl);
allinea_stimatore(t6_mdl);
if strcmp(t6_mdl, 'phantomx_sim_attitude')   % il pacco resta solo per C3P
    if t6_info.carico, commenta_carico(t6_mdl, 'off'); else, commenta_carico(t6_mdl); end
    if t6_info.assetto, spegni_assetto('off', t6_mdl); else, spegni_assetto('on', t6_mdl); end
end

fprintf('\nT6: controllore %s, sette ostacoli, %g s a velocita'' nominale\n', ...
        t6_ctrl, t6_dur);
OVERRIDE_C2 = t6_c2;                                           
clear OVERRIDE_GAIT                                            % andatura nominale
init_gait
t6_out = sim(t6_mdl, 'StopTime', num2str(t6_dur));
t6_run = adatta_simscape(t6_out, struct( ...
             'controller', t6_ctrl, 'task','T6', 'run',1, ...
             'condizione', 'ost1-7', 'vel_d', [t6_cfg.v_nom 0]));
clear OVERRIDE_C2
init_gait                                                      % ripristina cfg

%% ---- troncamento al bordo del pavimento ----
% Il pavimento finisce a x = 3.95 m, undici centimetri dopo l'ultimo
% ostacolo. Un piede che lo supera non trova piu' contatto e il robot cade
% dal bordo: roll_max va a pi greco e le metriche d'assetto diventano la
% geometria del pavimento invece del controllore. E' l'artefatto che
% prova_soglia_T6.m documenta e che script_T4D toglie gia' con
% t4d_taglia_bordo. Finche' la durata era 25 s il bordo non si raggiungeva e
% il troncamento non serviva; a 32 s serve.
[t6_run, t6_t_bordo] = t6_taglia_bordo(t6_run, t6_xbordo);
if ~isnan(t6_t_bordo)
    fprintf(2, ['\n  [bordo] Un piede raggiunge x = %.2f m a t = %.2f s.\n' ...
                '  La run e'' troncata li'': le metriche NON includono la caduta.\n'], ...
            t6_xbordo, t6_t_bordo);
end

%% ---- verifica x comune (TRONCAMENTO DISABILITATO) ----
t6_xfine = t6_cfg.terreno.T6_x_fine;
t6_kf = find(t6_run.p(:,1) >= t6_xfine, 1, 'first');
t6_arrivato = ~isempty(t6_kf);
t6_t_a_fine = NaN;

if t6_arrivato
    t6_t_a_fine = t6_run.t(t6_kf);
    fprintf('\n  Arrivato a x = %.2f m in %.2f s (run mantenuta intera fino a %g s).\n', ...
            t6_xfine, t6_t_a_fine, t6_dur);
else
    fprintf(2, ['\n  NON arriva a x = %.2f m entro i %g s: si ferma a %.2f m.\n'], ...
            t6_xfine, t6_dur, max(t6_run.p(:,1)));
end

% Metriche sull'intervallo effettivo della run (troncato al bordo, se serve)
t6_riga = metriche(t6_run, t6_cfg, struct('t_regime', 2*t6_cfg.T));
t6_riga.t_bordo   = t6_t_bordo;
t6_riga.x_fine    = t6_xfine;
t6_riga.arrivato  = t6_arrivato;
t6_riga.t_a_fine  = t6_t_a_fine;

%% ---- il passaggio ----
t6_P = t6_passaggio(t6_run, t6_soglia_su, t6_v_appoggio, t6_cfg, 2*t6_cfg.T);

%% ---- le colonne di contatto valgono solo se i sensori vedono tutto il peso ----
t6_chiusura = NaN;
if isfield(t6_run,'Fc') && ~isempty(t6_run.Fc)
    t6_sel = t6_run.t >= 2*t6_cfg.T;
    t6_chiusura = mean(sum(t6_run.Fc(t6_sel,3:3:18),2)) / (t6_cfg.mass*t6_cfg.g);
end
t6_riga.chiusura_peso = t6_chiusura;
if isnan(t6_chiusura) || abs(t6_chiusura - 1) > 0.05
    t6_contatto = intersect({'appoggio_medio','sotto3_frac','slip_tot','slip_per_passo', ...
        'distacchi','frazione_persa','Fz_max_norm','disp_carico', ...
        'appoggi_scartati','appoggi_totali'}, t6_riga.Properties.VariableNames);
    for t6_k = 1:numel(t6_contatto), t6_riga.(t6_contatto{t6_k}) = NaN; end
    t6_riga.note = t6_riga.note + sprintf( ...
        "; sensori chiudono al %.0f%% del peso (vedono solo il pavimento): colonne di contatto a NaN", ...
        100*t6_chiusura);
    fprintf(2, ['\n  I sensori chiudono al %.0f%% del peso: non vedono i piedi sull''ostacolo.\n' ...
                '  Colonne di contatto messe a NaN. L''assetto non ne risente.\n'], 100*t6_chiusura);
end

t6_riga.t_ingresso      = t6_P.t_ini;
t6_riga.t_uscita        = t6_P.t_fin;
t6_riga.x_ingresso      = t6_P.x_ini;
t6_riga.x_uscita        = t6_P.x_fin;
t6_riga.appoggi_su_ost  = t6_P.n_appoggi;
t6_riga.zampe_su_ost    = string(t6_P.zampe);
t6_riga.alt_ost         = t6_P.altezza;
t6_riga.roll_esc        = t6_P.roll_esc;
t6_riga.pitch_esc       = t6_P.pitch_esc;
t6_riga.roll_max_ost    = t6_P.roll_max;
t6_riga.pitch_max_ost   = t6_P.pitch_max;
t6_riga.roll_ref_piano  = t6_P.roll_ref;
t6_riga.pitch_ref_piano = t6_P.pitch_ref;
t6_riga.corpoZ_pp_ost   = t6_P.z_pp;
t6_riga.superato        = t6_P.uscito && t6_riga.causa_fallimento ~= "ribaltamento";

if ~isfolder('results'), mkdir('results'); end
t6_file = fullfile('results', sprintf('T6_%s.csv', t6_ctrl));
writetable(t6_riga, t6_file);

%% ---- lettura ----
fprintf('\n=============== T6 - %s ===============\n', t6_ctrl);
fprintf('  ostacolo alto ~%.1f mm (dalla quota dei piedi in appoggio)\n', 1e3*t6_P.altezza);
fprintf('  passaggio  t = %.2f .. %.2f s,  x = %.3f .. %.3f m\n', ...
        t6_P.t_ini, t6_P.t_fin, t6_P.x_ini, t6_P.x_fin);
fprintf('  appoggi sull''ostacolo: %d  (zampe: %s)\n', t6_P.n_appoggi, t6_P.zampe);
fprintf('\n                  escursione    massimo assoluto    media in piano\n');
fprintf('  rollio         %6.2f deg      %6.2f deg          %+6.2f deg\n', ...
        rad2deg(t6_P.roll_esc), rad2deg(t6_P.roll_max), rad2deg(t6_P.roll_ref));
fprintf('  beccheggio     %6.2f deg      %6.2f deg          %+6.2f deg\n', ...
        rad2deg(t6_P.pitch_esc), rad2deg(t6_P.pitch_max), rad2deg(t6_P.pitch_ref));
fprintf('  corpo in z     %6.1f mm picco-picco nella finestra\n', 1e3*t6_P.z_pp);
fprintf('\n  superato: %s   frazione del task %.0f%%   %s\n', ...
        t6_si(t6_riga.superato), 100*t6_riga.frazione_task, char(t6_riga.causa_fallimento));
fprintf('\n  scritto  %s\n\n', t6_file);

%% ---- grafico ----
figure;
subplot(3,1,1); hold on; grid on
plot(t6_run.t, rad2deg(t6_run.rpy(:,1)), 'DisplayName','rollio');
plot(t6_run.t, rad2deg(t6_run.rpy(:,2)), 'DisplayName','beccheggio');
t6_ombra(t6_P);
ylabel('[deg]'); legend('Location','best');
title(sprintf('T6 - %s: assetto (in grigio il passaggio sull''ostacolo)', t6_ctrl))
subplot(3,1,2); hold on; grid on
plot(t6_run.t, 1e3*t6_run.p(:,3));
t6_ombra(t6_P);
ylabel('z corpo [mm]');
subplot(3,1,3); hold on; grid on
plot(t6_run.t, 1e3*(t6_run.pf(:,3:3:18) - t6_P.z_piano));
yline(1e3*t6_soglia_su, 'k:');
t6_ombra(t6_P);
xlabel('t [s]'); ylabel('z piedi - piano [mm]');
salva_grafico(sprintf('T6_%s', t6_ctrl));

%% ================= helper =================
function [r, t_bordo] = t6_taglia_bordo(r, x_bordo)
%T6_TAGLIA_BORDO  Tronca la run al primo campione con un piede oltre il bordo.
% Stessa logica di t4d_taglia_bordo e di rm_taglia: taglia tutti i campi che
% hanno una riga per istante, lasciando intatto il resto della struttura.
t_bordo = NaN;
if ~isfield(r,'pf') || isempty(r.pf), return; end
N = numel(r.t);
k = find(any(r.pf(:, 1:3:18) > x_bordo, 2), 1, 'first');
if isempty(k), return; end
t_bordo = r.t(k);
f = fieldnames(r);
for i = 1:numel(f)
    v = r.(f{i});
    if (isnumeric(v) || islogical(v)) && size(v,1) == N && N > 1
        r.(f{i}) = v(1:k-1, :);
    end
end
end

function P = t6_passaggio(r, soglia, v_app, cfg, t_regime)
T = cfg.T;
if ~isfield(r,'pf') || isempty(r.pf)
    error('script_T6:dati', 'Serve la posizione dei piedi (run.pf).');
end
t  = r.t(:);
dt = median(diff(t));
zf = r.pf(:, 3:3:18);                  % [N x 6] quota dei piedi, ordine CAN
fermo = false(size(zf));
dmin  = round(0.25 * cfg.T_stance / dt);   % campioni: un quarto di appoggio
for i = 1:6
    c = 3*(i-1) + (1:3);
    v = vecnorm(gradient(r.pf(:,c).', dt).', 2, 2);
    g = v < v_app;
    ini = find(diff([false; g]) ==  1);
    fin = find(diff([g; false]) == -1);
    for k = 1:numel(ini)
        if fin(k) - ini(k) + 1 < dmin, g(ini(k):fin(k)) = false; end
    end
    fermo(:,i) = g;
end
giu = fermo;
z_piano = median(zf(giu));             % dominata dal pavimento
su = giu & (zf - z_piano > soglia);    % fermo E sopra il pavimento
qualcuno = any(su, 2);
if ~any(qualcuno)
    error('script_T6:mancato', ...
        ['Nessun piede fermo sopra il pavimento di %.0f mm: l''ostacolo non ' ...
         'e'' stato incontrato.\nO e'' fuori dalla traiettoria del robot, o la ' ...
         'durata e'' troppo corta. Guarda l''animazione prima di cambiare le soglie.'], ...
         1e3*soglia);
end
i1 = find(qualcuno, 1, 'first');
i2 = find(qualcuno, 1, 'last');
P.uscito = t(i2) < t(end) - T;
if ~P.uscito
    warning('script_T6:corta', ['La run finisce col robot ancora sull''ostacolo: ' ...
            'aumenta t6_dur. La riga e'' scritta, con superato = false.']);
end
P.t_ini = max(t(1),   t(i1) - T/2);
P.t_fin = min(t(end), t(i2) + T);
fin = t >= P.t_ini & t <= P.t_fin;
P.x_ini = r.p(find(fin,1,'first'), 1);
P.x_fin = r.p(find(fin,1,'last'),  1);
P.z_piano = z_piano;
P.altezza = median(zf(su) - z_piano);
P.z_pp    = max(r.p(fin,3)) - min(r.p(fin,3));

nomi = cfg.legNamesCAN;
n = zeros(1,6);
for i = 1:6
    n(i) = sum(diff([false; su(:,i)]) == 1);
end
P.n_appoggi = sum(n);
P.zampe = strjoin(nomi(n > 0), ' ');

ref = t >= t_regime & ~(t >= P.t_ini & t <= P.t_fin);
if sum(ref) < 2*T / max(median(diff(t)), eps)
    warning('script_T6:piano', ['Meno di due cicli in piano fuori dal passaggio: ' ...
            'il riferimento dell''assetto e'' debole.']);
end
if any(ref)
    P.roll_ref  = mean(r.rpy(ref,1));
    P.pitch_ref = mean(r.rpy(ref,2));
else
    P.roll_ref = NaN;  P.pitch_ref = NaN;
end
P.roll_esc  = max(abs(r.rpy(fin,1) - P.roll_ref));
P.pitch_esc = max(abs(r.rpy(fin,2) - P.pitch_ref));
P.roll_max  = max(abs(r.rpy(fin,1)));
P.pitch_max = max(abs(r.rpy(fin,2)));
end

function t6_ombra(P)
yl = ylim;
h = patch([P.t_ini P.t_fin P.t_fin P.t_ini], [yl(1) yl(1) yl(2) yl(2)], ...
          [0.85 0.85 0.85], 'EdgeColor','none', 'HandleVisibility','off');
uistack(h, 'bottom');
ylim(yl);
end

function s = t6_si(b)
if b, s = 'si'; else, s = 'NO'; end
end
