%% script_T5.m - T5: ostacolo singolo non modellato  [impianto SIMSCAPE]
%
%   Misura terreno ignoto al controllore. Il robot cammina dritto a velocita' nominale 
%   e incontra un ostacolo che il generatore di traiettoria non conosce. 
%
%   Controlli:
%     - nessun piede fermo sopra il pavimento -> l'ostacolo non e' stato
%       incontrato (fuori traiettoria, o durata troppo corta): errore;
%     - ultimo piede sull'ostacolo a meno di un ciclo dalla fine -> la run e'
%       finita col robot ancora sopra: avviso, e superato = false.
%
t5_cfg       = phantomx_config();
t5_ctrl      = 'C2';       % 'C1' | 'C2' | 'C3'
t5_dur       = 20;          % [s] a 1.0x sono ~2.8 m: abbondante per superarlo
t5_soglia_su = 0.015;       % [m] appoggio piu' alto del piano = sull'ostacolo
t5_v_appoggio = 0.05;       % [m/s] piede piu' lento di cosi' = fermo, in appoggio

if exist('OVERRIDE_CTRL','var') && ~isempty(OVERRIDE_CTRL)
    t5_ctrl = OVERRIDE_CTRL;
    fprintf(2, '  [OVERRIDE_CTRL] controllore forzato a %s\n', t5_ctrl);
end
[t5_mdl, t5_c2, t5_info] = scegli_controllore(t5_ctrl);

applica_terreno('T5', false, t5_mdl);

applica_inerzie(t5_mdl);

allinea_stimatore(t5_mdl);

if strcmp(t5_mdl, 'phantomx_sim_attitude')   % il pacco resta solo per C3P
    if t5_info.carico, commenta_carico(t5_mdl, 'off'); else, commenta_carico(t5_mdl); end
end

fprintf('\nT5: controllore %s, ostacolo singolo, %g s a velocita'' nominale\n', ...
        t5_ctrl, t5_dur);

OVERRIDE_C2 = t5_c2;                                           
clear OVERRIDE_GAIT                                            % andatura nominale
init_gait

t5_out = sim(t5_mdl, 'StopTime', num2str(t5_dur));
t5_run = adatta_simscape(t5_out, struct( ...
             'controller', t5_ctrl, 'task','T5', 'run',1, ...
             'condizione', 'ost1', 'vel_d', [t5_cfg.v_nom 0]));

clear OVERRIDE_C2
init_gait                                                      % ripristina cfg

t5_riga = metriche(t5_run, t5_cfg, struct('t_regime', 2*t5_cfg.T));

%% ---- il passaggio ----
t5_P = t5_passaggio(t5_run, t5_soglia_su, t5_v_appoggio, t5_cfg, 2*t5_cfg.T);

%% ---- le colonne di contatto valgono solo se i sensori vedono tutto il peso ----
t5_chiusura = NaN;
if isfield(t5_run,'Fc') && ~isempty(t5_run.Fc)
    t5_sel = t5_run.t >= 2*t5_cfg.T;
    t5_chiusura = mean(sum(t5_run.Fc(t5_sel,3:3:18),2)) / (t5_cfg.mass*t5_cfg.g);
end
t5_riga.chiusura_peso = t5_chiusura;
if isnan(t5_chiusura) || abs(t5_chiusura - 1) > 0.05
    t5_contatto = intersect({'appoggio_medio','sotto3_frac','slip_tot','slip_per_passo', ...
        'distacchi','frazione_persa','Fz_max_norm','disp_carico', ...
        'appoggi_scartati','appoggi_totali'}, t5_riga.Properties.VariableNames);
    for t5_k = 1:numel(t5_contatto), t5_riga.(t5_contatto{t5_k}) = NaN; end
    t5_riga.note = t5_riga.note + sprintf( ...
        "; sensori chiudono al %.0f%% del peso (vedono solo il pavimento): colonne di contatto a NaN", ...
        100*t5_chiusura);
    fprintf(2, ['\n  I sensori chiudono al %.0f%% del peso: non vedono i piedi sull''ostacolo.\n' ...
                '  Colonne di contatto messe a NaN. L''assetto non ne risente.\n'], 100*t5_chiusura);
end

t5_riga.t_ingresso      = t5_P.t_ini;
t5_riga.t_uscita        = t5_P.t_fin;
t5_riga.x_ingresso      = t5_P.x_ini;
t5_riga.x_uscita        = t5_P.x_fin;
t5_riga.appoggi_su_ost  = t5_P.n_appoggi;
t5_riga.zampe_su_ost    = string(t5_P.zampe);
t5_riga.alt_ost         = t5_P.altezza;
t5_riga.roll_esc        = t5_P.roll_esc;
t5_riga.pitch_esc       = t5_P.pitch_esc;
t5_riga.roll_max_ost    = t5_P.roll_max;
t5_riga.pitch_max_ost   = t5_P.pitch_max;
t5_riga.roll_ref_piano  = t5_P.roll_ref;
t5_riga.pitch_ref_piano = t5_P.pitch_ref;
t5_riga.corpoZ_pp_ost   = t5_P.z_pp;
t5_riga.superato        = t5_P.uscito && t5_riga.causa_fallimento ~= "ribaltamento";

if ~isfolder('results'), mkdir('results'); end
t5_file = fullfile('results', sprintf('T5_%s.csv', t5_ctrl));
writetable(t5_riga, t5_file);

%% ---- lettura ----
fprintf('\n=============== T5 - %s ===============\n', t5_ctrl);
fprintf('  ostacolo alto ~%.1f mm (dalla quota dei piedi in appoggio)\n', 1e3*t5_P.altezza);
fprintf('  passaggio  t = %.2f .. %.2f s,  x = %.3f .. %.3f m\n', ...
        t5_P.t_ini, t5_P.t_fin, t5_P.x_ini, t5_P.x_fin);
fprintf('  appoggi sull''ostacolo: %d  (zampe: %s)\n', t5_P.n_appoggi, t5_P.zampe);
fprintf('\n                  escursione    massimo assoluto    media in piano\n');
fprintf('  rollio         %6.2f deg      %6.2f deg          %+6.2f deg\n', ...
        rad2deg(t5_P.roll_esc), rad2deg(t5_P.roll_max), rad2deg(t5_P.roll_ref));
fprintf('  beccheggio     %6.2f deg      %6.2f deg          %+6.2f deg\n', ...
        rad2deg(t5_P.pitch_esc), rad2deg(t5_P.pitch_max), rad2deg(t5_P.pitch_ref));
fprintf('  corpo in z     %6.1f mm picco-picco nella finestra\n', 1e3*t5_P.z_pp);
fprintf('\n  superato: %s   frazione del task %.0f%%   %s\n', ...
        t5_si(t5_riga.superato), 100*t5_riga.frazione_task, char(t5_riga.causa_fallimento));
fprintf('\n  scritto  %s\n\n', t5_file);

%% ---- grafico ----
figure;
subplot(3,1,1); hold on; grid on
plot(t5_run.t, rad2deg(t5_run.rpy(:,1)), 'DisplayName','rollio');
plot(t5_run.t, rad2deg(t5_run.rpy(:,2)), 'DisplayName','beccheggio');
t5_ombra(t5_P);
ylabel('[deg]'); legend('Location','best');
title(sprintf('T5 - %s: assetto (in grigio il passaggio sull''ostacolo)', t5_ctrl))
subplot(3,1,2); hold on; grid on
plot(t5_run.t, 1e3*t5_run.p(:,3));
t5_ombra(t5_P);
ylabel('z corpo [mm]');
subplot(3,1,3); hold on; grid on
plot(t5_run.t, 1e3*(t5_run.pf(:,3:3:18) - t5_P.z_piano));
yline(1e3*t5_soglia_su, 'k:');
t5_ombra(t5_P);
xlabel('t [s]'); ylabel('z piedi - piano [mm]');
salva_grafico(sprintf('T5_%s', t5_ctrl));   

%% ================= helper =================
function P = t5_passaggio(r, soglia, v_app, cfg, t_regime)
%T5_PASSAGGIO  Finestra del passaggio sull'ostacolo, ricavata dai piedi.
T = cfg.T;
if ~isfield(r,'pf') || isempty(r.pf)
    error('script_T5:dati', 'Serve la posizione dei piedi (run.pf).');
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
    error('script_T5:mancato', ...
        ['Nessun piede fermo sopra il pavimento di %.0f mm: l''ostacolo non ' ...
         'e'' stato incontrato.\nO e'' fuori dalla traiettoria del robot, o la ' ...
         'durata e'' troppo corta. Guarda l''animazione prima di cambiare le soglie.'], ...
         1e3*soglia);
end

i1 = find(qualcuno, 1, 'first');
i2 = find(qualcuno, 1, 'last');
P.uscito = t(i2) < t(end) - T;
if ~P.uscito
    warning('script_T5:corta', ['La run finisce col robot ancora sull''ostacolo: ' ...
            'aumenta t5_dur. La riga e'' scritta, con superato = false.']);
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

% riferimento in piano: a regime, FUORI dalla finestra (prima o dopo)
ref = t >= t_regime & ~(t >= P.t_ini & t <= P.t_fin);
if sum(ref) < 2*T / max(median(diff(t)), eps)
    warning('script_T5:piano', ['Meno di due cicli in piano fuori dal passaggio: ' ...
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

function t5_ombra(P)
%T5_OMBRA  Fascia grigia sul passaggio. Va chiamata DOPO i plot: prima, ylim
% vale ancora [0 1] e la fascia coprirebbe l'intervallo sbagliato.
yl = ylim;
h = patch([P.t_ini P.t_fin P.t_fin P.t_ini], [yl(1) yl(1) yl(2) yl(2)], ...
          [0.85 0.85 0.85], 'EdgeColor','none', 'HandleVisibility','off');
uistack(h, 'bottom');
ylim(yl);
end

function s = t5_si(b)
if b, s = 'si'; else, s = 'NO'; end
end
