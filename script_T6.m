%% script_T6.m - T6: terreno con sette ostacoli  [impianto SIMSCAPE]
%
% E' script_T5 applicato a T6: stesso metodo, stesse metriche, stesse guardie.
% Il metodo (passaggio riconosciuto da piedi fermi sopra il pavimento, non dai
% sensori; assetto come escursione rispetto alla media in piano; colonne di
% contatto a NaN se i sensori non chiudono sul peso) e le ragioni di ogni
% scelta sono scritti in testa a script_T5.m, e valgono qui senza modifiche.
%
% Con piu' ostacoli la finestra del passaggio va dal primo appoggio sopra un
% ostacolo qualunque all'ultimo: e' il passaggio sull'intero campo. Il
% riferimento in piano e' il tratto prima e dopo il campo; se e' corto, lo
% script lo segnala.
%
% Nessuno spostamento: cfg.terreno.ost_dx.T6 non esiste, gli ostacoli stanno
% dove li mette il modello (ost1 incluso, che in T5 e' spostato).
%
% Progetto FSR PhantomX - A. Russo

t6_cfg       = phantomx_config();
% [29/9] Il controllore si sceglie per NOME. Modello e interruttore
% OVERRIDE_C2 li da' scegli_controllore, che e' l'unico posto dove sta
% scritto chi gira su cosa. Prima erano un booleano: con tre controllori
% quel booleano non sbagliava il calcolo, sbagliava il NOME DEL FILE, e
% una run di C3 sovrascriveva results/T*_C2.csv senza un errore.
t6_ctrl      = 'C2';       % 'C1' | 'C2' | 'C3'
t6_dur       = 25;          % [s] ABBONDANTE di proposito: la run non finisce
                            %     allo scadere del tempo ma a cfg.terreno.T6_x_fine,
                            %     vedi il taglio piu' sotto. 25 s a 0.134 m/s
                            %     portano il corpo a ~3.5 m, oltre l'ultimo
                            %     ostacolo (~3.3 m).
t6_soglia_su = 0.015;       % [m] appoggio piu' alto del piano = sull'ostacolo
t6_v_appoggio = 0.05;       % [m/s] piede piu' lento di cosi' = fermo, in appoggio
                            %       (lo swing va a ~0.24 m/s di media)
% [29/9] Scavalcabile dal workspace, per lanciare piu' task di fila senza
% aprire i file:
%     OVERRIDE_CTRL = 'C3'; script_T5; script_T6; clear OVERRIDE_CTRL
% NON viene cancellata dallo script: se lo facesse andrebbe riscritta prima
% di ogni task, che e' il problema che risolve. In cambio ogni run che la
% usa lo dichiara a schermo, perche' lo stato residuo deve vedersi - una
% OVERRIDE dimenticata nel workspace ci e' gia' costata una campagna.
if exist('OVERRIDE_CTRL','var') && ~isempty(OVERRIDE_CTRL)
    t6_ctrl = OVERRIDE_CTRL;
    fprintf(2, '  [OVERRIDE_CTRL] controllore forzato a %s\n', t6_ctrl);
end
[t6_mdl, t6_c2] = scegli_controllore(t6_ctrl);

% Il terreno si fissa qui, non si eredita.
applica_terreno('T6', false, t6_mdl);
% [23/9] Inerzie corrette in memoria: vedi applica_inerzie e piano_confronto 9.
applica_inerzie(t6_mdl);
% [25/9] E subito dopo lo stimatore, SEMPRE. Il blocco Inverse Dynamics che
% produce tau_attesa per il flag di contatto di C2 usa un rigidBodyTree
% importato dall'URDF: correggere le inerzie del robot e lasciare a lui
% quelle vecchie rende |tau_mis - tau_att| grande ovunque, il flag resta
% incollato a 1 e C2 smette di cercare il terreno. E' successo dal 22/9 al
% 25/9. Vedi allinea_stimatore e results/diagnostica/valida_soglia.csv.
allinea_stimatore(t6_mdl);

fprintf('\nT6: controllore %s, sette ostacoli, %g s a velocita'' nominale\n', ...
        t6_ctrl, t6_dur);

OVERRIDE_C2 = t6_c2;                                           %#ok<NASGU>
clear OVERRIDE_GAIT                                            % andatura nominale
init_gait

t6_out = sim(t6_mdl, 'StopTime', num2str(t6_dur));
t6_run = adatta_simscape(t6_out, struct( ...
             'controller', t6_ctrl, 'task','T6', 'run',1, ...
             'condizione', 'ost1-7', 'vel_d', [t6_cfg.v_nom 0]));

clear OVERRIDE_C2
init_gait                                                      % ripristina cfg

%% ---- taglio al bordo del pavimento, PRIMA delle metriche ----
% [25/9] MANCAVA, ED E' UN ERRORE CHE FALSIFICA IL TASK.
%   Il pavimento e' il cubo 8 x 8, quindi x in [-4, 4]. A velocita' nominale
%   30 s sono ~3.6 m e i piedi arrivano 15-20 cm piu' avanti del corpo: un
%   controllore un po' piu' veloce ESCE DAL PAVIMENTO e cade. metriche legge
%   quella caduta come "ribaltamento", e superato diventa false anche se il
%   percorso a ostacoli era stato completato.
%   E' successo il 25/9 confrontando due terne di soglia: la run migliore
%   usciva dagli ostacoli a t = 21.55 s, poi a t = 28.85 s un piede superava
%   x = 3.95 m e a t = 29.64 s il rollio passava 30 gradi. Senza questo
%   taglio, l'unica delle due che aveva superato il percorso risultava
%   l'unica ad aver fallito.
%   script_T4D lo fa gia' da giorni (t4d_taglia_bordo): qui mancava.
[t6_run, t6_t_bordo] = t6_taglia_bordo(t6_run, 4 - 0.05);
if ~isnan(t6_t_bordo)
    fprintf(2, ['\n  Un piede supera il bordo del pavimento a t = %.2f s:\n' ...
                '  la run e'' troncata li''. Quello che succede dopo e'' la caduta\n' ...
                '  dal bordo, non il comportamento del controllore.\n'], t6_t_bordo);
end

%% ---- taglio alla x comune: e' qui che finisce il task ----
% [26/9] Il taglio al bordo (sopra) e' una RETE DI SICUREZZA contro la caduta
% dal pavimento. Questo invece definisce il task: tutte le run di T6 coprono
% lo stesso tratto di pista, quindi incontrano gli stessi ostacoli, quindi
% alt_ost, appoggi_su_ost e l'assetto confrontano controllori e non percorsi.
% Vedi cfg.terreno.T6_x_fine per il perche'.
t6_xfine = t6_cfg.terreno.T6_x_fine;
t6_kf = find(t6_run.p(:,1) >= t6_xfine, 1, 'first');
t6_arrivato = ~isempty(t6_kf);
t6_t_a_fine = NaN;
if t6_arrivato
    t6_t_a_fine = t6_run.t(t6_kf);
    t6_run = t6_taglia_bordo_k(t6_run, t6_kf);
    fprintf('\n  Arrivato a x = %.2f m in %.2f s: run tagliata li''.\n', ...
            t6_xfine, t6_t_a_fine);
else
    fprintf(2, ['\n  NON arriva a x = %.2f m entro la fine della run: si ferma a\n' ...
                '  %.2f m. Le metriche coprono meno pista delle altre righe e NON\n' ...
                '  sono confrontabili con loro. Leggere arrivato = false.\n'], ...
            t6_xfine, max(t6_run.p(:,1)));
end

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
salva_grafico(sprintf('T6_%s', t6_ctrl));   % grafici/<tag>.fig, testi modificabili dopo

%% ================= helper =================
function P = t6_passaggio(r, soglia, v_app, cfg, t_regime)
%T6_PASSAGGIO  Finestra del passaggio sull'ostacolo, ricavata dai piedi.
%   L'appoggio e' CINEMATICO: piede fermo nel mondo. Non si usano i sensori,
%   che vedono solo il pavimento.
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
    % scarta i tratti fermi troppo brevi: agli estremi dello swing il piede
    % rallenta per un istante senza essere appoggiato
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

% appoggi sull'ostacolo per zampa, e quali. I nomi vengono da
% cfg.legNamesCAN: le colonne di pf sono in ordine CAN, che NON e' quello dei
% To Workspace (adatta_simscape riordina).
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
%T6_OMBRA  Fascia grigia sul passaggio. Va chiamata DOPO i plot: prima, ylim
% vale ancora [0 1] e la fascia coprirebbe l'intervallo sbagliato.
yl = ylim;
h = patch([P.t_ini P.t_fin P.t_fin P.t_ini], [yl(1) yl(1) yl(2) yl(2)], ...
          [0.85 0.85 0.85], 'EdgeColor','none', 'HandleVisibility','off');
uistack(h, 'bottom');
ylim(yl);
end

function s = t6_si(b)
if b, s = 'si'; else, s = 'NO'; end
end

function r = t6_taglia_bordo_k(r, k)
%T6_TAGLIA_BORDO_K  Tronca la run al campione k.
%   La lunghezza di riferimento si prende PRIMA del ciclo: troncando t per
%   primo, un confronto fatto dentro il ciclo userebbe gia' il valore nuovo e
%   lascerebbe gli altri campi interi (errore fatto il 26/9 in asimmetria_T6).
N = numel(r.t);
if isempty(k) || k >= N || k < 2, return; end
f = fieldnames(r);
for i = 1:numel(f)
    v = r.(f{i});
    if (isnumeric(v) || islogical(v)) && size(v,1) == N && N > 1
        r.(f{i}) = v(1:k, :);
    end
end
end

function [r, t_bordo] = t6_taglia_bordo(r, x_bordo)
%T6_TAGLIA_BORDO  Tronca la run al primo campione con un piede oltre x_bordo.
%   Stessa logica di t4d_taglia_bordo in script_T4D: il pavimento e' il cubo
%   8 x 8 (x in [-4, 4]), e cfg.floor_dim non lo descrive.
t_bordo = NaN;
N = numel(r.t);
if ~isfield(r,'pf') || isempty(r.pf), return; end
k = find(any(r.pf(:, 1:3:18) > x_bordo, 2), 1, 'first');
if isempty(k) || k < 2, return; end
t_bordo = r.t(k);
f = fieldnames(r);
for i = 1:numel(f)
    v = r.(f{i});
    if (isnumeric(v) || islogical(v)) && size(v,1) == N && N > 1
        r.(f{i}) = v(1:k-1, :);
    end
end
end
