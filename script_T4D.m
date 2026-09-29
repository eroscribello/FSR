%% script_T4D.m - T4D: dosso (salita, cima, discesa) non modellato  [impianto SIMSCAPE]
%
% COSA MISURA
%   Il comportamento in DISCESA, che T4 non vede. Il robot sale una rampa di
%   8 gradi (la stessa di T4), percorre una cima piana, scende di 8 gradi e
%   torna in piano. Dosso alto 14 cm (cfg.terreno.dosso). Nessun controllore conosce il terreno.
%   La discesa e' il caso per cui la ricerca del terreno del paper esiste: il
%   terreno si allontana dal piede, C1 lo appoggia alla quota prevista e la
%   zampa resta APPESA; C2 dovrebbe continuare a scendere finche' tocca.
%
% LA GEOMETRIA E' NOTA ALLO SCRIPT, NON AI CONTROLLORI
%   Il dosso e' scritto da applica_terreno('T4D') a partire da dosso_profilo,
%   la stessa funzione che qui da' le quote. Le finestre si ricavano quindi
%   dalla POSIZIONE DEI PIEDI rispetto agli spigoli, non da cio' che fa il
%   corpo: niente circolarita'.
%
% FINESTRE (dichiarate prima di guardare i dati)
%   Con xmin, xmax = piede piu' arretrato e piu' avanzato:
%     tratti    tutto il robot sullo stesso tratto
%               piano_prima  xmax < x0            (e t >= 2 cicli)
%               salita       x0 <= xmin, xmax <= x1
%               cima         x1 <= xmin, xmax <= x2
%               discesa      x2 <= xmin, xmax <= x3
%               piano_dopo   xmin >= x3
%     spigoli   il robot a cavallo di uno spigolo:  xmin < x_k < xmax
%               piede_salita (x0, concavo)   inizio_cima    (x1, convesso)
%               inizio_discesa (x2, convesso) fondo_discesa (x3, concavo)
%   Gli spigoli convessi (x1, x2) sono quelli in cui il terreno si abbassa
%   sotto i piedi anteriori: e' li' che C1 dovrebbe lasciare zampe appese.
%
% METRICHE PER FINESTRA
%   roll_esc      max |rollio - rollio medio in piano_prima|
%   incl_err_max  max |inclinazione - riferimento in piano - inclinazione attesa|
%                 dove l'attesa e' la retta fra il terreno sotto il piede piu'
%                 arretrato e quello sotto il piu' avanzato: in un tratto e' la
%                 pendenza, su uno spigolo e' la media dei due tratti
%   incl_err      lo stesso, medio e col segno (+ = muso piu' alto dell'atteso)
%   piedi_fermi   numero medio di piedi in appoggio CINEMATICO (fermi nel
%                 mondo). Una zampa appesa si muove col corpo e non conta:
%                 e' la misura diretta delle zampe appese.
%   sotto3        frazione del tempo con meno di tre piedi fermi
%   h_min, h_max  quota del corpo sul terreno sotto di lui, rispetto al piano
%                 [mm]: h_min < 0 = il corpo "cade" verso il terreno
%   v_rel         velocita' lungo la superficie / velocita' in piano (solo tratti)
%
% ESITO
%   attraversato = il piede piu' arretrato supera il fondo della discesa almeno
%   un ciclo prima della fine, e l'assetto resta entro 30 gradi rispetto
%   all'atteso (rollio e inclinazione). Stessa soglia di T4.
%
% BORDO DEL PAVIMENTO
%   Il pavimento finisce a x = 4 m (cubo 8 x 8), e a fine run il robot ci
%   arriva vicino. L'analisi si ferma al primo istante in cui un piede supera
%   il bordo meno 5 cm: oltre, il robot camminerebbe nel vuoto e le misure non
%   sarebbero piu' del dosso. La run viene TAGLIATA li' prima di metriche, cosi'
%   anche tau_max, cot e assetto massimo escludono la caduta. Colonna t_bordo.

% USCITE
%   results/T4D_<ctrl>.csv           riga di metriche + le colonne chiave
%   results/T4D_<ctrl>_finestre.csv  una riga per finestra, tutte le metriche
%   grafici/T4D_<ctrl>.fig          figura, testi e assi modificabili dopo (+ .png)
%
% COLONNE DI CONTATTO: come in T4-T6, a NaN se i sensori non chiudono sul peso.
%
% ATTENZIONE AI NOMI: variabili prefissate t4d_ (init_gait e' uno script).
%
% Progetto FSR PhantomX - A. Russo

t4d_cfg        = phantomx_config();
% [29/9] Il controllore si sceglie per NOME. Modello e interruttore
% OVERRIDE_C2 li da' scegli_controllore, che e' l'unico posto dove sta
% scritto chi gira su cosa. Prima erano un booleano: con tre controllori
% quel booleano non sbagliava il calcolo, sbagliava il NOME DEL FILE, e
% una run di C3 sovrascriveva results/T*_C2.csv senza un errore.
t4d_ctrl       = 'C2';     % 'C1' | 'C2' | 'C3'
t4d_dur        = 30;        % [s] fondo della discesa a ~27 s a velocita' nominale
t4d_v_appoggio = 0.05;      % [m/s] piede piu' lento di cosi' = fermo, in appoggio
% [29/9] Scavalcabile dal workspace, per lanciare piu' task di fila senza
% aprire i file:
%     OVERRIDE_CTRL = 'C3'; script_T5; script_T6; clear OVERRIDE_CTRL
% NON viene cancellata dallo script: se lo facesse andrebbe riscritta prima
% di ogni task, che e' il problema che risolve. In cambio ogni run che la
% usa lo dichiara a schermo, perche' lo stato residuo deve vedersi - una
% OVERRIDE dimenticata nel workspace ci e' gia' costata una campagna.
if exist('OVERRIDE_CTRL','var') && ~isempty(OVERRIDE_CTRL)
    t4d_ctrl = OVERRIDE_CTRL;
    fprintf(2, '  [OVERRIDE_CTRL] controllore forzato a %s\n', t4d_ctrl);
end
[t4d_mdl, t4d_c2] = scegli_controllore(t4d_ctrl);

% Il terreno si fissa qui, non si eredita.
applica_terreno('T4D', false, t4d_mdl);
% [23/9] Inerzie corrette in memoria: vedi applica_inerzie e piano_confronto 9.
applica_inerzie(t4d_mdl);
% [25/9] E subito dopo lo stimatore, SEMPRE. Il blocco Inverse Dynamics che
% produce tau_attesa per il flag di contatto di C2 usa un rigidBodyTree
% importato dall'URDF: correggere le inerzie del robot e lasciare a lui
% quelle vecchie rende |tau_mis - tau_att| grande ovunque, il flag resta
% incollato a 1 e C2 smette di cercare il terreno. E' successo dal 22/9 al
% 25/9. Vedi allinea_stimatore e docs/piano_confronto.md.
allinea_stimatore(t4d_mdl);
t4d_G = dosso_profilo(t4d_cfg.terreno.dosso, t4d_cfg.floor_top);

fprintf(['\nT4D: controllore %s, dosso %g/%g gradi, H %.0f mm, cima %.2f m, ' ...
         '%g s a velocita'' nominale\n'], t4d_ctrl, t4d_cfg.terreno.dosso.gradi_su, ...
         t4d_cfg.terreno.dosso.gradi_giu, 1e3*t4d_G.H, t4d_cfg.terreno.dosso.L_cima, t4d_dur);

OVERRIDE_C2 = t4d_c2;                                          %#ok<NASGU>
clear OVERRIDE_GAIT                                            % andatura nominale
init_gait

t4d_out = sim(t4d_mdl, 'StopTime', num2str(t4d_dur));
t4d_run = adatta_simscape(t4d_out, struct( ...
             'controller', t4d_ctrl, 'task','T4D', 'run',1, ...
             'condizione', sprintf('dosso%g-%g', t4d_cfg.terreno.dosso.gradi_su, ...
                                   t4d_cfg.terreno.dosso.gradi_giu), ...
             'vel_d', [t4d_cfg.v_nom 0]));

clear OVERRIDE_C2
init_gait                                                      % ripristina cfg

% [CORRETTO 22/9] La run si TAGLIA al bordo del pavimento PRIMA di metriche.
% La prima versione tagliava solo le finestre: le metriche globali (tau_max,
% cot, roll_max, pitch_max) contenevano la caduta dal bordo. Con C2, piu'
% veloce, il robot cadeva negli ultimi 2 s (corpo -95 mm, beccheggio -18 gradi)
% e tau_max usciva 20 N*m. Si taglia ogni campo con una riga per campione.
[t4d_run, t4d_t_bordo] = t4d_taglia_bordo(t4d_run, 4 - 0.05);
if ~isnan(t4d_t_bordo)
    fprintf(2, '  un piede supera il bordo a t = %.2f s: run tagliata li''.\n', t4d_t_bordo);
end

t4d_riga = metriche(t4d_run, t4d_cfg, struct('t_regime', 2*t4d_cfg.T));
t4d_riga.t_bordo = t4d_t_bordo;

%% ---- finestre e metriche ----
[t4d_F, t4d_S] = t4d_finestre(t4d_run, t4d_G, t4d_cfg, t4d_v_appoggio, 2*t4d_cfg.T);

%% ---- colonne di contatto ----
t4d_chiusura = NaN;
if isfield(t4d_run,'Fc') && ~isempty(t4d_run.Fc)
    t4d_sel = t4d_run.t >= 2*t4d_cfg.T;
    t4d_chiusura = mean(sum(t4d_run.Fc(t4d_sel,3:3:18),2)) / (t4d_cfg.mass*t4d_cfg.g);
end
t4d_riga.chiusura_peso = t4d_chiusura;
if isnan(t4d_chiusura) || abs(t4d_chiusura - 1) > 0.05
    t4d_contatto = intersect({'appoggio_medio','sotto3_frac','slip_tot','slip_per_passo', ...
        'distacchi','frazione_persa','Fz_max_norm','disp_carico', ...
        'appoggi_scartati','appoggi_totali'}, t4d_riga.Properties.VariableNames);
    for t4d_k = 1:numel(t4d_contatto), t4d_riga.(t4d_contatto{t4d_k}) = NaN; end
    t4d_riga.note = t4d_riga.note + sprintf( ...
        "; sensori chiudono al %.0f%% del peso (vedono solo il pavimento): colonne di contatto a NaN", ...
        100*t4d_chiusura);
end

%% ---- riga riassuntiva: metriche + colonne chiave della discesa ----
t4d_riga.attraversato = t4d_S.attraversato;
t4d_riga.causa_T4D    = string(t4d_S.causa);
t4d_riga.t_fondo      = t4d_S.t_fondo;
for t4d_nome = {'inizio_cima','inizio_discesa','discesa','fondo_discesa'}
    t4d_k = find(t4d_F.finestra == t4d_nome{1});
    for t4d_col = {'roll_esc','incl_err_max','piedi_fermi','sotto3','h_min'}
        t4d_v = NaN;  if ~isempty(t4d_k), t4d_v = t4d_F.(t4d_col{1})(t4d_k); end
        t4d_riga.([t4d_nome{1} '_' t4d_col{1}]) = t4d_v;
    end
end

if ~isfolder('results'), mkdir('results'); end
t4d_file  = fullfile('results', sprintf('T4D_%s.csv', t4d_ctrl));
t4d_fileF = fullfile('results', sprintf('T4D_%s_finestre.csv', t4d_ctrl));
writetable(t4d_riga, t4d_file);
writetable(t4d_F,    t4d_fileF);

%% ---- lettura ----
fprintf('\n=============== T4D - %s ===============\n', t4d_ctrl);
fprintf('  %-15s %6s %8s %9s %8s %7s %7s %7s %6s\n', 'finestra', 'dur', 'roll_esc', ...
        'incl_err', 'err_max', 'piedi', 'sotto3', 'h_min', 'v_rel');
fprintf('  %-15s %6s %8s %9s %8s %7s %7s %7s %6s\n', '', '[s]', '[deg]', '[deg]', '[deg]', ...
        'fermi', '[%]', '[mm]', '');
for t4d_k = 1:height(t4d_F)
    fprintf('  %-15s %6.1f %8.2f %+9.2f %8.2f %7.2f %7.1f %+7.1f %6.2f\n', ...
        t4d_F.finestra(t4d_k), t4d_F.durata(t4d_k), rad2deg(t4d_F.roll_esc(t4d_k)), ...
        rad2deg(t4d_F.incl_err(t4d_k)), rad2deg(t4d_F.incl_err_max(t4d_k)), ...
        t4d_F.piedi_fermi(t4d_k), 100*t4d_F.sotto3(t4d_k), 1e3*t4d_F.h_min(t4d_k), ...
        t4d_F.v_rel(t4d_k));
end
fprintf('\n  attraversato: %s   %s\n', t4d_si(t4d_S.attraversato), t4d_S.causa);
fprintf('  tau_max %.1f N*m   cot %.2f   frazione del task %.0f%%\n', ...
        t4d_riga.tau_max, t4d_riga.cot, 100*t4d_riga.frazione_task);
fprintf('\n  scritto  %s\n           %s\n\n', t4d_file, t4d_fileF);

%% ---- grafico ----
figure;
subplot(4,1,1); hold on; grid on
plot(t4d_run.t, rad2deg(-t4d_run.rpy(:,2)), 'DisplayName','inclinazione (-beccheggio)');
plot(t4d_run.t, rad2deg(t4d_S.incl_attesa + t4d_S.incl_ref), 'k--', ...
     'DisplayName','inclinazione attesa');
plot(t4d_run.t, rad2deg(t4d_run.rpy(:,1)), 'DisplayName','rollio');
t4d_ombra(t4d_F);
ylabel('[deg]'); legend('Location','best');
title(sprintf('T4D - %s  (grigio: robot a cavallo di uno spigolo)', t4d_ctrl))
subplot(4,1,2); hold on; grid on
plot(t4d_run.t, 1e3*t4d_S.h_rel);
yline(0, 'k:');
t4d_ombra(t4d_F);
ylabel('corpo sul terreno - piano [mm]');
subplot(4,1,3); hold on; grid on
plot(t4d_run.t, 1e3*t4d_S.piede_su_terreno);
t4d_ombra(t4d_F);
ylabel('piedi - terreno [mm]');
subplot(4,1,4); hold on; grid on
plot(t4d_run.t, sum(t4d_S.fermo,2));
yline(3, 'k:');
t4d_ombra(t4d_F);
xlabel('t [s]'); ylabel('piedi fermi');
salva_grafico(sprintf('T4D_%s', t4d_ctrl));   % grafici/<tag>.fig, testi modificabili dopo

%% ================= helper =================
function [F, S] = t4d_finestre(r, G, cfg, v_app, t_regime)
%T4D_FINESTRE  Finestre dai piedi rispetto agli spigoli, metriche per finestra.
T  = cfg.T;
t  = r.t(:);
xf = r.pf(:, 1:3:18);   zf = r.pf(:, 3:3:18);
% analisi fino al bordo del pavimento (vedi intestazione)
% [ATTENZIONE] non cfg.floor_dim, che vale [4 4 0.05] e non corrisponde alla
% mesh: il pavimento e' il cubo 8 x 8 di ProvaPianoImperfettoCube.stl, x in [-4, 4].
x_bordo = 4 - 0.05;
k_bordo = find(any(xf > x_bordo, 2), 1, 'first');
S.t_bordo = NaN;
if ~isempty(k_bordo)
    S.t_bordo = t(k_bordo);
    fprintf(2, '  un piede supera x = %.2f m a t = %.2f s: analisi fino a li''.\n', x_bordo, S.t_bordo);
end
utile = true(size(t));  if ~isempty(k_bordo), utile(k_bordo:end) = false; end
xmin = min(xf, [], 2);  xmax = max(xf, [], 2);
x = G.x;

S.fermo = appoggio_cinematico(r, cfg, v_app);
n_fermi = sum(S.fermo, 2);

% inclinazione attesa: retta fra il terreno sotto il piede piu' arretrato e
% quello sotto il piu' avanzato
S.incl_attesa = atan((G.zg(xmax) - G.zg(xmin)) ./ max(xmax - xmin, eps));
incl = -r.rpy(:,2);                          % ZYX: muso in su = beccheggio negativo

w = struct();
w.piano_prima = xmax < x(1) & t >= t_regime & utile;
S.incl_ref = mean(incl(w.piano_prima));
roll_ref   = mean(r.rpy(w.piano_prima,1));
err = incl - S.incl_ref - S.incl_attesa;

% quota del corpo sul terreno sotto di lui, in normale, rispetto al piano
xb = r.p(:,1);
h = (r.p(:,3) - G.zg(xb)) .* cos(G.pend(xb));
S.h_rel = h - mean(h(w.piano_prima));
% piedi rispetto al terreno sotto ciascuno: un piede "in appoggio nominale"
% ma alto sul terreno e' una zampa appesa
S.piede_su_terreno = zf - G.zg(xf);

w.salita         = x(1) <= xmin & xmax <= x(2) & utile;
w.cima           = x(2) <= xmin & xmax <= x(3) & utile;
w.discesa        = x(3) <= xmin & xmax <= x(4) & utile;
w.piano_dopo     = xmin >= x(4) & utile;
w.piede_salita   = xmin < x(1) & xmax > x(1) & utile;
w.inizio_cima    = xmin < x(2) & xmax > x(2) & utile;
w.inizio_discesa = xmin < x(3) & xmax > x(3) & utile;
w.fondo_discesa  = xmin < x(4) & xmax > x(4) & utile;
pend_tratto = struct('piano_prima',0, 'salita',deg2rad(cfg.terreno.dosso.gradi_su), ...
                     'cima',0, 'discesa',-deg2rad(cfg.terreno.dosso.gradi_giu), 'piano_dopo',0);

ordine = {'piano_prima','piede_salita','salita','inizio_cima','cima', ...
          'inizio_discesa','discesa','fondo_discesa','piano_dopo'};
vx = @(m) (xb(find(m,1,'last')) - xb(find(m,1,'first'))) / ...
          max(t(find(m,1,'last')) - t(find(m,1,'first')), eps);
v_piano = vx(w.piano_prima);

righe = cell(numel(ordine), 11);
for k = 1:numel(ordine)
    m = w.(ordine{k});
    if nnz(m) < 2
        righe(k,:) = {string(ordine{k}), 0, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN};
        continue
    end
    v_rel = NaN;
    if isfield(pend_tratto, ordine{k})
        v_rel = vx(m) / cos(pend_tratto.(ordine{k})) / v_piano;
    end
    righe(k,:) = {string(ordine{k}), nnz(m)*median(diff(t)), t(find(m,1,'first')), ...
        max(abs(r.rpy(m,1) - roll_ref)), mean(err(m)), max(abs(err(m))), ...
        mean(n_fermi(m)), mean(n_fermi(m) < 3), min(S.h_rel(m)), max(S.h_rel(m)), ...
        v_rel};
end
F = cell2table(righe, 'VariableNames', {'finestra','durata','t_inizio','roll_esc', ...
    'incl_err','incl_err_max','piedi_fermi','sotto3','h_min','h_max','v_rel'});

% ---- esito ----
S.t_fondo = NaN;
k = find(xmin > x(4) & utile, 1, 'first');
if ~isempty(k), S.t_fondo = t(k); end
t_fine = t(find(utile, 1, 'last'));
dopo = t >= t_regime & utile;
assetto_ko = max(abs(r.rpy(dopo,1) - roll_ref)) > deg2rad(30) || max(abs(err(dopo))) > deg2rad(30);
if isnan(S.t_fondo) || S.t_fondo > t_fine - T
    S.causa = 'non supera il fondo della discesa entro la run';
elseif assetto_ko
    S.causa = 'assetto oltre 30 gradi rispetto all''atteso';
else
    S.causa = 'dosso attraversato';
end
S.attraversato = strcmp(S.causa, 'dosso attraversato');
S.w = w;
end

function [r, t_bordo] = t4d_taglia_bordo(r, x_bordo)
%T4D_TAGLIA_BORDO  Tronca la run al primo campione con un piede oltre x_bordo.
% Il pavimento e' il cubo 8 x 8 (x in [-4, 4]); cfg.floor_dim non lo descrive.
t_bordo = NaN;
N = numel(r.t);
k = find(any(r.pf(:, 1:3:18) > x_bordo, 2), 1, 'first');
if isempty(k), return; end
t_bordo = r.t(k);
tieni = 1:k-1;
f = fieldnames(r);
for i = 1:numel(f)
    v = r.(f{i});
    if (isnumeric(v) || islogical(v)) && size(v,1) == N && N > 1
        r.(f{i}) = v(tieni, :);
    end
end
end

function t4d_ombra(F)
%T4D_OMBRA  Fasce grigie sugli spigoli. Va chiamata DOPO i plot.
yl = ylim;
sp = {'piede_salita','inizio_cima','inizio_discesa','fondo_discesa'};
col = [0.90 0.90 0.90; 0.80 0.80 0.80; 0.70 0.70 0.70; 0.85 0.85 0.85];
for k = 1:numel(sp)
    i = find(F.finestra == sp{k});
    if isempty(i) || isnan(F.t_inizio(i)), continue; end
    a = F.t_inizio(i);  b = a + F.durata(i);
    h = patch([a b b a], [yl(1) yl(1) yl(2) yl(2)], col(k,:), ...
              'EdgeColor','none', 'HandleVisibility','off');
    uistack(h, 'bottom');
end
ylim(yl);
end

function s = t4d_si(b)
if b, s = 'si'; else, s = 'NO'; end
end
