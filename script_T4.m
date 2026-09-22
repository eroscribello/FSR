%% script_T4.m - T4: salita su rampa non modellata  [impianto SIMSCAPE]
%
% COSA MISURA
%   Il robot cammina dritto a velocita' nominale e incontra una rampa di 8
%   gradi che il generatore di traiettoria non conosce. C1 continua a mettere
%   i piedi alla quota del piano; C2 dovrebbe trovare il terreno. A differenza
%   di T5 e T6 la rampa non finisce entro la run: non si misura un passaggio
%   ma un REGIME IN SALITA, confrontato col regime in piano della stessa run.
%
% LA RAMPA (verificata il 22/9, vedi phantomx_config)
%   Superficie dei piedi dal pavimento a x ~ 0.66 m, poi sale di 8 gradi fino
%   a oltre la fine del pavimento. A velocita' nominale il robot la raggiunge
%   verso i 4.5 s, dopo il transitorio, e in 20 s non arriva in cima.
%
% COME SI TROVA LA RAMPA, SENZA SAPERE DOVE STA
%   Come in T5: un piede FERMO nel mondo (appoggio cinematico, non dai
%   sensori, che vedono solo il pavimento) e piu' alto del pavimento di almeno
%   t4_soglia_su e' un piede sulla rampa.
%
%   [DIVERSO DA T5] La quota del pavimento non e' la mediana degli appoggi: in
%   T4 piu' di meta' degli appoggi sta sulla rampa, e la mediana cadrebbe li'.
%   Si usa il 10 percentile: gli appoggi in piano sono il ~20% del totale e
%   sono i piu' bassi.
%
% TRE FINESTRE, DICHIARATE PRIMA DI GUARDARE I DATI
%   piano        [2 cicli,  primo piede sulla rampa - T/2]
%   transizione  [primo piede sulla rampa - T/2,  ultima zampa sulla rampa + T]
%   rampa        [ultima zampa sulla rampa + T,  fine]
%   "ultima zampa sulla rampa" = l'istante in cui anche la sesta zampa ha fatto
%   il suo primo appoggio sulla rampa. Da li' in poi il robot e' tutto in
%   salita; un ciclo dopo e' a regime.
%
% METRICHE
%   pendenza_mis   pendenza della superficie ricavata dagli appoggi (retta z-x
%                  dei piedi fermi sulla rampa). Deve tornare 8 gradi: e' la
%                  verifica indipendente della posa impostata in cfg.
%   incl_err       inclinazione media del corpo sulla rampa, tolto l'offset in
%                  piano, meno la pendenza. 0 = corpo parallelo alla rampa.
%                  Positivo = muso piu' alto della rampa.
%   incl_pp        beccheggio picco-picco a regime sulla rampa.
%   roll_esc_*     max |rollio - rollio medio in piano|, in transizione e sulla
%                  rampa.
%   dh_corpo       distanza del corpo dalla superficie, misurata in normale,
%                  sulla rampa meno in piano [mm]. Negativo = corpo schiacciato
%                  verso la rampa (zampe anteriori che toccano prima).
%   v_rapporto     velocita' lungo la rampa / velocita' in piano. 1 = la salita
%                  non rallenta il robot.
%
%   Segno dell'inclinazione: adatta_simscape da' rpy in convenzione ZYX, dove
%   il muso in su e' beccheggio NEGATIVO. Qui incl = -pitch. Se in salita il
%   segno esce opposto alla pendenza, lo script lo dice invece di scrivere
%   un numero con il segno sbagliato.
%
% SALITA RIUSCITA - criterio dichiarato il 22/9, prima della ricerca del
% limite (script_T4_limite). Tutte e quattro:
%   1. tutte e sei le zampe fanno almeno un appoggio sulla rampa;
%   2. almeno due cicli a regime sulla rampa;
%   3. v_rapporto >= 0.5: il robot sale almeno a meta' della velocita' in piano;
%   4. assetto entro 30 gradi RISPETTO ALLA RAMPA: rollio rispetto al piano, e
%      inclinazione rispetto alla pendenza. La soglia e' quella di metriche
%      (30 gradi), ma metriche la applica al beccheggio ASSOLUTO: su una rampa
%      di 30 gradi un robot perfettamente allineato risulterebbe "ribaltato".
%      Per questo in T4 la colonna causa_fallimento di metriche non decide
%      l'esito; resta in tabella, e sopra i ~20 gradi va ignorata.
% La prima condizione violata va in causa_salita.
%
% COLONNE DI CONTATTO: come in T5, a NaN se i sensori non chiudono sul peso.
%
% USO DA ALTRI SCRIPT (script_T4_limite)
%   OVERRIDE_T4 = struct('c2',true, 'gradi',20, 'grafico',false);  script_T4
% Lo struct viene letto e CANCELLATO subito: non sopravvive alla run.
%
% ATTENZIONE AI NOMI: init_gait e' uno script e sovrascrive variabili del
% workspace. Tutte le variabili qui sono prefissate t4_.
%
% Progetto FSR PhantomX - A. Russo

t4_cfg        = phantomx_config();
t4_mdl        = 'phantomx_sim_zero';
t4_c2         = true;      % false = C1, anello aperto. true = C2.
t4_dur        = 20;         % [s] ~4.5 s in piano, il resto in salita
t4_soglia_su  = 0.015;      % [m] appoggio piu' alto del piano = sulla rampa
t4_v_appoggio = 0.05;       % [m/s] piede piu' lento di cosi' = fermo, in appoggio
t4_gradi      = t4_cfg.terreno.rampa_gradi;
t4_grafico    = true;
if exist('OVERRIDE_T4','var')
    if isfield(OVERRIDE_T4,'c2'),      t4_c2      = OVERRIDE_T4.c2;      end
    if isfield(OVERRIDE_T4,'gradi'),   t4_gradi   = OVERRIDE_T4.gradi;   end
    if isfield(OVERRIDE_T4,'grafico'), t4_grafico = OVERRIDE_T4.grafico; end
    clear OVERRIDE_T4                  % lo stato residuo e' gia' costato una campagna
end
t4_ctrl       = t4_nome_ctrl(t4_c2);

% Il terreno si fissa qui, non si eredita.
applica_terreno('T4', false, t4_mdl, struct('rampa_gradi', t4_gradi));

fprintf('\nT4: controllore %s, rampa di %g gradi, %g s a velocita'' nominale\n', ...
        t4_ctrl, t4_gradi, t4_dur);

OVERRIDE_C2 = t4_c2;                                           %#ok<NASGU>
clear OVERRIDE_GAIT                                            % andatura nominale
init_gait

t4_out = sim(t4_mdl, 'StopTime', num2str(t4_dur));
t4_run = adatta_simscape(t4_out, struct( ...
             'controller', t4_ctrl, 'task','T4', 'run',1, ...
             'condizione', sprintf('rampa%g', t4_gradi), ...
             'vel_d', [t4_cfg.v_nom 0]));

clear OVERRIDE_C2
init_gait                                                      % ripristina cfg

t4_riga = metriche(t4_run, t4_cfg, struct('t_regime', 2*t4_cfg.T));

%% ---- le finestre e le metriche di salita ----
t4_R = t4_salita(t4_run, t4_soglia_su, t4_v_appoggio, t4_cfg, 2*t4_cfg.T, deg2rad(t4_gradi));

%% ---- colonne di contatto: valgono solo se i sensori vedono tutto il peso ----
t4_chiusura = NaN;
if isfield(t4_run,'Fc') && ~isempty(t4_run.Fc)
    t4_sel = t4_run.t >= 2*t4_cfg.T;
    t4_chiusura = mean(sum(t4_run.Fc(t4_sel,3:3:18),2)) / (t4_cfg.mass*t4_cfg.g);
end
t4_riga.chiusura_peso = t4_chiusura;
if isnan(t4_chiusura) || abs(t4_chiusura - 1) > 0.05
    t4_contatto = intersect({'appoggio_medio','sotto3_frac','slip_tot','slip_per_passo', ...
        'distacchi','frazione_persa','Fz_max_norm','disp_carico', ...
        'appoggi_scartati','appoggi_totali'}, t4_riga.Properties.VariableNames);
    for t4_k = 1:numel(t4_contatto), t4_riga.(t4_contatto{t4_k}) = NaN; end
    t4_riga.note = t4_riga.note + sprintf( ...
        "; sensori chiudono al %.0f%% del peso (vedono solo il pavimento): colonne di contatto a NaN", ...
        100*t4_chiusura);
    fprintf(2, ['\n  I sensori chiudono al %.0f%% del peso: non vedono i piedi sulla rampa.\n' ...
                '  Colonne di contatto messe a NaN. Assetto e avanzamento non ne risentono.\n'], ...
                100*t4_chiusura);
end

t4_riga.gradi           = t4_gradi;
t4_riga.t_ingresso      = t4_R.t_ini;
t4_riga.t_tutti_su      = t4_R.t_tutti;
t4_riga.x_ingresso      = t4_R.x_ini;
t4_riga.durata_rampa    = t4_R.durata_rampa;
t4_riga.pendenza_mis    = t4_R.pendenza;
t4_riga.incl_ref_piano  = t4_R.incl_ref;
t4_riga.incl_media_rampa= t4_R.incl_media;
t4_riga.incl_err        = t4_R.incl_err;
t4_riga.incl_pp         = t4_R.incl_pp;
t4_riga.roll_ref_piano  = t4_R.roll_ref;
t4_riga.roll_esc_trans  = t4_R.roll_esc_trans;
t4_riga.roll_esc_rampa  = t4_R.roll_esc_rampa;
t4_riga.h_piano         = t4_R.h_piano;
t4_riga.dh_corpo        = t4_R.dh;
t4_riga.v_rapporto      = t4_R.v_rapporto;
t4_riga.salito          = t4_R.salito;
t4_riga.causa_salita    = string(t4_R.causa);

if ~isfolder('results'), mkdir('results'); end
if t4_gradi == t4_cfg.terreno.rampa_gradi
    t4_file = fullfile('results', sprintf('T4_%s.csv', t4_ctrl));
else                                   % le run della ricerca del limite
    if ~isfolder(fullfile('results','T4_limite')), mkdir(fullfile('results','T4_limite')); end
    t4_file = fullfile('results', 'T4_limite', sprintf('T4_%s_%gdeg.csv', t4_ctrl, t4_gradi));
end
writetable(t4_riga, t4_file);

%% ---- lettura ----
fprintf('\n=============== T4 - %s - %g gradi ===============\n', t4_ctrl, t4_gradi);
fprintf('  pendenza misurata dagli appoggi  %.2f deg  (cfg: %g)\n', ...
        rad2deg(t4_R.pendenza), t4_gradi);
fprintf('  primo piede sulla rampa  t = %.2f s, x = %.3f m\n', t4_R.t_ini, t4_R.x_ini);
fprintf('  tutte e sei sulla rampa  t = %.2f s   -> regime in salita per %.1f s\n', ...
        t4_R.t_tutti, t4_R.durata_rampa);
fprintf('\n  inclinazione del corpo   in piano %+6.2f deg   sulla rampa %+6.2f deg\n', ...
        rad2deg(t4_R.incl_ref), rad2deg(t4_R.incl_media));
fprintf('  errore rispetto alla rampa (tolto l''offset in piano)  %+6.2f deg\n', ...
        rad2deg(t4_R.incl_err));
fprintf('  beccheggio picco-picco sulla rampa                    %6.2f deg\n', ...
        rad2deg(t4_R.incl_pp));
fprintf('  rollio: escursione in transizione %.2f deg, sulla rampa %.2f deg\n', ...
        rad2deg(t4_R.roll_esc_trans), rad2deg(t4_R.roll_esc_rampa));
fprintf('  corpo sulla superficie: %+.1f mm rispetto al piano (%.1f mm)\n', ...
        1e3*t4_R.dh, 1e3*t4_R.h_piano);
fprintf('  velocita'' in salita / in piano: %.2f\n', t4_R.v_rapporto);
fprintf('\n  salito: %s   %s   frazione del task %.0f%%\n', ...
        t4_si(t4_R.salito), t4_R.causa, 100*t4_riga.frazione_task);
fprintf('\n  scritto  %s\n\n', t4_file);

%% ---- grafico ----
if t4_grafico && ~isnan(t4_R.t_tutti)
figure;
subplot(3,1,1); hold on; grid on
plot(t4_run.t, rad2deg(-t4_run.rpy(:,2)), 'DisplayName','inclinazione (-beccheggio)');
plot(t4_run.t, rad2deg(t4_run.rpy(:,1)),  'DisplayName','rollio');
yline(rad2deg(t4_R.pendenza + t4_R.incl_ref), 'k:', 'HandleVisibility','off');
t4_ombra(t4_R);
ylabel('[deg]'); legend('Location','best');
title(sprintf(['T4 - %s  (grigio chiaro: transizione, grigio scuro: rampa; ' ...
               'punteggiato: corpo parallelo alla rampa)'], t4_ctrl))
subplot(3,1,2); hold on; grid on
plot(t4_run.t, 1e3*t4_R.h_t);
yline(1e3*t4_R.h_piano, 'k:');
t4_ombra(t4_R);
ylabel('corpo - superficie [mm]');
subplot(3,1,3); hold on; grid on
plot(t4_run.t, 1e3*(t4_run.pf(:,3:3:18) - t4_R.z_piano));
yline(1e3*t4_soglia_su, 'k:');
t4_ombra(t4_R);
xlabel('t [s]'); ylabel('z piedi - piano [mm]');
end

%% ================= helper =================
function R = t4_salita(r, soglia, v_app, cfg, t_regime, pend_cmd)
%T4_SALITA  Finestre piano / transizione / rampa, metriche di salita ed esito.
%   Non si ferma con un errore se la salita fallisce: nella ricerca del limite
%   il fallimento E' il dato. Le metriche che non si possono calcolare restano
%   NaN e causa dice perche'.
T  = cfg.T;
t  = r.t(:);
dt = median(diff(t));
zf = r.pf(:, 3:3:18);                      % [N x 6] quota dei piedi, ordine CAN
xf = r.pf(:, 1:3:18);
fermo = appoggio_cinematico(r, cfg, v_app);

nan_ = NaN;
R = struct('t_ini',nan_, 't_tutti',nan_, 'x_ini',nan_, 'z_piano',nan_, ...
    'durata_rampa',0, 'a_regime',false, 'w_trans',[nan_ nan_], 'w_rampa',[nan_ nan_], ...
    'pendenza',nan_, 'incl_ref',nan_, 'incl_media',nan_, 'incl_err',nan_, 'incl_pp',nan_, ...
    'roll_ref',nan_, 'roll_esc_trans',nan_, 'roll_esc_rampa',nan_, ...
    'h_t',nan(size(t)), 'h_piano',nan_, 'dh',nan_, 'v_rapporto',nan_, ...
    'salito',false, 'causa','');

zs_ord  = sort(zf(fermo));                 % 10 percentile senza toolbox:
R.z_piano = zs_ord(max(1, round(0.10*numel(zs_ord))));   % vedi intestazione
su = fermo & (zf - R.z_piano > soglia);
if ~any(su(:))
    R.causa = 'non raggiunge la rampa';
    return
end

% primo appoggio sulla rampa, per zampa
t_primo = inf(1,6);
for i = 1:6
    k = find(su(:,i), 1, 'first');
    if ~isempty(k), t_primo(i) = t(k); end
end
R.t_ini = min(t_primo);
R.x_ini = r.p(find(t >= R.t_ini, 1, 'first'), 1);
w_piano = t >= t_regime & t <= R.t_ini - T/2;
if sum(w_piano)*dt < 2*T
    warning('script_T4:piano', 'Meno di due cicli in piano prima della rampa: riferimento debole.');
end
incl = -r.rpy(:,2);                        % ZYX: muso in su = beccheggio negativo
R.incl_ref = mean(incl(w_piano));
R.roll_ref = mean(r.rpy(w_piano,1));
dopo = t >= R.t_ini - T/2;
R.roll_esc_trans = max(abs(r.rpy(dopo,1) - R.roll_ref));   % ridefinito sotto se sale

% pendenza dagli appoggi sulla rampa (tutti quelli disponibili)
if nnz(su) > 10
    pq = polyfit(xf(su), zf(su), 1);
    R.pendenza = atan(pq(1));
else
    pq = [tan(pend_cmd) 0];  R.pendenza = pend_cmd;
end
pend = R.pendenza;

if any(isinf(t_primo))
    R.causa = sprintf('non tutte le zampe sulla rampa (%s)', ...
                      strjoin(cfg.legNamesCAN(isinf(t_primo)), ' '));
    R.salito = false;
    R.roll_esc_trans = max(abs(r.rpy(dopo,1) - R.roll_ref));
    % l'assetto conta anche qui: se si e' ribaltato, lo si dice
    if R.roll_esc_trans > deg2rad(30) || any(incl(dopo) - R.incl_ref > pend + deg2rad(30)) ...
            || any(incl(dopo) - R.incl_ref < -deg2rad(30))
        R.causa = [R.causa '; assetto oltre 30 gradi'];
    end
    return
end

R.t_tutti = max(t_primo);
w_trans = t >= R.t_ini - T/2 & t <= R.t_tutti + T;
w_rampa = t >  R.t_tutti + T;
R.durata_rampa = max(0, t(end) - (R.t_tutti + T));
R.a_regime = R.durata_rampa >= 2*T;
R.w_trans = [R.t_ini - T/2, R.t_tutti + T];
R.w_rampa = [R.t_tutti + T, t(end)];

R.roll_esc_trans = max(abs(r.rpy(w_trans,1) - R.roll_ref));
if any(w_rampa)
    R.incl_media = mean(incl(w_rampa));
    R.incl_err   = (R.incl_media - R.incl_ref) - pend;
    R.incl_pp    = max(incl(w_rampa)) - min(incl(w_rampa));
    R.roll_esc_rampa = max(abs(r.rpy(w_rampa,1) - R.roll_ref));
    if sign(R.incl_media - R.incl_ref) ~= sign(pend) && abs(R.incl_media - R.incl_ref) > deg2rad(2)
        warning('script_T4:segno', ['In salita il corpo risulta inclinato al contrario della ' ...
                'rampa (%.1f deg contro %.1f): controlla la convenzione di rpy prima di ' ...
                'usare incl_err.'], rad2deg(R.incl_media - R.incl_ref), rad2deg(pend));
    end
end

% distanza corpo-superficie in normale: in piano dal pavimento, poi dalla retta
zr   = polyval(pq, r.p(:,1));
zs   = max(R.z_piano, zr);
cosn = ones(size(t));  cosn(zr > R.z_piano) = cos(pend);
R.h_t     = (r.p(:,3) - zs) .* cosn;
R.h_piano = mean(R.h_t(w_piano));
if any(w_rampa), R.dh = mean(R.h_t(w_rampa)) - R.h_piano; end

% avanzamento: lungo la superficie in salita, lungo x in piano
vx = @(w) (r.p(find(w,1,'last'),1) - r.p(find(w,1,'first'),1)) / ...
          (t(find(w,1,'last')) - t(find(w,1,'first')));
if nnz(w_rampa) > 1 && nnz(w_piano) > 1
    R.v_rapporto = (vx(w_rampa) / cos(pend)) / vx(w_piano);
end

% ---- esito: i quattro criteri dell'intestazione, nell'ordine ----
rel_trans = incl(w_trans) - R.incl_ref;
assetto_ko = R.roll_esc_trans > deg2rad(30) || ...
             (any(w_rampa) && R.roll_esc_rampa > deg2rad(30)) || ...
             any(rel_trans > pend + deg2rad(30)) || any(rel_trans < -deg2rad(30)) || ...
             (any(w_rampa) && any(abs(incl(w_rampa) - R.incl_ref - pend) > deg2rad(30)));
if ~R.a_regime
    R.causa = 'meno di due cicli a regime sulla rampa';
elseif ~(R.v_rapporto >= 0.5)
    R.causa = sprintf('si ferma (v_rapporto %.2f < 0.5)', R.v_rapporto);
elseif assetto_ko
    R.causa = 'assetto oltre 30 gradi rispetto alla rampa';
else
    R.causa = 'salita riuscita';
end
R.salito = strcmp(R.causa, 'salita riuscita');
end

function t4_ombra(R)
%T4_OMBRA  Fasce sulla transizione e sulla rampa. Va chiamata DOPO i plot.
yl = ylim;
h1 = patch([R.w_trans(1) R.w_trans(2) R.w_trans(2) R.w_trans(1)], [yl(1) yl(1) yl(2) yl(2)], ...
           [0.92 0.92 0.92], 'EdgeColor','none', 'HandleVisibility','off');
h2 = patch([R.w_rampa(1) R.w_rampa(2) R.w_rampa(2) R.w_rampa(1)], [yl(1) yl(1) yl(2) yl(2)], ...
           [0.82 0.82 0.82], 'EdgeColor','none', 'HandleVisibility','off');
uistack([h1 h2], 'bottom');
ylim(yl);
end

function s = t4_nome_ctrl(c2)
if c2, s = 'C2'; else, s = 'C1'; end
end

function s = t4_si(b)
if b, s = 'si'; else, s = 'NO'; end
end
