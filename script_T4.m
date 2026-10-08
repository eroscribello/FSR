%% script_T4.m - T4: salita su rampa non modellata  [impianto SIMSCAPE]
%
%   Il robot cammina dritto a velocita' nominale e incontra una rampa di 8
%   gradi che il generatore di traiettoria non conosce. C1 continua a mettere
%   i piedi alla quota del piano; C2 dovrebbe trovare il terreno.
%
% METRICHE
%   pendenza_mis   pendenza della superficie ricavata dagli appoggi (retta z-x
%                  dei piedi fermi sulla rampa). 
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
%   il muso in su e' beccheggio NEGATIVO. Qui incl = -pitch. 
%
% SALITA RIUSCITA:
%   1. tutte e sei le zampe fanno almeno un appoggio sulla rampa;
%   2. almeno due cicli a regime sulla rampa;
%   3. v_rapporto >= 0.5: il robot sale almeno a meta' della velocita' in piano;
%   4. assetto entro 30 gradi RISPETTO ALLA RAMPA: rollio rispetto al piano, e
%      inclinazione rispetto alla pendenza.
% La prima condizione violata va in causa_salita.

t4_cfg        = phantomx_config();
t4_ctrl       = 'C2';      % 'C1' | 'C2' | 'C3'
t4_dur        = 20;         % [s] ~4.5 s in piano, il resto in salita
t4_soglia_su  = 0.015;      % [m] appoggio piu' alto del piano = sulla rampa
t4_v_appoggio = 0.05;       % [m/s] piede piu' lento di cosi' = fermo, in appoggio
t4_gradi      = t4_cfg.terreno.rampa_gradi;
t4_grafico    = true;
t4_forzato    = '';        % chi ha scavalcato t4_ctrl, per dirlo a schermo
if exist('OVERRIDE_CTRL','var') && ~isempty(OVERRIDE_CTRL)
    t4_ctrl = OVERRIDE_CTRL;  t4_forzato = 'OVERRIDE_CTRL';
end
if exist('OVERRIDE_T4','var')
    % Interfaccia vecchia, booleana: la usa ancora script_T4_limite.
    % Copre solo C1 e C2; per C3 serve OVERRIDE_T4.ctrl, che ha la precedenza.
    if isfield(OVERRIDE_T4,'c2')
        if OVERRIDE_T4.c2, t4_ctrl = 'C2'; else, t4_ctrl = 'C1'; end
        t4_forzato = 'OVERRIDE_T4.c2';
    end
    if isfield(OVERRIDE_T4,'ctrl')
        t4_ctrl = OVERRIDE_T4.ctrl;  t4_forzato = 'OVERRIDE_T4.ctrl';
    end
    if isfield(OVERRIDE_T4,'gradi'),   t4_gradi   = OVERRIDE_T4.gradi;   end
    if isfield(OVERRIDE_T4,'grafico'), t4_grafico = OVERRIDE_T4.grafico; end
    clear OVERRIDE_T4                  
end
if ~isempty(t4_forzato)
    fprintf(2, '  [%s] controllore forzato a %s\n', t4_forzato, t4_ctrl);
end
[t4_mdl, t4_c2, t4_info] = scegli_controllore(t4_ctrl);

applica_terreno('T4', false, t4_mdl, struct('rampa_gradi', t4_gradi));

applica_inerzie(t4_mdl);

allinea_stimatore(t4_mdl);

if strcmp(t4_mdl, 'phantomx_sim_attitude')   % il pacco resta solo per C3P
    if t4_info.carico, commenta_carico(t4_mdl, 'off'); else, commenta_carico(t4_mdl); end
    if t4_info.assetto, spegni_assetto('off', t4_mdl); else, spegni_assetto('on', t4_mdl); end
end

fprintf('\nT4: controllore %s, rampa di %g gradi, %g s a velocita'' nominale\n', ...
        t4_ctrl, t4_gradi, t4_dur);

OVERRIDE_C2 = t4_c2;                                          
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

%% ---- finestre e metriche di salita ----
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
salva_grafico(sprintf('T4_%s_%gdeg', t4_ctrl, t4_gradi));   % grafici/<tag>.fig
end

%% ================= helper =================
function R = t4_salita(r, soglia, v_app, cfg, t_regime, pend_cmd)
%T4_SALITA  Finestre piano / transizione / rampa, metriche di salita ed esito.
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

zs_ord  = sort(zf(fermo));
if isempty(zs_ord)
    % Senza questo controllo l'errore e' "Index exceeds array bounds" su
    % zs_ord, che non dice niente. Il problema non e' la soglia della rampa:
    % e' che nessun piede risulta mai fermo, cioe' o la run e' divergente o
    % e' troppo corta perche' un tratto fermo duri un quarto di appoggio.
    vmin = inf;
    for i_ = 1:6
        c_ = 3*(i_-1) + (1:3);
        vmin = min(vmin, min(vecnorm(gradient(r.pf(:,c_).', dt).', 2, 2)));
    end
    error('script_T4:appoggi', ...
        ['Nessun piede riconosciuto in appoggio in tutta la run.\n' ...
         '  durata %.2f s, %d campioni, dt %.4g s\n' ...
         '  tratto fermo minimo richiesto: %d campioni (un quarto di appoggio)\n' ...
         '  velocita'' minima di un piede: %.4f m/s, soglia %.3f m/s\n' ...
         'Se la velocita'' minima e'' ben sopra la soglia il robot non si ferma\n' ...
         'mai: guardare la quota del corpo e l''assetto prima di toccare le soglie.'], ...
         t(end), numel(t), dt, round(0.25*cfg.T_stance/dt), vmin, v_app);
end
R.z_piano = zs_ord(max(1, round(0.10*numel(zs_ord))));
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

% ---- esito ----
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
%T4_OMBRA  Fasce sulla transizione e sulla rampa.
yl = ylim;
h1 = patch([R.w_trans(1) R.w_trans(2) R.w_trans(2) R.w_trans(1)], [yl(1) yl(1) yl(2) yl(2)], ...
           [0.92 0.92 0.92], 'EdgeColor','none', 'HandleVisibility','off');
h2 = patch([R.w_rampa(1) R.w_rampa(2) R.w_rampa(2) R.w_rampa(1)], [yl(1) yl(1) yl(2) yl(2)], ...
           [0.82 0.82 0.82], 'EdgeColor','none', 'HandleVisibility','off');
uistack([h1 h2], 'bottom');
ylim(yl);
end

function s = t4_si(b)
if b, s = 'si'; else, s = 'NO'; end
end
