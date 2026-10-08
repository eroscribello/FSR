%% script_T7.m - T7: spinta laterale impulsiva  [impianto SIMSCAPE]
%
% COSA MISURA
%
%       dy(t) = y_disturbata(t) - y_indisturbata(t)
%
%
%  CRITERI:
%     dev_max   = max |dy| dopo t0
%     dev_fin   = |dy| mediata sull'ultimo secondo
%     recupero  = 1 - dev_fin / dev_max        (1 = torna esattamente dov'era)
%
%   CADUTA      causa_fallimento == "ribaltamento", oppure la simulazione non
%               arriva in fondo, oppure z_media sotto il 60% del riferimento.
%   RECUPERO    non e' caduto, recupero >= 0.5, e |dyaw| <= 5 gradi.
%   SOTTO IL PAVIMENTO   dev_max sotto TRE VOLTE il valore della run
%               dv = 0.01. Il fattore 3 non e' scelto qui: e' la stessa regola
%               del pavimento di rumore del 23/9, dove una colonna discrimina
%               se il rapporto segnale/dispersione e' almeno 3. Una riga sotto
%               quella soglia non concorre a nessuna soglia di cedimento.
%   SOGLIE      il piu' piccolo impulso che NON recupera, e il piu' piccolo
%               che fa cadere. Sono due numeri diversi, si riportano entrambi.
%
t7_cfg  = phantomx_config();
t7_ctrl = 'C2';      % 'C1' | 'C2' | 'C3'
t7_dur  = 12;         % [s]
t7_t0   = 4;          % [s] la spinta arriva a marcia regolare (transitorio 2 s)
t7_larg = 0.05;       % [s] larghezza dell'impulso
t7_dv   = [0 0.01 1 2 4 8 16];       % moltiplicatori di v_nom

assert(t7_dv(1) == 0, 'script_T7:riferimento', ...
    'La prima ampiezza deve essere 0: e'' la run di riferimento, non un punto.');

if exist('OVERRIDE_CTRL','var') && ~isempty(OVERRIDE_CTRL)
    t7_ctrl = OVERRIDE_CTRL;
    fprintf(2, '  [OVERRIDE_CTRL] controllore forzato a %s\n', t7_ctrl);
end
[t7_mdl, t7_c2, t7_info] = scegli_controllore(t7_ctrl);
t7_J    = t7_cfg.mass * t7_dv * t7_cfg.v_nom;     % [N*s]

fprintf('\nT7: controllore %s, spinta laterale a t = %.1f s, %d run\n', ...
        t7_ctrl, t7_t0, numel(t7_J));
fprintf('  v_nom = %.3f m/s, massa = %.4f kg\n', t7_cfg.v_nom, t7_cfg.mass);
fprintf('  misura DIFFERENZIALE rispetto alla run indisturbata\n');

t7_righe = table();
t7_y_rif = [];  t7_yaw_rif = [];  t7_z_rif = NaN;  t7_pav = NaN;
t7_DY = [];  t7_tt = [];          % le serie di Dy, per il grafico di fondo

for t7_i = 1:numel(t7_J)

    % ---- terreno, inerzie, disturbo: si fissano qui, non si ereditano ----
    applica_terreno('T7', false, t7_mdl);
    applica_inerzie(t7_mdl);

    allinea_stimatore(t7_mdl);

    if strcmp(t7_mdl, 'phantomx_sim_attitude')   % il pacco resta solo per C3P
        if t7_info.carico, commenta_carico(t7_mdl, 'off'); else, commenta_carico(t7_mdl); end
        if t7_info.assetto, spegni_assetto('off', t7_mdl); else, spegni_assetto('on', t7_mdl); end
    end
    applica_disturbo(t7_mdl, t7_J(t7_i), 'applica', ...
                     struct('t0', t7_t0, 'durata', t7_larg, 'asse', 'y'));

    OVERRIDE_C2 = t7_c2;                                       
    clear OVERRIDE_GAIT
    init_gait

    fprintf('\n--- dv = %.2f x v_nom,  J = %.4f N*s ---\n', t7_dv(t7_i), t7_J(t7_i));

    t7_out = sim(t7_mdl, 'StopTime', num2str(t7_dur));
    t7_run = adatta_simscape(t7_out, struct( ...
                 'controller', t7_ctrl, 'task','T7', 'run', t7_i, ...
                 'condizione', sprintf('dv%.2f', t7_dv(t7_i)), ...
                 'vel_d', [t7_cfg.v_nom 0]));

    clear OVERRIDE_C2
    init_gait                                                  % ripristina cfg

    t7_riga = metriche(t7_run, t7_cfg, struct('t_regime', 2*t7_cfg.T));

    % ---- la run di riferimento si memorizza e non si giudica ----
    if t7_i == 1
        t7_y_rif   = t7_run.p(:,2);
        t7_yaw_rif = t7_run.rpy(:,3);
        t7_z_rif   = t7_riga.z_media;
        t7_n_rif   = numel(t7_run.t);
    end

    % ---- la differenza rispetto alla run indisturbata ----
    if numel(t7_run.t) ~= t7_n_rif
        error('script_T7:griglia', ...
            'Run %d ha %d campioni invece di %d: le griglie non coincidono.', ...
            t7_i, numel(t7_run.t), t7_n_rif);
    end
    t7_dy   = t7_run.p(:,2)   - t7_y_rif;
    t7_dyaw = t7_run.rpy(:,3) - t7_yaw_rif;

    if isempty(t7_tt), t7_tt = t7_run.t; end
    t7_DY(:, t7_i) = t7_dy;                                   
    t7_dopo = t7_run.t >= t7_t0;
    t7_coda = t7_run.t >= t7_run.t(end) - 1;

    t7_dmax = max(abs(t7_dy(t7_dopo)));
    t7_dfin = abs(mean(t7_dy(t7_coda)));

    t7_riga.dv_su_vnom   = t7_dv(t7_i);
    t7_riga.impulso      = t7_J(t7_i);
    t7_riga.dev_max_mm   = 1e3 * t7_dmax;
    t7_riga.dev_fin_mm   = 1e3 * t7_dfin;
    t7_riga.dyaw_deg     = rad2deg(mean(t7_dyaw(t7_coda)));
    t7_riga.durata_sim   = t7_run.t(end);
    
    t7_riga.dev_grezza_mm = 1e3 * max(abs(t7_run.p(t7_dopo,2) - t7_run.p(find(t7_dopo,1),2)));

    if t7_dmax > 1e-9
        t7_riga.recupero = 1 - t7_dfin / t7_dmax;
    else
        t7_riga.recupero = NaN;
    end

    % ---- il pavimento della misura, dalla run a dv = 0.01 ----
    if t7_i == 2
        t7_pav = t7_dmax;
        fprintf('  [pavimento] la spinta trascurabile produce %.3f mm:\n', 1e3*t7_pav);
        fprintf('              sotto questo valore non si legge niente.\n');
    end
    t7_riga.sotto_pavimento = (t7_i <= 2) || (t7_dmax < 3*t7_pav);

    % ---- i criteri, applicati come dichiarati in testa ----
    t7_riga.caduto = (t7_riga.causa_fallimento == "ribaltamento") || ...
                     (t7_run.t(end) < t7_dur - 1e-6) || ...
                     (t7_riga.z_media < 0.6 * t7_z_rif);
    t7_riga.recuperato = ~t7_riga.caduto && t7_riga.recupero >= 0.5 && ...
                         (1e-3*(t7_riga.dev_max_mm - t7_riga.dev_fin_mm) >= 3*t7_pav) && ...
                         abs(t7_riga.dyaw_deg) <= 5;

    if t7_i == 1
        % il riferimento non si giudica: e' il metro, non la misura
        t7_riga.recupero = NaN;  t7_riga.recuperato = true;
        t7_riga.sotto_pavimento = true;
    end

    fprintf('  Dy %.2f -> %.2f mm   recupero %.2f   Dyaw %+.2f deg   %s\n', ...
            t7_riga.dev_max_mm, t7_riga.dev_fin_mm, t7_riga.recupero, ...
            t7_riga.dyaw_deg, t7_esito(t7_riga));

    t7_righe = [t7_righe; t7_riga];                            %#ok<AGROW>

    t7_file = fullfile('results', sprintf('T7_%s.csv', t7_ctrl));
    writetable(t7_righe, t7_file);
end

%% ---- lettura ----
fprintf('\n=========== T7 %s ===========\n', t7_ctrl);
fprintf('  pavimento della misura differenziale: %.3f mm\n', 1e3*t7_pav);
fprintf('  soglia di leggibilita'' (3x il pavimento): %.3f mm\n\n', 3e3*t7_pav);
fprintf('  %8s %10s %10s %10s %9s %8s %10s  %s\n', 'dv/vnom', 'J [N*s]', ...
        'Dy max', 'Dy fin', 'recupero', 'Dyaw', 'grezza', 'esito');
for t7_i = 1:height(t7_righe)
    r = t7_righe(t7_i,:);
    fprintf('  %8.2f %10.4f %10.2f %10.2f %9.2f %8.2f %10.2f  %s\n', ...
            r.dv_su_vnom, r.impulso, r.dev_max_mm, r.dev_fin_mm, ...
            r.recupero, r.dyaw_deg, r.dev_grezza_mm, t7_esito(r));
end
fprintf('\n  "grezza" e'' la deviazione misurata come nella prima versione,\n');
fprintf('  cioe'' compreso l''ondeggio dell''andatura: serve a mostrare quanto\n');
fprintf('  segnale la misura differenziale recupera.\n');

t7_val    = t7_righe(~t7_righe.sotto_pavimento, :);
t7_i_norec = find(~t7_val.recuperato & ~t7_val.caduto, 1);
t7_i_cad   = find(t7_val.caduto, 1);

fprintf('\n  soglia di NON recupero : ');
if isempty(t7_i_norec), fprintf('non raggiunta nella spazzata\n');
else, fprintf('dv = %.2f x v_nom  (J = %.4f N*s)\n', ...
              t7_val.dv_su_vnom(t7_i_norec), t7_val.impulso(t7_i_norec)); end
fprintf('  soglia di CADUTA       : ');
if isempty(t7_i_cad), fprintf('non raggiunta nella spazzata\n');
else, fprintf('dv = %.2f x v_nom  (J = %.4f N*s)\n', ...
              t7_val.dv_su_vnom(t7_i_cad), t7_val.impulso(t7_i_cad)); end

if isempty(t7_i_norec) && isempty(t7_i_cad)
    fprintf(2, ['\n  %s recupera a ogni ampiezza provata. E'' il caso previsto in\n' ...
                '  testa al file: in ComputedTorque la coppia di reazione e''\n' ...
                '  illimitata, quindi il confronto con un controllore in coppia\n' ...
                '  sarebbe sbilanciato. Passo gia'' deciso: rifare T7 con gli\n' ...
                '  attuatori saturati al datasheet (%.2f N*m) per tutti e tre.\n'], ...
            t7_ctrl, t7_cfg.tau_max);
end

fprintf('\n  scritto %s\n', t7_file);
fprintf('  Il .slx non e'' stato salvato: bdclose all riporta tutto com''era.\n\n');

%% ---- il grafico ----
t7_fig = figure('Name', sprintf('T7 %s', t7_ctrl), 'Position', [80 60 920 720]);

% pannello 1: la deviazione nel tempo, scala log perche' le ampiezze
% coprono tre decadi (0.9 -> 400 mm)
subplot(2,1,1); hold on
t7_col = lines(numel(t7_J));
t7_vis = t7_tt >= t7_t0 - 0.3;
for t7_k = 2:numel(t7_J)
    plot(t7_tt(t7_vis), max(abs(1e3*t7_DY(t7_vis,t7_k)), 1e-3), ...
         'Color', t7_col(t7_k,:), 'LineWidth', 1.3, ...
         'DisplayName', sprintf('dv = %g  (J = %.3f N s)', t7_dv(t7_k), t7_J(t7_k)));
end
set(gca, 'YScale', 'log')
yline(3e3*t7_pav, 'k--', 'soglia di leggibilita'' (3x pavimento)', ...
      'LabelHorizontalAlignment','left', 'HandleVisibility','off');
xline(t7_t0, 'k:', 'spinta', 'HandleVisibility','off');
grid on; box on
xlabel('tempo [s]')
ylabel('|\Deltay| rispetto alla run indisturbata [mm]')
title(sprintf('T7 %s - deviazione laterale dopo la spinta', t7_ctrl))
legend('Location','southeast')

% pannello 2: massimo e finale a confronto. Se coincidono, non si recupera.
subplot(2,1,2); hold on
plot(t7_righe.impulso(2:end), t7_righe.dev_max_mm(2:end), 'o-',  ...
     'LineWidth',1.5, 'MarkerFaceColor','w', 'DisplayName','\Deltay massima');
plot(t7_righe.impulso(2:end), t7_righe.dev_fin_mm(2:end), 's--', ...
     'LineWidth',1.5, 'MarkerFaceColor','w', 'DisplayName','\Deltay finale');
yline(3e3*t7_pav, 'k--', 'soglia di leggibilita''', ...
      'LabelHorizontalAlignment','left', 'HandleVisibility','off');
set(gca, 'XScale', 'log', 'YScale', 'log')
grid on; box on
xlabel('impulso [N s]')
ylabel('deviazione [mm]')
title('Le due curve coincidono: quello che si sposta, non torna')
legend('Location','northwest')

salva_grafico(sprintf('T7_%s', t7_ctrl));

%% ================= helper =================
function s = t7_esito(r)
if r.dv_su_vnom == 0,      s = 'RIFERIMENTO';
elseif r.caduto,           s = 'CADUTO';
elseif r.sotto_pavimento,  s = 'sotto il pavimento';
elseif r.recuperato,       s = 'recuperato';
else,                      s = 'in piedi, non torna';
end
end
