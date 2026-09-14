%% script_T2.m - T2: piano, rettilineo, tre velocita'
% Le tre velocita' si ottengono variando la CADENZA a passo costante:
% cosi' la geometria della gamba resta quella validata e la differenza fra
% le celle e' attribuibile alla velocita', non a un diverso punto di lavoro.
%
% ATTENZIONE AI NOMI: init_gait e' uno script, gira in questo workspace e
% sovrascrive k, j, L, cfg, alpha, side, offset, pen. Tutte le variabili di
% questo ciclo sono prefissate t2_ per non finirci sotto.

t2_cfg  = phantomx_config();
t2_mdl  = 'phantomx_sim_zero';
t2_fatt = [0.5 0.75 1.0 1.5 2.0];
t2_nCicli = 10;
T2 = table();
t2_runs = cell(1, numel(t2_fatt));

for t2_i = 1:numel(t2_fatt)

    t2_T     = t2_cfg.T / t2_fatt(t2_i);                      % cadenza
    t2_vnom  = t2_cfg.S / (t2_cfg.beta_stance * t2_T);        % v nominale di QUESTA cella
    t2_stop  = t2_nCicli * t2_T;                              % stesso numero di passi
    t2_etich = sprintf('v%.1fx', t2_fatt(t2_i));

    OVERRIDE_GAIT = struct('T', t2_T);
    init_gait                                                 % <-- da qui k, j, L sono persi

    t2_out = sim(t2_mdl, 'StopTime', num2str(t2_stop));

    t2_run = adatta_simscape(t2_out, struct( ...
                 'controller','C1', 'task','T2', 'run',1, ...
                 'condizione', t2_etich, ...
                 'vel_d', [t2_vnom 0]));

    t2_cfgLoc   = t2_cfg;
    t2_cfgLoc.T = t2_T;                     % serve a slip_per_passo
    t2_riga = metriche(t2_run, t2_cfgLoc, struct('t_regime', 2*t2_T)); 
    T2 = [T2; t2_riga];
    t2_runs{t2_i} = t2_run; 

    fprintf('\n=== %s : T = %.2f s | v_nom %.4f | misurata %.4f | piedi %.2f ===\n\n', ...
            t2_etich, t2_T, t2_vnom, t2_riga.vel_media, t2_riga.appoggio_medio);

end 

clear OVERRIDE_GAIT
init_gait                                                     % ripristina i valori di cfg

writetable(T2, 'T2_C1.csv');
disp(T2(:, {'condizione','vel_media','frazione_task','successo', ...
            'causa_fallimento','cot','frazione_saturo'}))

fprintf('\n');
for t2_j = 1:numel(t2_runs)
    t2_vx  = t2_runs{t2_j}.v(:,1);
    t2_sel = t2_runs{t2_j}.t >= 0.2 * t2_runs{t2_j}.t(end);
    fprintf('%-7s  v media %.4f | min %+.4f | max %.4f | oscillazione %3.0f%%\n', ...
        t2_runs{t2_j}.meta.condizione, mean(t2_vx(t2_sel)), ...
        min(t2_vx(t2_sel)), max(t2_vx(t2_sel)), ...
        100*std(t2_vx(t2_sel))/mean(t2_vx(t2_sel)));
end

figure; hold on; grid on
for t2_j = 1:numel(t2_runs)
    plot(t2_runs{t2_j}.t / t2_runs{t2_j}.t(end), t2_runs{t2_j}.v(:,1));
end
legend('0.5x','1.0x','2.0x'); xlabel('frazione della run'); ylabel('v_x [m/s]')