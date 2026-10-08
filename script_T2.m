%% script_T2.m - T2: piano, rettilineo, curva di velocita'  [impianto SIMSCAPE]
%
% DEFINIZIONE CANONICA DEL TASK
%   La variabile indipendente e' la VELOCITA' COMANDATA. Il periodo si ricava
%   da  T = S / (beta_stance * v), quindi la lunghezza del passo resta quella
%   validata e la differenza fra le celle e' attribuibile alla velocita' e non
%   a un diverso punto di lavoro della gamba.

% TARATURA PER VELOCITA'
%   Se cfg.t2_taratura esiste, ogni cella usa la SUA z0 e la SUA H, scelte da
%   taratura_T2 con un criterio dichiarato. T2 misura allora il MEGLIO
%   OTTENIBILE dal controllore a ciascuna velocita', non la robustezza di una
%   taratura unica: la differenza va detta in relazione, perche' il confronto
%   con C3 diventa "C1 al suo meglio" contro "C3 con una taratura".
%   Le tarature usate finiscono nelle colonne z0 e H della tabella.
%
%   I parametri del CONTATTO non cambiano mai fra le celle: descrivono il
%   terreno, non il controllore.

t2_cfg    = phantomx_config();
t2_ctrl    = 'C2';      % 'C1' | 'C2' | 'C3'
if exist('OVERRIDE_CTRL','var') && ~isempty(OVERRIDE_CTRL)
    t2_ctrl = OVERRIDE_CTRL;
    fprintf(2, '  [OVERRIDE_CTRL] controllore forzato a %s\n', t2_ctrl);
end
[t2_mdl, t2_c2, t2_info] = scegli_controllore(t2_ctrl);
t2_fatt   = t2_cfg.t2_fattori;
t2_nCicli = 10;
T2 = table();
t2_runs  = cell(1, numel(t2_fatt));
t2_etichette = cell(1, numel(t2_fatt));

applica_terreno('T2', false, t2_mdl); 

applica_inerzie(t2_mdl); 

allinea_stimatore(t2_mdl);

if strcmp(t2_mdl, 'phantomx_sim_attitude')   % il pacco resta solo per C3P
    if t2_info.carico, commenta_carico(t2_mdl, 'off'); else, commenta_carico(t2_mdl); end
    if t2_info.assetto, spegni_assetto('off', t2_mdl); else, spegni_assetto('on', t2_mdl); end
end

t2_haTar = isfield(t2_cfg,'t2_taratura') && ~isempty(t2_cfg.t2_taratura);
if t2_haTar
    fprintf('\nT2: taratura per velocita'' da cfg.t2_taratura\n');
else
    fprintf('\nT2: taratura UNICA (cfg.z0, cfg.H)\n');
end

for t2_i = 1:numel(t2_fatt)

    t2_vnom  = t2_fatt(t2_i) * t2_cfg.v_nom;                  
    t2_T     = t2_cfg.S / (t2_cfg.beta_stance * t2_vnom);      % periodo derivato
    t2_stop  = t2_nCicli * t2_T;                               % stesso numero di passi
    t2_etich = sprintf('v%.2fx', t2_fatt(t2_i));
    t2_etichette{t2_i} = t2_etich;

    t2_ov = struct('T', t2_T);
    if t2_haTar
        t2_r = find(abs(t2_cfg.t2_taratura(:,1) - t2_fatt(t2_i)) < 1e-9, 1);
        if isempty(t2_r)
            fprintf(2,'  %s: nessuna taratura in cfg.t2_taratura, uso la nominale\n', t2_etich);
        else
            t2_ov.z0 = t2_cfg.t2_taratura(t2_r, 2);
            t2_ov.H  = t2_cfg.t2_taratura(t2_r, 3);
            if t2_cfg.t2_taratura(t2_r, 4) == 0
                fprintf(2,['  %s: la taratura scelta NON era ammissibile (nessuna\n' ...
                           '      cella della griglia lo era). La run e'' informativa,\n' ...
                           '      non un punto della curva.\n'], t2_etich);
            end
        end
    end

    OVERRIDE_C2   = t2_c2;                                     
    OVERRIDE_GAIT = t2_ov;                                     
    init_gait                                                 

    t2_out = sim(t2_mdl, 'StopTime', num2str(t2_stop));

    t2_run = adatta_simscape(t2_out, struct( ...
                 'controller', t2_ctrl, 'task','T2', 'run',1, ...
                 'condizione', t2_etich, ...
                 'vel_d', [t2_vnom 0]));

    t2_cfgLoc   = t2_cfg;
    t2_cfgLoc.T = t2_T;                     
    t2_riga = metriche(t2_run, t2_cfgLoc, struct('t_regime', 2*t2_T));

   
    if isfield(t2_ov,'z0'), t2_riga.z0 = t2_ov.z0; else, t2_riga.z0 = t2_cfg.z0; end
    if isfield(t2_ov,'H'),  t2_riga.H  = t2_ov.H;  else, t2_riga.H  = t2_cfg.H;  end
    t2_riga.T_gait  = t2_T;
    t2_riga.fattore = t2_fatt(t2_i);

    T2 = [T2; t2_riga];                                       
    t2_runs{t2_i} = t2_run;

    fprintf('\n=== %s : v_nom %.4f | T %.2f s | z0 %.4f | H %.3f | misurata %.4f | piedi %.2f ===\n\n', ...
            t2_etich, t2_vnom, t2_T, t2_riga.z0, t2_riga.H, ...
            t2_riga.vel_media, t2_riga.appoggio_medio);

end

clear OVERRIDE_GAIT OVERRIDE_C2
init_gait                                                      % ripristina i valori di cfg

% I CSV di campagna vanno in results/, non nella radice
if ~isfolder('results'), mkdir('results'); end
t2_file = fullfile('results', sprintf('T2_%s.csv', t2_ctrl));
writetable(T2, t2_file);
fprintf('\nscritto  %s\n\n', t2_file);

% sotto3_frac e corpoZ_pp servono a leggere le celle FUORI dall'inviluppo
% (1.5x e 2.0x): li' frazione_task e' negativa e senza queste due colonne non
% si puo' dire perche'. A quelle velocita' non e' errore di inseguimento - i
% giunti eseguono la corsa comandata - e' il robot che non cammina.
disp(T2(:, {'condizione','vel_media','frazione_task','successo', ...
            'appoggio_medio','sotto3_frac','corpoZ_pp','corpoZ_vz', ...
            'potenza_max','cot'}))

fprintf('\n');
for t2_j = 1:numel(t2_runs)
    t2_vx  = t2_runs{t2_j}.v(:,1);
    t2_sel = t2_runs{t2_j}.t >= 0.2 * t2_runs{t2_j}.t(end);
    fprintf('%-8s  v media %.4f | min %+.4f | max %.4f | oscillazione %3.0f%%\n', ...
        t2_runs{t2_j}.meta.condizione, mean(t2_vx(t2_sel)), ...
        min(t2_vx(t2_sel)), max(t2_vx(t2_sel)), ...
        100*std(t2_vx(t2_sel))/mean(t2_vx(t2_sel)));
end

% La legenda viene generata dai fattori
figure; hold on; grid on
for t2_j = 1:numel(t2_runs)
    plot(t2_runs{t2_j}.t / t2_runs{t2_j}.t(end) * t2_nCicli, t2_runs{t2_j}.v(:,1));
end
legend(t2_etichette, 'Location','best');
xlabel('cicli di andatura'); ylabel('v_x [m/s]')
title(sprintf('T2 - curva di velocita'', %s su Simscape', t2_ctrl))
salva_grafico(sprintf('T2_%s', t2_ctrl));   

