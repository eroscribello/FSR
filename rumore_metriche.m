%% rumore_metriche.m
%
%   Una differenza fra C1 e C2 e' un risultato solo se e' piu' grande di
%   quanto le stesse metriche oscillano da sole. Questo script misura quel
%   "pavimento di rumore" e lo confronta con la differenza C1-C2 misurata in
%   campagna.
%
%   rapporto = |valore C1 - valore C2| in campagna / escursione fra le tre
%              perturbazioni (la peggiore fra i due controllori)
%     rapporto >= 3   la colonna DISCRIMINA: la differenza fra i controllori
%                     e' reale e si puo' scrivere in relazione
%     rapporto <  3   NON SI PUO' CONCLUDERE nulla da quella colonna su quel
%                     task: va tolta dalle conclusioni, non solo da qui
%   Il 3 e' arbitrario, ma e' fissato prima di vedere i numeri.
%

rm_cfg = phantomx_config();
rm_dz  = [0, +0.2e-3, -0.2e-3];        % [m] perturbazione su gait.z0
rm_tag    = 'slew';
rm_prove  = {'T2',  'C1', 10*rm_cfg.T, 'v1.00x'
             'T2',  'C2', 10*rm_cfg.T, 'v1.00x'
             'T2',  'C3', 10*rm_cfg.T, 'v1.00x'
             'T3',  'C1', 15,          'yaw+0.100'
             'T3',  'C2', 15,          'yaw+0.100'
             'T3',  'C3', 15,          'yaw+0.100'
             'T4D', 'C1', 30,          'dosso8-8'
             'T4D', 'C2', 30,          'dosso8-8'
             'T4D', 'C3', 30,          'dosso8-8'
             'T5',  'C1', 20,          'ost1'
             'T5',  'C2', 20,          'ost1'
             'T5',  'C3', 20,          'ost1'};
% le coppie da leggere, ADIACENTI: ognuna misura un meccanismo solo
rm_coppie = {'C1', 'C2'
             'C2', 'C3'};
% pavimenti gia' misurati da cui prendere il rumore dei controllori che non
% girano in questo giro: nessuno, girano tutti
rm_rumore_prec = {};

rm_xbordo = 4 - 0.05;                  % serve solo a T4D
rm_out_f  = fullfile('results','diagnostica', sprintf('rumore_%s.csv', rm_tag));

for rm_m = unique(cellfun(@scegli_controllore, rm_prove(:,2), 'UniformOutput', false))'
    if bdIsLoaded(rm_m{1}) && strcmp(get_param(rm_m{1},'Dirty'),'on')
        fprintf(2, ['\n  %s ha modifiche in memoria (normale). Terreno,\n' ...
                    '  inerzie e stimatore vengono reimpostati. Nessun salvataggio.\n'], rm_m{1});
    end
end
if ~isfolder(fullfile('results','diagnostica')), mkdir(fullfile('results','diagnostica')); end

RM = table();
rm_crono = tic;
rm_terreno_ora = '';

try
    for rm_i = 1:size(rm_prove,1)
        rm_t    = rm_prove{rm_i,1};
        rm_ctrl = rm_prove{rm_i,2};
        rm_dur  = rm_prove{rm_i,3};
        [rm_mdl, rm_c2, rm_info] = scegli_controllore(rm_ctrl);   % modello e interruttore

        if ~strcmp(rm_terreno_ora, [rm_t '|' rm_mdl])
            applica_imbardata(0, [], rm_mdl);  
            applica_terreno(rm_t, false, rm_mdl);
            applica_inerzie(rm_mdl);
            allinea_stimatore(rm_mdl); 
            if strcmp(rm_mdl, 'phantomx_sim_attitude')   % il pacco resta solo per C3P
                if rm_info.carico, commenta_carico(rm_mdl, 'off'); else, commenta_carico(rm_mdl); end
                if rm_info.assetto, spegni_assetto('off', rm_mdl); else, spegni_assetto('on', rm_mdl); end
            end
            rm_terreno_ora = [rm_t '|' rm_mdl];
        end

        rm_yaw  = 0;
        rm_S    = [];
        if strcmp(rm_t, 'T3')
            rm_yaw  = rm_cfg.yaw_d;
            rm_info = applica_imbardata(rm_yaw, [], rm_mdl);
            rm_S    = rm_info.S;
        end

        for rm_j = 1:numel(rm_dz)
            fprintf('\n---- %s, %s, z0 %+.1f mm ----\n', rm_t, rm_ctrl, 1e3*rm_dz(rm_j));

            OVERRIDE_C2   = rm_c2;                                 %#ok<NASGU>
            OVERRIDE_GAIT = struct('z0', rm_cfg.z0 + rm_dz(rm_j));
            if ~isempty(rm_S), OVERRIDE_GAIT.S = rm_S; end         %#ok<NASGU>
            clear OVERRIDE_C2_SOGLIA                               % soglia di campagna
            init_gait

            rm_run = adatta_simscape(sim(rm_mdl, 'StopTime', num2str(rm_dur)), ...
                         struct('controller', rm_ctrl, 'task', rm_t, 'run', 1, ...
                                'condizione', sprintf('z0%+.1fmm', 1e3*rm_dz(rm_j)), ...
                                'vel_d', [rm_cfg.v_nom 0], 'yaw_d', rm_yaw));

            if strcmp(rm_t,'T4D'), rm_run = rm_taglia(rm_run, rm_xbordo); end

            rm_riga = metriche(rm_run, rm_cfg, struct('t_regime', 2*rm_cfg.T));
            rm_riga.dz0 = rm_dz(rm_j);
            RM = [RM; rm_riga];                                    %#ok<AGROW>
            writetable(RM, rm_out_f);      
        end
    end
    for rm_m = unique(cellfun(@scegli_controllore, rm_prove(:,2), 'UniformOutput', false))'
        applica_imbardata(0, [], rm_m{1});
    end
    clear OVERRIDE_C2 OVERRIDE_GAIT
    init_gait
catch rm_err
    for rm_m = unique(cellfun(@scegli_controllore, rm_prove(:,2), 'UniformOutput', false))'
        try applica_imbardata(0, [], rm_m{1}); end %#ok<TRYNC>
    end
    clear OVERRIDE_C2 OVERRIDE_GAIT
    fprintf(2,'\n  Errore: modello NON salvato. Le run gia'' fatte sono in %s\n', rm_out_f);
    rethrow(rm_err);
end

%% ---- lettura: il rumore contro la differenza fra i controllori ----
rm_col = {'roll_max','pitch_max','dev_lat_max','yaw_err_fin','cot','energia', ...
          'tau_max','tau_rms','z_media','vel_media','frazione_task'};

if ~exist('rm_coppie', 'var'), rm_coppie = rm_coppia; end
for rm_iq = 1:size(rm_coppie, 1)
rm_coppia = rm_coppie(rm_iq, :);
rm_pool = RM(:, intersect([{'controller','task'}, rm_col], RM.Properties.VariableNames, 'stable'));
rm_pool.controller = string(rm_pool.controller);
rm_pool.task       = string(rm_pool.task);
rm_qui = unique(rm_pool.controller);
for rm_f = rm_rumore_prec'
    if ~isfile(rm_f{1}), continue; end
    rm_P = readtable(rm_f{1}, 'TextType', 'string');
    rm_P = rm_P(ismember(rm_P.controller, string(rm_coppia)) & ~ismember(rm_P.controller, rm_qui), :);
    if isempty(rm_P), continue; end
    rm_cc_ok = intersect(rm_pool.Properties.VariableNames, rm_P.Properties.VariableNames, 'stable');
    rm_pool  = [rm_pool(:, rm_cc_ok); rm_P(:, rm_cc_ok)];                  %#ok<AGROW>
end

fprintf('\n\n======= PAVIMENTO DI RUMORE (z0 +-0.2 mm), coppia %s-%s =======\n', rm_coppia{:});
for rm_t = unique(string(RM.task))'
    rm_T = rm_pool(strcmp(string(rm_pool.task), rm_t), :);
    rm_cella = rm_prove{find(strcmp(rm_prove(:,1), rm_t), 1), 4};

    rm_A = rm_campagna(rm_t, rm_coppia{1}, rm_cella);
    rm_B = rm_campagna(rm_t, rm_coppia{2}, rm_cella);

    fprintf('\n--- %s, cella %s ---\n', rm_t, rm_cella);
    fprintf('  %-14s %10s %10s %11s %10s %10s %8s   %s\n', '', rm_coppia{1}, rm_coppia{2}, ...
            sprintf('|%s-%s|', rm_coppia{:}), ['rum.' rm_coppia{1}], ['rum.' rm_coppia{2}], 'rapp.', 'verdetto');

    for rm_k = 1:numel(rm_col)
        rm_c = rm_col{rm_k};
        if ~ismember(rm_c, rm_T.Properties.VariableNames), continue; end

        % rumore = il peggiore fra i due controllori, ciascuno misurato su se'
        rm_rr = NaN(1, 2);
        for rm_q = 1:2
            rm_sel = strcmp(string(rm_T.controller), rm_coppia{rm_q});
            if any(rm_sel)
                rm_v = rm_T.(rm_c)(rm_sel);
                rm_rr(rm_q) = max(rm_v) - min(rm_v);
            end
        end
        if any(isnan(rm_rr))
            fprintf('  %-14s   manca il rumore di %s: nessun verdetto\n', rm_c, ...
                    strjoin(rm_coppia(isnan(rm_rr)), ', '));
            continue
        end
        rm_r = max(rm_rr);

        if isempty(rm_A) || isempty(rm_B) || ~ismember(rm_c, rm_A.Properties.VariableNames)
            fprintf('  %-14s %10s %10s %11s %10.4g %10.4g %8s   %s\n', rm_c, '-', '-', '-', ...
                    rm_rr(1), rm_rr(2), '-', '(manca la riga di campagna)');
            continue
        end
        rm_a = rm_A.(rm_c)(1);  rm_b = rm_B.(rm_c)(1);
        rm_d = abs(rm_a - rm_b);
        if rm_r > 0, rm_rap = rm_d / rm_r; else, rm_rap = Inf; end
        if rm_rap >= 3, rm_ver = 'DISCRIMINA'; else, rm_ver = 'non concludente'; end
        if isinf(rm_rap), rm_ver = 'rumore nullo: guardare a parte'; end
        fprintf('  %-14s %10.4g %10.4g %11.4g %10.4g %10.4g %8.1f   %s\n', ...
                rm_c, rm_a, rm_b, rm_d, rm_rr(1), rm_rr(2), rm_rap, rm_ver);
    end
end
end   % rm_coppie

fprintf('\n  scritto  %s   (%.0f s)\n', rm_out_f, toc(rm_crono));
fprintf(['  "non concludente" non vuol dire che i numeri sono sbagliati: vuol\n' ...
         '  dire che quella differenza non si puo'' attribuire al controllore.\n\n']);

fprintf('\n\n===================== FINITO =====================\n');
fprintf('  %d run in %.0f minuti\n', height(RM), toc(rm_crono)/60);
fprintf('  scritto  %s\n', rm_out_f);
fprintf('  Nessun CSV di campagna toccato, modello non salvato.\n');
fprintf('==================================================\n\n');

%% ================= helper =================
function r = rm_taglia(r, x_bordo)
%RM_TAGLIA  Tronca la run al bordo del pavimento, come script_T4D.
N = numel(r.t);
k = find(any(r.pf(:, 1:3:18) > x_bordo, 2), 1, 'first');
if isempty(k), return; end
f = fieldnames(r);
for i = 1:numel(f)
    v = r.(f{i});
    if (isnumeric(v) || islogical(v)) && size(v,1) == N && N > 1
        r.(f{i}) = v(1:k-1, :);
    end
end
end

function T = rm_campagna(task, ctrl, cella)
%RM_CAMPAGNA  La riga della campagna con cui confrontarsi.
T = [];
f = fullfile('results', sprintf('%s_%s.csv', task, ctrl));
if ~isfile(f), return; end
T = readtable(f);
if ~isempty(cella) && ismember('condizione', T.Properties.VariableNames)
    sel = strcmp(string(T.condizione), cella);
    if any(sel), T = T(sel, :); end
end
end
