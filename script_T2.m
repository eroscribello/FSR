%% script_T2.m - T2: piano, rettilineo, curva di velocita'  [impianto SIMSCAPE]
%
% DEFINIZIONE CANONICA DEL TASK
%   I fattori di velocita' stanno in cfg.t2_fattori, non qui: li legge anche
%   esegui_misure per l'impianto ridotto (C3). Finche' erano scritti due volte
%   - tre fattori la' variando v, cinque qui variando T - i due CSV non erano
%   confrontabili e la curva non si poteva disegnare.
%
%   La variabile indipendente e' la VELOCITA' COMANDATA. Il periodo si ricava
%   da  T = S / (beta_stance * v), quindi la lunghezza del passo resta quella
%   validata e la differenza fra le celle e' attribuibile alla velocita' e non
%   a un diverso punto di lavoro della gamba.
%
%   L'etichetta della cella ha lo STESSO formato dei due impianti: v0.50x.
%
% ATTENZIONE AI NOMI: init_gait e' uno script, gira in questo workspace e
% sovrascrive k, j, L, cfg, alpha, side, offset, pen. Tutte le variabili di
% questo ciclo sono prefissate t2_ per non finirci sotto.

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
t2_mdl    = 'phantomx_sim_zero';
t2_fatt   = t2_cfg.t2_fattori;
t2_nCicli = 10;
t2_c2     = false;          % false = C1, anello aperto. true = C2.
T2 = table();
t2_runs  = cell(1, numel(t2_fatt));
t2_etichette = cell(1, numel(t2_fatt));

% Il terreno si fissa qui: ereditarlo dalla chiamata precedente ha gia'
% fatto girare una campagna T2 sul gradino di T5.
applica_terreno('T2', false, t2_mdl);

t2_haTar = isfield(t2_cfg,'t2_taratura') && ~isempty(t2_cfg.t2_taratura);
if t2_haTar
    fprintf('\nT2: taratura per velocita'' da cfg.t2_taratura\n');
else
    fprintf('\nT2: taratura UNICA (cfg.z0, cfg.H) - lancia taratura_T2 per ritarare\n');
end

for t2_i = 1:numel(t2_fatt)

    t2_vnom  = t2_fatt(t2_i) * t2_cfg.v_nom;                   % <-- il comando
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

    OVERRIDE_C2   = t2_c2;                                     %#ok<NASGU>
    OVERRIDE_GAIT = t2_ov;                                     %#ok<NASGU>
    init_gait                                                  % <-- da qui k, j, L sono persi

    t2_out = sim(t2_mdl, 'StopTime', num2str(t2_stop));

    t2_run = adatta_simscape(t2_out, struct( ...
                 'controller', t2_nome_ctrl(t2_c2), 'task','T2', 'run',1, ...
                 'condizione', t2_etich, ...
                 'vel_d', [t2_vnom 0]));

    t2_cfgLoc   = t2_cfg;
    t2_cfgLoc.T = t2_T;                     % serve a slip_per_passo
    t2_riga = metriche(t2_run, t2_cfgLoc, struct('t_regime', 2*t2_T));

    % la taratura usata va in tabella: senza, la riga non e' riproducibile
    if isfield(t2_ov,'z0'), t2_riga.z0 = t2_ov.z0; else, t2_riga.z0 = t2_cfg.z0; end
    if isfield(t2_ov,'H'),  t2_riga.H  = t2_ov.H;  else, t2_riga.H  = t2_cfg.H;  end
    t2_riga.T_gait  = t2_T;
    t2_riga.fattore = t2_fatt(t2_i);

    T2 = [T2; t2_riga];                                        %#ok<AGROW>
    t2_runs{t2_i} = t2_run;

    fprintf('\n=== %s : v_nom %.4f | T %.2f s | z0 %.4f | H %.3f | misurata %.4f | piedi %.2f ===\n\n', ...
            t2_etich, t2_vnom, t2_T, t2_riga.z0, t2_riga.H, ...
            t2_riga.vel_media, t2_riga.appoggio_medio);

end

clear OVERRIDE_GAIT OVERRIDE_C2
init_gait                                                      % ripristina i valori di cfg

writetable(T2, 'T2_C1.csv');
disp(T2(:, {'condizione','vel_media','frazione_task','successo', ...
            'causa_fallimento','cot','frazione_saturo'}))

fprintf('\n');
for t2_j = 1:numel(t2_runs)
    t2_vx  = t2_runs{t2_j}.v(:,1);
    t2_sel = t2_runs{t2_j}.t >= 0.2 * t2_runs{t2_j}.t(end);
    fprintf('%-8s  v media %.4f | min %+.4f | max %.4f | oscillazione %3.0f%%\n', ...
        t2_runs{t2_j}.meta.condizione, mean(t2_vx(t2_sel)), ...
        min(t2_vx(t2_sel)), max(t2_vx(t2_sel)), ...
        100*std(t2_vx(t2_sel))/mean(t2_vx(t2_sel)));
end

% La legenda va GENERATA dai fattori: scritta a mano elencava tre celle
% mentre il ciclo ne produceva cinque, e le curve risultavano attribuite
% alla velocita' sbagliata.
figure; hold on; grid on
for t2_j = 1:numel(t2_runs)
    plot(t2_runs{t2_j}.t / t2_runs{t2_j}.t(end), t2_runs{t2_j}.v(:,1));
end
legend(t2_etichette, 'Location','best');
xlabel('frazione della run'); ylabel('v_x [m/s]')
title(sprintf('T2 - curva di velocita'', %s su Simscape', t2_nome_ctrl(t2_c2)))

%% ================= helper =================
function s = t2_nome_ctrl(c2)
if c2, s = 'C2'; else, s = 'C1'; end
end
