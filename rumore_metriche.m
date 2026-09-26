%% rumore_metriche.m - quali colonne discriminano davvero?   [PRIMA DI SCRIVERE]
%
% LA DOMANDA
%   Una differenza fra C1 e C2 e' un risultato solo se e' piu' grande di
%   quanto le stesse metriche oscillano da sole. Questo script misura quel
%   "pavimento di rumore" e lo confronta con la differenza C1-C2 misurata in
%   campagna.
%
% COME
%   Si tiene tutto fermo e si cambia una cosa IRRILEVANTE: gait.z0 di
%   +-0.2 mm, cioe' un quarto della penetrazione statica del contatto e un
%   decimo di cfg.c2.tol. Il simulatore e' deterministico (seed NaN), quindi
%   ripetere la stessa run non direbbe nulla: la perturbazione va messa
%   apposta, ed e' l'unico modo di stimare una dispersione senza statistica.
%
% IL CRITERIO, SCRITTO PRIMA DI LANCIARE
%   rapporto = |valore C1 - valore C2| in campagna / escursione fra le tre
%              perturbazioni (la peggiore fra i due controllori)
%     rapporto >= 3   la colonna DISCRIMINA: la differenza fra i controllori
%                     e' reale e si puo' scrivere in relazione
%     rapporto <  3   NON SI PUO' CONCLUDERE nulla da quella colonna su quel
%                     task: va tolta dalle conclusioni, non solo da qui
%   Il 3 e' arbitrario, ma e' fissato prima di vedere i numeri.
%
% [MISURATO 23/9] SU DOSSO E OSTACOLO
%   results/diagnostica/rumore_metriche.csv. Su T4D e T5 nessuna colonna di
%   picco sopravvive: 0.2 mm bastano a cambiare quale piede tocca per primo
%   uno spigolo, e con l'ordine degli urti cambiano tau_max, roll_max, la
%   deriva e l'imbardata. Sopravvivono z_media (5-6x) e, in T4D, cot (3.7x) e
%   pitch_max (3.3x). Non e' un difetto del simulatore: un robot vero fa lo
%   stesso, ed e' il motivo per cui gli esperimenti si ripetono.
%
% ADESSO TOCCA AL PIANO
%   In piano non ci sono spigoli, quindi il rumore dovrebbe essere molto
%   minore - ma "dovrebbe" non e' una misura, e in T2 c'e' la conclusione
%   portante della campagna (C2 costa il 16% in piu' di C1 a 1x). Se il
%   rumore sul cot in piano vale piu' del 5%, quella frase non si puo'
%   scrivere come sta.
%
% COSA NON TOCCA
%   Nessun CSV di campagna, nessuna figura, nessun save_system. Scrive
%   results/diagnostica/rumore_<tag>.csv, aggiornandolo DOPO OGNI RUN: se
%   interrompi con Ctrl+C non perdi quelle gia' fatte.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   rumore_metriche
%
% Progetto FSR PhantomX - A. Russo

rm_cfg = phantomx_config();
rm_mdl = 'phantomx_sim_zero';
rm_dz  = [0, +0.2e-3, -0.2e-3];        % [m] perturbazione su gait.z0

% {task, controllore, durata [s], cella della campagna da confrontare}
%
% [25/9] RIMISURA DOPO L'ALLINEAMENTO DELLO STIMATORE
%   Il pavimento misurato il 23/9 (results/diagnostica/rumore_metriche.csv) e'
%   stato preso su un C2 che NON cercava il terreno: il flag di contatto era
%   incollato a 1 e il comando degenerava in una costante (vedi
%   allinea_stimatore e docs/piano_confronto.md sezione 11). La sensibilita'
%   di quel controllore non dice niente su quella di questo, quindi quel file
%   non e' un metro valido per le righe nuove - ne' per dichiarare le
%   differenze, ne' per escluderle.
%
%   PREVISIONE, SCRITTA PRIMA DI LANCIARE: il C2 riparato dovrebbe essere
%   MENO sensibile a z0 di quello rotto, perche' assorbire un errore di quota
%   di 0.2 mm e' esattamente il mestiere della ricerca del terreno. Se invece
%   risulta piu' sensibile, e' una cattiva notizia su C2 e va detta.
%
%   Il file vecchio NON viene sovrascritto: resta come documento dello stato
%   precedente. Questa rimisura scrive rumore_ostacoli.csv.
%
%   La configurazione precedente, sul piano, era:
%     rm_tag = 'T2';  rm_prove = {'T2','C1',10*rm_cfg.T,'v1.00x'
%                                 'T2','C2',10*rm_cfg.T,'v1.00x'};
%   [26/9] SECONDO GIRO: piano e curva. Su T5 e T4D il rumore e' gia'
%   misurato (rumore_ostacoli.csv, 12 run). T2 e T3 non hanno un metro, e
%   senza metro i loro numeri non si possono dichiarare ne' escludere.
rm_tag   = 'piano';
rm_prove = {'T2', 'C1', 10*rm_cfg.T, 'v1.00x'
            'T2', 'C2', 10*rm_cfg.T, 'v1.00x'
            'T3', 'C1', 15,          'yaw+0.100'
            'T3', 'C2', 15,          'yaw+0.100'};

rm_xbordo = 4 - 0.05;                  % serve solo a T4D
rm_out_f  = fullfile('results','diagnostica', sprintf('rumore_%s.csv', rm_tag));

% [CORRETTO 25/9] Era un error. Ma applica_terreno, applica_inerzie e
% allinea_stimatore lavorano con set_param in memoria: dopo la prima chiamata
% il modello E' dirty per costruzione, e la guardia fermava sul nascere una
% sessione lanciata e lasciata sola. Lo stato viene comunque reimpostato qui
% sotto in modo esplicito, e questo script non salva mai il .slx.
if bdIsLoaded(rm_mdl) && strcmp(get_param(rm_mdl,'Dirty'),'on')
    fprintf(2, ['\n  Il modello ha modifiche in memoria (normale). Terreno,\n' ...
                '  inerzie e stimatore vengono reimpostati. Nessun salvataggio.\n']);
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

        if ~strcmp(rm_terreno_ora, rm_t)
            applica_imbardata(0);        % si esce sempre puliti dal task precedente
            applica_terreno(rm_t, false, rm_mdl);
            applica_inerzie(rm_mdl);
            allinea_stimatore(rm_mdl);   % [25/9] sempre insieme: vedi sezione 11
            rm_terreno_ora = rm_t;
        end

        % [26/9] T3 NON E' SOLO UN TERRENO. La curva si realizza ruotando la
        % direzione del passo di ciascuna zampa (applica_imbardata), e il
        % passo S cambia di conseguenza. Senza queste due righe le run
        % "T3" sarebbero rettilinei su terreno piano, cioe' un altro task -
        % e il rumore misurato non sarebbe quello di T3.
        rm_yaw  = 0;
        rm_S    = [];
        if strcmp(rm_t, 'T3')
            rm_yaw  = rm_cfg.yaw_d;
            rm_info = applica_imbardata(rm_yaw);
            rm_S    = rm_info.S;
        end

        for rm_j = 1:numel(rm_dz)
            fprintf('\n---- %s, %s, z0 %+.1f mm ----\n', rm_t, rm_ctrl, 1e3*rm_dz(rm_j));

            OVERRIDE_C2   = strcmp(rm_ctrl,'C2');                  %#ok<NASGU>
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
            writetable(RM, rm_out_f);       % dopo ogni run: Ctrl+C non cancella nulla
        end
    end
    applica_imbardata(0);
    clear OVERRIDE_C2 OVERRIDE_GAIT
    init_gait
catch rm_err
    try, applica_imbardata(0); end %#ok<TRYNC>
    clear OVERRIDE_C2 OVERRIDE_GAIT
    fprintf(2,'\n  Errore: modello NON salvato. Le run gia'' fatte sono in %s\n', rm_out_f);
    rethrow(rm_err);
end

%% ---- lettura: il rumore contro la differenza fra i controllori ----
rm_col = {'roll_max','pitch_max','dev_lat_max','yaw_err_fin','cot','energia', ...
          'tau_max','tau_rms','z_media','vel_media','frazione_task'};

fprintf('\n\n======= PAVIMENTO DI RUMORE (z0 +-0.2 mm) =======\n');
for rm_t = unique(string(RM.task))'
    rm_T = RM(strcmp(string(RM.task), rm_t), :);
    rm_cella = rm_prove{find(strcmp(rm_prove(:,1), rm_t), 1), 4};

    rm_C1 = rm_campagna(rm_t, 'C1', rm_cella);
    rm_C2 = rm_campagna(rm_t, 'C2', rm_cella);

    fprintf('\n--- %s, cella %s ---\n', rm_t, rm_cella);
    fprintf('  %-14s %10s %10s %11s %11s %8s   %s\n', ...
            '', 'C1', 'C2', '|C1-C2|', 'rumore', 'rapp.', 'verdetto');

    for rm_k = 1:numel(rm_col)
        rm_c = rm_col{rm_k};
        if ~ismember(rm_c, rm_T.Properties.VariableNames), continue; end

        rm_r = 0;                          % rumore = il peggiore fra i controllori
        for rm_cc = {'C1','C2'}
            rm_sel = strcmp(string(rm_T.controller), rm_cc{1});
            if any(rm_sel)
                rm_v = rm_T.(rm_c)(rm_sel);
                rm_r = max(rm_r, max(rm_v) - min(rm_v));
            end
        end

        if isempty(rm_C1) || isempty(rm_C2) || ~ismember(rm_c, rm_C1.Properties.VariableNames)
            fprintf('  %-14s %10s %10s %11s %11.4g %8s   %s\n', rm_c, '-', '-', '-', rm_r, '-', ...
                    '(manca la riga di campagna)');
            continue
        end
        rm_a = rm_C1.(rm_c)(1);  rm_b = rm_C2.(rm_c)(1);
        rm_d = abs(rm_a - rm_b);
        if rm_r > 0, rm_rap = rm_d / rm_r; else, rm_rap = Inf; end
        if rm_rap >= 3, rm_ver = 'DISCRIMINA'; else, rm_ver = 'non concludente'; end
        fprintf('  %-14s %10.4g %10.4g %11.4g %11.4g %8.1f   %s\n', ...
                rm_c, rm_a, rm_b, rm_d, rm_r, rm_rap, rm_ver);
    end
end

fprintf('\n  scritto  %s   (%.0f s)\n', rm_out_f, toc(rm_crono));
fprintf(['  "non concludente" non vuol dire che i numeri sono sbagliati: vuol\n' ...
         '  dire che quella differenza non si puo'' attribuire al controllore.\n\n']);

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

fprintf('\n\n===================== FINITO =====================\n');
fprintf('  %d run in %.0f minuti\n', height(RM), toc(rm_crono)/60);
fprintf('  scritto  %s\n', rm_out_f);
fprintf('  Nessun CSV di campagna toccato, modello non salvato.\n');
fprintf('  Le righe di campagna T2 e T3 sono gia'' quelle con lo stimatore\n');
fprintf('  allineato, quindi i rapporti qui sopra sono leggibili subito.\n');
fprintf('==================================================\n\n');
