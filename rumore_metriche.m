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
rm_dz  = [0, +0.2e-3, -0.2e-3];        % [m] perturbazione su gait.z0
% [1/10] Il modello non e' piu' fisso: lo da' scegli_controllore, run per run.

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
%
%   La configurazione del secondo giro era:
%     rm_tag   = 'piano';
%     rm_prove = {'T2','C1',10*rm_cfg.T,'v1.00x';  'T2','C2',10*rm_cfg.T,'v1.00x'
%                 'T3','C1',15,'yaw+0.100';        'T3','C2',15,'yaw+0.100'};
%     rm_coppia = {'C1','C2'};
%
% [1/10] TERZO GIRO: C3, SUGLI STESSI QUATTRO TASK
%   LA DOMANDA. Le righe di C3 ci sono su tutti e sette i task (campagna
%   dell'1/10), ma senza un pavimento di rumore misurato SU C3 nessuna
%   differenza C2-C3 si puo' dichiarare ne' escludere. La regola e' quella
%   di sempre: il rumore si misura sullo stesso controllore - usare quello di
%   C2 per C3 e' l'errore che il 26/9 ci ha fatto credere che non fosse
%   dichiarabile niente.
%
%   STESSO PROTOCOLLO DEI DUE GIRI PRECEDENTI, per poter confrontare i
%   pavimenti: z0 di 0 e +-0.2 mm, stesse durate (T2 10 s, T3 15 s, T4D 30 s
%   tagliata al bordo, T5 20 s), stesse celle. 12 run.
%
%   LA SOGLIA. Invariata: rapporto = |C2 - C3| in campagna / rumore >= 3,
%   dove il rumore e' il PEGGIORE fra C2 (rumore_piano.csv e
%   rumore_ostacoli.csv, gia' misurato) e C3 (questo giro). Si prende il
%   peggiore per la stessa ragione di prima: una differenza vale solo se
%   regge l'oscillazione di tutti e due.
%
%   PREVISIONE, SCRITTA PRIMA DI LANCIARE: C3 e' C2 piu' un anello chiuso
%   sull'assetto, quindi su roll_max e pitch_max dovrebbe essere MENO
%   sensibile di C2: la retroazione esiste per respingere proprio i
%   disturbi. Su tau_max, dev_lat_max e yaw_err_fin no: li decide l'ordine
%   degli urti sugli spigoli, e l'anello d'assetto non lo cambia.
%
%   CONCLUSIONI POSSIBILI
%     - rumore di C3 dello stesso ordine di C2: il pavimento combinato e'
%       quello di C2, e le differenze C2-C3 si leggono col criterio solito;
%     - rumore di C3 MAGGIORE su assetto: l'anello amplifica una
%       perturbazione di 0.2 mm invece di respingerla. Va detto, ed e' un
%       risultato su C3, non un difetto della misura;
%     - una colonna con rumore nullo (rapporto Inf): non e' una differenza
%       infinitamente certa, e' una colonna che il protocollo non perturba.
%       Va guardata a parte.
%
%   COSA CAMBIA NEL CODICE, perche' com'era non poteva misurare C3:
%     - il modello non e' piu' fisso: si prende da scegli_controllore,
%       insieme a OVERRIDE_C2. Prima era strcmp(rm_ctrl,'C2'), che per C3
%       dava false: avrebbe misurato C3 SENZA ricerca del terreno, cioe' un
%       altro controllore;
%     - applica_imbardata riceve il modello (lo stesso difetto corretto l'1/10
%       in script_T3: senza, su C3 la curva non c'e');
%     - sul modello di C3 si toglie il carico (commenta_carico);
%     - la lettura confronta la coppia rm_coppia invece di C1-C2 fisso, e
%       prende il rumore dei controllori che in questo giro non girano dai
%       file dei giri precedenti.
%
%   La configurazione del terzo giro era:
%     rm_tag = 'C3';  rm_prove = {T2, T3, T4D, T5} x {'C3'};
%     rm_coppia = {'C2','C3'};  rumore di C2 da rumore_piano e rumore_ostacoli.
%   Quei tre file sono in results/storico/pre_slew_20261001/diagnostica.
%
% [1/10, sera] QUARTO GIRO: TUTTI E TRE, DOPO LO SLEW RATE
%   COSA E' CAMBIATO. Un Rate Limiter di +-0.4 m/s per piede e' stato
%   salvato in phantomx_sim_zero (C1 e C2: prima non c'era) e portato da
%   +-0.3 a +-0.4 in phantomx_sim_attitude (C3). E' un impianto diverso per
%   tutti e tre: nessuno dei pavimenti misurati prima vale piu', nemmeno
%   quello di C1. Si rimisura tutto, 4 task x 3 controllori x 3 run = 36.
%
%   LA DOMANDA. Quanto oscillano le metriche di ciascun controllore con il
%   limitatore, e quali differenze C1-C2 e C2-C3 reggono il >= 3x.
%   Adesso C2 -> C3 differisce per UN meccanismo (l'anello d'assetto; la
%   saturazione su C2 non interverrebbe, vedi CLAUDE.md), quindi i verdetti
%   C2-C3 si attribuiscono all'anello.
%
%   STESSO PROTOCOLLO DEI GIRI PRECEDENTI: z0 0 e +-0.2 mm, stesse durate,
%   stesse celle, stessa soglia (3x sul peggiore dei due rumori).
%
%   PREVISIONE, SCRITTA PRIMA DI LANCIARE:
%     - C1: rumore quasi invariato. All'andatura nominale il limitatore non
%       interviene (picco del piede 0.375 m/s < 0.4), e C1 non ha altri
%       comandi veloci;
%     - C2: rumore sull'assetto MINORE di prima. La ricerca del terreno
%       produceva gradini di quota che il limitatore ora smussa: e' il
%       meccanismo con cui lo slew rate ha tolto la deriva su T6;
%     - C3: simile al terzo giro (aveva gia' un limitatore, a 0.3). Resta da
%       vedere se l'energia di C3 e' ancora 7-17x piu' sensibile di quella
%       di C2: se si', e' l'anello; se no, era l'interazione col limite a 0.3.
%
%   CONCLUSIONI POSSIBILI
%     - previsioni confermate: i pavimenti nuovi sostituiscono i vecchi;
%     - C2 PIU' sensibile di prima: il limitatore introduce un ritardo che
%       amplifica le perturbazioni invece di smussarle. Va detto;
%     - energia di C3 non piu' anomala: la sensibilita' del terzo giro era
%       del limite a 0.3, non dell'anello, e il punto 7.6 si chiude cosi'.
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

% [CORRETTO 25/9] Era un error. Ma applica_terreno, applica_inerzie e
% allinea_stimatore lavorano con set_param in memoria: dopo la prima chiamata
% il modello E' dirty per costruzione, e la guardia fermava sul nascere una
% sessione lanciata e lasciata sola. Lo stato viene comunque reimpostato qui
% sotto in modo esplicito, e questo script non salva mai il .slx.
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
        [rm_mdl, rm_c2, rm_info] = scegli_controllore(rm_ctrl);   % [1/10] modello e interruttore

        % [1/10] Il terreno si reimposta quando cambia il task OPPURE il
        % modello: con due modelli in gioco, "stesso task" non basta piu'.
        if ~strcmp(rm_terreno_ora, [rm_t '|' rm_mdl])
            applica_imbardata(0, [], rm_mdl);   % si esce sempre puliti dal task precedente
            applica_terreno(rm_t, false, rm_mdl);
            applica_inerzie(rm_mdl);
            allinea_stimatore(rm_mdl);   % [25/9] sempre insieme: vedi sezione 11
            if strcmp(rm_mdl, 'phantomx_sim_attitude')   % [2/10] il pacco resta solo per C3P
                if rm_info.carico, commenta_carico(rm_mdl, 'off'); else, commenta_carico(rm_mdl); end
            end
            rm_terreno_ora = [rm_t '|' rm_mdl];
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
            writetable(RM, rm_out_f);       % dopo ogni run: Ctrl+C non cancella nulla
        end
    end
    for rm_m = unique(cellfun(@scegli_controllore, rm_prove(:,2), 'UniformOutput', false))'
        applica_imbardata(0, [], rm_m{1});
    end
    clear OVERRIDE_C2 OVERRIDE_GAIT
    init_gait
catch rm_err
    for rm_m = unique(cellfun(@scegli_controllore, rm_prove(:,2), 'UniformOutput', false))'
        try, applica_imbardata(0, [], rm_m{1}); end %#ok<TRYNC>
    end
    clear OVERRIDE_C2 OVERRIDE_GAIT
    fprintf(2,'\n  Errore: modello NON salvato. Le run gia'' fatte sono in %s\n', rm_out_f);
    rethrow(rm_err);
end

%% ---- lettura: il rumore contro la differenza fra i controllori ----
rm_col = {'roll_max','pitch_max','dev_lat_max','yaw_err_fin','cot','energia', ...
          'tau_max','tau_rms','z_media','vel_media','frazione_task'};

% [1/10] La coppia non e' piu' C1-C2 fissa (rm_coppia, in testa). Il rumore
% dei controllori della coppia che NON girano in questo giro si prende dai
% pavimenti gia' misurati (rm_rumore_prec): stesso protocollo, stesse celle.
% [1/10, sera] Piu' coppie in un giro: rm_coppie, una riga per coppia. Un
% giro configurato con la sola rm_coppia (terzo giro) resta valido.
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

% [CORRETTO 1/10] Questo blocco stava DOPO le funzioni locali, dove in uno
% script MATLAB non puo' stare: lo script non partiva. Spostato qui.
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
