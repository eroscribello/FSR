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
% [29/9] Il controllore si sceglie per NOME. Modello e interruttore
% OVERRIDE_C2 li da' scegli_controllore, che e' l'unico posto dove sta
% scritto chi gira su cosa. Prima erano un booleano: con tre controllori
% quel booleano non sbagliava il calcolo, sbagliava il NOME DEL FILE, e
% una run di C3 sovrascriveva results/T*_C2.csv senza un errore.
t2_ctrl    = 'C2';      % 'C1' | 'C2' | 'C3'
% [29/9] Scavalcabile dal workspace, per lanciare piu' task di fila senza
% aprire i file:
%     OVERRIDE_CTRL = 'C3'; script_T5; script_T6; clear OVERRIDE_CTRL
% NON viene cancellata dallo script: se lo facesse andrebbe riscritta prima
% di ogni task, che e' il problema che risolve. In cambio ogni run che la
% usa lo dichiara a schermo, perche' lo stato residuo deve vedersi - una
% OVERRIDE dimenticata nel workspace ci e' gia' costata una campagna.
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

% Il terreno si fissa qui: ereditarlo dalla chiamata precedente ha gia'
% fatto girare una campagna T2 sul gradino di T5.
applica_terreno('T2', false, t2_mdl);
% [23/9] Inerzie corrette in memoria (il .slx non viene salvato): quelle nel
% file vengono dall'URDF e sono ~1000 volte troppo grandi. Vedi applica_inerzie
% e docs/piano_confronto.md sezione 9. Se il collega correggera' il .slx questa
% riga diventa inutile e si toglie.
applica_inerzie(t2_mdl);
% [25/9] E subito dopo lo stimatore, SEMPRE. Il blocco Inverse Dynamics che
% produce tau_attesa per il flag di contatto di C2 usa un rigidBodyTree
% importato dall'URDF: correggere le inerzie del robot e lasciare a lui
% quelle vecchie rende |tau_mis - tau_att| grande ovunque, il flag resta
% incollato a 1 e C2 smette di cercare il terreno. E' successo dal 22/9 al
% 25/9. Vedi allinea_stimatore e docs/piano_confronto.md.
allinea_stimatore(t2_mdl);
% [1/10] Il modello di C3 porta un carico, attivo su disco: va tolto, o si
% misura C3 carico contro C1 e C2 scarichi. Su C1 e C2 non fa niente.
if strcmp(t2_mdl, 'phantomx_sim_attitude')   % [2/10] il pacco resta solo per C3P
    if t2_info.carico, commenta_carico(t2_mdl, 'off'); else, commenta_carico(t2_mdl); end
end

t2_haTar = isfield(t2_cfg,'t2_taratura') && ~isempty(t2_cfg.t2_taratura);
if t2_haTar
    fprintf('\nT2: taratura per velocita'' da cfg.t2_taratura\n');
else
    fprintf('\nT2: taratura UNICA (cfg.z0, cfg.H)\n');
    % [2/10] Tolto il rimando a taratura_T2: e' in archivio/ e NON chiama
    % applica_inerzie, quindi rilanciarla oggi tarerebbe su un robot diverso.
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
                 'controller', t2_ctrl, 'task','T2', 'run',1, ...
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

% IL NOME DEL FILE DEVE CONTENERE IL CONTROLLORE.
% Era cablato a 'T2_C1.csv': lanciare lo script sul controllore C2 sovrascriveva
% la campagna C1 con dati C2, senza dire niente e senza modo di accorgersene
% dopo. Un CSV con l'etichetta sbagliata nel nome e' peggio di nessun CSV.
% I CSV di campagna vanno in results/, non nella radice: sono il risultato e
% stanno insieme a quelli di esegui_misure e taratura_T2, che ci scrivevano
% gia'. Da quando .gitignore ignora results/ per ESTENSIONE e non per
% cartella, i .csv la' dentro sono tracciati e i .mat no.
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

% La legenda va GENERATA dai fattori: scritta a mano elencava tre celle
% mentre il ciclo ne produceva cinque, e le curve risultavano attribuite
% alla velocita' sbagliata.
figure; hold on; grid on
for t2_j = 1:numel(t2_runs)
    plot(t2_runs{t2_j}.t / t2_runs{t2_j}.t(end) * t2_nCicli, t2_runs{t2_j}.v(:,1));
end
legend(t2_etichette, 'Location','best');
xlabel('cicli di andatura'); ylabel('v_x [m/s]')
title(sprintf('T2 - curva di velocita'', %s su Simscape', t2_ctrl))
salva_grafico(sprintf('T2_%s', t2_ctrl));   % grafici/<tag>.fig, testi modificabili dopo

