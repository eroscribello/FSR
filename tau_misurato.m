%% tau_misurato.m - le coppie statiche LETTE dal modello che funziona
%
% PERCHE' ESISTE
%   [24/9] S0 non passa, in tre modi diversi: a gradino il robot schizza in
%   alto, a rampa cade, con lo smorzamento schizza lo stesso. Tre tentativi
%   sullo STESSO ingresso, cioe' il vettore analitico di tau_statico. Finche'
%   quel vettore non e' verificato, ogni altra modifica e' un tentativo alla
%   cieca: e' esattamente l'errore che con applica_imbardata era costato tre
%   giorni, e la regola del progetto e' "non si ragiona, si misura".
%
%   Il modello base ha gia' tutto quello che serve per misurarlo:
%     - i 18 giunti sono in ComputedTorque, cioe' Simscape CALCOLA da sola
%       la coppia che serve per tenere la posa comandata;
%     - quella coppia e' gia' sensorizzata e loggata (torque_sens): e' la
%       "tau_misurata" del flag di contatto di C2;
%     - FORZA_STATICO azzera gait.S e gait.H, cioe' l'IK comanda esattamente
%       la posa nominale u = [0, 0, z0] - la stessa su cui tau_statico
%       calcola il suo Jacobiano.
%   Quindi la verita' e' gia' nel repo: basta leggerla.
%
% COSA FA
%   1. Simula il modello BASE fermo in posa nominale, terreno T1, inerzie
%      corrette. Solo lettura: nessun save_system, il .slx non cambia.
%   2. Media torque_sens sull'ultimo mezzo secondo -> 18 coppie in ordine CAN.
%   3. Le riporta nell'ordine del Mux di INGRESSO (cfg.jointIN), che e'
%      l'ordine in cui il modello vuole le coppie. Sono DUE convenzioni
%      diverse nello stesso modello (legNamesMUX vs legNamesOUT): e' gia'
%      documentato in phantomx_config, e qui e' il punto in cui conta.
%   4. Confronta con tau_statico, giunto per giunto: segno, rapporto, scarto.
%   5. Scrive results/diagnostica/tau_statico_misurato.csv, che prova_S0
%      usa come ingresso al posto del vettore analitico.
%
% COME SI LEGGE IL CONFRONTO
%   - rapporto ~ +1 su tutti e 18: tau_statico e' giusto, e il problema di S0
%     e' altrove (dinamica, non mappa).
%   - rapporto ~ -1 su tutti: segno globale invertito.
%   - rapporto ~ -1 solo su tre zampe: e' il mirroring destra/sinistra.
%   - valori giusti ma nelle caselle sbagliate: e' l'ordine dei giunti.
%   Sono quattro diagnosi diverse che nessuna animazione avrebbe distinto.
%
% ATTENZIONE
%   Queste sono le coppie che tengono il robot in posa nominale SUL PIANO,
%   contatto gia' carico. Sono l'ingresso giusto per S0 e per niente altro.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   tau_misurato
%
% Progetto FSR PhantomX - A. Russo

function [tau_mux, info] = tau_misurato(durata)

% [24/9] 2 s non bastavano: l'oscillazione residua arrivava a 0.45 N*m su una
% media di 0.28. Con 6 s il confronto fra due finestre diverse (sotto) dice se
% i numeri sono a regime davvero, invece di limitarsi ad avvisare.
if nargin < 1 || isempty(durata), durata = 6.0; end

tm_mdl = 'phantomx_sim_zero';
cfg    = phantomx_config();

fprintf('\n=========== COPPIE STATICHE MISURATE ===========\n');
fprintf('  modello %s, fermo in posa nominale, %.1f s\n', tm_mdl, durata);

%% ---- la run: posa ferma, C1, piano liscio ----
tm_ripristina = onCleanup(@() evalin('base', ...
    'clear FORZA_STATICO OVERRIDE_C2'));
assignin('base', 'FORZA_STATICO', true);
assignin('base', 'OVERRIDE_C2',   false);    % C1: nessuna logica di contatto

if ~bdIsLoaded(tm_mdl), load_system(tm_mdl); end
applica_terreno('T1', false, tm_mdl);
applica_inerzie(tm_mdl);

tm_out = sim(tm_mdl, 'StopTime', num2str(durata));
r = adatta_simscape(tm_out, struct('controller','C1', 'task','S0', 'run',1, ...
                                   'condizione','statico', 'vel_d',[0 0]));

if ~isfield(r,'tau') || isempty(r.tau)
    error('tau_misurato:tau', ...
          ['torque_sens non e'' arrivato nel log: senza quello non c''e''\n' ...
           'niente da misurare. Controlla abilita_log / verifica_log.']);
end

%% ---- media a regime, e la verifica che sia davvero a regime ----
sel  = r.t >= r.t(end) - 0.5;
sel2 = r.t >= r.t(end) - 1.5;
tau_can  = mean(r.tau(sel,:),  1);          % 1x18, ordine CAN
tau_can2 = mean(r.tau(sel2,:), 1);
osc      = max(r.tau(sel,:), [], 1) - min(r.tau(sel,:), [], 1);

% Il criterio NON e' l'ampiezza dell'oscillazione: un ripple simmetrico non
% sposta la media. Il criterio e' che la media non cambi cambiando finestra -
% se ultimo mezzo secondo e ultimo secondo e mezzo danno lo stesso numero,
% quel numero e' il valore a regime anche con del ripple sopra.
deriva = max(abs(tau_can - tau_can2));
fprintf('\n  ripple max (ultimo 0.5 s)            : %.4f N*m\n', max(osc));
fprintf('  media su 0.5 s vs 1.5 s, scarto max  : %.4f N*m\n', deriva);
if deriva > 0.005
    fprintf(2, '  La media dipende dalla finestra: NON e'' a regime, i numeri\n');
    fprintf(2, '  sotto non sono utilizzabili. Allunga la durata.\n');
else
    fprintf('  Le due finestre coincidono: il valor medio e'' quello a regime.\n');
end
if max(osc) > 0.02
    gg = {'coxa','femore','tibia'};
    [~, kw] = maxk(osc, 3);
    fprintf('  ripple concentrato su:');
    for kk = kw(:).'
        fprintf('  %s/%s %.3f', cfg.legNamesCAN{floor((kk-1)/3)+1}, gg{mod(kk-1,3)+1}, osc(kk));
    end
    fprintf('\n');
end

%% ---- da ordine CAN a ordine del Mux di INGRESSO ----
tau_mux = zeros(18,1);
for i = 1:6
    for j = 1:3
        tau_mux(cfg.jointIN(i,j)) = tau_can(3*(i-1) + j);
    end
end

%% ---- il confronto ----
[tau_an, info_an] = tau_statico(false);     % analitico, ordine Mux
nomi   = cfg.legNamesCAN;
gnomi  = {'coxa','femore','tibia'};

fprintf('\n  %-5s %-7s %11s %11s %9s %11s\n', ...
        'zampa', 'giunto', 'misurato', 'analitico', 'rapporto', 'scarto');
for i = 1:6
    for j = 1:3
        m = tau_can(3*(i-1) + j);
        a = tau_an(cfg.jointIN(i,j));
        if abs(m) > 1e-6, rap = a/m; else, rap = NaN; end
        fprintf('  %-5s %-7s %11.4f %11.4f %9.3f %11.4f\n', ...
                nomi{i}, gnomi{j}, m, a, rap, a - m);
    end
end

scarto = max(abs(tau_an(:) - tau_mux(:)));
fprintf('\n  scarto massimo analitico - misurato : %.4f N*m\n', scarto);
fprintf('  norma misurato / norma analitico    : %.3f\n', norm(tau_mux)/norm(tau_an));
fprintf('  coppia massima misurata             : %.4f N*m   (datasheet %.2f)\n', ...
        max(abs(tau_mux)), cfg.tau_max);

if scarto < 0.01
    fprintf('\n  I due vettori coincidono: tau_statico e'' giusto e il problema di S0\n');
    fprintf('  NON e'' la mappa forze-coppie. Va cercato nella dinamica.\n');
else
    fprintf(2, '\n  I due vettori NON coincidono. Prima di toccare altro, guarda la\n');
    fprintf(2, '  colonna rapporto con la legenda in testa al file: dice quale dei\n');
    fprintf(2, '  quattro errori e'' (segno globale, mirroring, ordine, scala).\n');
end

%% ---- che forza al piede spiegherebbe le coppie misurate ----
% [24/9] Se il rapporto fosse lo stesso su tutti i giunti si tratterebbe di
% una forza sbagliata in MODULO. Non lo e' (femore 0.79, tibia 0.42), quindi
% la forza vera al piede non e' verticale. Con lo Jacobiano analitico si puo'
% invertire la mappa e chiedersi: quale f rende conto delle coppie misurate?
%   f = -(J')^-1 * tau
% Se ne esce f_z vicino a m*g/6 con una componente ORIZZONTALE non nulla, lo
% Jacobiano e' giusto e le zampe si stanno semplicemente spingendo l'una
% contro l'altra: con sei piedi a terra e i giunti comandati in POSIZIONE il
% sistema e' iperstatico (18 vincoli per 6 gradi di liberta' del corpo), e
% le forze interne sono la conseguenza normale, non un errore.
fprintf('\n  Forza al piede che spiega le coppie misurate (frame corpo):\n');
fprintf('  %-5s %9s %9s %9s %11s\n', 'zampa', 'fx [N]', 'fy [N]', 'fz [N]', '|f_oriz|');
fz_att = cfg.mass * cfg.g / 6;
for i = 1:6
    tau_i = tau_can(3*(i-1) + (1:3)).';
    f_i   = -(info_an(i).J.') \ tau_i;
    fprintf('  %-5s %9.3f %9.3f %9.3f %11.3f\n', nomi{i}, f_i(1), f_i(2), f_i(3), norm(f_i(1:2)));
end
fprintf('  atteso da solo peso: fz = %.3f N, orizzontale = 0\n', fz_att);

%% ---- il file, che poi prova_S0 legge ----
tm_dir = fullfile('results','diagnostica');
if ~isfolder(tm_dir), mkdir(tm_dir); end
tm_csv = fullfile(tm_dir, 'tau_statico_misurato.csv');

T = table((1:18).', tau_mux, tau_an(:), tau_an(:) - tau_mux, ...
          'VariableNames', {'slot_mux','misurato','analitico','scarto'});
writetable(T, tm_csv);
fprintf('\n  scritto %s\n\n', tm_csv);

info = struct('tau_can', tau_can, 'tau_mux', tau_mux, 'tau_analitico', tau_an(:), ...
              'scarto', scarto, 'oscillazione', max(osc), 'csv', tm_csv, 'run', r);
end
