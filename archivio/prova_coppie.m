%% prova_coppie.m - le coppie comandate arrivano davvero dove pensiamo?
%
% PRIMA DI TUTTO: CORREZIONE A s0_vuoto
%   [24/9] La prova 1 di s0_vuoto (gravita' zero, coppia zero) NON era la
%   prova nulla che avevo dichiarato. Tolta la gravita' resta il PRECARICO
%   DEL CONTATTO: cfg.body_z0 parte gia' 2.1 mm dentro il pavimento, quindi
%   le molle di contatto spingono i piedi verso l'alto con ~2.5 N per zampa
%   e non c'e' piu' il peso a bilanciarle. Per questo le zampe si muovono e
%   si richiudono, come si vede nell'animazione. Colpa mia: "niente forze"
%   era falso.
%   Quello che la prova dimostra comunque, e non e' poco: le zampe hanno
%   attraversato mezza corsa fino a richiudersi, per 3 s interi, SENZA un
%   solo messaggio di massa degenere. Quindi la distribuzione di massa non e'
%   degenere ne' in posa nominale ne' in un ampio intorno. Il modello regge.
%
% LA DOMANDA CHE RESTA
%   Con coppia zero il robot cade e muore a 0.22 s: e' una caduta libera da
%   15 cm, che dura 0.17 s - tempi coerenti, nessun mistero.
%   Con la coppia statica MISURATA il robot va in ALTO e muore a 0.16 s. E
%   questo non torna: quelle coppie sono esattamente quelle che il modello
%   base usa per stare fermo. Se il robot sale, le coppie che ARRIVANO ai
%   giunti non sono quelle che abbiamo comandato.
%
%   Le cose che possono essersi rotte fra il Constant e i giunti sono tre, e
%   nessuna e' mai stata verificata:
%     - l'ORDINE: crea_modello_mpc ha cancellato il Mux dell'IK e ci ha messo
%       un Constant. Se il fan-out a valle non e' nell'ordine del Mux di
%       ingresso, le coppie finiscono sui giunti sbagliati;
%     - il SEGNO di singoli giunti;
%     - le UNITA': i convertitori sono passati da Unit = 1 a N*m. Se la
%       conversione non e' quella che crediamo, il modulo e' scalato.
%
% COME SI VERIFICA, SENZA IPOTESI
%   I giunti della copia sono in InputTorque, ma la sensorizzazione della
%   coppia (torque_sens) c'e' ancora, ed e' la stessa che il modello base usa
%   per il flag di contatto di C2. Quindi si puo' leggere la coppia che
%   ARRIVA a ogni giunto e confrontarla con quella COMANDATA.
%   La run dura 0.12 s: prima dei 0.16 s in cui il robot esplode, e abbastanza
%   da avere qualche decina di campioni. Non serve che la simulazione vada
%   bene - serve solo che i primi campioni siano leggibili.
%
% COME SI LEGGE
%   rapporto arrivato/comandato, giunto per giunto:
%     ~ +1 ovunque          -> il percorso e' giusto. Allora il problema e'
%                              davvero dinamico e S0 in anello aperto va
%                              ridefinito, non riparato.
%     ~ -1 ovunque          -> segno globale nel convertitore.
%     valori giusti ma in caselle sbagliate -> ORDINE: e' il fan-out dopo il
%                              Constant, e si sistema in crea_modello_mpc.
%     tutti scalati uguale  -> UNITA' del convertitore.
%     zeri                  -> le coppie non arrivano affatto.
%
% SOLO LETTURA sul modello base. Sulla copia scrive solo parametri in memoria.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   prova_coppie
%
% Progetto FSR PhantomX - A. Russo

function info = prova_coppie(durata)

if nargin < 1 || isempty(durata), durata = 0.12; end

pc_mdl = 'phantomx_sim_mpc';
cfg    = phantomx_config();

pc_csv = fullfile('results','diagnostica','tau_statico_misurato.csv');
if ~isfile(pc_csv)
    error('prova_coppie:csv', 'Manca %s: lancia prima tau_misurato.', pc_csv);
end
cmd_mux = readtable(pc_csv).misurato;          % ordine Mux di INGRESSO

fprintf('\n=========== COPPIE COMANDATE vs ARRIVATE ===========\n');
fprintf('  run di %.2f s sulla copia (esplode a ~0.16: qui non importa)\n', durata);

%% ---- il comando: costante, nessuna rampa, nessuno smorzamento ----
% Niente rampa e niente smorzamento apposta: qui non si cerca di far stare in
% piedi il robot, si guarda solo cosa arriva ai giunti. Ogni ingrediente in
% piu' e' una variabile in piu' da escludere dopo.
assignin('base', 'tau_ts', timeseries([cmd_mux.'; cmd_mux.'], [0; durata]));

if ~bdIsLoaded(pc_mdl), load_system(pc_mdl); end
applica_terreno('T1', false, pc_mdl);
applica_inerzie(pc_mdl);

pc_out = sim(pc_mdl, 'StopTime', num2str(durata));
r = adatta_simscape(pc_out, struct('controller','C3', 'task','S0', 'run',1, ...
                                   'condizione','coppie', 'vel_d',[0 0]));

if ~isfield(r,'tau') || isempty(r.tau)
    error('prova_coppie:tau', 'torque_sens assente: senza quello non si misura niente.');
end

%% ---- la coppia arrivata, mediata sui primi campioni utili ----
% Si scarta il primissimo istante (il convertitore e il solutore si assestano)
% e si media su una finestra corta, prima che il moto diventi grande.
sel = r.t >= 0.01 & r.t <= min(0.05, r.t(end));
if nnz(sel) < 3
    sel = r.t <= r.t(end);
    fprintf(2, '  Pochi campioni: la finestra e'' tutta la run (%.3f s).\n', r.t(end));
end
arr_can = mean(r.tau(sel,:), 1);               % ordine CAN

%% ---- il comando, portato in ordine CAN per poterli confrontare ----
cmd_can = zeros(1,18);
for i = 1:6
    for j = 1:3
        cmd_can(3*(i-1)+j) = cmd_mux(cfg.jointIN(i,j));
    end
end

%% ---- il confronto ----
gnomi = {'coxa','femore','tibia'};
fprintf('\n  %-5s %-7s %11s %11s %9s\n', 'zampa', 'giunto', 'comandato', 'arrivato', 'rapporto');
for i = 1:6
    for j = 1:3
        k = 3*(i-1)+j;
        c = cmd_can(k);  a = arr_can(k);
        if abs(c) > 1e-5, rap = a/c; else, rap = NaN; end
        fprintf('  %-5s %-7s %11.4f %11.4f %9.3f\n', cfg.legNamesCAN{i}, gnomi{j}, c, a, rap);
    end
end

%% ---- e se fossero solo nell'ordine sbagliato? ----
% Se il modulo dei 18 valori e' lo stesso ma distribuito diversamente, non e'
% un problema di segno ne' di unita': e' una permutazione. Confrontare gli
% insiemi ORDINATI lo dice subito, senza doverla cercare a mano.
fprintf('\n  valori ordinati per modulo, comandati vs arrivati:\n');
sc = sort(abs(cmd_can), 'descend');  sa = sort(abs(arr_can), 'descend');
fprintf('    comandati : ');  fprintf('%.3f ', sc(1:6));  fprintf('...\n');
fprintf('    arrivati  : ');  fprintf('%.3f ', sa(1:6));  fprintf('...\n');
stessi_valori = max(abs(sc - sa)) < 0.02;
al_posto      = max(abs(cmd_can - arr_can)) < 0.02;

fprintf('\n  scarto max, giunto per giunto : %.4f N*m\n', max(abs(cmd_can - arr_can)));
fprintf('  scarto max, valori ordinati    : %.4f N*m\n', max(abs(sc - sa)));
fprintf('  quota corpo: %.1f -> %.1f mm in %.3f s\n', ...
        1e3*r.p(1,3), 1e3*r.p(end,3), r.t(end));

fprintf('\n=========== VERDETTO ===========\n');
if al_posto
    fprintf(['  Le coppie ARRIVANO GIUSTE, al giunto giusto e col segno giusto.\n' ...
             '  Percorso Constant -> convertitori -> giunti: verificato.\n' ...
             '  Allora non resta nessuna ipotesi sulla mappa: il problema e''\n' ...
             '  dinamico, e S0 in anello aperto va ridefinito, non riparato.\n']);
elseif stessi_valori
    fprintf(2, ['  Stessi valori, caselle diverse: e'' una PERMUTAZIONE.\n' ...
                '  Il fan-out a valle del Constant non e'' nell''ordine del Mux di\n' ...
                '  ingresso. Si sistema in crea_modello_mpc, non qui.\n']);
else
    fprintf(2, ['  Le coppie arrivate non sono quelle comandate, e nemmeno una loro\n' ...
                '  permutazione. Guarda la colonna rapporto: costante e negativo e''\n' ...
                '  un segno, costante e diverso da 1 sono le unita'', zeri vogliono\n' ...
                '  dire che non arriva niente.\n']);
end
fprintf('\n');

info = struct('cmd_can', cmd_can, 'arr_can', arr_can, ...
              'al_posto', al_posto, 'stessi_valori', stessi_valori, 'run', r);
end
