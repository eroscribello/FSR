%% posa_iniziale.m - la copia parte davvero dalla stessa posa del modello base?
%
% IL NUMERO CHE HA CAMBIATO LA DIAGNOSI
%   [24/9] Nell'output di prova_coppie, in mezzo a tutto il resto:
%       zampe a terra medie 2.32 su 6
%   Nella stessa condizione, sul modello base (tau_misurato), era 6.00 su 6.
%   Nei primi 0.12 s la copia ha QUATTRO ZAMPE PER ARIA. E il corpo intanto
%   sale di 0.7 mm, cioe' quasi niente: quindi non e' il corpo che si alza
%   staccando i piedi, sono le zampe che si muovono per conto loro.
%
%   Questo spiega tutto quello che finora non tornava. Le coppie statiche
%   sono giuste per una zampa CARICA. Una zampa che non tocca terra non ha
%   nessuna forza da equilibrare: la sua coppia statica la accelera e basta,
%   con la sola inerzia del link davanti. Quattro zampe che frustano mentre
%   due spingono danno esattamente il "robot che schizza in alto" e la
%   configurazione singolare che fa degenerare la matrice di massa.
%   Non e' un problema di coppie. E' un problema di POSA DI PARTENZA.
%
% PERCHE' PROPRIO LA COPIA
%   Il sospetto ha un nome: PositionTargetPriority = 'Low'. E' stato messo
%   per risolvere il sovra-vincolo in fase di assemblaggio, ed e' l'unica
%   modifica non documentata fatta a mano sulla copia. "Low" vuol dire che
%   il solutore CERCA di partire da quegli angoli ma non e' tenuto a
%   riuscirci: se ci arriva vicino invece che esatto, i piedi non sono piu'
%   sul pavimento e il precarico del contatto se ne va.
%
% COSA FA
%   Fa partire i due modelli per pochi centesimi di secondo e confronta il
%   PRIMO campione: angoli di giunto, quota dei piedi, forza di contatto.
%   Non serve simulare a lungo - la domanda riguarda solo t = 0.
%     - base: FORZA_STATICO, posa nominale comandata (il riferimento);
%     - copia: coppia zero, cosi' niente puo' falsare il primo istante.
%
% COME SI LEGGE
%   - angoli uguali entro un grado e sei piedi a terra in tutti e due:
%     la posa iniziale non c'entra, e va cercato altro.
%   - angoli diversi, o piedi staccati nella copia:
%     e' l'assemblaggio. Si rimette PositionTargetPriority a 'High' e si
%     rimisura, sapendo pero' che con 'High' era comparso il sovra-vincolo:
%     in quel caso i target vanno messi solo sui 18 giunti E il 6-DOF Joint
%     va lasciato libero, oppure il contrario, non tutti e due.
%
% SOLO LETTURA sui parametri: applica_terreno e applica_inerzie scrivono in
% memoria, nessun save_system su nessuno dei due modelli.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   posa_iniziale
%
% Progetto FSR PhantomX - A. Russo

function info = posa_iniziale(durata)

if nargin < 1 || isempty(durata), durata = 0.06; end
cfg = phantomx_config();

%% ---- il modello base, in posa nominale ----
pi_pulisci = onCleanup(@() evalin('base','clear FORZA_STATICO OVERRIDE_C2'));
assignin('base','FORZA_STATICO', true);
assignin('base','OVERRIDE_C2',   false);
rb = pi_run('phantomx_sim_zero', durata, 'C1');

%% ---- la copia, a coppia zero ----
evalin('base','clear FORZA_STATICO OVERRIDE_C2');
assignin('base','tau_ts', timeseries(zeros(2,18), [0; durata]));
rc = pi_run('phantomx_sim_mpc', durata, 'C3');

%% ---- angoli di giunto al primo campione ----
gnomi = {'coxa','femore','tibia'};
fprintf('\n=========== POSA A t = 0 ===========\n');
fprintf('  %-5s %-7s %10s %10s %10s\n', 'zampa','giunto','base [deg]','copia','differenza');
dmax = 0;
for i = 1:6
    for j = 1:3
        k = 3*(i-1)+j;
        qb = rad2deg(rb.q(1,k));  qc = rad2deg(rc.q(1,k));
        dmax = max(dmax, abs(qb-qc));
        fprintf('  %-5s %-7s %10.3f %10.3f %10.3f\n', ...
                cfg.legNamesCAN{i}, gnomi{j}, qb, qc, qc-qb);
    end
end
fprintf('\n  differenza massima di angolo: %.3f deg\n', dmax);

%% ---- piedi e contatto ----
fprintf('\n  %-5s %12s %12s %12s %12s\n', 'zampa', 'z piede base', 'z piede cop', 'Fz base', 'Fz copia');
for i = 1:6
    zb = NaN; zc = NaN; fb = NaN; fc = NaN;
    if isfield(rb,'pf') && ~isempty(rb.pf), zb = 1e3*rb.pf(1, 3*(i-1)+3); end
    if isfield(rc,'pf') && ~isempty(rc.pf), zc = 1e3*rc.pf(1, 3*(i-1)+3); end
    if isfield(rb,'Fc') && ~isempty(rb.Fc), fb = rb.Fc(1, 3*(i-1)+3); end
    if isfield(rc,'Fc') && ~isempty(rc.Fc), fc = rc.Fc(1, 3*(i-1)+3); end
    fprintf('  %-5s %12.2f %12.2f %12.3f %12.3f\n', cfg.legNamesCAN{i}, zb, zc, fb, fc);
end
fprintf('  (quote in mm, piano a %.1f mm; Fz in N, atteso %.2f per zampa)\n', ...
        1e3*cfg.floor_top, cfg.mass*cfg.g/6);

nb = pi_aterra(rb);  nc = pi_aterra(rc);
fprintf('\n  piedi a terra a t=0:  base %d/6,  copia %d/6\n', nb, nc);
fprintf('  quota corpo a t=0  :  base %.2f mm, copia %.2f mm (nominale %.2f)\n', ...
        1e3*rb.p(1,3), 1e3*rc.p(1,3), 1e3*cfg.body_z0);

%% ---- verdetto ----
fprintf('\n=========== VERDETTO ===========\n');
if dmax < 1.0 && nc >= 6
    fprintf(['  La copia parte dalla stessa posa del base, con sei piedi a terra.\n' ...
             '  Allora la posa iniziale NON e'' la causa, e le zampe si staccano\n' ...
             '  DOPO: e'' instabilita'' vera, e S0 va ridefinito con retroazione.\n']);
elseif dmax >= 1.0
    fprintf(2, ['  La copia NON parte dalla stessa posa: fino a %.2f gradi di\n' ...
                '  differenza. Con angoli diversi le coppie statiche misurate sul\n' ...
                '  base non sono piu'' quelle di equilibrio, e le zampe partono\n' ...
                '  gia'' sbilanciate. E'' l''assemblaggio: PositionTargetPriority.\n'], dmax);
else
    fprintf(2, ['  Gli angoli coincidono ma la copia ha solo %d piedi a terra su 6.\n' ...
                '  Allora non sono i giunti: e'' la quota del corpo o la posa del\n' ...
                '  pavimento nella copia. Guarda la colonna "z piede".\n'], nc);
end
fprintf('\n');

info = struct('dmax_deg', dmax, 'piedi_base', nb, 'piedi_copia', nc, ...
              'base', rb, 'copia', rc);
end

%% ================= helper =================
function r = pi_run(mdl, durata, ctrl)
if ~bdIsLoaded(mdl), load_system(mdl); end
applica_terreno('T1', false, mdl);
applica_inerzie(mdl);
out = sim(mdl, 'StopTime', num2str(durata));
r = adatta_simscape(out, struct('controller',ctrl, 'task','S0', 'run',1, ...
                                'condizione','posa', 'vel_d',[0 0]));
end

function n = pi_aterra(r)
n = NaN;
if isfield(r,'Fc') && ~isempty(r.Fc)
    n = nnz(r.Fc(1, 3:3:18) > 0.5);
end
end
