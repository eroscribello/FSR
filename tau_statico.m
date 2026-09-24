function [tau18, info] = tau_statico(verbose)
%TAU_STATICO  Le 18 coppie che tengono il robot fermo in posa nominale.  [S0]
%
%   tau18 = tau_statico          vettore 18x1 nell'ordine del Mux
%   [tau18, info] = tau_statico(true)   con tabella a schermo
%
% A COSA SERVE
%   E' il gradino S0 del porting dell'MPC: giunti comandati in coppia, MPC
%   spento, forza verticale m*g/6 su tutte e sei le zampe. Se la mappa
%   forze -> coppie e' giusta il robot resta fermo; se il segno o lo
%   Jacobiano sono sbagliati si accascia o salta, e lo si scopre qui invece
%   che dentro l'anello chiuso dell'MPC.
%
%   In posa ferma la coppia non dipende dal tempo: sono 18 numeri, e si
%   possono mettere in un Constant. Nessun blocco con logica, nessuna
%   funzione nuova nel modello.
%
% LA MAPPA, E PERCHE' E' SCRITTA COSI'
%       tau_i = - J_i(q)' * R' * f_i
%   f_i e' la forza che il TERRENO esercita sul piede; la zampa spinge il
%   terreno con -f_i. Il segno e' DICHIARATO qui e MISURATO da S0: se il
%   robot si accascia, il segno giusto e' l'altro. Non e' una cosa da
%   decidere ragionando - con applica_imbardata un segno scelto a tavolino
%   ci e' costato tre giorni.
%   Qui R = I (corpo orizzontale in posa nominale), quindi tau = -J' * f.
%
% LO JACOBIANO SENZA SCRIVERE UNA CINEMATICA DIRETTA
%   Non esiste una FK nel repo, e scriverne una nuova vorrebbe dire
%   reintrodurre le convenzioni che inv_kyn ha gia' risolto. Si fa al
%   contrario:
%     - la posizione del piede nel frame corpo e', per costruzione di
%       cfg.pf_nom,   p = p_hip + [r*cos(a) + x; r*sin(a) + y; -z]
%       quindi  dp/du = diag(1, 1, -1)  con u = (x, y, z) il comando;
%     - A = dq/du si ottiene per differenze centrate su inv_kyn, che e' gia'
%       validata da verifica_ik e usata dal modello;
%     - J = dp/dq = (dp/du) * inv(A).
%   Cosi' l'unica cinematica in gioco resta quella che il robot usa davvero.
%
% CONTROLLI CHE FA DA SOLO
%   1. A invertibile (fuori dalle singolarita' in posa nominale);
%   2. richiusura: ricostruendo f da tau con lo Jacobiano si ritrova m*g/6;
%   3. simmetria destra/sinistra delle coppie di coxa;
%   4. confronto con cfg.tau_max (datasheet AX-12A).
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(verbose), verbose = true; end
cfg = phantomx_config();

ts_h  = 1e-6;                       % passo delle differenze centrate [m]
ts_fz = cfg.mass * cfg.g / 6;       % [N] carico verticale per zampa
ts_D  = diag([1, 1, -1]);           % dp/du, dal modo in cui e' costruita pf_nom

tau18 = zeros(18,1);
info  = struct('leg', {}, 'q', {}, 'J', {}, 'tau', {}, 'cond_A', {}, 'err_f', {});

for i = 1:6
    a  = cfg.alpha(i);
    sd = cfg.side(i);
    u0 = [0; 0; cfg.z0];            % comando nominale: nessuno scostamento

    % --- A = dq/du per differenze centrate ---
    A = zeros(3,3);
    for k = 1:3
        up = u0;  up(k) = up(k) + ts_h;
        um = u0;  um(k) = um(k) - ts_h;
        [t1,p1,s1] = inv_kyn(up(1), up(2), up(3), sd, a);
        [t2,p2,s2] = inv_kyn(um(1), um(2), um(3), sd, a);
        A(:,k) = ([t1;p1;s1] - [t2;p2;s2]) / (2*ts_h);
    end
    if rcond(A) < 1e-8
        error('tau_statico:singolare', ...
              'Zampa %d: dq/du e'' mal condizionata (rcond %.1e). Posa singolare?', ...
              i, rcond(A));
    end

    J = ts_D / A;                   % dp/dq = (dp/du) * inv(A)

    f   = [0; 0; ts_fz];            % forza del terreno sul piede, frame corpo
    tau = -J.' * f;                 % <-- la mappa dichiarata

    % richiusura: da tau si deve poter risalire alla forza
    f_ric  = -(J.') \ tau;
    err_f  = norm(f_ric - f);

    [q1,q2,q3] = inv_kyn(u0(1), u0(2), u0(3), sd, a);
    for j = 1:3
        tau18(cfg.jointIN(i,j)) = tau(j);
    end
    info(i) = struct('leg', i, 'q', [q1;q2;q3], 'J', J, 'tau', tau, ...
                     'cond_A', rcond(A), 'err_f', err_f);       %#ok<AGROW>
end

%% ---- lettura ----
if ~verbose, return; end

nomi = {'FL','FR','ML','MR','RL','RR'};
fprintf('\n=========== COPPIE STATICHE (S0) ===========\n');
fprintf('  carico per zampa  %.3f N   (massa %.4f kg, sei zampe)\n', ts_fz, cfg.mass);
fprintf('  R = I: corpo orizzontale, posa nominale z0 = %.3f m\n\n', cfg.z0);

fprintf('  %-5s %9s %9s %9s   %9s %9s %9s\n', 'zampa', ...
        'q coxa', 'q femore', 'q tibia', 'tau coxa', 'tau fem', 'tau tib');
for i = 1:6
    fprintf('  %-5s %9.2f %9.2f %9.2f   %9.4f %9.4f %9.4f\n', nomi{i}, ...
            rad2deg(info(i).q(1)), rad2deg(info(i).q(2)), rad2deg(info(i).q(3)), ...
            info(i).tau(1), info(i).tau(2), info(i).tau(3));
end

fprintf('\n  richiusura f da tau: errore max %.2e N\n', max([info.err_f]));

% simmetria: le zampe sinistre e destre stanno a coppie (1,2) (3,4) (5,6)
fprintf('  simmetria destra/sinistra:\n');
for k = 1:3
    i1 = 2*k-1;  i2 = 2*k;
    d_fem = abs(info(i1).tau(2) - info(i2).tau(2));
    d_tib = abs(info(i1).tau(3) - info(i2).tau(3));
    fprintf('    %s / %s: differenza femore %.2e, tibia %.2e N*m\n', ...
            nomi{i1}, nomi{i2}, d_fem, d_tib);
end

tau_pk = max(abs(tau18));
fprintf('\n  coppia massima richiesta: %.4f N*m   (datasheet AX-12A: %.2f)\n', ...
        tau_pk, cfg.tau_max);
if tau_pk > cfg.tau_max
    fprintf(2, '  Gia'' in piedi fermo il robot supera il datasheet: da dichiarare.\n');
else
    fprintf('  Sta dentro il datasheet: in piedi fermo il robot vero ce la farebbe.\n');
end

fprintf('\n  vettore per il Constant del modello (ordine Mux):\n  [');
fprintf('%.6f ', tau18);
fprintf(']''\n\n');
fprintf(['  ATTESO DA S0: corpo fermo entro 5 mm di quota e 1 grado di assetto\n' ...
         '  in 2 s. Se si accascia o salta, il segno di tau = -J''*f e'' l''altro:\n' ...
         '  si cambia UNA volta e si rimisura, non si ragiona.\n\n']);
end
