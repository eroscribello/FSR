function p = get_params(gait)

p.predHorizon = 6;
p.simTimeStep = 1/200;
p.Tmpc = 4/100;             % MPC prediction step time
p.gait = gait;
p.Umax = 50;
p.decayRate = 1;
p.freq = 30;
p.Rground = eye(3);
p.Qf = diag([1e5 2e5 3e5 5e2 1e3 150 1e3 1e4 800 40 40 10]);

p.nLeg = 6;                 % <-- ESAPODE (era implicitamente 4)

% ---- gait ----
% NB: per l'esapode si usa gait = 0 (ramo "trot" -> TRIPODE).
% Gli altri rami sono legacy quadrupede: lasciati per riferimento, non usati.
% Q e Qf restano 12x12 (pesano lo stato del corpo). Solo R scala a 3*nLeg.
if gait == 1                % 1 - bound (legacy quad)
    p.Tst = 0.1;
    p.Tsw = 0.18;
    p.predHorizon = 7;
    p.simTimeStep = 1/100;
    p.Tmpc = 2/100;
    p.decayRate = 1;
    p.R = diag(repmat([0.1 0.1 0.1]',[p.nLeg,1]));
    p.Q = diag([5e4 2e4 1e6 4e3 5e2 5e2 1e4 5e4 1e3 1e2 5e2 1e2]);
    p.Qf = diag([2e5 5e4 5e6 8e3 5e2 5e2 1e4 5e4 5e3 1e2 1e2 1e2]);
elseif gait == 2            % 2 - pacing (legacy quad)
    p.Tst = 0.12;
    p.Tsw = 0.12;
    p.R = diag(repmat([0.1 0.2 0.1]',[p.nLeg,1]));
    p.Q = diag([5e3 5e3 9e4 5e2 5e2 5e2 7e3 7e3 7e3 5e1 5e1 5e1]);
elseif gait == 3            % 3 - gallop (legacy quad)
    p.Tst = 0.08;
    p.Tsw = 0.2;
    p.R = diag(repmat([0.1 0.2 0.1]',[p.nLeg,1]));
    p.Q = diag([3e3 3e3 4e6 5e2 1e3 150 1e4 1e4 800 1e2 5e1 5e1]);
elseif gait == 4            % 4 - trot run (legacy quad)
    p.Tst = 0.12;
    p.Tsw = 0.2;
    p.Tmpc = 3/100;
    p.predHorizon = 6;
    p.decayRate = 1;
    p.R = diag(repmat([0.1 0.18 0.08]',[p.nLeg,1]));
    p.Q = diag([1e5 1e5 1e5 1e3 1e3 1e3 2e3 1e4 800 100 40 10]);
    p.Qf = diag([1e5 1.5e5 2e4 1.5e3 1e3 100 2e3 2e3 800 100 60 10]);
elseif gait == 5            % 5 - crawl (legacy quad)
    p.Tst = 0.3;
    p.Tsw = 0.1;
    p.R = diag(repmat([0.1 0.2 0.1]',[p.nLeg,1]));
    p.Q = diag([5e5 5e5 9e5 5 5 5 3e3 3e3 3e3 3 3 3]);
else                        % 0 - trot -> TRIPODE (ramo usato)
    p.predHorizon = 6;
    p.simTimeStep = 1/100;
    p.Tmpc = 8/100;
    p.Tst = 0.20;           % <-- avvicinato a Tsw per tripode alternato (era 0.3)
    p.Tsw = 0.20;           % <-- (era 0.15). Per un po' di overlap: Tst leggermente > Tsw
    p.R = diag(repmat([0.1 0.2 0.1]',[p.nLeg,1]));   % 18x18 (era [4,1])
    p.Q = diag([1e5 2e5 3e5 5e2 1e3 1e3 1e3 1e4 800 40 40 10]);
    p.Qf = p.Q;
end

%% Physical Parameters — PhantomX AX Mark II (da phantomx.urdf)
p.mass = 1.56;              % massa totale [kg]        (era 5.5)
p.J = diag([0.008, 0.010, 0.016]);   % stima box -> CONFERMARE in Simscape (era diag([0.026 0.112 0.075]))
p.g = 9.81;
p.mu = 0.5;                % attrito gomma (era 0.2)
p.z0 = 0.08;               % altezza nominale corpo [m] (era 0.2)

% Posizioni delle 6 anche (giunti j_c1_*) rispetto al centro corpo.
% Ordine canonico: 1=lf 2=rf 3=lm 4=rm 5=lr 6=rr. Layout ESAGONALE.
p.p_hip = [ 0.1248,  0.1248,  0.0000,  0.0000, -0.1248, -0.1248;
            0.0616, -0.0616,  0.1034, -0.1034,  0.0616, -0.0616;
            0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000];   % 3x6

% Stance neutra: piedi sotto le anche, a terra (z=0). (sostituisce p.pf34)
p.pf36 = p.p_hip;          % 3x6

p.L = 0.250;    % span anche x  (solo per il disegno del corpo)
p.W = 0.207;    % span anche y  (solo per il disegno del corpo)
p.d = 0.054;    % coxa offset
p.h = 0.05;     % altezza corpo (disegno)
p.l1 = 0.066;   % femore
p.l2 = 0.075;   % tibia (stima -> confermare da mesh)

%% Swing phase
p.Kp_sw = 300;  % Kp for swing phase

%% color
p.body_color    = [42 80 183]/255;
p.leg_color     = [7 179 128]/255;
p.ground_color  = [195 232 243]/255;