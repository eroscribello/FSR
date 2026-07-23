function p = get_params(gait)
% Parametri per ESAPODE PhantomX AX Metal Mark II.
% Ordine gambe (canonico, usato in TUTTI i file):
%   1=LF  2=RF  3=LM  4=RM  5=LH  6=RH
%   Tripode A = {1,4,5}   Tripode B = {2,3,6}

p.nLeg = 6;

%% ---- Geometria anche (da phantomx.urdf) ----
% Layout ESAGONALE: le gambe centrali sporgono in y piu' di ant./post.
% Le colonne seguono l'ordine canonico sopra.
p.p_hip = [ 0.1248,  0.1248,  0.0000,  0.0000, -0.1248, -0.1248;
            0.0616, -0.0616,  0.1034, -0.1034,  0.0616, -0.0616;
            0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000];

%% ---- MPC ----
p.predHorizon = 6;
p.simTimeStep = 1/200;
p.Tmpc = 4/100;
p.gait = gait;
p.Umax = 50;
p.decayRate = 1;
p.freq = 30;
p.Rground = eye(3);
p.Qf = diag([1e5 2e5 3e5 5e2 1e3 150 1e3 1e4 800 40 40 10]);

% ---- gait ----
% Per l'esapode si usa gait = 0 -> TRIPODE.
% Q/Qf restano 12x12 (pesano lo stato del corpo); solo R scala a 3*nLeg.
if gait == 5                % wave/crawl: una zampa alla volta
    p.Tst = 0.30;
    p.Tsw = 0.10;
    p.R = diag(repmat([0.1 0.2 0.1]',[p.nLeg,1]));
    p.Q = diag([5e5 5e5 9e5 5 5 5 3e3 3e3 3e3 3 3 3]);
else                        % 0 - TRIPODE (ramo usato)
    p.predHorizon = 6;
    p.simTimeStep = 1/100;
    p.Tmpc = 8/100;
    % Tst = Tsw -> alternanza pulita dei due tripodi.
    % (con Tst ~= Tsw restano istanti con tutte e 6 le zampe a terra: non e'
    %  un errore, ma il tripode non e' piu' perfettamente alternato)
    p.Tst = 0.20;           % era 0.3 (quad)
    p.Tsw = 0.20;           % era 0.15 (quad)
    p.R = diag(repmat([0.1 0.2 0.1]',[p.nLeg,1]));
    p.Q = diag([1e5 2e5 3e5 5e2 1e3 1e3 1e3 1e4 800 40 40 10]);
    p.Qf = p.Q;
end

%% ---- Parametri fisici PhantomX (da URDF) ----
p.mass = 1.56;              % [kg] massa totale       (era 11 / quad 5.5)
p.J = diag([0.008, 0.010, 0.016]);   % stima box -> CONFERMARE in Simscape
p.g = 9.81;
p.mu = 0.6;                 % attrito gomma-superficie dura (era 1.4: irrealistico)
p.z0 = 0.15;                % [m] altezza nominale corpo (era 0.2: irraggiungibile,
                            %     femore+tibia = 0.141 m)

% ---- lunghezze link (da URDF) ----
p.d  = 0.054;   % coxa   (era 0.05, offset ABAD del quadrupede)
p.l1 = 0.066;   % femore (era 0.14)
p.l2 = 0.075;   % tibia  (era 0.14) -> confermare da mesh

% ---- ingombro corpo: SOLO per il disegno ----
p.L = 0.250;
p.W = 0.207;
p.h = 0.05;

%% ---- Stance nominale ----
% NOME UNIFICATO: p.pf36 (prima p.pf34, fuorviante con 6 zampe).
% NON si scrive a mano: si deriva da p_hip spingendo i piedi RADIALMENTE
% verso l'esterno. Cosi' non puo' andare fuori sincrono con le anche.
% (prima i piedi delle zampe medie erano piu' INTERNI delle loro anche
%  -> gambe disegnate verso l'interno)
p.spread = 0.07;            % quanto i piedi stanno fuori dalle anche [m]
u_rad = p.p_hip(1:2,:) ./ vecnorm(p.p_hip(1:2,:));   % direzione radiale per anca
p.pf36 = zeros(3,p.nLeg);
p.pf36(1:2,:) = p.p_hip(1:2,:) + p.spread * u_rad;
p.pf36(3,:)   = 0;                                    % piedi a terra

%% ---- Costanti di andatura (scala PhantomX) ----
p.stepLenRef  = 0.05;       % lunghezza passo di riferimento (era 0.2 quad)
p.stepClamp   = 0.05;       % clamp capture point           (era 0.15 quad)
p.swingHeight = 0.025;      % sollevamento piede in volo    (era 0.1 quad!)

%% ---- Swing phase ----
p.Kp_sw = 300;

%% ---- colori ----
p.body_color    = [42 80 183]/255;
p.leg_color     = [7 179 128]/255;
p.ground_color  = [195 232 243]/255;