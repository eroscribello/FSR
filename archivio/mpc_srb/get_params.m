function p = get_params(gait)
%GET_PARAMS  Parametri dell'MPC sul simulatore ridotto (modello SRB).
%
%   p = get_params        % TRIPODE, l'andatura del confronto
%   p = get_params(0)     % tripode  (3 zampe a terra)
%   p = get_params(1)     % ripple   (4 zampe a terra)
%   p = get_params(2)     % wave     (5 zampe a terra)
%
% Tutto cio' che e' fisico viene da phantomx_config: masse, inerzie,
% geometria, andatura, attrito, limiti. Qui restano i parametri
% dell'ottimizzatore e quelli di disegno, che non hanno un corrispettivo
% nel modello Simscape.
%
% ATTENZIONE ALL'ANDATURA
%   Il vecchio MAIN.m aveva gait = 1, cioe' RIPPLE. Il baseline Simscape
%   cammina a TRIPODE. Per il confronto servono la stessa andatura e la
%   stessa temporizzazione: usa gait = 0.
%
% Ordine gambe (CAN): 1=FL 2=FR 3=ML 4=MR 5=RL 6=RR
%
% Progetto FSR PhantomX - A. Russo

cfg = phantomx_config();

if nargin < 1 || isempty(gait)
    gait = 0;                 % tripode
end
p.gait = gait;
p.cfg  = cfg;

%% ===== fisica, da phantomx_config =====
p.g     = cfg.g;
p.mass  = cfg.mass;
p.J     = cfg.J;
p.nLeg  = 6;
p.mu    = cfg.mu_mpc;         % piu' conservativo dell'attrito del modello fisico
p.z0    = cfg.z0;

p.d     = cfg.lc;             % coxa   (nome storico nel codice MPC)
p.l1    = cfg.lf;             % femore
p.l2    = cfg.lt;             % tibia  (era 0.075 "da confermare dalla mesh")

p.p_hip = cfg.p_hip;
p.alpha = cfg.alpha;
p.side  = cfg.side;

% Ingombro del corpo, ricavato dalle anche. Serve a fcn_get_disturbance,
% che oggi lo usa come punto di applicazione della spinta: e' sbagliato,
% ma finche' non lo riscriviamo questi campi devono esistere.
p.L = max(cfg.p_hip(1,:)) - min(cfg.p_hip(1,:));   % 0.2496
p.W = max(cfg.p_hip(2,:)) - min(cfg.p_hip(2,:));   % 0.2068
p.h = 0.05;                                        % semi-altezza, solo per il disegno

%% ===== appoggio nominale =====
% NB: pf36 e' espresso nel frame MONDO, con i piedi A TERRA (z = 0).
% Non e' cfg.pf_nom, che invece e' nel frame CORPO (z = -z0).
% Scambiarli mette i piedi 10 cm sotto il pavimento.
p.pf36 = [cfg.pf_nom(1:2,:); zeros(1,6)];

%% ===== attuatori =====
p.tau_max = cfg.tau_max;
p.qd_max  = cfg.qd_max;
p.q_min   = cfg.q_min;
p.q_max   = cfg.q_max;

%% ===== andatura =====
switch gait
    case 1      % RIPPLE (tetrapode): 4 zampe a terra
        p.T = 0.45;  p.beta = 2/3;
        p.phase = [0; 2/3; 1/3; 0; 2/3; 1/3];
    case 2      % WAVE (metacronale): 5 zampe a terra
        p.T = 0.72;  p.beta = 5/6;
        p.phase = [1/3; 5/6; 1/6; 2/3; 0; 1/2];
    otherwise   % 0 - TRIPODE: 3 zampe a terra
        p.T     = cfg.T;             % dal baseline Simscape
        p.beta  = cfg.beta_stance;
        p.phase = cfg.phase;
end

p.Tst = p.beta * p.T;          % tempo di appoggio, usato da fcn_FSM
p.Tsw = (1 - p.beta) * p.T;    % tempo di volo

if gait ~= 0
    warning('phantomx:params:gait', ...
        ['gait = %d: temporizzazione NON allineata al baseline Simscape.\n' ...
         'Va bene per esplorare, non per il confronto fra controllori.'], gait);
end

% costanti del generatore di appoggi (fcn_FSM le legge con isfield)
p.stepLenRef  = cfg.S;    % lunghezza passo di riferimento
p.stepClamp   = cfg.S;    % clamp sul capture point
p.swingHeight = cfg.H;    % sollevamento del piede in volo

p.Kp_sw = 300;            % guadagno di inseguimento della zampa in volo
p.v_nom = cfg.v_nom;
p.S     = cfg.S;
p.H     = cfg.H;

%% ===== comando di riferimento (MAIN puo' sovrascriverlo) =====
p.vel_d = [cfg.v_nom; 0];   % [vx; vy] desiderati
p.yaw_d = 0;                % velocita' di imbardata desiderata
p.acc_d = 1;                % accelerazione della rampa iniziale

%% ===== terreno =====
p.Rground = eye(3);         % piano orizzontale; ruotala per i task in pendenza

%% ===== disturbo esterno =====
p.dist = cfg.dist;

%% ===== ottimizzatore =====
p.predHorizon = 6;
p.Tmpc        = 8/100;      % passo di ricalcolo del QP [s]
p.simTimeStep = 1/100;
p.Umax        = 50;
p.decayRate   = 1;

% pesi sullo stato: [pos(3) vel(3) eta(3) omega(3)]
p.Q  = diag([1e5 2e5 3e5   5e2 1e3 1e3   1e3 1e4 800   40 40 10]);
p.Qf = p.Q;

% pesi sugli ingressi: 18 forze (6 zampe x 3 componenti).
% Con 6 zampe la distribuzione delle forze e' sovra-determinata nelle fasi
% a supporto multiplo, e R e' l'unica cosa che sceglie fra le infinite
% combinazioni equivalenti. Sul quadrupede pesava molto meno: va giustificato
% nel report, non subito.
p.R = diag(repmat([0.1 0.2 0.1]', [p.nLeg, 1]));

%% ===== disegno e animazione =====
p.freq          = 30;
p.playSpeed     = 1;
p.flag_movie    = 0;                      % 1 = registra il video
p.forceScale    = 0.02;                   % metri di freccia per newton
p.body_color    = [ 42  80 183]/255;
p.leg_color     = [  7 179 128]/255;
p.ground_color  = [195 232 243]/255;

%% ===== controlli di coerenza =====
assert(size(p.R,1) == 3*p.nLeg, 'phantomx:params:R', ...
    'R deve essere %dx%d per %d zampe.', 3*p.nLeg, 3*p.nLeg, p.nLeg);

assert(size(p.Q,1) == 12, 'phantomx:params:Q', ...
    'Q deve essere 12x12: lo stato ridotto resta a 12 anche con 6 zampe.');

assert(all(abs(p.pf36(3,:)) < 1e-12), 'phantomx:params:pf36', ...
    'pf36 e'' nel frame MONDO: i piedi devono stare a z = 0.');

if gait == 0
    assert(abs(p.T - cfg.T) < 1e-12 && abs(p.beta - cfg.beta_stance) < 1e-12, ...
        'phantomx:params:disallineato', ...
        'I tempi dell''andatura non coincidono con phantomx_config.');
end

end