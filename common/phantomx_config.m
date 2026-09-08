function cfg = phantomx_config(verbose)
%PHANTOMX_CONFIG  Unica fonte di verita' dei parametri del PhantomX.
%
% PROVENIENZA DEI VALORI
%   [URDF]      letto da urdf/phantomx.urdf
%   [MESH]      misurato sulla geometria STL
%   [MODELLO]   letto dai blocchi di phantomx_sim_zero.slx
%   [DATASHEET] specifica del servo
%   [TARATO]    trovato sperimentalmente
%
% Progetto FSR PhantomX

if nargin < 1, verbose = false; end

%% ===== guardia sui doppioni =====
copie = which('phantomx_config','-all');
copie = copie(~contains(copie,'_cestino'));
if numel(copie) > 1
    warning('phantomx:config:duplicati', ...
        ['Esistono %d copie di phantomx_config sul path. MATLAB ne usa una\n' ...
         'sola e potresti star modificando l''altra:\n%s'], ...
         numel(copie), sprintf('    %s\n', copie{:}));
end
copie = which('inv_kyn','-all');
copie = copie(~contains(copie,'_cestino'));
if numel(copie) > 1
    warning('phantomx:config:duplicati', ...
        'Esistono %d copie di inv_kyn sul path:\n%s', ...
        numel(copie), sprintf('    %s\n', copie{:}));
end

%% ===== costanti fisiche =====
cfg.g = 9.81;

%% ===== ordine delle zampe =====
cfg.legNamesCAN = {'FL','FR','ML','MR','RL','RR'};
cfg.legNamesMUX = {'RR','MR','FR','RL','ML','FL'};
cfg.mux2can = [6 4 2 5 3 1];
cfg.can2mux = [6 3 5 2 4 1];
cfg.jointIndex = @(i,j) 3*(cfg.can2mux(i)-1) + j;

%% ===== geometria della gamba =====
cfg.lc = 0.054;    % [URDF] coxa,   da j_c2_* xyz = [0 -0.054 0]
cfg.lf = 0.0661;   % [URDF] femore, da j_tibia_* xyz = [0 -0.0645 -0.0145]
                   %        norm([0.0645 0.0145]) = 0.066112

% Posizione della sfera di contatto nel frame della tibia, come sta nei sei
% blocchi Rigid Transform1..6 del modello. E' il piede: l'IK comanda questo
% punto, e la sfera tocca terra un raggio piu' sotto.
cfg.foot_xyz    = [0.03, -0.15, 0];              % [MODELLO]
cfg.foot_offset = norm(cfg.foot_xyz(1:2));       % 0.152971 m
cfg.lt          = cfg.foot_offset;               % tibia vista dall'IK

% Punta geometrica della tibia, misurata sulla mesh tibia_l.STL: il centroide
% della zona terminale sta a [0.00155, 0.15985, 0.02879] nel frame del link,
% e l'asse del ginocchio e' x, quindi la lunghezza e' nel piano y-z.
cfg.lt_mesh = norm([0.15985, 0.02879]);          % [MESH] 0.162422 m

cfg.reachMax = cfg.lf + cfg.lt;                  % 0.219071 m

%% ===== posizione delle anche nel frame corpo, ordine CAN =====
cfg.p_hip = [ ...
     0.1248     0.1248     0.0      0.0     -0.1248    -0.1248 ; ...
     0.06164   -0.06164    0.1034  -0.1034   0.06164   -0.06164 ; ...
     0.001116   0.001116   0.001116 0.001116 0.001116   0.001116 ];   % [URDF]

cfg.alpha = deg2rad([45, -45, 90, -90, 135, -135]);   % [URDF] yaw di j_c1_* meno 90 gradi
cfg.side  = [ +1, -1, +1, -1, +1, -1 ];

%% ===== posa nominale =====
cfg.r_offset = 0.14;   % [TARATO] estensione radiale del piede dall'asse coxa
cfg.z0       = 0.14;   % [TARATO] profondita' d'appoggio sotto l'anca

cfg.pf_nom = zeros(3,6);
for i = 1:6
    cfg.pf_nom(:,i) = cfg.p_hip(:,i) + ...
        [cfg.r_offset*cos(cfg.alpha(i)); cfg.r_offset*sin(cfg.alpha(i)); -cfg.z0];
end

%% ===== masse e inerzie =====
cfg.m_body = 0.975599;
cfg.m_link = 0.024357719;
cfg.mass   = cfg.m_body + 24*cfg.m_link;

cfg.J       = diag([0.01444, 0.01751, 0.02889]);   % [TARATO] stimata, non misurata
cfg.I_body  = [3.557e-03, 5.154e-03, 8.565e-03];
cfg.I_c1    = [3.654e-06, 7.746e-06, 7.746e-06];
cfg.I_c2    = [3.654e-06, 3.654e-06, 3.654e-06];
cfg.I_thigh = [3.095e-06, 1.014e-05, 1.070e-05];
cfg.I_tibia = [2.081e-06, 3.004e-05, 3.050e-05];

%% ===== andatura a tripode =====
cfg.T           = 1.00;
cfg.beta_stance = 0.60;
cfg.duty_swing  = 1 - cfg.beta_stance;
cfg.T_stance    = cfg.beta_stance * cfg.T;
cfg.T_swing     = cfg.duty_swing  * cfg.T;
cfg.S           = 0.06;
cfg.v_nom       = cfg.S / cfg.T_stance;
cfg.H           = 0.03;

cfg.phase       = [0; 1/2; 1/2; 0; 0; 1/2];

%% ===== contatto e terreno =====
cfg.contact.k      = 1e4;
cfg.contact.c      = 100;
cfg.contact.w      = 1e-4;
cfg.contact.vcrit  = 1e-2;
cfg.contact.foot_r = 0.01;
cfg.mu_plant = 0.9;
cfg.mu_mpc   = 0.6;

cfg.floor_dim = [4 4 0.05];
cfg.floor_off = [0 0 +cfg.floor_dim(3)/2];
cfg.floor_top = -cfg.floor_off(3) + cfg.floor_dim(3)/2;

cfg.body_z0_geom = cfg.z0 + cfg.contact.foot_r + cfg.floor_top - cfg.p_hip(3,1);
cfg.body_z0      = cfg.body_z0_geom + 0.002;   % 2 mm: nasce appena sopra

%% ===== attuatori =====
cfg.tau_max = 1.5;          % [DATASHEET] l'URDF dichiara 2.8: ottimistico
cfg.qd_max  = 5.6548668;    % [URDF]
cfg.q_min   = -2.6179939;   % [URDF]
cfg.q_max   =  2.6179939;   % [URDF]

%% ===== disturbo esterno =====
cfg.dist.t0    = 2.0;
cfg.dist.dur   = 0.2;
cfg.dist.dir   = [0; 1; 0];
cfg.dist.frac  = 0.25;
cfg.dist.point = [0; 0; 0];
cfg.dist.F     = cfg.dist.frac * cfg.mass * cfg.g;

%% ===== controlli di coerenza =====
% Estensione della gamba nel caso PEGGIORE del ciclo, non nella posa ferma.
% Durante l'appoggio il piede si sposta di +/- S/2 rispetto al nominale, e per
% le zampe montate a 45 gradi quello spostamento si somma quasi tutto alla
% componente radiale: controllare solo il nominale lascia passare pose che a
% meta' passo arrivano oltre il 90%.
reach     = 0;
reach_nom = norm([cfg.r_offset - cfg.lc, cfg.z0]) / cfg.reachMax;
for i = 1:6
    for xs = [-cfg.S/2, +cfg.S/2]
        xl = cfg.r_offset + xs*cos(cfg.alpha(i));
        yl = -xs*sin(cfg.alpha(i))*cfg.side(i);
        tx = hypot(xl, yl) - cfg.lc;
        reach = max(reach, hypot(tx, cfg.z0) / cfg.reachMax);
    end
end

assert(reach < 0.85, 'phantomx:config:reach', ...
    ['A meta'' passo la gamba usa il %.1f%% dell''estensione massima\n' ...
     '(%.1f%% nella posa ferma). Sopra l''85%% l''IK si mal condiziona:\n' ...
     'a 89%% il robot camminava all''indietro. Riduci r_offset o z0.'], ...
     100*reach, 100*reach_nom);

assert(abs(cfg.lt - cfg.foot_offset) < 1e-12, 'phantomx:config:tibia', ...
    'lt e foot_offset devono coincidere: l''IK e la sfera di contatto\ndevono riferirsi allo stesso punto.');

assert(cfg.foot_offset < cfg.lt_mesh, 'phantomx:config:sfera', ...
    ['Il centro della sfera (%.5f) e'' oltre la punta della tibia (%.5f):\n' ...
     'la sfera sporgerebbe fuori dal piede.'], cfg.foot_offset, cfg.lt_mesh);

assert(cfg.mu_mpc <= cfg.mu_plant, 'phantomx:config:mu', ...
    'L''MPC assume un attrito superiore a quello del modello fisico.');

assert(abs(cfg.S - cfg.v_nom*cfg.T_stance) < 1e-12, 'phantomx:config:passo', ...
    'S deve valere v_nom*T_stance, altrimenti il piede striscia in appoggio.');

assert(abs(cfg.mass - 1.560184) < 1e-5, 'phantomx:config:massa', ...
    'La massa totale non torna con m_body + 24*m_link.');

%% ===== riepilogo =====
if verbose
    fprintf('\n=== PHANTOMX CONFIG ===\n');
    fprintf('  gamba        lc %.4f  lf %.4f  lt %.6f   -> reach %.6f m\n', ...
            cfg.lc, cfg.lf, cfg.lt, cfg.reachMax);
    fprintf('               punta della tibia %.6f, sfera un raggio dentro\n', cfg.lt_mesh);
    fprintf('  posa         r_offset %.3f  z0 %.3f  -> %.1f%% da fermo, %.1f%% a meta'' passo\n', ...
            cfg.r_offset, cfg.z0, 100*reach_nom, 100*reach);
    fprintf('  andatura     T %.2f s  appoggio %.2f s  passo %.3f m  -> %.3f m/s\n', ...
            cfg.T, cfg.T_stance, cfg.S, cfg.v_nom);
    fprintf('  quota corpo  %.3f m (geometrica)\n', cfg.body_z0);
    fprintf('  attrito      modello %.2f  MPC %.2f\n\n', cfg.mu_plant, cfg.mu_mpc);
end

end