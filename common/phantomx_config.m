 function cfg = phantomx_config(verbose)
% Contiene tutti i parametri necessari per la configurazione del progetto. 
%
% PROVENIENZA DEI VALORI
%   [URDF]      letto da urdf/phantomx.urdf
%   [MESH]      misurato sulla geometria STL
%   [MODELLO]   letto dai blocchi di phantomx_sim_zero.slx
%   [DATASHEET] specifica del servo
%   [TARATO]    trovato sperimentalmente
%

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

% Mux di INGRESSO: porta gli angoli ai giunti
cfg.legNamesMUX = {'RR','MR','FR','RL','ML','FL'};
cfg.mux2can = [6 4 2 5 3 1];
cfg.can2mux = [6 3 5 2 4 1];

% Mux di USCITA: Fleg e torque_sens
cfg.legNamesOUT = {'FL','ML','RL','FR','MR','RR'};
cfg.out2can = [1 3 5 2 4 6];
cfg.can2out = [1 4 2 5 3 6];

%cfg.jointIndex = @(i,j) 3*(cfg.can2mux(i)-1) + j;

cfg.jointIN  = @(i,j) 3*(cfg.can2mux(i)-1) + j;   % vettore verso i giunti (Mux ingresso)
cfg.jointOUT = @(i,j) 3*(cfg.can2out(i)-1) + j;   % vettori dai To Workspace (Mux uscita)

%% ===== geometria della gamba =====
cfg.lc = 0.054;    % [URDF] coxa,   da j_c2_* xyz = [0 -0.054 0]
cfg.lf = 0.0661;   % [URDF] femore, da j_tibia_* xyz = [0 -0.0645 -0.0145]
                   %        norm([0.0645 0.0145]) = 0.066112

% Posizione della sfera di contatto nel frame della tibia
cfg.foot_xyz    = [0.03, -0.15, 0];              % [MODELLO]
cfg.foot_offset = norm(cfg.foot_xyz(1:2));       % 0.152971 m
cfg.lt          = cfg.foot_offset;               % tibia vista dall'IK

% Punta geometrica della tibia, misurata sulla mesh tibia_l.STL
cfg.lt_mesh = norm([0.15985, 0.02879]);          % [MESH] 0.162422 m
cfg.contact.foot_r = 0.01;

cfg.reachMax = cfg.lf + cfg.lt;                  % 0.219071 m

%% ===== posizione delle anche nel frame corpo, ordine CAN =====
cfg.p_hip = [ ...
     0.1248     0.1248     0.0      0.0     -0.1248    -0.1248 ; ...
     0.06164   -0.06164    0.1034  -0.1034   0.06164   -0.06164 ; ...
     0.001116   0.001116   0.001116 0.001116 0.001116   0.001116 ];   % [URDF]

cfg.alpha = deg2rad([45, -45, 90, -90, 135, -135]);   % [URDF] yaw di j_c1_* meno 90 gradi
cfg.side  = [ +1, -1, +1, -1, +1, -1 ];

%% ===== posa nominale =====
cfg.r_offset = 0.14;    % [TARATO] estensione radiale del piede dall'asse coxa
cfg.z0       = 0.14;    % [TARATO] profondita' d'appoggio sotto l'anca
cfg.z0_eff = 0.147344;  % [MISURATO] dall'equilibrio statico del modello:
                        % 0.155710 - foot_r + p_hip(3) + mg/(6k).

cfg.pf_nom = zeros(3,6);
for i = 1:6
    cfg.pf_nom(:,i) = cfg.p_hip(:,i) + ...
        [cfg.r_offset*cos(cfg.alpha(i)); cfg.r_offset*sin(cfg.alpha(i)); -cfg.z0];
end

%% ===== masse e inerzie =====
cfg.m_body = 0.975599;
cfg.m_link = 0.024357719;
cfg.m_foot = 1000 * (4/3)*pi*cfg.contact.foot_r^3;
cfg.mass   = cfg.m_body + 24*cfg.m_link + 6*cfg.m_foot;

% Inerzia del ROBOT INTERO visto come corpo rigido.
cfg.J       = diag([0.01444, 0.01751, 0.02889]);
cfg.I_body  = [3.557e-03, 5.154e-03, 8.565e-03];
cfg.I_c1    = [3.654e-06, 7.746e-06, 7.746e-06];
cfg.I_c2    = [3.654e-06, 3.654e-06, 3.654e-06];
cfg.I_thigh = [3.095e-06, 1.014e-05, 1.070e-05];
cfg.I_tibia = [2.081e-06, 3.004e-05, 3.050e-05];

%% ===== andatura a tripode =====
%       T = S / (beta_stance * v_nom)

cfg.phase       = [0; 1/2; 1/2; 0; 0; 1/2]; % definizione dell'andatura alternata

cfg.beta_stance = 0.50; % 0.50 e' l'unico duty che da' appoggio continuo con tre zampe
cfg.duty_swing  = 1 - cfg.beta_stance;

cfg.S           = 0.06;

cfg.H           = 0.05;
cfg.T           = 1.00;

cfg.T_stance    = cfg.beta_stance * cfg.T;
cfg.T_swing     = cfg.duty_swing  * cfg.T;
cfg.v_nom       = cfg.S / cfg.T_stance;

%% ===== T2: curva di velocita' =====
cfg.t2_fattori = [0.5 1.0 1.20 1.5 2.0];

cfg.t2_limite    = 1.20;              % [x] ultima cella con moto stabile
cfg.t2_confronto = [0.5 1.0 1.20];    % moto del corpo stabile
cfg.t2_fuori     = [1.5 2.0];         % fuori: casi non funzionanti

%% ===== contatto e terreno =====
cfg.contact.k = 1200;    % penetrazione statica 2.2 mm, in tripode 4.3 mm
cfg.contact.c = 50;      % 2*sqrt(k*mass/3)
cfg.contact.w = 2e-3;    % la forza sale su 2 mm invece di 1
cfg.contact.vcrit = 1e-2;   % [TARATO] regolarizzazione attrito, 10 mm/s

cfg.mu_plant = 0.9;
cfg.floor_dim = [8 8 0.1];    % bounding box della mesh
cfg.floor_off = [0 0 +cfg.floor_dim(3)/2];
cfg.floor_top = -cfg.floor_off(3) + cfg.floor_dim(3)/2;

cfg.body_z0_geom = cfg.z0_eff + cfg.contact.foot_r + cfg.floor_top - cfg.p_hip(3,1);
cfg.body_z0 = cfg.body_z0_geom - cfg.mass*cfg.g/(6*cfg.contact.k);
% cfg.body_z0 = cfg.body_z0_geom + 0.002;          % 0.148366

% Rampa di T4.
cfg.terreno.rampa_x_inizio = 0.66;         % [m, mondo] dove la superficie esce dal pavimento
cfg.terreno.rampa_gradi    = 8;            % salita verso +x
cfg.terreno.rampa_stl      = 'simscape\props\ProvaPianoImperfettoCube.stl';   % file originale del solido

% Dosso di T4D: salita, cima piana, discesa, poi di nuovo piano.
cfg.terreno.dosso = struct('x_inizio', 0.66, 'H', 0.14, 'gradi_su', 8, ...
                           'gradi_giu', 8, 'L_cima', 0.6, 'larghezza', 2, ...
                           'spessore', 0.02);

%% ===== terreno per task =====
cfg.terreno.n_prop   = 7;
cfg.terreno.z_spento = +5;        % [m] quota di parcheggio
cfg.terreno.ost_dz = 0;           % [m]

cfg.terreno.ost_dx.T5 = [-0.5 0 0 0 0 0 0];  % [m] uno per ostacolo

cfg.terreno.task.T1 = [];         % piano
cfg.terreno.task.T2 = [];         % piano, curva di velocita'
cfg.terreno.task.T3 = [];         % piano, traiettoria curva
cfg.terreno.task.T4 = [];         % rampa (rampa_x_inizio, rampa_gradi)
cfg.terreno.task.T4D = [];        % dosso: salita, cima, discesa (terreno.dosso)
cfg.terreno.task.T5 = 1;          % ostacolo singolo
cfg.terreno.task.T6 = 1:7;        % ostacoli multipli

cfg.terreno.T6_x_fine = Inf;      % [m] Inf = run intera
cfg.terreno.task.T7 = [];         % piano, disturbo impulsivo

%% ===== attuatori =====
%  FONTE: docs/ROBOTIS_AX-12A_emanual.pdf
cfg.tau_max    = 1.5;       % [DATASHEET] stallo a 12 V
cfg.tau_lavoro = cfg.tau_max/5;   % [DATASHEET] carico per moto stabile, 1/5 dello stallo
cfg.qd_max  = 5.6548668;    % [URDF] il datasheet da' 6.18 rad/s a 12 V
cfg.q_min   = -2.6179939;   % [URDF]
cfg.q_max   =  2.6179939;   % [URDF]

%% ===== disturbo esterno =====
cfg.dist.t0    = 2.0;
cfg.dist.dur   = 0.2;
cfg.dist.dir   = [0; 1; 0];
cfg.dist.frac  = 0.25;
cfg.dist.point = [0; 0; 0];
cfg.dist.F     = cfg.dist.frac * cfg.mass * cfg.g;



%% ===== switch C1 / C2 =====
cfg.c2.attiva     = true;    % false = C1, anello aperto vero
cfg.c2.soglia_tau = [0.385 0.136 0.105];   % [N*m] coxa, femore, tibia

% Parametri della ricerca del terreno 
cfg.c2.z_nom     = cfg.z0;   % [m] profondita' oltre cui si considera
                             %     "fase di abbassamento".
cfg.c2.v_search  = 0.06;     % [m/s] velocita' di discesa in ricerca
cfg.c2.z_ext_max = 0.03;     % [m] estensione massima sotto la nominale
cfg.c2.t_reset   = 0.5;      % [s] nessuna ricerca nel transitorio iniziale
cfg.c2.tol       = 0.002;    % [m] tolleranza sulla soglia di abbassamento

%% ===== T3: imbardata =====
cfg.yaw_d = 0.1;             % [rad/s]

%% ===== controlli di coerenza =====
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



assert(abs(cfg.S - cfg.v_nom*cfg.T_stance) < 1e-12, 'phantomx:config:passo', ...
    'S deve valere v_nom*T_stance, altrimenti il piede striscia in appoggio.');

assert(abs(cfg.mass - 1.585317) < 1e-5, 'phantomx:config:massa', ...
    'La massa non torna con m_body + 24*m_link + 6*m_foot.');

assert(cfg.mass*cfg.g/(3*cfg.contact.k) > 2e-3, 'phantomx:config:contatto', ...
    ['Penetrazione in tripode %.2f mm: troppo poco per assorbire gli errori\n' ...
     'di assetto del corpo (misurati ~3.3 mm). Sotto i 2 mm il robot torna a\n' ...
     'camminare a una zampa alla volta.'], 1000*cfg.mass*cfg.g/(3*cfg.contact.k));

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
    fprintf('  attrito      modello %.2f\n\n', cfg.mu_plant);
end

end