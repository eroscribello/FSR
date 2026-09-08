function cfg = phantomx_config(verbose)
%PHANTOMX_CONFIG  Unica fonte di verita' dei parametri del PhantomX.
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
cfg.lc          = 0.054;     % [URDF] coxa
cfg.lf          = 0.0661;    % [URDF] femore
cfg.lt          = 0.16;      % [MODELLO] Tibia a 16 cm (fissata per tutti i codici)
cfg.foot_offset = 0.16;      % [MODELLO] Coincide esattamente con lt
cfg.lt_mesh     = 0.15297;   % [MESH] Geometria STL originale (non usata)
cfg.reachMax    = cfg.lf + cfg.lt; % 0.2261 m

%% ===== posizione delle anche nel frame corpo, ordine CAN =====
cfg.p_hip = [ ...
     0.1248     0.1248     0.0      0.0     -0.1248    -0.1248 ; ...
     0.06164   -0.06164    0.1034  -0.1034   0.06164   -0.06164 ; ...
     0.001116   0.001116   0.001116 0.001116 0.001116   0.001116 ];

cfg.alpha = deg2rad([45, -45,  90, -90, 135, -135]);
cfg.side  = [ +1,  -1,  +1,  -1,  +1,  -1 ];

%% ===== posa nominale =====
cfg.r_offset = 0.18;   % [TARATO] estensione radiale adattata alla tibia da 16 cm
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

cfg.J = diag([0.01444, 0.01751, 0.02889]);
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
cfg.mu_plant       = 0.9;   
cfg.mu_mpc         = 0.6;   
cfg.floor_dim      = [4 4 0.05];        
cfg.floor_off      = [0 0 -cfg.floor_dim(3)/2];   
cfg.floor_top      = cfg.floor_off(3) + cfg.floor_dim(3)/2;   

% QUOTA INIZIALE CORRETTA PER EVITARE LA CADUTA:
cfg.body_z0_geom = cfg.z0 + cfg.contact.foot_r + cfg.floor_top; % 0.150 m
cfg.body_z0      = cfg.body_z0_geom; 

%% ===== attuatori =====
cfg.tau_max = 1.5;          
cfg.qd_max  = 5.6548668;    
cfg.q_min   = -2.6179939;   
cfg.q_max   =  2.6179939;   

%% ===== disturbo esterno =====
cfg.dist.t0    = 2.0;                 
cfg.dist.dur   = 0.2;                 
cfg.dist.dir   = [0; 1; 0];           
cfg.dist.frac  = 0.25;                
cfg.dist.point = [0; 0; 0];           
cfg.dist.F     = cfg.dist.frac * cfg.mass * cfg.g;   

%% ===== controlli di coerenza =====
reach = norm([cfg.r_offset - cfg.lc, cfg.z0]) / cfg.reachMax;
assert(reach < 0.85, 'phantomx:config:reach', ...
    'La posa nominale supera l''85%% dell''estensione massima.');
assert(cfg.mu_mpc <= cfg.mu_plant, 'phantomx:config:mu', ...
    'L''MPC assume un attrito superiore al modello.');
assert(abs(cfg.S - cfg.v_nom*cfg.T_stance) < 1e-12, 'phantomx:config:passo', ...
    'S deve valere v_nom*T_stance.');
assert(abs(cfg.mass - 1.560184) < 1e-5, 'phantomx:config:massa', ...
    'La massa totale non torna.');
assert(abs(cfg.lt - cfg.foot_offset) < 1e-12, 'phantomx:config:tibia', ...
    'lt e foot_offset devono coincidere.');

if verbose
    fprintf('\n=== PHANTOMX CONFIG CARICATO ===\n');
    fprintf('Tibia: %.2f m | body_z0: %.3f m | reach: %.1f%%\n\n', ...
            cfg.lt, cfg.body_z0, 100*reach);
end
end
