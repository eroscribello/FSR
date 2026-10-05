%% init_gait.m - parametri di andatura e contatto, verifica e taratura giunti

clear gait

cfg = phantomx_config();

%% ====================================================================
%  INTERRUTTORE DI TARATURA
%  0 = simulazione normale (usa la cinematica inversa)
%  1 = giunti tutti a zero  -> mostra la posa di riferimento dell'URDF
%  2 = solo FEMORE  RR a -0.5 rad
%  3 = solo TIBIA   RR a -0.5 rad
%  4 = solo COXA    RR a -0.5 rad
%  ====================================================================
TARATURA = 0;

%% ---------------- Andatura ----------------
gait.T  = cfg.T;      % [s] periodo del ciclo completo
gait.S  = cfg.S;      % [m] lunghezza del passo
gait.H  = cfg.H;      % [m] altezza di sollevamento in volo
gait.z0 = cfg.z0;     % [m] profondita' di appoggio (POSITIVA = verso il basso)
gait.duty = cfg.duty_swing;

%% ---------------- Geometria di contatto ----------------
foot_r     = cfg.contact.foot_r;   % [m] raggio sfera del piede
tibia_len  = cfg.foot_offset;      % [m] traslazione sfera in punta alla tibia
floor_dim  = cfg.floor_dim;        % [m] dimensioni del Brick Solid pavimento
floor_off  = cfg.floor_off;        % faccia superiore del pavimento a z = 0
body_z0    = cfg.body_z0;          % [m] quota iniziale del corpo (6-DOF Joint)
foot_xyz = cfg.foot_xyz;

%% ---------------- Parametri del contatto ----------------
contact_k     = cfg.contact.k;       % [N/m]      rigidezza normale
contact_c     = cfg.contact.c;       % [N/(m/s)]  smorzamento
contact_w     = cfg.contact.w;       % [m]        transition region width
contact_vcrit = cfg.contact.vcrit;   % [m/s]      velocita' critica attrito

%% ---------------- Zampe: ordine del Mux, lato, montaggio ----------------
% L'ordine e' quello verificato sul Mux8: RR MR FR RL ML FL
zampe_mux = cfg.legNamesMUX;

alpha = struct(); side = struct();
for k = 1:6
    L = cfg.legNamesCAN{k};
    alpha.(L) = cfg.alpha(k);
    side.(L)  = cfg.side(k);
end

%% ---------------- Correzione permanente degli zeri ----------------
% Differenza costante fra lo zero dei giunti nell'URDF e lo zero che
% assume inv_kyn
q_corr = zeros(18,1);

%% ---------------- posa iniziale dei giunti ----------------
q0 = zeros(18,1);
for k = 1:6
    L = zampe_mux{k};
    [th, ph, ps] = inv_kyn(0, 0, gait.z0, side.(L), alpha.(L));
    q0(3*k-2 : 3*k) = [th; ph; ps];
end

%% ====================================================================
%  Costruzione del vettore "offset" che il modello sommera' alla IK
%  ====================================================================
if TARATURA == 0
    offset = q_corr;                       % funzionamento normale
else
    gait.S = 0;                            % la taratura richiede posa fissa
    gait.H = 0;

    % uscita della cinematica inversa a comando nullo (x=0, y=0, z=z0)
    q_ik0 = zeros(18,1);
    for k = 1:6
        L = zampe_mux{k};
        [th, ph, ps] = inv_kyn(0, 0, gait.z0, side.(L), alpha.(L));
        q_ik0(3*k-2 : 3*k) = [th; ph; ps];
    end

    q_test = zeros(18,1);
    switch TARATURA
        case 2, q_test(2) = -0.5;          % femore RR
        case 3, q_test(3) = -0.5;          % tibia  RR
        case 4, q_test(1) = -0.5;          % coxa   RR
    end

    offset = q_test - q_ik0;               % ai giunti arriva esattamente q_test
end



%% ====================================================================
TOLL = 1e-4;

trueX_c = cfg.r_offset - cfg.lc;
im_c    = hypot(trueX_c, cfg.z0);
cphi_c  = max(-1, min(1, (cfg.lf^2 + im_c^2 - cfg.lt^2)/(2*im_c*cfg.lf)));
cpsi_c  = max(-1, min(1, (cfg.lf^2 + cfg.lt^2 - im_c^2)/(2*cfg.lf*cfg.lt)));
phi_cfg = atan2(cfg.z0, trueX_c) - acos(cphi_c);
psi_cfg = -(pi/2 - acos(cpsi_c));

[~, phi_ik, psi_ik] = inv_kyn(0, 0, cfg.z0, +1, cfg.alpha(1));

if abs(phi_ik - phi_cfg) > TOLL || abs(psi_ik - psi_cfg) > TOLL
    warning('phantomx:initgait:divergenza', ...
       ['inv_kyn NON e'' allineata a phantomx_config.\n' ...
        '  phi:  inv_kyn %+9.4f deg   cfg %+9.4f deg   (scarto %.1e rad)\n' ...
        '  psi:  inv_kyn %+9.4f deg   cfg %+9.4f deg   (scarto %.1e rad)\n' ...
        'Controlla r_offset, lc, lf, lt dentro inv_kyn.m: devono valere\n' ...
        '%.8f  %.6f  %.6f  %.8f.'], ...
        rad2deg(phi_ik), rad2deg(phi_cfg), abs(phi_ik-phi_cfg), ...
        rad2deg(psi_ik), rad2deg(psi_cfg), abs(psi_ik-psi_cfg), ...
        cfg.r_offset, cfg.lc, cfg.lf, cfg.lt);
end

%% ---- oggetto rigidBodyTree per il blocco Inverse Dynamics ----
if ~exist('robotModel','var')
    Setup_robot_object;
end

%% ---- interruttore per le prove statiche ----
if exist('FORZA_STATICO','var') && FORZA_STATICO
    gait.S = 0;
    gait.H = 0;
    fprintf('\n*** PROVA STATICA: gait.S e gait.H forzati a zero ***\n');
end

%% ---- override per la campagna ----
if exist('OVERRIDE_GAIT','var') && isstruct(OVERRIDE_GAIT)
    ovNomi = fieldnames(OVERRIDE_GAIT);
    for ovI = 1:numel(ovNomi)
        gait.(ovNomi{ovI}) = OVERRIDE_GAIT.(ovNomi{ovI});
        fprintf('  [override] gait.%s = %g\n', ovNomi{ovI}, OVERRIDE_GAIT.(ovNomi{ovI}));
    end
    clear ovNomi ovI
end

%% ---- interruttore C1 / C2 ----
if exist('OVERRIDE_C2','var') && ~isempty(OVERRIDE_C2)
    cfg.c2.attiva = logical(OVERRIDE_C2);
    fprintf('  [override] cfg.c2.attiva = %d\n', cfg.c2.attiva);
end

if exist('OVERRIDE_C2_SOGLIA','var') && ~isempty(OVERRIDE_C2_SOGLIA)
    cfg.c2.soglia_tau = OVERRIDE_C2_SOGLIA;
    fprintf('  [override] cfg.c2.soglia_tau = %s N*m\n', mat2str(cfg.c2.soglia_tau, 4));
end

if cfg.c2.attiva
    c2_soglia = cfg.c2.soglia_tau;
else
    c2_soglia = inf(1,3);    % il flag di contatto non scatta mai
end

if isscalar(c2_soglia), c2_soglia = repmat(c2_soglia, 1, 3); end
c2_soglia = reshape(c2_soglia, 1, []);
if numel(c2_soglia) ~= 3
    error('init_gait:soglia', ['c2_soglia ha %d elementi, ne servono 3 ' ...
        '(coxa, femore, tibia).'], numel(c2_soglia));
end

%% ---- quota degli ostacoli ----
if ~exist('ost_dz','var') || isempty(ost_dz)
    ost_dz = cfg.terreno.ost_dz;
end

c2_par = [double(cfg.c2.attiva), ...
          cfg.c2.z_nom, ...
          cfg.c2.v_search, ...
          cfg.c2.z_ext_max, ...
          cfg.c2.t_reset, ...
          cfg.c2.tol];

if cfg.c2.attiva && abs(c2_par(2) - gait.z0) > cfg.c2.tol
    fprintf(2, ['  [c2] z_nom = %.4f ma gait.z0 = %.4f: la soglia di ricerca\n' ...
                '       non e'' allineata alla profondita'' comandata.\n'], ...
            c2_par(2), gait.z0);
end

%% ====================================================================
%  Stampa di controllo
%  ====================================================================
if TARATURA == 0
    fprintf('\n=== MODO NORMALE - cinematica inversa attiva ===\n');
    fprintf('\n--- Postura statica (x=0, y=0, z=%.3f) ---\n', gait.z0);
    fprintf('%-4s %10s %10s %10s\n','zampa','theta[deg]','phi[deg]','psi[deg]');
    for k = 1:6
        L = zampe_mux{k};
        [th, ph, ps] = inv_kyn(0, 0, gait.z0, side.(L), alpha.(L));
        fprintf('%-4s %10.2f %10.2f %10.2f\n', L, rad2deg(th), rad2deg(ph), rad2deg(ps));
    end

    fprintf('\n--- Comando "avanti" (x=+S/2): i lati devono avere theta OPPOSTI ---\n');
    for k = 1:6
        L = zampe_mux{k};
        [th, ~, ~] = inv_kyn(gait.S/2, 0, gait.z0, side.(L), alpha.(L));
        fprintf('%-4s theta = %+7.2f deg\n', L, rad2deg(th));
    end

    fprintf('\n--- Andatura ---\n');
    fprintf('T %.2f s   appoggio %.2f s   volo %.2f s   passo %.3f m\n', ...
            gait.T, cfg.T_stance, cfg.T_swing, gait.S);
    fprintf('velocita'' attesa: %.4f m/s   (S / T_stance)\n', cfg.v_nom);

    fprintf('\n--- Contatto ---\n');
    pen = 1000*(cfg.mass*cfg.g/3)/contact_k;
    fprintf('penetrazione statica stimata: %.2f mm  (transition region: %.2f mm)\n', ...
            pen, 1000*contact_w);
    fprintf('smorzamento critico stimato:  %.0f N/(m/s)  -> impostato %g\n', ...
            2*sqrt(contact_k*cfg.mass/3), contact_c);

    fprintf('\n--- Quota iniziale ---\n');
    caduta = 1000*(body_z0 - cfg.body_z0_geom);
    fprintf('body_z0 %.3f m   (geometrica %.3f m)\n', body_z0, cfg.body_z0_geom);
    if caduta > 5
        fprintf(2,'i piedi partono %.0f mm sopra il pavimento: impatto a %.2f m/s\n', ...
                caduta, sqrt(2*cfg.g*caduta/1000));
    end

    fprintf('duty (volo) %.2f   -> appoggio %.2f s, volo %.2f s\n', ...
        gait.duty, (1-gait.duty)*gait.T, gait.duty*gait.T);
    fprintf('\n');

else
    descr = {'giunti tutti a ZERO (posa di riferimento URDF)', ...
             'solo FEMORE RR a -0.5 rad', ...
             'solo TIBIA  RR a -0.5 rad', ...
             'solo COXA   RR a -0.5 rad'};
    fprintf('\n=== MODO TARATURA %d: %s ===\n', TARATURA, descr{TARATURA});
    fprintf('gait.S e gait.H forzati a 0. La cinematica inversa e'' annullata.\n');
    fprintf('Verifica che il blocco Constant "offset" contenga la variabile  offset.\n\n');
    fprintf('%-4s %10s %10s %10s\n','zampa','coxa[rad]','femore[rad]','tibia[rad]');
    for k = 1:6
        q = q_test(3*k-2 : 3*k);
        fprintf('%-4s %10.3f %10.3f %10.3f\n', zampe_mux{k}, q(1), q(2), q(3));
    end
    fprintf('\nSimula 2 secondi e salva una VISTA LATERALE.\n\n');
end
