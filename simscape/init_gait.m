%% init_gait.m - parametri di andatura e contatto, verifica e taratura giunti
%
%  Eseguire PRIMA della simulazione, o metterlo in
%  Modeling > Model Settings > Model Properties > Callbacks > InitFcn.
%
%  Nel blocco Constant "offset" scrivi   offset   (la variabile, non zeros(18,1)).
%  Nei Constant di T, S, H, z0 scrivi    gait.T   gait.S   gait.H   gait.z0
%  Nel Constant dello sfasamento scrivi  gait.T/2
%
%  DIFFERENZA RISPETTO ALLA VERSIONE PRECEDENTE
%  I numeri non sono piu' scritti qui: vengono da phantomx_config. I valori
%  sono gli stessi di prima, quindi la simulazione si comporta in modo
%  identico. Cambia solo che adesso esiste un posto solo dove modificarli,
%  e che una divergenza fra questo file e l'MPC non puo' piu' passare
%  inosservata.

clear gait

cfg = phantomx_config();

%% ====================================================================
%  INTERRUTTORE DI TARATURA
%  0 = simulazione normale (usa la cinematica inversa)
%  1 = giunti tutti a zero  -> mostra la posa di riferimento dell'URDF
%  2 = solo FEMORE  RR a -0.5 rad
%  3 = solo TIBIA   RR a -0.5 rad
%  4 = solo COXA    RR a -0.5 rad
%
%  Con TARATURA diverso da 0 la cinematica inversa viene annullata e ai
%  giunti arriva esattamente il vettore di prova. Non serve toccare lo
%  schema: si passa dallo stesso blocco "offset" che c'e' gia'.
%  ====================================================================
TARATURA = 0;

%% ---------------- Andatura ----------------
gait.T  = cfg.T;      % [s] periodo del ciclo completo
gait.S  = cfg.S;      % [m] lunghezza del passo
gait.H  = cfg.H;      % [m] altezza di sollevamento in volo
gait.z0 = cfg.z0;     % [m] profondita' di appoggio (POSITIVA = verso il basso)

%% ---------------- Geometria di contatto ----------------
foot_r     = cfg.contact.foot_r;   % [m] raggio sfera del piede
tibia_len  = cfg.foot_offset;      % [m] traslazione sfera in punta alla tibia
floor_dim  = cfg.floor_dim;        % [m] dimensioni del Brick Solid pavimento
floor_off  = cfg.floor_off;        % faccia superiore del pavimento a z = 0
body_z0    = cfg.body_z0;          % [m] quota iniziale del corpo (6-DOF Joint)

%% ---------------- Parametri del contatto ----------------
contact_k     = cfg.contact.k;       % [N/m]      rigidezza normale
contact_c     = cfg.contact.c;       % [N/(m/s)]  smorzamento
contact_w     = cfg.contact.w;       % [m]        transition region width
contact_vcrit = cfg.contact.vcrit;   % [m/s]      velocita' critica attrito

%% ---------------- Zampe: ordine del Mux, lato, montaggio ----------------
% L'ordine e' quello verificato sul Mux8: RR MR FR RL ML FL,
% e dentro ogni terna: coxa, femore, tibia.
% alpha e side vengono da cfg (ordine CAN) e sono riportati in ordine Mux
% tramite cfg.mux2can: nessun numero riscritto a mano.
zampe_mux = cfg.legNamesMUX;

alpha = struct(); side = struct();
for k = 1:6
    L = cfg.legNamesCAN{k};
    alpha.(L) = cfg.alpha(k);
    side.(L)  = cfg.side(k);
end

%% ---------------- Correzione permanente degli zeri ----------------
% Differenza costante fra lo zero dei giunti nell'URDF e lo zero che
% assume inv_kyn. La taratura ha detto che e' nulla: si lascia a zero.
% Ordine: [coxa femore tibia] per RR MR FR RL ML FL.
q_corr = zeros(18,1);

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
%  GUARDIA: inv_kyn e' dentro sei blocchi MATLAB Function e non puo'
%  chiamare phantomx_config a runtime, quindi le sue costanti restano
%  duplicate. Qui verifichiamo che non siano divergute, ricalcolando la
%  stessa posa con i valori di cfg e confrontando.
%  E' il controllo che avrebbe smascherato subito la tibia rimasta a 0.12.
%  ====================================================================
trueX_c = cfg.r_offset - cfg.lc;
im_c    = hypot(trueX_c, cfg.z0);
cphi_c  = max(-1, min(1, (cfg.lf^2 + im_c^2 - cfg.lt^2)/(2*im_c*cfg.lf)));
cpsi_c  = max(-1, min(1, (cfg.lf^2 + cfg.lt^2 - im_c^2)/(2*cfg.lf*cfg.lt)));
phi_cfg = atan2(cfg.z0, trueX_c) - acos(cphi_c);
psi_cfg = pi/2 - acos(cpsi_c);

[~, phi_ik, psi_ik] = inv_kyn(0, 0, cfg.z0, +1, cfg.alpha(1));

if abs(phi_ik - phi_cfg) > 1e-6 || abs(psi_ik - psi_cfg) > 1e-6
    warning('phantomx:initgait:divergenza', ...
       ['inv_kyn NON e'' allineata a phantomx_config.\n' ...
        '  phi:  inv_kyn %+8.3f deg   cfg %+8.3f deg\n' ...
        '  psi:  inv_kyn %+8.3f deg   cfg %+8.3f deg\n' ...
        'Controlla r_offset, lc, lf, lt dentro inv_kyn.m: devono valere\n' ...
        '%.5f  %.4f  %.4f  %.4f.'], ...
        rad2deg(phi_ik), rad2deg(phi_cfg), rad2deg(psi_ik), rad2deg(psi_cfg), ...
        cfg.r_offset, cfg.lc, cfg.lf, cfg.lt);
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