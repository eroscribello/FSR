function [theta, phi, psi] = inv_kyn(x, y, z, side, alpha)
%INV_KYN  Cinematica inversa di una zampa PhantomX (coxa - femore - tibia).
%#codegen
%
%  ------------------------------------------------------------------
%  CONVENZIONI  (identiche per tutte e sei le zampe: NON modificare i
%  segni caso per caso, il parametro "side" esiste apposta)
%  ------------------------------------------------------------------
%   x      [m]   comando piede lungo l'asse X del corpo   (+ = AVANTI)
%   y      [m]   comando piede lungo l'asse Y del corpo   (+ = SINISTRA)
%   z      [m]   profondita' del piede sotto il corpo     (+ = VERSO IL BASSO)
%   side   [-]   +1 zampe sinistre (FL, ML, RL)
%                -1 zampe destre   (FR, MR, RR)
%   alpha  [rad] angolo di montaggio della zampa rispetto a +X del corpo
%                FL=+45  ML=+90  RL=+135  FR=-45  MR=-90  RR=-135  (gradi)
%
%   theta  [rad] angolo giunto coxa    (j_c1_*)
%   phi    [rad] angolo giunto femore  (j_thigh_*)
%   psi    [rad] angolo giunto tibia   (j_tibia_*)
%
%  ATTENZIONE: z e' POSITIVO VERSO IL BASSO. Il generatore di traiettoria
%  deve quindi usare z0 positivo e ALZARE il piede SOTTRAENDO l'altezza di
%  volo:  z = z0 - H*sin(...).
%
%  ------------------------------------------------------------------
%  PERCHE' LE COSTANTI SONO DUPLICATE QUI
%  ------------------------------------------------------------------
%  Questa funzione gira dentro sei blocchi MATLAB Function e non puo'
%  chiamare phantomx_config a runtime. Le costanti restano quindi duplicate,
%  ma NON in silenzio: init_gait ricalcola la posa con i valori di cfg e la
%  confronta con l'uscita di questa funzione, segnalando ogni divergenza.
%  Se cambi un valore qui, cambialo anche in phantomx_config.m.

    % ---------- Geometria dei link [m] ----------
    r_offset = 0.14;       % estensione radiale a riposo   (era 0.12)
    lc       = 0.054;      % coxa                          [URDF]
    lf       = 0.0661;     % femore                        [URDF]
    lt       = 0.15; % era 0.152971: l'arrotondamento faceva scattare
                            % la guardia di init_gait

    % ---------- 1. Dal frame CORPO al frame ZAMPA: R_z(-alpha) ----------
    % componente radiale (lungo l'asse della zampa)
    x_loc = r_offset + x*cos(alpha) + y*sin(alpha);
    % componente tangenziale, specchiata sul lato destro
    y_loc = (-x*sin(alpha) + y*cos(alpha)) * side;
    % componente verticale
    z_loc = z;

    % ---------- 2. Giunto COXA ----------
    % un'unica formula per tutte le zampe: la simmetria destra/sinistra
    % e' gia' gestita da "side".
    theta = atan2(y_loc, x_loc) * side;

    % ---------- 3. Piano femore/tibia (2R planare) ----------
    trueX = sqrt(x_loc*x_loc + y_loc*y_loc) - lc;  % distanza orizzontale femore-piede
    im    = sqrt(trueX*trueX + z_loc*z_loc);       % distanza diretta   femore-piede

    % Femore
    c_phi = (lf*lf + im*im - lt*lt) / (2*im*lf);
    c_phi = max(-1.0, min(1.0, c_phi));            % clamp anti-NaN (fuori portata)
    phi   = atan2(z_loc, trueX) - acos(c_phi);

    % Tibia
    c_psi = (lf*lf + lt*lt - im*im) / (2*lf*lt);
    c_psi = max(-1.0, min(1.0, c_psi));            % clamp anti-NaN
    psi   = -(pi/2 - acos(c_psi));
end