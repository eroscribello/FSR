function dXdt = dynamics_SRB(t,Xt,Ut,Xd,U_ext,p)
% Modello a singolo corpo rigido (SRB) — esteso a nLeg zampe.
% Stato:  X = [pc; dpc; vec(R); wb; pf]   ->  18 + 3*nLeg  (=36 per l'esapode)
% Ingresso: Ut = forze di contatto ai piedi (3*nLeg)

%% parameters
mass = p.mass;
J    = p.J;        % tensore d'inerzia in terna corpo {B}
g    = 9.81;
nLeg = p.nLeg;     % <-- 6 per l'esapode (era implicitamente 4)

%% decompose
% X = [pc dpc vR wb pf]'
pc  = reshape(Xt(1:3),[3,1]);
dpc = reshape(Xt(4:6),[3,1]);
R   = reshape(Xt(7:15),[3,3]);
wb  = reshape(Xt(16:18),[3,1]);

idx_pf = 18 + (1:3*nLeg);          % 19:36 con nLeg=6 (era 19:30)
pf  = reshape(Xt(idx_pf),[3,nLeg]);    % era reshape(Xt(19:30),[3,4])
pfd = reshape(Xd(idx_pf),[3,nLeg]);    % era reshape(Xd(19:30),[3,4])

% braccio dal baricentro a ogni piede
r = pf - repmat(pc,[1,nLeg]);          % era repmat(pc,[1,4])

% GRF (una forza 3x1 per piede)
f = reshape(Ut,[3,nLeg]);              % era reshape(Ut,[3,4])

%% dynamics
% --- traslazione (Newton): somma delle GRF + gravita' ---
ddpc = 1/mass * (sum(f,2) + U_ext) + [0;0;-g];

% --- orientamento: Rdot = R * [wb]x ---
dR = R * hatMap(wb);

% --- coppia attorno al baricentro: somma su TUTTI i piedi ---
tau_s = zeros(3,1);      % coppia del corpo espressa in {S} (mondo)
for ii = 1:nLeg          % era for ii = 1:4
    tau_s = tau_s + hatMap(r(:,ii)) * f(:,ii);
end
tau_ext = hatMap(R * p.p_ext) * U_ext;
tau_tot = tau_s + tau_ext;

% --- Eulero in terna corpo ---
dwb = J \ (R' * tau_tot - hatMap(wb) * J * wb);

% --- piedi: inseguitore proporzionale verso il riferimento ---
dpf = p.Kp_sw * (pfd(:) - pf(:));

dXdt = [dpc;ddpc;dR(:);dwb;dpf];

end