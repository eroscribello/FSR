function fig_plot_robot_d(Xd,Ud,p)
% Disegno della posa DESIDERATA (corpo trasparente + GRF di riferimento).
% Le gambe non vengono disegnate (come nell'originale).

nLeg = p.nLeg;
h    = p.h;
body_color = p.body_color;

%% unpack
pcom = reshape(Xd(1:3),[3,1]);
R    = reshape(Xd(7:15),[3,3]);

idx_pf = 18 + (1:3*nLeg);
pf = reshape(Xd(idx_pf),[3,nLeg]);
f  = reshape(Ud,[3,nLeg]);

%% ---- corpo trasparente: prisma esagonale ----
perim  = [1 3 5 6 4 2];
hip_lo = p.p_hip(:,perim);
hip_up = hip_lo + repmat([0;0;h],[1,nLeg]);

P_lo = R*hip_lo + repmat(pcom,[1,nLeg]);
P_up = R*hip_up + repmat(pcom,[1,nLeg]);

f1 = fill3(P_lo(1,:),P_lo(2,:),P_lo(3,:),body_color);  alpha(f1,0.2)
f2 = fill3(P_up(1,:),P_up(2,:),P_up(3,:),body_color);  alpha(f2,0.2)
for k = 1:nLeg
    k2 = mod(k,nLeg) + 1;
    face = [P_lo(:,k), P_lo(:,k2), P_up(:,k2), P_up(:,k)];
    fk = fill3(face(1,:),face(2,:),face(3,:),body_color);
    alpha(fk,0.2)
end

%% ---- GRF di riferimento ----
if isfield(p,'forceScale'), scale = p.forceScale; else, scale = 2e-2; end
for i_leg = 1:nLeg
    chain_f = [pf(:,i_leg), pf(:,i_leg) + scale * f(:,i_leg)];
    plot3(chain_f(1,:),chain_f(2,:),chain_f(3,:),'g','linewidth',1.5)
end

end