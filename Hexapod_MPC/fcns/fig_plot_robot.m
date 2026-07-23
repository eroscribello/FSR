function fig_plot_robot(Xt,Ut,Ue,p)
% Disegno del robot ESAPODE (PhantomX).
% - corpo: esagono costruito dalle vere posizioni delle anche (p.p_hip)
% - gambe: cinematica PhantomX (coxa in yaw + femore + tibia)
% - GRF: una freccia per piede
%
% NB: non usa piu' fcn_invKin3 (era tarato su layout rettangolare + ABAD).

nLeg = p.nLeg;
h    = p.h;

body_color   = p.body_color;
leg_color    = p.leg_color;
ground_color = p.ground_color;

%% unpack
% X = [pc dpc vR wb pf]'
pcom = reshape(Xt(1:3),[3,1]);
R    = reshape(Xt(7:15),[3,3]);

idx_pf = 18 + (1:3*nLeg);              % 19:36 per nLeg=6
pf = reshape(Xt(idx_pf),[3,nLeg]);     % piedi in terna mondo
f  = reshape(Ut,[3,nLeg]);             % GRF (era [3,4])

%% ---- corpo: prisma esagonale ----
% giro attorno al perimetro: lf -> lm -> lr -> rr -> rm -> rf
perim  = [1 3 5 6 4 2];
hip_lo = p.p_hip(:,perim);                       % faccia inferiore (z=0)
hip_up = hip_lo + repmat([0;0;h],[1,nLeg]);      % faccia superiore

P_lo = R*hip_lo + repmat(pcom,[1,nLeg]);
P_up = R*hip_up + repmat(pcom,[1,nLeg]);

fill3(P_lo(1,:),P_lo(2,:),P_lo(3,:),body_color)
fill3(P_up(1,:),P_up(2,:),P_up(3,:),body_color)
for k = 1:nLeg
    k2 = mod(k,nLeg) + 1;
    face = [P_lo(:,k), P_lo(:,k2), P_up(:,k2), P_up(:,k)];
    fill3(face(1,:),face(2,:),face(3,:),body_color)
end

%% ---- gambe + piedi ----
for i_leg = 1:nLeg
    chain = legChain(pcom,R,p.p_hip(:,i_leg),pf(:,i_leg),p);
    plot3(chain(1,:),chain(2,:),chain(3,:),'linewidth',2.5,'color',leg_color)
    plot3(pf(1,i_leg),pf(2,i_leg),pf(3,i_leg),'o',...
          'MarkerFaceColor',leg_color,'MarkerEdgeColor',leg_color)
end

%% ---- GRF ----
% scala tarata sul PhantomX (forze ~5 N per piede, non ~13 N del quadrupede)
if isfield(p,'forceScale'), scale = p.forceScale; else, scale = 2e-2; end
for i_leg = 1:nLeg
    chain_f = [pf(:,i_leg), pf(:,i_leg) + scale * f(:,i_leg)];
    plot3(chain_f(1,:),chain_f(2,:),chain_f(3,:),'r','linewidth',1.5)
end

%% ---- forza esterna ----
if isfield(p,'p_ext')
    p_ext_R = R * p.p_ext + pcom;
    chain_Ue = [p_ext_R, p_ext_R + scale * reshape(Ue,[3,1])];
    plot3(chain_Ue(1,:),chain_Ue(2,:),chain_Ue(3,:),'c','linewidth',1.5)
end

%% ---- terreno ----
goffset = 0.3;      % ridotto: il PhantomX e' piccolo (era 0.5)
chain0 = [[pcom(1)-goffset pcom(2)+goffset 0]',...
          [pcom(1)+goffset pcom(2)+goffset 0]',...
          [pcom(1)+goffset pcom(2)-goffset 0]',...
          [pcom(1)-goffset pcom(2)-goffset 0]'];
fill3(chain0(1,:),chain0(2,:),chain0(3,:),ground_color)

end


function chain = legChain(pcom,R,p_hip_b,p_foot_w,p)
% Catena anca -> fine coxa -> ginocchio -> piede, in terna mondo.
% Cinematica PhantomX: coxa ruota in yaw, femore e tibia nel piano verticale.
d  = p.d;      % lunghezza coxa
l1 = p.l1;     % femore
l2 = p.l2;     % tibia

% vettore anca->piede espresso in terna corpo
v = R'*(p_foot_w - pcom) - p_hip_b;

yaw = atan2(v(2),v(1));
u   = [cos(yaw); sin(yaw); 0];       % direzione radiale della gamba

a = norm(v(1:2)) - d;                % avanzamento orizzontale oltre la coxa
b = v(3);                            % quota (negativa verso il basso)

% IK planare a 2 link, con clamp per evitare acos di argomenti fuori range
Lr   = hypot(a,b);
Lr   = min(max(Lr, abs(l1-l2)+1e-4), l1+l2-1e-4);
cosA = (Lr^2 + l1^2 - l2^2) / (2*Lr*l1);
cosA = min(max(cosA,-1),1);
ang  = atan2(b,a) + acos(cosA);      % inclinazione del femore

p_coxa_b = p_hip_b + d*u;
p_knee_b = p_coxa_b + l1*(cos(ang)*u + [0;0;sin(ang)]);

P = R*[p_hip_b, p_coxa_b, p_knee_b] + repmat(pcom,[1,3]);
chain = [P, p_foot_w];
end