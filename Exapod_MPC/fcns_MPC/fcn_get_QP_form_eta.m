function [H,g,Aineq,bineq,Aeq,beq] = fcn_get_QP_form_eta(Xt,Ut,Xd,Ud,p)
% min. 0.5 * x' * H *x + g' * x
% s.t. Aineq *x <= bineq
%      Aeq * x <= beq
% Xt = [pc dpc vR wb pf]': [36,1]  <-- 18 CoM + 18 per i 6 piedi
% q = [pc dpc eta wb]: [12 1]      <-- Lo stato ridotto rimane 12
% lb/ub - [6,n_hor]                 <-- Forze verticali per 6 zampe
%% parameters
mu = p.mu;
n_hor = p.predHorizon;
Umax = p.Umax;
decayRate = p.decayRate;
R = p.R;   % Ora è 18x18 (assegnata in get_params)
Q = p.Q;
Qf = p.Qf;
[Qx,Qv,Qeta,Qw] = deal(Q(1:3,1:3),Q(4:6,4:6),Q(7:9,7:9),Q(10:12,10:12));
[Qxf,Qvf,Qetaf,Qwf] = deal(Qf(1:3,1:3),Qf(4:6,4:6),Qf(7:9,7:9),Qf(10:12,10:12));
nX = 12;   
nU = 18;   % --- MODIFICA 1: Ingressi a 18 (6 zampe x 3 forze) ---

%%%%%%% A,B,d matrices for linear dynamics %%%%%%%%%%%
[A,B,d] = fcn_get_ABD_eta(Xt,Ut,p);

%% Decompose
Rt = reshape(Xt(7:15,1),[3,3]);
qt = [Xt(1:6);[0;0;0];Xt(16:18)];

% lb <= Fz <= ub
% --- MODIFICA 2: Fz desiderate per 6 zampe ---
Fzd = Ud([3 6 9 12 15 18],:); 
lb = -1 * Fzd;
ub = 2 * Fzd;

%% Matrices for QP
H = zeros((nX + nU) * n_hor);
g = zeros(size(H,1),1);
Aeq = zeros(nX * n_hor,(nX+nU) * n_hor);
beq = zeros(size(Aeq,1),1);

if p.gait == -2
    Aineq_unit = [1 0 0;-1 0 0;0 1 0;0 -1 0;0 0 1;0 0 -1];
else
    Aineq_unit = [1 0 -mu;-1 0 -mu;0 1 -mu;0 -1 -mu;0 0 1; 0 0 -1];
end
nAineq_unit = size(Aineq_unit,1);

% --- MODIFICA 3: Spazio per i vincoli di 6 zampe (6 * nAineq_unit per passo dell'orizzonte) ---
Aineq = zeros(6*nAineq_unit*n_hor,(nX+nU)*n_hor);
bineq = zeros(size(Aineq,1),1);

for i_hor = 1:n_hor
    xd = Xd(1:3,i_hor);
    vd = Xd(4:6,i_hor);
    Rd = reshape(Xd(7:15,i_hor),[3,3]);
    wd = Xd(16:18,i_hor);
    
    %% Objective function
    idx_u = (i_hor-1) * (nX + nU) + (1:nU);
    idx_x = (i_hor-1) * (nX + nU) + nU + (1:nX);
    if i_hor == n_hor
        H(idx_x,idx_x) = Qf * decayRate^(i_hor-1);
        g(idx_x) = [-Qxf * xd;
                    -Qvf * vd;
                     Qetaf * veeMap(logm(Rd' * Rt));
                    -Qwf * wd] * decayRate^(i_hor-1);
    else
        H(idx_x,idx_x) = Q * decayRate^(i_hor-1);
        g(idx_x) = [-Qx * xd;
                    -Qv * vd;
                     Qeta * veeMap(logm(Rd' * Rt));
                    -Qw * wd] * decayRate^(i_hor-1);
    end
    H(idx_u,idx_u) = R * decayRate^(i_hor-1);
    g(idx_u) = R' * (Ut - Ud(:,i_hor)) * decayRate^(i_hor-1);
                
    %% Equality constraints
    if i_hor == 1
        Aeq(1:nX,1:(nU+nX)) = [-B,eye(nX)];
        beq(1:nX) = A * qt + d;
    else
        Aeq((i_hor-1)*nX+(1:nX),(i_hor-2)*(nX+nU)+nU+(1:(2*nX+nU)))= [-A -B eye(nX)];
        beq((i_hor-1)*nX+(1:nX)) = d;
    end
    
    %% Inequality constraints
    % --- MODIFICA 4: Matrice Fi dimensionata 36x18 ---
    Fi = zeros(6*nAineq_unit,18);
    hi = zeros(size(Fi,1),1);
    for i_leg = 1:6 % --- MODIFICA 5: Loop esteso a 6 zampe ---
        idx_F = (i_leg-1)*nAineq_unit + (1:nAineq_unit);
        idx_u = (i_leg-1)*3 + (1:3);
        Fi(idx_F,idx_u) = Aineq_unit;
        if p.gait == -2
            hi(idx_F) = [Umax-Ut(idx_u(1));Umax+Ut(idx_u(1));...
                         Umax-Ut(idx_u(2));Umax+Ut(idx_u(2));...
                         Umax-Ut(idx_u(3));Umax+Ut(idx_u(3))];
        else
            hi(idx_F) = [mu*Ut(idx_u(3))-Ut(idx_u(1));
                         mu*Ut(idx_u(3))+Ut(idx_u(1));
                         mu*Ut(idx_u(3))-Ut(idx_u(2));
                         mu*Ut(idx_u(3))+Ut(idx_u(2));
                         ub(i_leg,i_hor)-Ut(idx_u(3))+Ud(idx_u(3),i_hor);
                        -lb(i_leg,i_hor)+Ut(idx_u(3))-Ud(idx_u(3),i_hor)];
        end
    end
    % --- MODIFICA 6: Assegnazione finale con indici corretti ---
    idx_A = (i_hor-1) * 6*nAineq_unit + (1:6*nAineq_unit);
    idx_z = (i_hor-1) * (nX+nU) + (1:nU);
    Aineq(idx_A,idx_z) = Fi;
    bineq(idx_A) = hi;
end
end