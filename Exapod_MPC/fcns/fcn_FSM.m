function [FSMout,Xd,Ud,Xt] = fcn_FSM(t_,Xt,p)
%% parameters
[L,W,d] = deal(p.L,p.W,p.d);
gait = p.gait;
Tst_ = p.Tst;
Tst = min(Tst_,0.2/norm(Xt(4:5)));
Tsw = p.Tsw;
T = Tst + Tsw;
Tair = 1/2 * (Tsw - Tst);
[pc,dpc,vR,wb] = deal(Xt(1:3),Xt(4:6),Xt(7:15),Xt(16:18));
R = reshape(vR,[3,3]);

% --- MODIFICA 1: Indici per 6 zampe (18 elementi invece di 12) ---
idx_pf = 19:36; 
pf34 = reshape(Xt(idx_pf),[3,6]);

%% initialization
persistent FSM Ta Tb pf_R_trans
if isempty(FSM)
    % --- MODIFICA 2: Inizializzazione per 6 elementi ---
    FSM = zeros(6,1);
    Ta = zeros(6,1);
    Tb = ones(6,1);
    pf_R_trans = Xt(idx_pf);
end
t = t_(1);      % current time
s = zeros(6,1); % 6 zampe

%% FSM
% 1 - stance
% 2 - swing
for i_leg = 1:6 % --- MODIFICA 3: Ciclo esteso a 6 zampe ---
    s(i_leg) = (t - Ta(i_leg)) ./ (Tb(i_leg) - Ta(i_leg));
    s(s<0) = 0;
    s(s>1) = 1;
    % --- FSM ---
    if FSM(i_leg) == 0          % init to stance
        if gait == -1           % pose control
            Ta(i_leg) = 0;
            Tb(i_leg) = -1;
        elseif gait == 5        % crawl (esapode)
            % Sequenza sequenziale per 6 zampe
            Ta(1) = t;
            Ta(2) = t + Tsw;
            Ta(3) = t + Tsw*2;
            Ta(4) = t + Tsw*3;
            Ta(5) = t + Tsw*4;
            Ta(6) = t + Tsw*5;
            Tb(i_leg) = Ta(i_leg) + Tst;
        else                    % Tripod Gait (Default per esapode, ex trot)
            % Gruppo A: LF (1), RM (4), LH (5)
            % Gruppo B: RF (2), LM (3), RH (6)
            Ta([1,4,5]) = [t;t;t];
            Ta([2,3,6]) = [1;1;1] * (t + 1/2*(Tst + Tsw));
            Tb(i_leg) = Ta(i_leg) + Tst;
        end
        FSM(i_leg) = FSM(i_leg) + 1;
        pf_R_trans = Xt(idx_pf);
    elseif FSM(i_leg) == 1 && (s(i_leg) >= 1 - 1e-7)    % stance to swing
         FSM(i_leg) = FSM(i_leg) + 1;
        Ta(i_leg) = t;
        Tb(i_leg) = Ta(i_leg) + Tsw;
        pf_R_trans = Xt(idx_pf);
    elseif FSM(i_leg) == 2 && (s(i_leg) >= 1 - 1e-7)    % swing to stance
        FSM(i_leg) = 1;
        Ta(i_leg) = t;
        Tb(i_leg) = Ta(i_leg) + Tst;
        pf_R_trans = Xt(idx_pf);
    end
end
s = (t - Ta) ./ (Tb - Ta);
s(s<0) = 0;
s(s>1) = 1;

%% FSM in prediction horizon
FSM_ = repmat(FSM,[1,p.predHorizon]);
for i_leg = 1:6 % --- MODIFICA 4: Ciclo esteso a 6 zampe ---
    for ii = 2:p.predHorizon
        if t_(ii) <= Ta(i_leg)
            FSM_(i_leg,ii) = 1;
        elseif (Ta(i_leg) < t_(ii)) && (t_(ii) < Tb(i_leg))
            FSM_(i_leg,ii) = FSM(i_leg);
        elseif Ta(i_leg) + Tst + Tsw < t_(ii)
            FSM_(i_leg,ii) = FSM(i_leg);
        else
            if FSM(i_leg) == 1
                FSM_(i_leg,ii) = 2;
            else
                FSM_(i_leg,ii) = 1;
            end
        end
    end
end
if gait == -1       % pose
    FSM_ = ones(size(FSM_));
end
% [6,predHorizon]: bool matrix
bool_inStance = (FSM_ == 1);

%% Gen ref traj
% --- Xd/Ud ---
[Xd,Ud] = fcn_gen_XdUd(t_,Xt,bool_inStance,p);

%% swing leg kinematics
% --- MODIFICA 5: Posizioni degli hip (anche per le zampe medie) ---
% Struttura: [LF, RF, LM, RM, LH, RH]
p_hip_b = [ ...
    [ L/2;  W/2+d; 0], ... % 1. LF
    [ L/2; -W/2-d; 0], ... % 2. RF
    [   0;  W/2+d; 0], ... % 3. LM (Medio sx)
    [   0; -W/2-d; 0], ... % 4. RM (Medio dx)
    [-L/2;  W/2+d; 0], ... % 5. LH
    [-L/2; -W/2-d; 0]  ... % 6. RH
];
p_hip_R = R * p_hip_b;
ws = R * wb;
v_hip_R = repmat(dpc,[1,6]) + hatMap(ws) * p_hip_R;

% capture point
p_cap = zeros(2,6); % --- MODIFICA 6: Esteso a 6 zampe ---
vd = Xd(4:5,1);
for i_leg = 1:6
    temp = 0.8 * Tst * vd + sqrt(p.z0/p.g) * (v_hip_R(1:2,i_leg) - vd);
    temp(temp < -0.15) = -0.15;
    temp(temp > 0.15) = 0.15;
    p_cap(:,i_leg) = pc(1:2) + p_hip_R(1:2,i_leg) + temp;
end

% desired foot placement
if p.gait == -2     % GOT
    Rg = fcn_GOT_Rg_BB(t,p);
    % --- MODIFICA 7: Aggiornamento dimensioni per 6 zampe ---
    Xt(idx_pf) = reshape(Rg * p.pf34,[18,1]); 
    Xd(idx_pf,:) = repmat(Xt(idx_pf),[1,p.predHorizon]);
else
    pfd = Xt(idx_pf);
    for i_leg = 1:6 % --- MODIFICA 8: Ciclo esteso a 6 zampe ---
        idx = 3*(i_leg-1) + (1:3);
        if FSM(i_leg) == 2
            co_x = linspace(pf_R_trans(idx(1)),p_cap(1,i_leg),6);
            co_y = linspace(pf_R_trans(idx(2)),p_cap(2,i_leg),6);
            co_z = [0 0 0.1 0.1 0 -0.002];
            pfd(idx) = [polyval_bz(co_x,s(i_leg));
                        polyval_bz(co_y,s(i_leg));
                        polyval_bz(co_z,s(i_leg))];
        end
    end
    Xd(idx_pf,:) = repmat(pfd,[1,p.predHorizon]);
end

%% output
FSMout = FSM;
end