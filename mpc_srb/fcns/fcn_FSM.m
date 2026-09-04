function [FSMout,Xd,Ud,Xt] = fcn_FSM(t_,Xt,p)
% FSM di andatura per esapode PhantomX.
% Ordine gambe: 1=LF  2=RF  3=LM  4=RM  5=LH  6=RH
%
% L'andatura e' definita da get_params tramite:
%   p.phase : fase iniziale di ogni zampa (frazione di ciclo, 6x1)
%   p.Tst/p.Tsw : tempi di appoggio e volo (derivati da duty factor e periodo)
% Esempi:  tripode  phase = [0;1/2;1/2;0;0;1/2]   beta = 1/2
%          ripple   phase = [0;2/3;1/3;0;2/3;1/3] beta = 2/3
%          wave     phase = [1/3;5/6;1/6;2/3;0;1/2] beta = 5/6

%% parameters
nLeg = p.nLeg;
gait = p.gait;

% --- costanti di SCALA (PhantomX) ---
if isfield(p,'stepLenRef'),  stepLenRef  = p.stepLenRef;  else, stepLenRef  = 0.05; end
if isfield(p,'stepClamp'),   stepClamp   = p.stepClamp;   else, stepClamp   = 0.05; end
if isfield(p,'swingHeight'), swingHeight = p.swingHeight; else, swingHeight = 0.025; end

Tst_ = p.Tst;
vxy  = norm(Xt(4:5));
if vxy > 1e-6
    Tst = min(Tst_, stepLenRef/vxy);
else
    Tst = Tst_;
end
Tsw = p.Tsw;

[pc,dpc,vR,wb] = deal(Xt(1:3),Xt(4:6),Xt(7:15),Xt(16:18));
R = reshape(vR,[3,3]);

idx_pf = 18 + (1:3*nLeg);              % 19:36

%% initialization
persistent FSM Ta Tb pf_R_trans
if isempty(FSM)
    FSM = zeros(nLeg,1);
    Ta  = zeros(nLeg,1);
    Tb  = ones(nLeg,1);
    pf_R_trans = Xt(idx_pf);
end

t = t_(1);
s = zeros(nLeg,1);

%% FSM
% 1 - stance   2 - swing
for i_leg = 1:nLeg
    s(i_leg) = (t - Ta(i_leg)) / (Tb(i_leg) - Ta(i_leg));
    s(s<0) = 0;  s(s>1) = 1;

    if FSM(i_leg) == 0              % init: schedulazione dal vettore di fasi
        Ta(i_leg) = t + p.phase(i_leg) * (Tst + Tsw);
        Tb(i_leg) = Ta(i_leg) + Tst;
        FSM(i_leg) = 1;
        pf_R_trans = Xt(idx_pf);
    elseif FSM(i_leg) == 1 && (s(i_leg) >= 1 - 1e-7)    % stance -> swing
        FSM(i_leg) = 2;
        Ta(i_leg)  = t;
        Tb(i_leg)  = Ta(i_leg) + Tsw;
        pf_R_trans = Xt(idx_pf);
    elseif FSM(i_leg) == 2 && (s(i_leg) >= 1 - 1e-7)    % swing -> stance
        FSM(i_leg) = 1;
        Ta(i_leg)  = t;
        Tb(i_leg)  = Ta(i_leg) + Tst;
        pf_R_trans = Xt(idx_pf);
    end
end

s = (t - Ta) ./ (Tb - Ta);
s(s<0) = 0;  s(s>1) = 1;

%% FSM lungo l'orizzonte di predizione
FSM_ = repmat(FSM,[1,p.predHorizon]);
for i_leg = 1:nLeg
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

if gait == -1       % pose control: tutte le zampe sempre in appoggio
    FSM_ = ones(size(FSM_));
end

bool_inStance = (FSM_ == 1);       % [nLeg, predHorizon]

%% riferimento
[Xd,Ud] = fcn_gen_XdUd(t_,Xt,bool_inStance,p);

%% swing leg
p_hip_b = p.p_hip;                 % 3 x nLeg (layout esagonale da URDF)
p_hip_R = R * p_hip_b;
p_nom_R = R * p.pf36;              % stance nominale allargata

ws = R * wb;
v_hip_R = repmat(dpc,[1,nLeg]) + hatMap(ws) * p_hip_R;

% capture point (centrato sulla stance nominale, NON sulle anche)
p_cap = zeros(2,nLeg);
vd = Xd(4:5,1);
for i_leg = 1:nLeg
    temp = 0.8 * Tst * vd + sqrt(p.z0/p.g) * (v_hip_R(1:2,i_leg) - vd);
    temp(temp < -stepClamp) = -stepClamp;
    temp(temp >  stepClamp) =  stepClamp;
    p_cap(:,i_leg) = pc(1:2) + p_nom_R(1:2,i_leg) + temp;
end

% traiettoria del piede in volo
pfd = Xt(idx_pf);
for i_leg = 1:nLeg
    idx = 3*(i_leg-1) + (1:3);
    if FSM(i_leg) == 2
        co_x = linspace(pf_R_trans(idx(1)),p_cap(1,i_leg),6);
        co_y = linspace(pf_R_trans(idx(2)),p_cap(2,i_leg),6);
        co_z = [0 0 swingHeight swingHeight 0 -0.002];
        pfd(idx) = [polyval_bz(co_x,s(i_leg));
                    polyval_bz(co_y,s(i_leg));
                    polyval_bz(co_z,s(i_leg))];
    end
end
Xd(idx_pf,:) = repmat(pfd,[1,p.predHorizon]);

%% output
FSMout = FSM;
end