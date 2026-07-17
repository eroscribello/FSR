% Representation-Free Model Predictive Control for Dynamic Quadruped Panther
% Author: Yanran Ding
% Last modified: 2020/12/21
% 
% Code accompanying the paper:
% Yanran Ding, Abhishek Pandala, Chuanzheng Li, Young-Ha Shin, Hae-Won Park
% "Representation-Free Model Predictive Control for Dynamic Motions in Quadrupeds"
% Transactions on Robotics
% 
% preprint available at: https://arxiv.org/abs/2012.10002
% video available at: https://www.youtube.com/watch?v=iMacEwQisoQ&t=101s

%% initialization
clear all;close all;clc
addpath fcns fcns_MPC

opts.MAXITER=2;

%% --- parameters ---
% ---- gait ----
% 0-trot; 1-bound; 2-pacing 3-gallop; 4-trot run; 5-crawl
gait = 0; 
p = get_params(gait);
p.playSpeed =1;
p.flag_movie = 0;       % 1 - make movie

dt_sim = p.simTimeStep;
SimTimeDuration = 4.5;  % [sec]
MAX_ITER = floor(SimTimeDuration/p.simTimeStep);

% desired trajectory
p.acc_d = 1;
p.vel_d = [0.5;0];
p.yaw_d = 0;

%% Model Predictive Control
% --- initial condition ---
% Xt = [pc dpc vR wb pf]': [30,1]
if gait == 1
    [p,Xt,Ut] = fcn_bound_ref_traj(p);
else
    [Xt,Ut] = fcn_gen_XdUd(0,[],[1;1;1;1],p);
end

% --- logging ---
tstart = 0;
tend = dt_sim;

[tout,Xout,Uout,Xdout,Udout,Uext,FSMout] = deal([]);

% --- simulation ----
h_waitbar = waitbar(0,'Calculating...');
tic
for ii = 1:MAX_ITER
    % --- time vector ---
    t_ = dt_sim * (ii-1) + p.Tmpc * (0:p.predHorizon-1);
    
    % --- FSM ---
    if gait == 1

        [FSM,Xd,Ud,Xt] = fcn_FSM_bound(t_,Xt,p);
    else
        [FSM,Xd,Ud,Xt] = fcn_FSM(t_,Xt,p);
    end

    % --- MPC ----
    % form QP
    [H,g,Aineq,bineq,Aeq,beq] = fcn_get_QP_form_eta(Xt,Ut,Xd,Ud,p);

       %%
    % Considering the matrices for the QP obtained from function fcn_get_QP_form_eta, use the QP solver qpSWIFT to 
    %  solve the quadratic problem with the following form 
    %  min. 0.5 * x' * H *x + g' * x
    %  s.t. Aineq *x <= bineq
    %      Aeq * x <= beq
    % 
    % The result of the QP problem should be stored in a variable called zval in order to be used in the following
    G=[Aineq;Aeq];
    h=[bineq;beq];
    [sol,basic_info,adv_info] = qpSWIFT(sparse(H),g,sparse(Aeq),beq,sparse(Aineq),bineq);
    
end