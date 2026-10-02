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

%% --- parameters ---
% ---- gait ----
% 0-tripode; 1-ripple; 2-wave
gait = 0; 
p = get_params(gait);
p.playSpeed =1;
p.flag_movie = 0;       % 1 - make movie

SimTimeDuration = 10;
[out, info] = mpc_loop(p, SimTimeDuration);

tout = out.tout;  Xout = out.Xout;  Uout = out.Uout;
Xdout = out.Xdout; Udout = out.Udout; Uext = out.Uext; FSMout = out.FSMout;

%% Animation
[t,EA,EAd] = fig_animate(tout,Xout,Uout,Xdout,Udout,Uext,p);



%% GRAFICI

%% Sincronizzazione temporale
t_start = tout(1);
t_end   = tout(end);
t_X   = linspace(t_start, t_end, size(Xout,1));   % Tempo per stato reale CoM
t_Xd  = linspace(t_start, t_end, size(Xdout,1));  % Tempo per stato desiderato CoM
t_U   = linspace(t_start, t_end, size(Uout,1));   % Tempo per Forze reali
t_Ud  = linspace(t_start, t_end, size(Udout,1));  % Tempo per Forze desiderate

% Mappatura indici per l'esapode (6 piedi)
% Forze U: Fx, Fy, Fz per ogni piede (totale 18 canali)
idx_Fz = 3:3:18; 
idx_Fx = 1:3:16;
idx_Fy = 2:3:17;

% Posizioni piedi in Xout: partono da indice 19 a 36 (x, y, z per 6 piedi)
idx_Pz = 21:3:36; 

%% =========================================================================
%% FIGURA 1: PANORAMICA STATO CoM & FORZE VERTICALI TOTALI
%% =========================================================================
figure('Name', 'CoM Overview & Global Forces', 'Color', 'white');
N = 2; M = 2;

% 1. Posizione del CoM
subplot(N,M,1);
plot(t_X,  Xout(:,1),  'r',   t_X,  Xout(:,2),  'g',   t_X,  Xout(:,3),  'b', ...
     t_Xd, Xdout(:,1), 'r--', t_Xd, Xdout(:,2), 'g--', t_Xd, Xdout(:,3), 'b--', 'LineWidth', 1.5)
grid on; xlim([t_start t_end]);
title('CoM Position [m]', 'FontWeight', 'bold'); xlabel('Time [s]'); ylabel('Position [m]');
legend('x', 'y', 'z', 'x_{des}', 'y_{des}', 'z_{des}', 'Location', 'best');

% 2. Velocità Lineare del CoM
subplot(N,M,2);
plot(t_X,  Xout(:,4),  'r',   t_X,  Xout(:,5),  'g',   t_X,  Xout(:,6),  'b', ...
     t_Xd, Xdout(:,4), 'r--', t_Xd, Xdout(:,5), 'g--', t_Xd, Xdout(:,6), 'b--', 'LineWidth', 1.5)
grid on; xlim([t_start t_end]);
title('CoM Linear Velocity [m/s]', 'FontWeight', 'bold'); xlabel('Time [s]'); ylabel('Velocity [m/s]');
legend('v_x', 'v_y', 'v_z', 'v_{x,des}', 'v_{y,des}', 'v_{z,des}', 'Location', 'best');

% 3. Velocità Angolare del CoM
subplot(N,M,3);
plot(t_X,  Xout(:,16),  'r',   t_X,  Xout(:,17),  'g',   t_X,  Xout(:,18),  'b', ...
     t_Xd, Xdout(:,16), 'r--', t_Xd, Xdout(:,17), 'g--', t_Xd, Xdout(:,18), 'b--', 'LineWidth', 1.5)
grid on; xlim([t_start t_end]);
title('CoM Angular Velocity [rad/s]', 'FontWeight', 'bold'); xlabel('Time [s]'); ylabel('Ang. Velocity [rad/s]');
legend('\omega_x', '\omega_y', '\omega_z', '\omega_{x,des}', '\omega_{y,des}', '\omega_{z,des}', 'Location', 'best');

% 4. Sguardo d'insieme su Fz (tutte e 6 le forze a terra)
subplot(N,M,4);
colors = {'r', 'g', 'b', 'k', 'm', 'c'};
hold on;
for i = 1:6
    plot(t_U, Uout(:, idx_Fz(i)), colors{i}, 'LineWidth', 1.2);
end
grid on; xlim([t_start t_end]);
title('All Foot Reaction Forces Fz [N]', 'FontWeight', 'bold'); xlabel('Time [s]'); ylabel('Force [N]');
legend('Piede 1', 'Piede 2', 'Piede 3', 'Piede 4', 'Piede 5', 'Piede 6', 'Location', 'best');

%% =========================================================================
%% FIGURA 2: TRACCIAMENTO FORZE VERTICALI SINGOLE (Griglia 3x2)
%% =========================================================================
figure('Name', 'Individual Vertical Forces (Fz)', 'Color', 'white');
for i = 1:6
    subplot(3, 2, i);
    plot(t_U,  Uout(:, idx_Fz(i)), 'b', ...
         t_Ud, Udout(:, idx_Fz(i)), 'r--', 'LineWidth', 1.2)
    grid on; xlim([t_start t_end]);
    ylabel('Force [N]');
    title(['Foot ', num2str(i), ' - Vertical Force Fz'], 'FontWeight', 'bold');
    if i >= 5; xlabel('Time [s]'); end
    legend('Real', 'Desired', 'Location', 'best');
end
sgtitle('Reaction Forces (Fz) per Foot', 'FontSize', 16, 'FontWeight', 'bold');

%% =========================================================================
%% FIGURA 3: TRAIETTORIA VERTICALE DEI PIEDI (Tracking dello Swing)
%% =========================================================================
% Questo grafico ti mostra se i piedi si alzano a terra seguendo il riferimento (fase di swing)
figure('Name', 'Foot Swing Trajectories (Z)', 'Color', 'white');
for i = 1:6
    subplot(3, 2, i);
    plot(t_X,  Xout(:, idx_Pz(i)), 'b', ...
         t_Xd, Xdout(:, idx_Pz(i)), 'r--', 'LineWidth', 1.2)
    grid on; xlim([t_start t_end]);
    ylabel('Height [m]');
    title(['Foot ', num2str(i), ' - Height (z)'], 'FontWeight', 'bold');
    if i >= 5; xlabel('Time [s]'); end
    legend('Real', 'Desired', 'Location', 'best');
end
sgtitle('Foot Vertical Trajectories', 'FontSize', 16, 'FontWeight', 'bold');

%% =========================================================================
%% FIGURA 4: CONI D'ATTRITO PER OGNI PIEDE (Friction Cone Angle)
%% =========================================================================
% Calcoliamo l'angolo di inclinazione della forza rispetto alla verticale per ogni zampa.
% Se l'angolo supera l'angolo limite dell'attrito statico (atan(mu)), la zampa slitta.
figure('Name', 'Friction Cone Analysis', 'Color', 'white');
limit_angle = atan(p.mu);

for i = 1:6
    subplot(3, 2, i);
    
    % Estrai le componenti per la zampa i-esima
    Fx = Uout(:, idx_Fx(i));
    Fy = Uout(:, idx_Fy(i));
    Fz = Uout(:, idx_Fz(i));
    
    % Forza tangenziale totale sul piano del terreno
    F_tangential = sqrt(Fx.^2 + Fy.^2);
    
    % Angolo della forza di reazione rispetto alla normale
    theta_foot = atan2(F_tangential, Fz);
    
    % Filtro: se la zampa non è a terra (forza verticale quasi nulla), azzera l'angolo per pulire il grafico
    theta_foot(Fz < 5) = 0; 
    
    % Plotting
    plot(t_U, theta_foot, 'b', 'LineWidth', 1);
    hold on;
    yline(limit_angle, 'r--', 'LineWidth', 1.5);
    grid on; xlim([t_start t_end]);
    ylabel('Angle [rad]');
    title(['Foot ', num2str(i), ' - Friction Angle'], 'FontWeight', 'bold');
    if i >= 5; xlabel('Time [s]'); end
    legend('Force Angle', 'Friction Limit', 'Location', 'best');
end
sgtitle('Friction Cone Angle vs Slipping Limit', 'FontSize', 16, 'FontWeight', 'bold');

%% =========================================================================
%% FIGURA 5: FORZE TANGENZIALI TOTALI (Sforzo di Trazione X e Y)
%% =========================================================================
figure('Name', 'Total Horizontal Forces', 'Color', 'white');

% Somma delle forze su tutte e 6 le zampe
Fx_total_real = sum(Uout(:, idx_Fx), 2);
Fy_total_real = sum(Uout(:, idx_Fy), 2);
Fx_total_des  = sum(Udout(:, idx_Fx), 2);
Fy_total_des  = sum(Udout(:, idx_Fy), 2);

subplot(2,1,1);
plot(t_U, Fx_total_real, 'b', t_Ud, Fx_total_des, 'r--', 'LineWidth', 1.2);
grid on; xlim([t_start t_end]);
ylabel('Force [N]'); title('Total Traction Force along X-axis', 'FontWeight', 'bold');
legend('Real', 'Desired');

subplot(2,1,2);
plot(t_U, Fy_total_real, 'g', t_Ud, Fy_total_des, 'r--', 'LineWidth', 1.2);
grid on; xlim([t_start t_end]);
ylabel('Force [N]'); title('Total Traction Force along Y-axis', 'FontWeight', 'bold');
legend('Real', 'Desired');
xlabel('Time [s]');