% 1. Crea il solutore di Cinematica Inversa per il tuo ragno
ik = inverseKinematics('RigidBodyTree', my_spider);

% 2. Definisci l'importanza dei parametri (Weights)
% [Orientamento(Roll,Pitch,Yaw), Posizione(X,Y,Z)]
% Visto che ci interessa solo dove poggia il piede, diamo peso alla posizione
weights = [0, 0, 0, 1, 1, 1]; 

% 3. Definisci la Matrice T Obiettivo (Dove vuoi che vada il piede)
T_target = trvec2tform([0.2, 0.1, -0.1]); % Inserisci coordinate X, Y, Z sensate per il robot

% 4. Calcola gli angoli dei giunti necessari
initial_guess = homeConfiguration(my_spider); % Punto di partenza per l'algoritmo
[config_sol, sol_info] = ik('tibia_lf', T_target, weights, initial_guess);

% 5. Mostra gli angoli calcolati!
disp('Angoli calcolati per la zampa:');
struct2table(config_sol)