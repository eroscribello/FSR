function joint_angles = fcn(rl_actions)
    % Inizializzazione del vettore di uscita (18 angoli per i giunti)
    joint_angles = zeros(18, 1);
    
    % Definizione dei pesi per il solutore IK (ci interessa solo la posizione X,Y,Z)
    weights = [0, 0, 0, 1, 1, 1];
    
    % Manteniamo in memoria il robot e il solutore per non ricalcolarli a ogni passo
    persistent ik_solver robot initial_guess
    if isempty(ik_solver)
        % IMPORTANTE: Sostituisci 'crea_tuo_robot' con la funzione o la 
        % variabile presente nel tuo workspace che genera il RigidBodyTree 'my_spider'
        robot = evalin('base', 'my_spider'); 
        ik_solver = inverseKinematics('RigidBodyTree', robot);
        initial_guess = homeConfiguration(robot);
    end
    
    % Nomi degli end-effector nel RigidBodyTree
    end_effectors = {'tibia_lf', 'tibia_lm', 'tibia_lr', 'tibia_rf', 'tibia_rm', 'tibia_rr'};
    
    for i = 1:6
        % 1. Estrai le 3 coordinate generate dall'RL Agent per la zampa i
        idx = (i-1)*3 + 1;
        X = rl_actions(idx);
        Y = rl_actions(idx+1);
        Z = rl_actions(idx+2);
        
        % 2. GESTIONE SPECULARITÀ: Se stiamo calcolando le zampe destre (indici 4, 5, 6)
        % invertiamo l'asse Y per far sì che i movimenti siano simmetrici rispetto al corpo
        if i > 3
            Y = -Y; 
        end
        
        % 3. Crea la matrice di trasformazione omogenea obiettivo
        T_target = trvec2tform([X, Y, Z]);
        
        % 4. Calcola la cinematica inversa per l'end-effector corrente
        [config_sol, ~] = ik_solver(end_effectors{i}, T_target, weights, initial_guess);
        
        % 5. Mappatura sui 18 giunti di uscita
        % Estraiamo i valori dei 3 giunti specifici per questa zampa dalla struttura
        % NOTA: L'ordine esatto dipende da come sono scritti i nomi dei giunti nel tuo file URDF/RigidBodyTree
        joint_angles(idx)   = config_sol(idx).JointPosition;   % Giunto Anca (Coxa)
        joint_angles(idx+1) = config_sol(idx+1).JointPosition; % Giunto Coscia (Thigh)
        joint_angles(idx+2) = config_sol(idx+2).JointPosition; % Giunto Tibia
    end
end