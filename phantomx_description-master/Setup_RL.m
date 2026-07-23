%% 1. Configurazione Ambiente
modelName = 'phantomx'; % Metti qui il nome del tuo file .slx
load_system(modelName);

% Definizione Spazio Azioni (18 posizioni X,Y,Z per le 6 zampe)
% Limiti: supponiamo che il piede possa muoversi di +/- 10cm (0.1m)
numActions = 18;
actInfo = rlNumericSpec([numActions 1], 'LowerLimit', -0.1, 'UpperLimit', 0.1);

% Definizione Spazio Osservazioni (es. 24 sensori)
numObs = 24; 
obsInfo = rlNumericSpec([numObs 1]);

% Creazione ambiente Simulink
env = rlSimulinkEnv(modelName, [modelName '/RL Agent'], obsInfo, actInfo);

%% 2. Creazione dell'Agente (PPO - standard per la robotica)
% Usiamo una rete neurale semplice per iniziare
obsPath = [
    featureInputLayer(numObs)
    fullyConnectedLayer(128)
    reluLayer()
    fullyConnectedLayer(128)
    reluLayer()
    fullyConnectedLayer(numActions)
    ];

actor = rlContinuousDeterministicActor(obsPath, obsInfo, actInfo);
agent = rlPPOAgent(obsInfo, actInfo);

%% 3. Parametri di Training
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 1000, ...
    'MaxStepsPerEpisode', 500, ...
    'Verbose', true, ...
    'Plots', 'training-progress');