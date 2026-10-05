function run = run_vuoto(N)
% Struttura normalizzata di una simulazione
%   run = run_vuoto        % struttura vuota, per vedere i campi
%   run = run_vuoto(N)     % preallocata su N campioni
%
% CAMPI OBBLIGATORI            
%   t        [N x 1]   tempo [s]
%   p        [N x 3]   posizione del CoM nel frame mondo [m]
%   rpy      [N x 3]   rollio, beccheggio, imbardata [rad]
%
% CAMPI OPZIONALI              
%   v        [N x 3]   velocita' del CoM [m/s]        - se manca si deriva da p
%   w        [N x 3]   velocita' angolare [rad/s]
%   q        [N x 18]  angoli di giunto [rad]
%   qd       [N x 18]  velocita' di giunto [rad/s]
%   tau      [N x 18]  coppie di giunto [N*m]
%   pf       [N x 18]  posizioni dei piedi nel frame mondo [m], 6 x (x,y,z)
%   Fc       [N x 18]  forze di contatto nel frame mondo [N], 6 x (x,y,z)
%   contact  [N x 6]   logico: true se la zampa e' in appoggio DAVVERO
%                      (misurato: forza trasmessa o flag del contatto)
%   contact_sched
%            [N x 6]   logico: true se la zampa DOVREBBE essere in appoggio
%                      secondo il ciclo di andatura nominale.
%                      Serve a distinguere i due casi: una zampa schedulata a
%                      terra che non trasmette forza e' un DISTACCO NON
%                      PREVISTO (D.distacchi, D.frazione_persa); senza questo
%                      campo resta solo un criterio indiretto sul numero di
%                      transizioni, e frazione_persa vale NaN.
%
% ORDINE DELLE ZAMPE
%   Sempre l'ordine CAN di phantomx_config: FL FR ML MR RL RR.
%   Il baseline Simscape lavora in ordine Mux: converti con cfg.mux2can
%   PRIMA di riempire questa struttura, non dopo.
%
% METADATI                     
%   meta.controller  'C1' | 'C2' | 'C3'
%   meta.task        'T1' ... 'T7'
%   meta.run         numero della ripetizione (1..5)
%   meta.seed        seme del generatore casuale usato per questa run
%   meta.vel_d       [1 x 2] velocita' comandata [m/s]
%   meta.yaw_d       velocita' di imbardata comandata [rad/s]
%   meta.condizione  'nominale' | 'massa-10' | 'attrito-04' | ...
%   meta.note        testo libero

if nargin < 1, N = 0; end

z1 = zeros(N,1);  z3 = zeros(N,3);  z6 = zeros(N,6);  z18 = zeros(N,18);

run = struct( ...
    't',       z1,  ...
    'p',       z3,  ...
    'v',       z3,  ...
    'rpy',     z3,  ...
    'w',       z3,  ...
    'q',       z18, ...
    'qd',      z18, ...
    'tau',     z18, ...
    'pf',      z18, ...
    'Fc',      z18, ...
    'contact',       false(N,6), ...
    'contact_sched', false(N,6), ...
    'meta',    struct('controller','', 'task','', 'run',1, 'seed',NaN, ...
                      'vel_d',[NaN NaN], 'yaw_d',NaN, ...
                      'condizione','nominale', 'note',''));

if N == 0
    % struttura vuota: azzera i campi array cosi' e' evidente che va riempita
    for f = {'t','p','v','rpy','w','q','qd','tau','pf','Fc'}
        run.(f{1}) = [];
    end
    run.contact       = logical([]);
    run.contact_sched = logical([]);
end

end