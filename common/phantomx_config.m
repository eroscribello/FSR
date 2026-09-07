function cfg = phantomx_config(verbose)
%PHANTOMX_CONFIG  Unica fonte di verita' dei parametri del PhantomX.
%
%   cfg = phantomx_config        % silenzioso
%   cfg = phantomx_config(true)  % stampa il riepilogo e i controlli derivati
%
% Letta da: get_params.m (MPC su simulatore ridotto)
%           init_gait.m  (baseline Simscape)
% NON duplicare questo file. Se ne esistono due copie, i due simulatori
% divergono senza che nessun errore lo segnali: e' gia' successo.
%
% PROVENIENZA DEI VALORI  (marcata riga per riga)
%   [URDF]      letto da urdf/phantomx.urdf
%   [MESH]      misurato sulla geometria STL
%   [DATASHEET] specifica del servo
%   [TARATO]    trovato sperimentalmente in simulazione
%   [VERIFICA]  da confermare, vedi note in fondo
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1, verbose = false; end

%% ===== guardia sui doppioni =====
copie = which('phantomx_config','-all');
copie = copie(~contains(copie,'_cestino'));
if numel(copie) > 1
    warning('phantomx:config:duplicati', ...
        ['Esistono %d copie di phantomx_config sul path. MATLAB ne usa una\n' ...
         'sola e potresti star modificando l''altra:\n%s'], ...
         numel(copie), sprintf('    %s\n', copie{:}));
end
copie = which('inv_kyn','-all');
copie = copie(~contains(copie,'_cestino'));
if numel(copie) > 1
    warning('phantomx:config:duplicati', ...
        'Esistono %d copie di inv_kyn sul path:\n%s', ...
        numel(copie), sprintf('    %s\n', copie{:}));
end

%% ===== costanti fisiche =====
cfg.g = 9.81;

%% ===== ordine delle zampe =====
% Due convenzioni convivono nel progetto:
%   ordine CAN  (questo file, get_params, MPC) : FL FR ML MR RL RR
%   ordine MUX8 (blocco Mux del modello Simscape): RR MR FR RL ML FL
cfg.legNamesCAN = {'FL','FR','ML','MR','RL','RR'};
cfg.legNamesMUX = {'RR','MR','FR','RL','ML','FL'};

cfg.mux2can = [6 4 2 5 3 1];   % indice CAN della i-esima zampa nell'ordine Mux
cfg.can2mux = [6 3 5 2 4 1];   % indice Mux della i-esima zampa nell'ordine CAN

% indice del giunto j (1=coxa, 2=femore, 3=tibia) della zampa CAN i,
% dentro il vettore a 18 elementi in ordine Mux
cfg.jointIndex = @(i,j) 3*(cfg.can2mux(i)-1) + j;

%% ===== geometria della gamba =====
cfg.lc = 0.054;      % [URDF] coxa,   da j_c2_*  xyz = [0 -0.054 0]
cfg.lf = 0.0661;     % [URDF] femore, da j_tibia_* xyz = [0 -0.0645 -0.0145]
                     %        norm([0.0645 0.0145]) = 0.066112
cfg.lt = 0.12;       % [MODELLO] tibia EFFETTIVA del simulatore.
                     % La punta della mesh sta a 0.15297 (norm([0.03 0.15])),
                     % ma la sfera di contatto e' traslata di 0.12 lungo la
                     % tibia, quindi il piede simulato e' a 0.12. inv_kyn usa
                     % lo stesso valore: il modello e' COERENTE.
                     % Per passare alla tibia vera vanno cambiati INSIEME
                     % questo valore, inv_kyn e cfg.foot_offset. Vedi NOTE.
cfg.lt_mesh = 0.15297;  % [MESH] tibia geometrica reale, oggi non usata
cfg.foot_offset = 0.12; % [MODELLO] traslazione della sfera lungo la tibia

cfg.reachMax = cfg.lf + cfg.lt;   % 0.1861 m, estensione massima della gamba

%% ===== posizione delle anche nel frame corpo, ordine CAN =====
% [URDF] origini dei giunti j_c1_*, colonna per zampa
%        FL          FR          ML       MR        RL           RR
cfg.p_hip = [ ...
     0.1248     0.1248     0.0      0.0     -0.1248    -0.1248 ; ...
     0.06164   -0.06164    0.1034  -0.1034   0.06164   -0.06164 ; ...
     0.001116   0.001116   0.001116 0.001116 0.001116   0.001116 ];

% angolo di montaggio della gamba nel piano XY del corpo, ordine CAN.
% [URDF] e' lo yaw del giunto j_c1_* meno 90 gradi (il frame della coxa e'
%        ruotato di pi/2 rispetto al corpo).
%        FL   FR   ML   MR   RL    RR
cfg.alpha = deg2rad([45, -45,  90, -90, 135, -135]);

% segno di specularita' usato da inv_kyn: +1 sinistra, -1 destra
cfg.side  = [ +1,  -1,  +1,  -1,  +1,  -1 ];

%% ===== posa nominale =====
cfg.r_offset = 0.12;   % [TARATO] distanza radiale del piede dall'asse coxa
cfg.z0       = 0.10;   % [TARATO] quota del piede sotto l'anca

% piedi nominali nel frame corpo, 3x6, ordine CAN
cfg.pf_nom = zeros(3,6);
for i = 1:6
    cfg.pf_nom(:,i) = cfg.p_hip(:,i) + ...
        [cfg.r_offset*cos(cfg.alpha(i)); cfg.r_offset*sin(cfg.alpha(i)); -cfg.z0];
end

%% ===== masse e inerzie =====
cfg.m_body = 0.975599;      % corpo
cfg.m_link = 0.024357719;   % ciascuno dei 24 link di gamba (6 x 4)
cfg.mass   = cfg.m_body + 24*cfg.m_link;   % 1.560184 kg

% inerzia del corpo rigido singolo, per il modello SRB dell'MPC
cfg.J = diag([0.01444, 0.01751, 0.02889]);

% inerzie per link, per i blocchi Inertia del modello Simscape [Ixx Iyy Izz]
cfg.I_body  = [3.557e-03, 5.154e-03, 8.565e-03];
cfg.I_c1    = [3.654e-06, 7.746e-06, 7.746e-06];
cfg.I_c2    = [3.654e-06, 3.654e-06, 3.654e-06];
cfg.I_thigh = [3.095e-06, 1.014e-05, 1.070e-05];
cfg.I_tibia = [2.081e-06, 3.004e-05, 3.050e-05];

%% ===== andatura a tripode =====
cfg.T           = 1.00;   % [TARATO] periodo del ciclo [s]. Era 1.0: con 2.0
                          %          l'andatura oscilla meno.
cfg.beta_stance = 0.60;   % frazione del ciclo in appoggio
cfg.duty_swing  = 1 - cfg.beta_stance;   % 0.40, frazione in volo

cfg.T_stance = cfg.beta_stance * cfg.T;  % 1.20 s
cfg.T_swing  = cfg.duty_swing  * cfg.T;  % 0.80 s

cfg.S     = 0.06;                        % [TARATO] lunghezza del passo [m]
cfg.v_nom = cfg.S / cfg.T_stance;        % 0.050 m/s, velocita' che ne consegue
cfg.H = 0.03;                            % [TARATO] altezza di sollevamento

% sfasamento del ciclo, ordine CAN. Tripode A = FL MR RL, tripode B = FR ML RR
cfg.phase = [0; 1/2; 1/2; 0; 0; 1/2];

%% ===== contatto e terreno =====
cfg.contact.k     = 1e4;    % [TARATO] rigidezza [N/m]
cfg.contact.c     = 100;    % [TARATO] smorzamento [N/(m/s)]
cfg.contact.w     = 1e-4;   % larghezza della regione di transizione [m]
cfg.contact.vcrit = 1e-2;   % velocita' critica stick-slip [m/s]
cfg.contact.foot_r = 0.01;  % raggio della sfera di contatto al piede [m]

cfg.mu_plant = 0.9;   % attrito del modello fisico (Simscape)
cfg.mu_mpc   = 0.6;   % attrito assunto dall'MPC: piu' conservativo del vero

cfg.floor_dim = [4 4 0.05];        % [x y z] del pavimento [m]
cfg.floor_off = [0 0 -cfg.floor_dim(3)/2];   % centro del pavimento
cfg.floor_top = cfg.floor_off(3) + cfg.floor_dim(3)/2;   % quota della superficie

% quota iniziale del corpo
cfg.body_z0 = 0.25;   % [TARATO] margine di caduta iniziale: i giunti partono
                      % dalla posa URDF, non da quella di appoggio, quindi il
                      % robot deve avere spazio per sistemarsi mentre cade.
                      % Con la geometrica (0.11) le sfere partono dentro il
                      % pavimento e il robot ci passa attraverso. Provato.
                      % Si potra' scendere a body_z0_geom quando i giunti
                      % avranno q0 (posa IK) come posizione iniziale.
% derivazione geometrica, per confronto (vedi controllo in fondo)
cfg.body_z0_geom = cfg.z0 + cfg.contact.foot_r + cfg.floor_top;   % 0.110 m

%% ===== attuatori =====
cfg.tau_max = 1.5;          % [DATASHEET] coppia di stallo del servo [N*m]
                            % l'URDF dichiara effort=2.8: ottimistico, non usarlo
cfg.qd_max  = 5.6548668;    % [URDF] velocita' massima [rad/s]
cfg.q_min   = -2.6179939;   % [URDF] -150 gradi
cfg.q_max   =  2.6179939;   % [URDF] +150 gradi

%% ===== disturbo esterno (task di robustezza T6/T7) =====
% letto da fcn_get_disturbance dopo la riscrittura
cfg.dist.t0    = 2.0;                 % istante di inizio della spinta [s]
cfg.dist.dur   = 0.2;                 % durata [s]
cfg.dist.dir   = [0; 1; 0];           % direzione nel frame mondo
cfg.dist.frac  = 0.25;                % ampiezza come frazione di m*g
cfg.dist.point = [0; 0; 0];           % punto di applicazione nel frame corpo
cfg.dist.F     = cfg.dist.frac * cfg.mass * cfg.g;   % 3.83 N

%% ===== controlli di coerenza =====
% Non sono decorazioni: ognuno di questi ha gia' trovato un errore vero.
reach = norm([cfg.r_offset - cfg.lc, cfg.z0]) / cfg.reachMax;

assert(reach < 0.85, 'phantomx:config:reach', ...
    ['La posa nominale usa il %.1f%% dell''estensione della gamba.\n' ...
     'Sopra l''85%% la gamba lavora quasi dritta: l''IK diventa mal condizionata\n' ...
     'e il robot cammina all''indietro. Abbassa z0 o r_offset.'], 100*reach);

assert(cfg.mu_mpc <= cfg.mu_plant, 'phantomx:config:mu', ...
    ['L''MPC assume mu=%.2f ma il modello fisico ne ha %.2f.\n' ...
     'Un controllore che conta su piu'' attrito di quello disponibile\n' ...
     'pianifica forze irrealizzabili.'], cfg.mu_mpc, cfg.mu_plant);

assert(abs(cfg.S - cfg.v_nom*cfg.T_stance) < 1e-12, 'phantomx:config:passo', ...
    'S deve valere v_nom*T_stance, altrimenti il piede striscia in appoggio.');

assert(abs(cfg.mass - 1.560184) < 1e-5, 'phantomx:config:massa', ...
    'La massa totale non torna con m_body + 24*m_link.');

assert(abs(cfg.lt - cfg.foot_offset) < 1e-12, 'phantomx:config:tibia', ...
    ['lt (%.5f) e foot_offset (%.5f) devono coincidere: l''IK e la sfera di\n' ...
     'contatto devono riferirsi allo STESSO punto, altrimenti il piede\n' ...
     'finisce dove il controllore non crede e in appoggio striscia.'], ...
     cfg.lt, cfg.foot_offset);

if abs(cfg.body_z0 - cfg.body_z0_geom) > 0.01
    warning('phantomx:config:quota', ...
        ['body_z0 = %.3f m ma la geometria ne suggerisce %.3f m.\n' ...
         'Alla prima run il corpo scendera'' o salira'' di %.0f mm: verifica\n' ...
         'che il transitorio iniziale sia accettabile.'], ...
         cfg.body_z0, cfg.body_z0_geom, 1000*abs(cfg.body_z0-cfg.body_z0_geom));
end

%% ===== riepilogo =====
if verbose
    fprintf('\n=== PHANTOMX CONFIG ===\n');
    fprintf('  massa totale      %.4f kg  (corpo %.4f + 24 x %.6f)\n', ...
            cfg.mass, cfg.m_body, cfg.m_link);
    fprintf('  gamba             lc %.4f   lf %.4f   lt %.5f   -> reach %.5f m\n', ...
            cfg.lc, cfg.lf, cfg.lt, cfg.reachMax);
    fprintf('                    (tibia geometrica reale %.5f, non usata)\n', cfg.lt_mesh);
    fprintf('  posa nominale     r_offset %.3f   z0 %.3f   -> %.1f%% dell''estensione\n', ...
            cfg.r_offset, cfg.z0, 100*reach);
    fprintf('  impronta          %.3f m in lunghezza, %.3f m in larghezza\n', ...
            max(cfg.pf_nom(1,:))-min(cfg.pf_nom(1,:)), ...
            max(cfg.pf_nom(2,:))-min(cfg.pf_nom(2,:)));
    fprintf('  andatura          T %.2f s   beta %.2f   S %.3f m   H %.3f m\n', ...
            cfg.T, cfg.beta_stance, cfg.S, cfg.H);
    fprintf('  velocita'' teorica  %.4f m/s   (S / T_stance)\n', cfg.S/cfg.T_stance);
    fprintf('  attrito           modello %.2f   MPC %.2f\n', cfg.mu_plant, cfg.mu_mpc);
    fprintf('  quota corpo       %.3f m   (geometrica %.3f m)\n', ...
            cfg.body_z0, cfg.body_z0_geom);
    fprintf('  coppia max        %.2f N*m  (URDF dichiara 2.8, non usato)\n', cfg.tau_max);
    fprintf('  disturbo T6/T7    %.2f N per %.2f s da t = %.1f s\n', ...
            cfg.dist.F, cfg.dist.dur, cfg.dist.t0);
    fprintf('\n  piedi nominali (frame corpo, ordine CAN):\n');
    for i = 1:6
        fprintf('    %-3s  [% .4f % .4f % .4f]   alpha % 6.1f deg   fase %.2f\n', ...
            cfg.legNamesCAN{i}, cfg.pf_nom(:,i), rad2deg(cfg.alpha(i)), cfg.phase(i));
    end
    fprintf('\n');
end

end

% =====================================================================
% NOTE
%
% [DECISIONE APERTA] tibia 0.12 o 0.15297
%   Oggi il simulatore rappresenta un robot con tibia da 12 cm, in modo
%   internamente coerente (IK e sfera di contatto d'accordo). Il robot vero
%   ne ha 15.3. Passare al valore vero significa cambiare INSIEME:
%     - lt in inv_kyn.m  (dentro i sei blocchi MATLAB Function)
%     - cfg.lt e cfg.foot_offset
%     - la traslazione delle sei sfere di contatto -> [0 -0.15297 0]
%   e poi ritarare z0, perche' la posa nominale cambia. Va fatto prima
%   della campagna di misura, non durante, e va scritto nel report come
%   scelta di modellazione.
%
% [VERIFICA 2] cfg.J
%   L'inerzia del corpo rigido singolo e' stata stimata, non misurata dalle
%   mesh. Per il confronto MPC / cinematico e' un parametro che entra
%   direttamente nel modello di predizione: vale la pena ricalcolarla con
%   importrobot sull'URDF e confrontare.
%
%
% [VERIFICA 4] tau_max
%   1.5 N*m e' la coppia di stallo di datasheet, che un servo non eroga in
%   continuo. Se nelle metriche di sforzo attuativo la saturazione risulta
%   frequente, e' un risultato da discutere, non da nascondere alzando il
%   limite.
% =====================================================================