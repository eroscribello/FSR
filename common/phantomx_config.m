function cfg = phantomx_config(verbose)
%PHANTOMX_CONFIG  Unica fonte di verita' dei parametri del PhantomX.
%
% PROVENIENZA DEI VALORI
%   [URDF]      letto da urdf/phantomx.urdf
%   [MESH]      misurato sulla geometria STL
%   [MODELLO]   letto dai blocchi di phantomx_sim_zero.slx
%   [DATASHEET] specifica del servo
%   [TARATO]    trovato sperimentalmente
%
% Progetto FSR PhantomX

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
cfg.legNamesCAN = {'FL','FR','ML','MR','RL','RR'};

% Mux di INGRESSO: porta gli angoli ai giunti. Verificato sul Mux8, e
% confermato dal fatto che il robot cammina dritto alla velocita' attesa.
cfg.legNamesMUX = {'RR','MR','FR','RL','ML','FL'};
cfg.mux2can = [6 4 2 5 3 1];
cfg.can2mux = [6 3 5 2 4 1];

% Mux di USCITA: Fleg e torque_sens. ORDINE DIVERSO da quello di ingresso,
% letto dal cablaggio dei due blocchi: lf lm lr rf rm rr.
% Non e' un refuso: nello stesso modello convivono due convenzioni. Se ne
% e' accorto solo lo scivolamento, perche' e' l'unica metrica per zampa:
% tutte le metriche di coppia sono aggregate e quindi invarianti all'ordine.
cfg.legNamesOUT = {'FL','ML','RL','FR','MR','RR'};
cfg.out2can = [1 3 5 2 4 6];
cfg.can2out = [1 4 2 5 3 6];

%cfg.jointIndex = @(i,j) 3*(cfg.can2mux(i)-1) + j;

cfg.jointIN  = @(i,j) 3*(cfg.can2mux(i)-1) + j;   % vettore verso i giunti (Mux ingresso)
cfg.jointOUT = @(i,j) 3*(cfg.can2out(i)-1) + j;   % vettori dai To Workspace (Mux uscita)

%% ===== geometria della gamba =====
cfg.lc = 0.054;    % [URDF] coxa,   da j_c2_* xyz = [0 -0.054 0]
cfg.lf = 0.0661;   % [URDF] femore, da j_tibia_* xyz = [0 -0.0645 -0.0145]
                   %        norm([0.0645 0.0145]) = 0.066112

% Posizione della sfera di contatto nel frame della tibia, come sta nei sei
% blocchi Rigid Transform1..6 del modello. E' il piede: l'IK comanda questo
% punto, e la sfera tocca terra un raggio piu' sotto.
cfg.foot_xyz    = [0.03, -0.15, 0];              % [MODELLO]
cfg.foot_offset = norm(cfg.foot_xyz(1:2));       % 0.152971 m
cfg.lt          = cfg.foot_offset;               % tibia vista dall'IK

% Punta geometrica della tibia, misurata sulla mesh tibia_l.STL: il centroide
% della zona terminale sta a [0.00155, 0.15985, 0.02879] nel frame del link,
% e l'asse del ginocchio e' x, quindi la lunghezza e' nel piano y-z.
cfg.lt_mesh = norm([0.15985, 0.02879]);          % [MESH] 0.162422 m
cfg.contact.foot_r = 0.01;

cfg.reachMax = cfg.lf + cfg.lt;                  % 0.219071 m

%% ===== posizione delle anche nel frame corpo, ordine CAN =====
cfg.p_hip = [ ...
     0.1248     0.1248     0.0      0.0     -0.1248    -0.1248 ; ...
     0.06164   -0.06164    0.1034  -0.1034   0.06164   -0.06164 ; ...
     0.001116   0.001116   0.001116 0.001116 0.001116   0.001116 ];   % [URDF]

cfg.alpha = deg2rad([45, -45, 90, -90, 135, -135]);   % [URDF] yaw di j_c1_* meno 90 gradi
cfg.side  = [ +1, -1, +1, -1, +1, -1 ];

%% ===== posa nominale =====
cfg.r_offset = 0.14;    % [TARATO] estensione radiale del piede dall'asse coxa
cfg.z0       = 0.14;    % [TARATO] profondita' d'appoggio sotto l'anca
cfg.z0_eff = 0.147344;  % [MISURATO] dall'equilibrio statico del modello:
                        % 0.155710 - foot_r + p_hip(3) + mg/(6k).
                        % NON dalla media di verifica_ik: quella usa una
                        % ricostruzione del frame URDF accurata a ~4 mm.                        % sotto l'anca, da verifica_ik con lt = 0.152971

cfg.pf_nom = zeros(3,6);
for i = 1:6
    cfg.pf_nom(:,i) = cfg.p_hip(:,i) + ...
        [cfg.r_offset*cos(cfg.alpha(i)); cfg.r_offset*sin(cfg.alpha(i)); -cfg.z0];
end

%% ===== masse e inerzie =====
cfg.m_body = 0.975599;
cfg.m_link = 0.024357719;
cfg.m_foot = 1000 * (4/3)*pi*cfg.contact.foot_r^3;
cfg.mass   = cfg.m_body + 24*cfg.m_link + 6*cfg.m_foot;

% [VERIFICATO 24/9] Inerzia del ROBOT INTERO visto come corpo rigido.
% [2/10] RESIDUO: la usava solo il modello di predizione dell'MPC, archiviato;
% nessun file vivo la legge. Si tiene perche' e' una grandezza misurata e
% verificata, e toglierla da cfg non guadagna niente. Non e' cfg.I_body,
% che e' del solo telaio: J comprende anche le zampe, che pesano il 38% del
% totale e stanno fino a 20 cm dall'asse.
% Verificata con calcola_J: composizione con gli assi paralleli dai valori
% corretti dei link. Sta dentro la forchetta fra i due casi limite (masse
% tutte all'anca / tutte al piede) e circa il 15% sotto il modello
% equispaziato.
% NON si puo' ricavare con importrobot: le inerzie dell'URDF sono quelle
% sbagliate di ~1000 volte (vedi applica_inerzie).
cfg.J       = diag([0.01444, 0.01751, 0.02889]);
cfg.I_body  = [3.557e-03, 5.154e-03, 8.565e-03];
cfg.I_c1    = [3.654e-06, 7.746e-06, 7.746e-06];
cfg.I_c2    = [3.654e-06, 3.654e-06, 3.654e-06];
cfg.I_thigh = [3.095e-06, 1.014e-05, 1.070e-05];
cfg.I_tibia = [2.081e-06, 3.004e-05, 3.050e-05];

%% ===== andatura a tripode =====
%
% DA DOVE VENGONO QUESTI VALORI.  [scritto il 2/10, dopo la domanda al
% ricevimento. Prima questa sezione era l'unica del file senza provenienza.]
%
% IL PAPER NON LI DA'. Arrigoni et al. delegano l'andatura a NUKE e lo
% dichiarano (p. 6):
%
%   "The gait engine employed is not fundamental to describe the desired
%    formulation and will not be discussed as descriptions are already
%    available in the literature (the gait engine employed in this treatment
%    is the one provided by NUKE)."
%
% Quindi periodo, duty, passo e sollevamento andavano scelti. Non sono
% arbitrari: tre su cinque sono fissati da un vincolo misurato, e il grado di
% liberta' residuo e' UNO SOLO. Il legame e'
%
%       T = S / (beta_stance * v_nom)
%
% cioe' T non e' una scelta indipendente: discende da S (geometria) e da
% v_nom (inviluppo di stabilita'). In relazione va presentato cosi'.

% --- phase: NON E' UN PARAMETRO, E' LA DEFINIZIONE ----------------------
% In ordine CAN (FL FR ML MR RL RR) questo vettore da' i due gruppi
%     {FL, MR, RL}   e   {FR, ML, RR}
% cioe' i due triangoli d'appoggio alternati. E' la definizione del tripode:
% non c'e' niente da giustificare e niente da tarare.
cfg.phase       = [0; 1/2; 1/2; 0; 0; 1/2];

% --- beta_stance: L'UNICO VALORE AMMISSIBILE ---------------------------
% [DEFINIZIONE] 0.50 e' l'unico duty che da' appoggio continuo con ESATTAMENTE
% tre zampe:
%   < 0.5  esistono istanti con MENO di tre piedi a terra, fuori dalla
%          stabilita' statica - che e' la condizione su cui il paper fonda
%          tutta l'architettura ("static stability in all intermediate
%          positions during movement", p. 11);
%   > 0.5  compaiono finestre a SEI piedi: 18 vincoli di posizione per 6
%          gradi di liberta', e i due tripodi si contendono il moto
%          attraverso la cedevolezza del contatto;
%   = 0.5  transizione istantanea fra i due triangoli, nessuna delle due.
cfg.beta_stance = 0.50;
cfg.duty_swing  = 1 - cfg.beta_stance;

% --- S: LIMITATO DALL'ESTENSIONE DELLA GAMBA ---------------------------
% [MISURATO] Il vincolo e' l'assert "phantomx:config:reach" in fondo a questo
% file: a meta' passo la gamba non deve superare l'85% di reachMax = lf + lt
% = 0.219 m. Con le zampe montate a +/-45 gradi lo spostamento di mezzo passo
% si somma quasi tutto alla componente radiale, quindi il vincolo morde.
%
%     S [m]     estensione usata a meta' passo
%     0.000     75.0%   (posa ferma)
%     0.060     80.9%   <- adottato
%     0.080     83.2%
%     0.095     85.0%   <- scatta l'assert
%     0.120     88.2%
%
% S = 0.06 sta al 63% del massimo geometrico. La soglia dell'85% non e'
% convenzionale: e' misurata, "a 89% il robot camminava all'indietro".
cfg.S           = 0.06;

% --- H: FISSATO DAGLI OSTACOLI DA SCAVALCARE ---------------------------
% [MISURATO] L'ostacolo di T5 misura 32.8 mm, letti dalla quota dei piedi in
% appoggio (colonna alt_ost di results/T5_C1.csv, non dalla geometria della
% mesh). T6 usa gli stessi sette prop. H = 50 mm e' 1.5 volte quell'altezza,
% e nella run tutte e sei le zampe ci salgono sopra (zampe_su_ost completo).
cfg.H           = 0.05;

% --- T: LA VARIABILE DIPENDENTE ----------------------------------------
% [VERIFICATO] Con S e beta fissati, T fissa la velocita':
%     v_nom = S / T_stance = 0.06 / 0.50 = 0.120 m/s
%
% La verifica sta in limite_velocita, che ha spazzato sei fattori fra 1.0x e
% 1.5x guardando il rimbalzo del corpo (soglia 10 mm, dichiarata prima):
%
%     fattore   rimbalzo
%       1.00     3.2 mm   <- nominale
%       1.20     3.4 mm   <- ultima cella con moto stabile
%       1.30     9.4 mm   <- al bordo
%       1.40    19.7 mm   <- cede
%
% Il nominale sta all'83% del limite misurato. E il numero di Froude
%     v^2/(g*h) = 0.120^2 / (9.81 * 0.154) = 0.0095
% colloca la marcia in regime quasi-statico, coerente con l'ipotesi di
% stabilita' statica su cui poggia l'andatura scelta: le forze d'inerzia
% valgono l'1% della gravita'.
cfg.T           = 1.00;

cfg.T_stance    = cfg.beta_stance * cfg.T;
cfg.T_swing     = cfg.duty_swing  * cfg.T;
cfg.v_nom       = cfg.S / cfg.T_stance;

%% ===== T2: la curva di velocita' =====
% DEFINIZIONE CANONICA, un posto solo. La leggono sia script_T2 (impianto
% Simscape, C1/C2) sia esegui_misure (impianto ridotto, C3): finche' stava
% scritta due volte i due CSV non erano confrontabili - esegui_misure usava
% tre fattori variando v, script_T2 cinque variando T.
%
% LA VARIABILE INDIPENDENTE E' LA VELOCITA' COMANDATA, non il periodo.
% Due ragioni:
%   1. tutte le metriche della famiglia A sono definite RISPETTO al comando
%      (meta.vel_d, err_vx_rms, avanzamento, frazione_task): se il comando
%      non e' la variabile di controllo, l'ascissa della curva e' una
%      grandezza derivata;
%   2. e' l'unica grandezza comune ai tre controllori. C1 e C2 accettano un
%      periodo, C3 accetta una velocita': confrontarli su "fattore di
%      cadenza" non avrebbe senso per C3.
% Il periodo si ricava:  T = S / (beta_stance * v).
%
% CINQUE PUNTI, NON TRE. Con 0.5x/1x/2x il risultato si legge come "degrada
% ad alta velocita'": i punti intermedi mostrano invece che l'ottimo e'
% STRETTO e che il cedimento e' di due tipi opposti (a bassa velocita'
% assestamento asimmetrico dentro la cedevolezza del contatto, ad alta
% perdita di appoggio). E' un risultato, e con tre punti non si vede.
%
% [MISURATO] 0.75x SOSTITUITO DA 1.20x, L'ULTIMA CELLA CON MOTO STABILE.
% Il terzo punto era 0.75x, scelto a occhio fra 0.5x e 1x. Ora e' 1.20x,
% scelto da limite_velocita su sei fattori fra 1.0 e 1.5.
%
% [CORRETTO 21/9] 1.20x NON SI CHIAMA PIU' "LIMITE DI C1".
% Il criterio dichiarato prima di guardare i dati aveva tre condizioni:
% tripode presente oltre il 95% del tempo, rimbalzo del corpo sotto 10 mm,
% avanzamento positivo. La prima era misurata con le forze RICOSTRUITE dalla
% penetrazione, e con quelle 1.20x la passava (2.2% sotto tre piedi).
%
% Dal 21/9 le forze vengono dai sensori del modello (Fleg), validati allo
% 0.0% sul peso. Con i sensori la stessa condizione boccia anche l'andatura
% NOMINALE: a 1.0x il robot sta sotto tre piedi l'11.7% del tempo. Un
% criterio che rifiuta il nominale e' tarato male, non e' un risultato: la
% soglia del 5% era calibrata sull'altra misura e non si trasferisce.
% Riscalarla sul nominale DOPO aver visto i dati vorrebbe dire scegliere il
% limite: x2 da' 1.0x, x3 da' 1.2x. Non si fa.
%
% Resta il criterio che non dipende dalla fonte delle forze, dichiarato prima
% e invariato: il moto del corpo.
%
%   fattore   task    rimbalzo   sotto 3 (sensori)
%     1.00   +116%     3.2 mm      11.7%
%     1.10   +115%     3.3 mm        -
%     1.20   +112%     3.4 mm      31.0%   <- ultima cella con moto stabile
%     1.30   +107%     9.4 mm        -        al bordo della soglia
%     1.40    +99%    19.7 mm        -
%     1.50    +83%    33.3 mm      51.5%
%
% In relazione vanno riportati DUE inviluppi, perche' dicono cose diverse:
%   - moto del corpo: C1 regge fino a 1.20x, e' al bordo a 1.30x (9.4 mm su
%     una soglia di 10, da una sola run: non basta per spostare il punto), e
%     cede a 1.40x;
%   - contatto misurato: il tripode e' al meglio a 1.0x (11.7%) e peggiora
%     GIA' a 1.20x (31%), dove pero' il corpo e' il piu' quieto di tutta la
%     curva (corpoZ_vz 0.018 m/s, il minimo).
% Fonderli in un numero solo nasconderebbe proprio questa divergenza.
%
% Un punto a mezza velocita' e uno all'ultima cella stabile dicono piu' di
% due punti bassi: 0.5x e' l'altro regime di cedimento, 1.20x il bordo.
cfg.t2_fattori = [0.5 1.0 1.20 1.5 2.0];

% QUALI PUNTI SONO CONFRONTABILI E QUALI NO.
% I primi tre hanno moto del corpo stabile e sono i punti su cui si
% confrontano C1, C2 e C3. Gli ultimi due sono FUORI: a 1.5x il robot
% saltella (rimbalzo 33 mm, 52% del tempo sotto tre piedi) e a 2.0x
% indietreggia.
%
% Vanno riportati, non scartati, ma con le metriche giuste: sotto3_frac e
% corpoZ_pp, non err_vx_rms. A quelle velocita' l'errore non e' un errore di
% inseguimento - i giunti eseguono la corsa comandata al 118% a tutte le
% velocita' - e' il robot che non cammina. Chiamarlo errore di inseguimento
% attribuirebbe a C1 un difetto che non ha, e a un controllore in anello
% chiuso un merito che non si e' guadagnato.
%
% E' anche l'ipotesi da verificare su C2 e C3: se camminano dove C1 rimbalza,
% il risultato e' che la retroazione ESTENDE L'INVILUPPO, con la soglia di C1
% misurata in anticipo invece che trovata a posteriori.
% Il nome t2_limite resta per compatibilita' con gli script: il valore e'
% l'ultima cella con moto stabile, vedi sopra.
cfg.t2_limite    = 1.20;              % [x] ultima cella con moto stabile
cfg.t2_confronto = [0.5 1.0 1.20];    % moto del corpo stabile
cfg.t2_fuori     = [1.5 2.0];         % fuori: casi non funzionanti

%% ===== contatto e terreno =====
% [TARATO] La rigidezza non e' una misura: e' un'assunzione sul terreno.
% Criterio: la penetrazione disponibile deve essere confrontabile con gli
% errori di assetto del corpo, altrimenti il modello di contatto diventa il
% fattore dominante del risultato invece del controllore.
%   k = 5000 -> penetrazione in tripode 1.04 mm, contro 3.29 mm di dislivello
%               ai piedi prodotto dall'inclinazione del corpo (media su 10 s,
%               picco 6.28). Due terzi dei piedi non potevano toccare: il
%               robot camminava a una zampa alla volta, 1.70 piedi a terra,
%               5% di tripode, 9-10 N per zampa invece di 5.18.
%   k = 1200 -> penetrazione 4.3 mm: 2.97 piedi a terra, 95% di tripode,
%               5.0-5.5 N per zampa.
% Lo stesso k vale per C1, C2 e C3: il confronto fra controllori non e'
% alterato da una proprieta' del terreno uguale per tutti.

cfg.contact.k = 1200;    % penetrazione statica 2.2 mm, in tripode 4.3 mm
cfg.contact.c = 50;      % 2*sqrt(k*mass/3)
cfg.contact.w = 2e-3;    % la forza sale su 2 mm invece di 1
cfg.contact.vcrit = 1e-2;   % [TARATO] regolarizzazione attrito, 10 mm/s

cfg.mu_plant = 0.9;
% [2/10] Tolto cfg.mu_mpc: era l'attrito che il QP dell'MPC assumeva nei
% vincoli di cono. L'MPC e' archiviato e nessun file vivo lo leggeva.

% [CORRETTO 25/9] floor_dim DESCRIVEVA UN BLOCCO CHE NON ESISTE PIU'.
%   Valeva [4 4 0.05]: erano le dimensioni del Brick Solid usato come pavimento
%   nelle prime versioni. Oggi il pavimento e' il File Solid con la mesh
%   ProvaPianoImperfettoCube.stl, che il bounding box misura
%       x,y in [-4, +4]   z in [-0.05, +0.05]     cioe' 8 x 8 x 0.1 m
%   Il vecchio valore era quindi la META' su ogni componente.
%   script_T4D lo aveva gia' annotato due volte ("cfg.floor_dim non lo
%   descrive") senza correggerlo.
%
%   IL RISULTATO NON CAMBIA, E VA VERIFICATO CHE NON CAMBI.
%   Con il valore vero, floor_off diventa 0.05 - che e' esattamente l'offset
%   scritto a mano nel Rigid Transform del pavimento - e la compensazione
%   cfg.terreno.ost_dz non serve piu' (vedi li'). La somma che arriva ai
%   blocchi e' la stessa di prima:
%       prima:  floor_off 0.025 + ost_dz 0.025 = 0.05
%       adesso: floor_off 0.050 + ost_dz 0     = 0.05
%   e floor_top resta 0. Quindi una campagna rifatta deve dare CSV identici:
%   se non li da', la correzione e' sbagliata e va rimessa indietro.
cfg.floor_dim = [8 8 0.1];    % [MISURATO 25/9] bounding box della mesh
cfg.floor_off = [0 0 +cfg.floor_dim(3)/2];
cfg.floor_top = -cfg.floor_off(3) + cfg.floor_dim(3)/2;

cfg.body_z0_geom = cfg.z0_eff + cfg.contact.foot_r + cfg.floor_top - cfg.p_hip(3,1);
cfg.body_z0 = cfg.body_z0_geom - cfg.mass*cfg.g/(6*cfg.contact.k);
% cfg.body_z0 = cfg.body_z0_geom + 0.002;          % 0.148366

% Rampa di T4.
% [MISURATO 22/9] PERCHE' I SEGNI DEI RIGID TRANSFORM DEL TERRENO SONO OPPOSTI.
% Tutti i Rigid Transform del terreno (pavimento, rampa, ostacoli) hanno il
% SOLIDO sulla porta B e il MONDO sulla porta F (ispeziona_rampa). L'offset
% scritto nel blocco e' quindi la posa del mondo vista dal solido, e il solido
% nel mondo sta nella posa INVERSA:  p = -R' * t,  rotazione R'.
% Per le sole traslazioni vuol dire x e z col segno cambiato: e' la regola
% trovata a tentativi su floor_off, z_spento e ost_dx.
%
% La rampa e' lo stesso cubo del pavimento (ProvaPianoImperfettoCube.stl,
% 8 x 8 x 0.1 m) ruotato. Con la posa del collega [-0.8 0 0.1], +Y 8 gradi:
% centro nel mondo a x = +0.81 m, superficie che esce dal pavimento a
% x = 0.36 m e SALE di 8 gradi verso +x (verificato a occhio). Il segno
% negativo di -0.8 era giusto: la rampa e' davanti.
% La meta' posteriore del cubo resta sotto il pavimento. E' solo estetica: i
% piedi non ci arrivano, e il contatto sopra il pavimento non cambia.
%
% [SCELTO 22/9] Spostata di 0.3 m in avanti, con lo stesso criterio di T5:
% a 0.36 m il robot ci arrivava verso i 2 s, alla fine del transitorio. Ora
% la superficie esce dal pavimento a x = 0.66 m.
%
% [CAMBIATO 22/9, sera] Non si scrive piu' l'offset del blocco ma DOVE la
% rampa esce dal pavimento: l'offset lo calcola applica_terreno (posa_rampa)
% per qualunque angolo. Serve alla ricerca dell'angolo limite, dove la rampa
% deve cominciare sempre nello stesso punto. A 8 gradi la superficie sopra il
% pavimento e' la stessa delle misure di T4 (stesso piano, stesso punto
% d'uscita); cambia solo quanto cubo resta interrato.
cfg.terreno.rampa_x_inizio = 0.66;         % [m, mondo] dove la superficie esce dal pavimento
cfg.terreno.rampa_gradi    = 8;            % salita verso +x
cfg.terreno.rampa_stl      = 'simscape\props\ProvaPianoImperfettoCube.stl';   % file originale del solido

% [SCELTO 22/9] Dosso di T4D: salita, cima piana, discesa, poi di nuovo piano.
% Geometria in dosso_profilo, STL scritto da applica_terreno('T4D').
%   - stessa partenza e stessa pendenza di T4: la salita di T4D e' confrontabile
%   - H 14 cm [CAMBIATO 22/9, era 12]: discesa di 1.0 m, quasi il doppio
%     dell'ingombro del robot (~0.53 m fra piede piu' arretrato e piu' avanzato),
%     quindi qualche ciclo con tutte e sei le zampe in discesa. 14 cm e' il
%     massimo che sta nel pavimento (finisce a x = 4 m) tenendo partenza a
%     0.66 m, cima di 0.6 m e ~2.5 cicli in piano dopo il dosso. L'altezza non
%     cambia la difficolta' degli spigoli (dipende dall'angolo), allunga solo
%     i tratti inclinati.
%   - cima 0.6 m: il robot ci sta tutto per un tratto, le due transizioni in
%     alto (salita->cima, cima->discesa) non si sovrappongono
%   - larghezza 2 m: la deriva laterale misurata e' sotto i 0.3 m
% Fondo della discesa a x ~ 3.25 m: a velocita' nominale ~27 s, da cui i 30 s
% di script_T4D, che taglia l'analisi se un piede arriva al bordo del pavimento.
cfg.terreno.dosso = struct('x_inizio', 0.66, 'H', 0.14, 'gradi_su', 8, ...
                           'gradi_giu', 8, 'L_cima', 0.6, 'larghezza', 2, ...
                           'spessore', 0.02);

%% ===== terreno per task =====
% I sette Rigid Transform degli ostacoli leggono terreno.off(:,k): quale
% ostacolo e' presente si sceglie da script con applica_terreno, senza mai
% modificare il .slx. Un ostacolo "spento" viene parcheggiato sotto il
% pavimento, dove non incontra mai un piede.
cfg.terreno.n_prop   = 7;
% [MISURATO] Positivo = verso il basso: i Rigid Transform del pavimento e
% degli ostacoli hanno l'asse z opposto a quello del mondo, come gia' visto
% su floor_off. Con -5 gli ostacoli finiscono in aria sopra il robot.
cfg.terreno.z_spento = +5;        % [m] quota di parcheggio
% [RITIRATO 25/9] ERA LA COMPENSAZIONE DI UN floor_dim SBAGLIATO.
%   Il commento precedente diceva: "gli ostacoli sono posati sul Cube, la cui
%   superficie sta 25 mm sopra quella del pavimento liscio (Brick: spessore
%   0.05 centrato a 0.025 -> 0.050; Cube: mesh +-0.05 -> 0.075)".
%   Era vero quando il pavimento era un Brick Solid. Oggi il pavimento E' il
%   Cube: la sua superficie e quella su cui poggiano gli ostacoli sono la
%   stessa cosa, e non c'e' nessun dislivello da compensare.
%   I 25 mm servivano solo perche' floor_dim era la meta' del vero, quindi
%   floor_off veniva 0.025 invece di 0.05. Corretto floor_dim, la somma torna
%   da sola e questo termine va a zero.
%
%   MISURATO sulle mesh (bounding box, 25/9): gli ostacoli hanno la base a
%   z = +0.05015 locale, cioe' 0.15 mm sopra la faccia superiore del Cube
%   (+0.05). Poggiano gia' sul pavimento per costruzione.
%
%   Resta il meccanismo, se un giorno servisse: positivo = verso il basso,
%   perche' la z di quei Rigid Transform e' opposta a quella del mondo.
cfg.terreno.ost_dz = 0;           % [m]

% [SCELTO 21/9] Spostamento degli ostacoli lungo il percorso, PER TASK.
% L'ostacolo 1 sta a 16 cm dalla partenza: in T5 il robot ci arrivava a 1.2 s,
% prima della fine del transitorio (2 s), senza un tratto di avvicinamento in
% piano, e l'impatto si sovrapponeva all'avvio. Spostato di 0.5 m in avanti,
% il robot ci arriva dopo circa 4.5 s di marcia regolare.
% Per task e non globale: l'ostacolo 1 e' anche in T6, e spostarlo li'
% cambierebbe la disposizione degli altri sei, che non e' stata verificata.
% applica_terreno lo somma a floor_off a runtime: il .slx non si tocca.
% [MISURATO] Segno NEGATIVO per andare avanti. Con +0.5 script_T5 ha dato
% x_ingresso 0.079 m invece di 0.165, e il primo appoggio a 0.61 s invece di
% 1.21: l'ostacolo si era AVVICINATO. Come la z, anche la x dei Rigid
% Transform degli ostacoli e' opposta a quella del mondo.
cfg.terreno.ost_dx.T5 = [-0.5 0 0 0 0 0 0];  % [m] uno per ostacolo

cfg.terreno.task.T1 = [];         % piano
cfg.terreno.task.T2 = [];         % piano, curva di velocita'
cfg.terreno.task.T3 = [];         % piano, traiettoria curva
cfg.terreno.task.T4 = [];         % rampa (rampa_x_inizio, rampa_gradi)
cfg.terreno.task.T4D = [];        % dosso: salita, cima, discesa (terreno.dosso)
cfg.terreno.task.T5 = 1;          % ostacolo singolo
cfg.terreno.task.T6 = 1:7;        % ostacoli multipli

% [26/9] TUTTE LE RUN DI T6 FINISCONO ALLA STESSA x, NON ALLO STESSO TEMPO.
%   Con una durata fissa un controllore piu' veloce percorre piu' strada, e
%   siccome gli ostacoli di T6 stanno a x fisse, incontra ostacoli diversi:
%   alt_ost, appoggi_su_ost e l'assetto finiscono per confrontare percorsi
%   diversi invece che controllori. E' successo il 25/9 fra due terne di
%   soglia (3.09 m contro 3.97 m) e succedera' di nuovo con C3, che se
%   funziona sara' piu' veloce.
%   Tagliando a una x comune ogni riga copre LO STESSO TRATTO DI PISTA, e
%   "chi e' piu' veloce" diventa una colonna esplicita (t_a_fine) invece di
%   un effetto collaterale che sporca tutte le altre.
%   3.40 m: l'ultimo ostacolo e' a ~3.3 m, il bordo del pavimento a 3.95 m e
%   i piedi precedono il corpo di ~25 cm, quindi restano ~30 cm di margine.
%   [DISATTIVATO 26/9] Inf = nessun taglio, le run di T6 restano intere.
%   Il motivo: su T6 la differenza fra i controllori e' GROSSOLANA - C1 si
%   ferma a 2.44 m e non raggiunge gli ostacoli finali, C2 arriva a 3.71 m.
%   1.3 metri sono 13-45 volte il rumore misurato su distanza (0.03-0.10 m):
%   per dirlo non serve una finestra comune. E con x_fine = 3.40 C1 non ci
%   arriva nemmeno, quindi il taglio produceva solo un avviso e una riga
%   incomparabile.
%   RESTA PERO' VERO che roll, pitch, cot e tau su T6 dipendono dal percorso
%   e NON vanno citati: quei confronti li danno T5 e T4D, che hanno il loro
%   pavimento di rumore. Da T6 si prendono distanza, frazione_task e
%   arrivato, che sono robusti al percorso.
%   Per riattivarlo basta rimettere un valore finito: la macchina del taglio
%   e' in script_T6 ed e' documentata li'.
cfg.terreno.T6_x_fine = Inf;      % [m] Inf = run intera
cfg.terreno.task.T7 = [];         % piano, disturbo impulsivo

%% ===== attuatori =====
% [VERIFICATO 1/10] FONTE: docs/ROBOTIS_AX-12A_emanual.pdf, pag. 2-3, cioe'
%   https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/ (consultato 1/10/2026),
%   lo stesso riferimento [40] di Arrigoni et al. (pag. 4: il PhantomX AX
%   Mark II monta 18 Dynamixel AX-12A). Fino all'1/10 il valore era scritto
%   [DATASHEET] senza fonte nel repo: era stato dato a memoria e mai letto.
%     Stall Torque    1.5 N*m a 12 V, 1.5 A          -> tau_max
%     No Load Speed   59 rpm a 12 V = 6.18 rad/s     (l'URDF dice 5.65)
%     Input Voltage   9-12 V, raccomandata 11.1 V    (a 11.1 V lo stallo e' ~1.4)
%     NOTA ROBOTIS    "Stable motions are possible with robots designed for
%                     loads with 1/5 or less of the stall torque" -> 0.3 N*m
%   Lo stallo e' un massimo istantaneo, non un limite di lavoro: il carico
%   continuo raccomandato e' tau_lavoro.
cfg.tau_max    = 1.5;       % [DATASHEET] stallo a 12 V. L'URDF dichiara 2.8: ottimistico
cfg.tau_lavoro = cfg.tau_max/5;   % [DATASHEET] carico per moto stabile, 1/5 dello stallo
cfg.qd_max  = 5.6548668;    % [URDF] il datasheet da' 6.18 rad/s a 12 V
cfg.q_min   = -2.6179939;   % [URDF]
cfg.q_max   =  2.6179939;   % [URDF]

%% ===== disturbo esterno =====
cfg.dist.t0    = 2.0;
cfg.dist.dur   = 0.2;
cfg.dist.dir   = [0; 1; 0];
cfg.dist.frac  = 0.25;
cfg.dist.point = [0; 0; 0];
cfg.dist.F     = cfg.dist.frac * cfg.mass * cfg.g;



%% ===== switch C1 / C2 =====
% cfg.c2.attiva e' L'INTERRUTTORE VERO, e arriva ai due blocchi MATLAB
% Function come c2_par(1) (init_gait lo assembla).
%
% [CORREZIONE] La versione precedente di questo commento diceva che bastava
% portare la soglia di coppia a infinito per avere l'anello aperto. E' FALSO:
% con la soglia a infinito il flag di contatto e' sempre falso, e con il flag
% falso la ricerca del terreno entra nel ramo di DISCESA a ogni appoggio -
% z0 = 0.14 contro la soglia z_nom - tol = 0.138 - scendendo di z_ext_max a
% v_search. E' il contrario dell'anello aperto.
cfg.c2.attiva     = true;    % false = C1, anello aperto vero
% [MISURATO 23/9] LA SOGLIA NON E' UNO SCALARE, E NON E' 0.5.
% Nel modello il confronto e' fatto da un Relational Operator fra un Mux di
% tre coppie (coxa, femore, tibia) e il Constant [0.04 0.1 0.1]: una soglia
% PER GIUNTO. Il flag di contatto della zampa e'
%     cont = OR( |tau_misurata - tau_attesa| > soglia )  sui tre giunti,
% quindi basta un giunto fuori tolleranza per dichiarare contatto. E' la
% forma del paper (Arrigoni et al. §5), non una soglia di coppia secca.
% Il valore 0.5 che stava qui prima NON e' mai arrivato al modello: init_gait
% calcolava c2_soglia e nessun blocco la leggeva (verificato con
% trova_nel_modello e Simulink.findVars il 23/9). Adesso il Constant vale
% c2_soglia e questo e' il valore vero, quindi il comportamento non cambia.
% [MISURATO 25/9] LA TERNA E' STATA TARATA DA NOI, E NON E' PIU' UN OR SU TRE
% GIUNTI: LA COXA E' DI FATTO ESCLUSA.
%   Fino al 25/9 qui c'era [0.04 0.1 0.1], il valore che sta nel Constant del
%   modello. Validandolo (valida_soglia, dopo allinea_stimatore) e' venuto
%   fuori che sulla COXA le due distribuzioni di |tau_mis - tau_att| si
%   sovrappongono: p90 in volo 0.0421 contro p10 a terra 0.0469. La coxa
%   ruota attorno all'asse verticale, quindi il contatto - che e' verticale -
%   le trasmette pochissima coppia, mentre in volo porta tutta
%   l'accelerazione laterale della zampa. Rapporto segnale/rumore basso per
%   costruzione: con 0.04 il flag scattava sul 39.7% della fase di volo.
%   soglia_ottima ha scelto da solo di portarla a 0.385, sopra la sua stessa
%   mediana in appoggio: non e' una taratura piu' fine, e' spegnerla. Il
%   contatto lo rilevano femore e tibia, che separano bene.
%
%   MISURA CHE L'HA DECISA (results/diagnostica/prova_soglia_T6*.csv):
%   T6, C2, due run, metriche su finestra comune fino a x = 3.043 m:
%                        [0.04 0.1 0.1]   [0.385 0.136 0.105]
%     esce dagli ostacoli      MAI            t = 21.55 s
%     roll_max                0.160            0.122
%     roll_rms                0.0580           0.0286
%     dev_lat_max             0.539            0.281
%     yaw_err_fin            -0.546           +0.098
%     cot                      2.085            1.330
%     energia [J]            100.2             63.2
%     tau_max [N*m]            6.86             8.38   <-- unico peggioramento
%   Su T2, terreno piano, le due sono equivalenti: e' su T6 che si decide,
%   perche' li' la ricerca del terreno serve davvero.
%
%   TRE CIFRE E NON QUATTRO: l'ottimo viene da una run sola, la quarta cifra
%   sarebbe una precisione che non abbiamo.
%
%   VA DICHIARATO IN RELAZIONE: questo parametro NON e' piu' quello del
%   modello di partenza, l'abbiamo scelto noi con il criterio qui sopra.
cfg.c2.soglia_tau = [0.385 0.136 0.105];   % [N*m] coxa, femore, tibia

% Parametri della ricerca del terreno (Arrigoni et al. §5). Erano cablati
% dentro i due blocchi MATLAB Function, quindi invisibili a git e duplicati:
% z_nominal valeva 0.14, cioe' cfg.z0 riscritto a mano.
cfg.c2.z_nom     = cfg.z0;   % [m] profondita' oltre cui si considera
                             %     "fase di abbassamento". Legata a z0: se si
                             %     ritara z0, la soglia segue
cfg.c2.v_search  = 0.06;     % [m/s] velocita' di discesa in ricerca
cfg.c2.z_ext_max = 0.03;     % [m] estensione massima sotto la nominale
cfg.c2.t_reset   = 0.5;      % [s] nessuna ricerca nel transitorio iniziale
cfg.c2.tol       = 0.002;    % [m] tolleranza sulla soglia di abbassamento

%% ===== T3: imbardata =====
% Il comando di imbardata per la traiettoria curva. Lato cinematico si
% realizza ruotando la direzione del passo di ciascuna zampa: vedi
% applica_imbardata, che riscrive i sei Constant di alpha a runtime senza
% salvare il modello.
cfg.yaw_d = 0.1;             % [rad/s]

%% ===== controlli di coerenza =====
% Estensione della gamba nel caso PEGGIORE del ciclo, non nella posa ferma.
% Durante l'appoggio il piede si sposta di +/- S/2 rispetto al nominale, e per
% le zampe montate a 45 gradi quello spostamento si somma quasi tutto alla
% componente radiale: controllare solo il nominale lascia passare pose che a
% meta' passo arrivano oltre il 90%.
reach     = 0;
reach_nom = norm([cfg.r_offset - cfg.lc, cfg.z0]) / cfg.reachMax;
for i = 1:6
    for xs = [-cfg.S/2, +cfg.S/2]
        xl = cfg.r_offset + xs*cos(cfg.alpha(i));
        yl = -xs*sin(cfg.alpha(i))*cfg.side(i);
        tx = hypot(xl, yl) - cfg.lc;
        reach = max(reach, hypot(tx, cfg.z0) / cfg.reachMax);
    end
end

assert(reach < 0.85, 'phantomx:config:reach', ...
    ['A meta'' passo la gamba usa il %.1f%% dell''estensione massima\n' ...
     '(%.1f%% nella posa ferma). Sopra l''85%% l''IK si mal condiziona:\n' ...
     'a 89%% il robot camminava all''indietro. Riduci r_offset o z0.'], ...
     100*reach, 100*reach_nom);

assert(abs(cfg.lt - cfg.foot_offset) < 1e-12, 'phantomx:config:tibia', ...
    'lt e foot_offset devono coincidere: l''IK e la sfera di contatto\ndevono riferirsi allo stesso punto.');

assert(cfg.foot_offset < cfg.lt_mesh, 'phantomx:config:sfera', ...
    ['Il centro della sfera (%.5f) e'' oltre la punta della tibia (%.5f):\n' ...
     'la sfera sporgerebbe fuori dal piede.'], cfg.foot_offset, cfg.lt_mesh);



assert(abs(cfg.S - cfg.v_nom*cfg.T_stance) < 1e-12, 'phantomx:config:passo', ...
    'S deve valere v_nom*T_stance, altrimenti il piede striscia in appoggio.');

assert(abs(cfg.mass - 1.585317) < 1e-5, 'phantomx:config:massa', ...
    'La massa non torna con m_body + 24*m_link + 6*m_foot.');

assert(cfg.mass*cfg.g/(3*cfg.contact.k) > 2e-3, 'phantomx:config:contatto', ...
    ['Penetrazione in tripode %.2f mm: troppo poco per assorbire gli errori\n' ...
     'di assetto del corpo (misurati ~3.3 mm). Sotto i 2 mm il robot torna a\n' ...
     'camminare a una zampa alla volta.'], 1000*cfg.mass*cfg.g/(3*cfg.contact.k));

%% ===== riepilogo =====
if verbose
    fprintf('\n=== PHANTOMX CONFIG ===\n');
    fprintf('  gamba        lc %.4f  lf %.4f  lt %.6f   -> reach %.6f m\n', ...
            cfg.lc, cfg.lf, cfg.lt, cfg.reachMax);
    fprintf('               punta della tibia %.6f, sfera un raggio dentro\n', cfg.lt_mesh);
    fprintf('  posa         r_offset %.3f  z0 %.3f  -> %.1f%% da fermo, %.1f%% a meta'' passo\n', ...
            cfg.r_offset, cfg.z0, 100*reach_nom, 100*reach);
    fprintf('  andatura     T %.2f s  appoggio %.2f s  passo %.3f m  -> %.3f m/s\n', ...
            cfg.T, cfg.T_stance, cfg.S, cfg.v_nom);
    fprintf('  quota corpo  %.3f m (geometrica)\n', cfg.body_z0);
    fprintf('  attrito      modello %.2f\n\n', cfg.mu_plant);
end

end