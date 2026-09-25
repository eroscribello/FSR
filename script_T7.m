%% script_T7.m - T7: spinta laterale impulsiva  [impianto SIMSCAPE]
%
% COSA MISURA, E PERCHE' PROPRIO QUESTO
%   C1 e' in anello aperto. C2 corregge la quota dei piedi con la ricerca del
%   terreno, ma non l'assetto ne' la direzione del corpo. Nessuno dei due ha
%   retroazione sullo stato del CORPO: dopo una spinta non hanno l'informazione
%   per tornare dov'erano.
%   E' l'unica affermazione su un controllore in forza che sia STRUTTURALE e
%   non numerica. Le altre sono cadute il 24/9: i limiti di coppia non servono
%   (in RMS il robot sta nel datasheet su ogni task) e il vantaggio in cot e'
%   al limite del pavimento di rumore.
%
% ================== [RITIRATO 24/9] LA PRIMA VERSIONE ==================
%   Misurava la deviazione rispetto a y(t0), cioe' alla quota laterale del
%   corpo un istante prima della spinta. Sbagliato, e si e' visto subito:
%
%       dv/vnom   0.00   0.25   0.50   1.00   2.00   4.00
%       dev_max   3.5    2.6    1.9    2.1    2.7    14.6   [mm]
%
%   La deviazione SENZA disturbo (3.5 mm) e' piu' grande che con disturbo a
%   quattro ampiezze su cinque. Non e' rumore numerico: e' l'ONDEGGIO LATERALE
%   PROPRIO DELL'ANDATURA A TRIPODE, che alterna i due terzetti e fa oscillare
%   il corpo di qualche millimetro per conto suo. Fino a dv = 2 la spinta e'
%   PIU' PICCOLA dell'ondeggio, quindi i numeri sono non monotoni e non
%   misurano niente.
%   Lo script infatti dichiarava "soglia di non recupero: dv = 0.00", cioe'
%   bocciava la propria run di controllo. Quando un esperimento boccia il
%   controllo, e' l'esperimento a essere sbagliato.
%
% ================== COME SI MISURA ADESSO: PER DIFFERENZA ==================
%   Il simulatore e' DETERMINISTICO - lo abbiamo misurato il 23/9, ed e' il
%   motivo per cui abbiamo deciso di non ripetere le campagne tre volte.
%   Quindi la run disturbata e quella indisturbata differiscono SOLO per il
%   disturbo, e la deviazione vera e'
%
%       dy(t) = y_disturbata(t) - y_indisturbata(t)
%
%   L'ondeggio dell'andatura si cancella esattamente, perche' e' identico
%   nelle due run. Resta solo la risposta alla spinta. Stessa cosa per
%   l'imbardata.
%
%   La run a dv = 0 e' quindi il RIFERIMENTO, non un punto della spazzata, e
%   deve stare per prima. Lo script lo impone.
%
% ================== IL PAVIMENTO DELLA MISURA DIFFERENZIALE ==============
%   Una differenza fra due run non e' esattamente zero nemmeno a parita' di
%   ingressi: il passo variabile del solutore lascia un residuo. Per sapere
%   quanto vale, la spazzata include dv = 0.01, cioe' una spinta trascurabile:
%   quello che ne esce e' il rumore della misura, non un effetto. Ogni
%   deviazione sotto quel valore va letta come zero.
%   E' la stessa regola di rumore_metriche, applicata qui.
%
% ================== I CRITERI, DICHIARATI PRIMA DELLE RUN ================
%     dev_max   = max |dy| dopo t0
%     dev_fin   = |dy| mediata sull'ultimo secondo
%     recupero  = 1 - dev_fin / dev_max        (1 = torna esattamente dov'era)
%
%   CADUTA      causa_fallimento == "ribaltamento", oppure la simulazione non
%               arriva in fondo, oppure z_media sotto il 60% del riferimento.
%   RECUPERO    non e' caduto, recupero >= 0.5, e |dyaw| <= 5 gradi.
%   SOTTO IL PAVIMENTO   dev_max sotto TRE VOLTE il valore della run
%               dv = 0.01. Il fattore 3 non e' scelto qui: e' la stessa regola
%               del pavimento di rumore del 23/9, dove una colonna discrimina
%               se il rapporto segnale/dispersione e' almeno 3. Una riga sotto
%               quella soglia non concorre a nessuna soglia di cedimento.
%   SOGLIE      il piu' piccolo impulso che NON recupera, e il piu' piccolo
%               che fa cadere. Sono due numeri diversi, si riportano entrambi.
%
% ================== AMPIEZZE ==================
%   J = m * dv, con dv multiplo di v_nom: "dv = 1" e' una spinta che vale
%   tutta la velocita' di marcia, leggibile senza conoscere la massa.
%   La spazzata sale fino a 16 perche' a 4 si era appena sopra l'ondeggio.
%   Stima del ribaltamento: portare il baricentro (154 mm) sul bordo del
%   poligono d'appoggio chiede ~0.9 m/s, cioe' dv ~ 7; con le zampe piantate
%   e l'attrito, di piu'. La spazzata deve arrivarci sopra.
%
% ================== [DECISO PRIMA 24/9] SE REGGONO TUTTO ================
%   Se C1 e C2 recuperano a ogni ampiezza, la causa probabile e' che in
%   ComputedTorque la loro coppia di reazione e' ILLIMITATA, mentre un
%   controllore in coppia avrebbe 1.5 N*m. In quel caso si rifa' T7 con gli
%   attuatori saturati al datasheet, per tutti e tre.
%   Deciso adesso, non dopo aver visto i numeri.
%
% ================== [DA FARE PRIMA DI RIVENDICARE IL RISULTATO] =========
%   dev_lat e yaw_err sono nella lista delle colonne RITIRATE dal pavimento di
%   rumore del 23/9. Quel pavimento era pero' misurato su run INDISTURBATE:
%   non dice che non sappiano risolvere una spinta. Qui la riga dv = 0.01
%   fornisce il pavimento specifico di questa misura, che e' il controllo che
%   serviva. Va citato in relazione insieme al risultato.
%
% COSA TOCCA DEL MODELLO
%   applica_disturbo aggiunge tre blocchi a run-time e NON salva mai: il .slx
%   del collega resta identico, basta bdclose. Come applica_terreno.
%
% ATTENZIONE AI NOMI: init_gait e' uno script e sovrascrive variabili del
% workspace. Tutte le variabili qui sono prefissate t7_.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   script_T7                      % C1 (t7_c2 = false)
%   poi si mette t7_c2 = true e si rilancia
%
% Progetto FSR PhantomX - A. Russo

t7_cfg  = phantomx_config();
t7_mdl  = 'phantomx_sim_zero';
t7_c2   = true;      % false = C1, anello aperto. true = C2.
t7_dur  = 12;         % [s]
t7_t0   = 4;          % [s] la spinta arriva a marcia regolare (transitorio 2 s)
t7_larg = 0.05;       % [s] larghezza dell'impulso
t7_dv   = [0 0.01 1 2 4 8 16];       % moltiplicatori di v_nom; il primo e' il
                                     % RIFERIMENTO, il secondo il PAVIMENTO

assert(t7_dv(1) == 0, 'script_T7:riferimento', ...
    'La prima ampiezza deve essere 0: e'' la run di riferimento, non un punto.');

t7_ctrl = t7_nome_ctrl(t7_c2);
t7_J    = t7_cfg.mass * t7_dv * t7_cfg.v_nom;     % [N*s]

fprintf('\nT7: controllore %s, spinta laterale a t = %.1f s, %d run\n', ...
        t7_ctrl, t7_t0, numel(t7_J));
fprintf('  v_nom = %.3f m/s, massa = %.4f kg\n', t7_cfg.v_nom, t7_cfg.mass);
fprintf('  misura DIFFERENZIALE rispetto alla run indisturbata\n');

t7_righe = table();
t7_y_rif = [];  t7_yaw_rif = [];  t7_z_rif = NaN;  t7_pav = NaN;
t7_DY = [];  t7_tt = [];          % le serie di Dy, per il grafico di fondo

for t7_i = 1:numel(t7_J)

    % ---- terreno, inerzie, disturbo: si fissano qui, non si ereditano ----
    applica_terreno('T7', false, t7_mdl);
    applica_inerzie(t7_mdl);
    % [25/9] E subito dopo lo stimatore, SEMPRE. Il blocco Inverse Dynamics che
    % produce tau_attesa per il flag di contatto di C2 usa un rigidBodyTree
    % importato dall'URDF: correggere le inerzie del robot e lasciare a lui
    % quelle vecchie rende |tau_mis - tau_att| grande ovunque, il flag resta
    % incollato a 1 e C2 smette di cercare il terreno. E' successo dal 22/9 al
    % 25/9. Vedi allinea_stimatore e docs/piano_confronto.md.
    allinea_stimatore(t7_mdl);
    applica_disturbo(t7_mdl, t7_J(t7_i), 'applica', ...
                     struct('t0', t7_t0, 'durata', t7_larg, 'asse', 'y'));

    OVERRIDE_C2 = t7_c2;                                       %#ok<NASGU>
    clear OVERRIDE_GAIT
    init_gait

    fprintf('\n--- dv = %.2f x v_nom,  J = %.4f N*s ---\n', t7_dv(t7_i), t7_J(t7_i));

    t7_out = sim(t7_mdl, 'StopTime', num2str(t7_dur));
    t7_run = adatta_simscape(t7_out, struct( ...
                 'controller', t7_ctrl, 'task','T7', 'run', t7_i, ...
                 'condizione', sprintf('dv%.2f', t7_dv(t7_i)), ...
                 'vel_d', [t7_cfg.v_nom 0]));

    clear OVERRIDE_C2
    init_gait                                                  % ripristina cfg

    t7_riga = metriche(t7_run, t7_cfg, struct('t_regime', 2*t7_cfg.T));

    % ---- la run di riferimento si memorizza e non si giudica ----
    if t7_i == 1
        t7_y_rif   = t7_run.p(:,2);
        t7_yaw_rif = t7_run.rpy(:,3);
        t7_z_rif   = t7_riga.z_media;
        t7_n_rif   = numel(t7_run.t);
    end

    % ---- la differenza rispetto alla run indisturbata ----
    % adatta_simscape ricampiona tutte le run sulla stessa griglia a 200 Hz,
    % quindi i vettori sono confrontabili campione per campione. Se non lo
    % fossero, meglio fermarsi che allineare a occhio.
    if numel(t7_run.t) ~= t7_n_rif
        error('script_T7:griglia', ...
            'Run %d ha %d campioni invece di %d: le griglie non coincidono.', ...
            t7_i, numel(t7_run.t), t7_n_rif);
    end
    t7_dy   = t7_run.p(:,2)   - t7_y_rif;
    t7_dyaw = t7_run.rpy(:,3) - t7_yaw_rif;

    if isempty(t7_tt), t7_tt = t7_run.t; end
    t7_DY(:, t7_i) = t7_dy;                                    %#ok<SAGROW>

    t7_dopo = t7_run.t >= t7_t0;
    t7_coda = t7_run.t >= t7_run.t(end) - 1;

    t7_dmax = max(abs(t7_dy(t7_dopo)));
    t7_dfin = abs(mean(t7_dy(t7_coda)));

    t7_riga.dv_su_vnom   = t7_dv(t7_i);
    t7_riga.impulso      = t7_J(t7_i);
    t7_riga.dev_max_mm   = 1e3 * t7_dmax;
    t7_riga.dev_fin_mm   = 1e3 * t7_dfin;
    t7_riga.dyaw_deg     = rad2deg(mean(t7_dyaw(t7_coda)));
    t7_riga.durata_sim   = t7_run.t(end);
    % la deviazione grezza, quella della prima versione: si tiene per mostrare
    % in relazione quanto vale l'ondeggio dell'andatura rispetto al segnale
    t7_riga.dev_grezza_mm = 1e3 * max(abs(t7_run.p(t7_dopo,2) - t7_run.p(find(t7_dopo,1),2)));

    if t7_dmax > 1e-9
        t7_riga.recupero = 1 - t7_dfin / t7_dmax;
    else
        t7_riga.recupero = NaN;
    end

    % ---- il pavimento della misura, dalla run a dv = 0.01 ----
    if t7_i == 2
        t7_pav = t7_dmax;
        fprintf('  [pavimento] la spinta trascurabile produce %.3f mm:\n', 1e3*t7_pav);
        fprintf('              sotto questo valore non si legge niente.\n');
    end
    % [CORRETTO 24/9] Non basta stare SOPRA il pavimento: serve un margine.
    % Il progetto ha gia' una regola per questo, dal pavimento di rumore del
    % 23/9: una quantita' discrimina se il rapporto segnale/dispersione e'
    % almeno 3. Qui vale identica. Senza il fattore 3 le righe a dv = 1 e 2
    % (1.46 e 1.64 mm contro un pavimento di 0.91) entravano nel conto delle
    % soglie pur essendo indistinguibili dal rumore, e la "soglia di non
    % recupero" usciva a dv = 0.01.
    % Le prime due righe sono il riferimento e il pavimento: non si giudicano.
    t7_riga.sotto_pavimento = (t7_i <= 2) || (t7_dmax < 3*t7_pav);

    % ---- i criteri, applicati come dichiarati in testa ----
    t7_riga.caduto = (t7_riga.causa_fallimento == "ribaltamento") || ...
                     (t7_run.t(end) < t7_dur - 1e-6) || ...
                     (t7_riga.z_media < 0.6 * t7_z_rif);

    % [AGGIUNTO 24/9] Il recupero non si dichiara su un RAPPORTO soltanto.
    % recupero = 1 - fin/max e' una frazione: puo' valere 0.6 anche quando il
    % robot ha recuperato due millimetri, cioe' niente. Perche' conti, la
    % QUANTITA' recuperata (max - fin) deve superare la stessa soglia di
    % leggibilita' di 3x il pavimento usata per il resto.
    % Scritto ADESSO, prima di misurare C3: se lo aggiungessimo dopo aver
    % visto un C3 che "recupera", sarebbe una soglia costruita sul risultato.
    t7_riga.recuperato = ~t7_riga.caduto && t7_riga.recupero >= 0.5 && ...
                         (1e-3*(t7_riga.dev_max_mm - t7_riga.dev_fin_mm) >= 3*t7_pav) && ...
                         abs(t7_riga.dyaw_deg) <= 5;

    if t7_i == 1
        % il riferimento non si giudica: e' il metro, non la misura
        t7_riga.recupero = NaN;  t7_riga.recuperato = true;
        t7_riga.sotto_pavimento = true;
    end

    fprintf('  Dy %.2f -> %.2f mm   recupero %.2f   Dyaw %+.2f deg   %s\n', ...
            t7_riga.dev_max_mm, t7_riga.dev_fin_mm, t7_riga.recupero, ...
            t7_riga.dyaw_deg, t7_esito(t7_riga));

    t7_righe = [t7_righe; t7_riga];                            %#ok<AGROW>

    t7_file = fullfile('results', sprintf('T7_%s.csv', t7_ctrl));
    writetable(t7_righe, t7_file);
end

%% ---- lettura ----
fprintf('\n=========== T7 %s ===========\n', t7_ctrl);
fprintf('  pavimento della misura differenziale: %.3f mm\n', 1e3*t7_pav);
fprintf('  soglia di leggibilita'' (3x il pavimento): %.3f mm\n\n', 3e3*t7_pav);
fprintf('  %8s %10s %10s %10s %9s %8s %10s  %s\n', 'dv/vnom', 'J [N*s]', ...
        'Dy max', 'Dy fin', 'recupero', 'Dyaw', 'grezza', 'esito');
for t7_i = 1:height(t7_righe)
    r = t7_righe(t7_i,:);
    fprintf('  %8.2f %10.4f %10.2f %10.2f %9.2f %8.2f %10.2f  %s\n', ...
            r.dv_su_vnom, r.impulso, r.dev_max_mm, r.dev_fin_mm, ...
            r.recupero, r.dyaw_deg, r.dev_grezza_mm, t7_esito(r));
end
fprintf('\n  "grezza" e'' la deviazione misurata come nella prima versione,\n');
fprintf('  cioe'' compreso l''ondeggio dell''andatura: serve a mostrare quanto\n');
fprintf('  segnale la misura differenziale recupera.\n');

t7_val    = t7_righe(~t7_righe.sotto_pavimento, :);
t7_i_norec = find(~t7_val.recuperato & ~t7_val.caduto, 1);
t7_i_cad   = find(t7_val.caduto, 1);

fprintf('\n  soglia di NON recupero : ');
if isempty(t7_i_norec), fprintf('non raggiunta nella spazzata\n');
else, fprintf('dv = %.2f x v_nom  (J = %.4f N*s)\n', ...
              t7_val.dv_su_vnom(t7_i_norec), t7_val.impulso(t7_i_norec)); end
fprintf('  soglia di CADUTA       : ');
if isempty(t7_i_cad), fprintf('non raggiunta nella spazzata\n');
else, fprintf('dv = %.2f x v_nom  (J = %.4f N*s)\n', ...
              t7_val.dv_su_vnom(t7_i_cad), t7_val.impulso(t7_i_cad)); end

if isempty(t7_i_norec) && isempty(t7_i_cad)
    fprintf(2, ['\n  %s recupera a ogni ampiezza provata. E'' il caso previsto in\n' ...
                '  testa al file: in ComputedTorque la coppia di reazione e''\n' ...
                '  illimitata, quindi il confronto con un controllore in coppia\n' ...
                '  sarebbe sbilanciato. Passo gia'' deciso: rifare T7 con gli\n' ...
                '  attuatori saturati al datasheet (%.2f N*m) per tutti e tre.\n'], ...
            t7_ctrl, t7_cfg.tau_max);
end

fprintf('\n  scritto %s\n', t7_file);
fprintf('  Il .slx non e'' stato salvato: bdclose all riporta tutto com''era.\n\n');

%% ---- il grafico ----
% [24/9] Mancava: c'era la chiamata a salva_grafico ma nessuna figura, e
% salva_grafico avvisa e non salva niente. Due pannelli, e il secondo e' il
% risultato in una figura sola.
t7_fig = figure('Name', sprintf('T7 %s', t7_ctrl), 'Position', [80 60 920 720]);

% pannello 1: la deviazione nel tempo, scala log perche' le ampiezze
% coprono tre decadi (0.9 -> 400 mm)
subplot(2,1,1); hold on
t7_col = lines(numel(t7_J));
t7_vis = t7_tt >= t7_t0 - 0.3;
for t7_k = 2:numel(t7_J)
    plot(t7_tt(t7_vis), max(abs(1e3*t7_DY(t7_vis,t7_k)), 1e-3), ...
         'Color', t7_col(t7_k,:), 'LineWidth', 1.3, ...
         'DisplayName', sprintf('dv = %g  (J = %.3f N s)', t7_dv(t7_k), t7_J(t7_k)));
end
set(gca, 'YScale', 'log')
yline(3e3*t7_pav, 'k--', 'soglia di leggibilita'' (3x pavimento)', ...
      'LabelHorizontalAlignment','left', 'HandleVisibility','off');
xline(t7_t0, 'k:', 'spinta', 'HandleVisibility','off');
grid on; box on
xlabel('tempo [s]')
ylabel('|\Deltay| rispetto alla run indisturbata [mm]')
title(sprintf('T7 %s - deviazione laterale dopo la spinta', t7_ctrl))
legend('Location','southeast')

% pannello 2: massimo e finale a confronto. Se coincidono, non si recupera.
subplot(2,1,2); hold on
plot(t7_righe.impulso(2:end), t7_righe.dev_max_mm(2:end), 'o-',  ...
     'LineWidth',1.5, 'MarkerFaceColor','w', 'DisplayName','\Deltay massima');
plot(t7_righe.impulso(2:end), t7_righe.dev_fin_mm(2:end), 's--', ...
     'LineWidth',1.5, 'MarkerFaceColor','w', 'DisplayName','\Deltay finale');
yline(3e3*t7_pav, 'k--', 'soglia di leggibilita''', ...
      'LabelHorizontalAlignment','left', 'HandleVisibility','off');
set(gca, 'XScale', 'log', 'YScale', 'log')
grid on; box on
xlabel('impulso [N s]')
ylabel('deviazione [mm]')
title('Le due curve coincidono: quello che si sposta, non torna')
legend('Location','northwest')

salva_grafico(sprintf('T7_%s', t7_ctrl));

%% ================= helper =================
function s = t7_nome_ctrl(c2)
if c2, s = 'C2'; else, s = 'C1'; end
end

function s = t7_esito(r)
if r.dv_su_vnom == 0,      s = 'RIFERIMENTO';
elseif r.caduto,           s = 'CADUTO';
elseif r.sotto_pavimento,  s = 'sotto il pavimento';
elseif r.recuperato,       s = 'recuperato';
else,                      s = 'in piedi, non torna';
end
end
