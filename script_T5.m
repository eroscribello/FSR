%% script_T5.m - T5: ostacolo singolo non modellato  [impianto SIMSCAPE]
%
% COSA MISURA
%   Lo scenario centrale di Arrigoni et al.: il terreno e' IGNOTO al
%   controllore. Il robot cammina dritto a velocita' nominale e incontra un
%   ostacolo che il generatore di traiettoria non conosce. C1 lo subisce, C2
%   dovrebbe adattare la quota delle zampe con la ricerca del terreno. La
%   quantita' su cui l'articolo misura il guadagno e' l'ASSETTO del corpo
%   durante il passaggio.
%
% COME SI TROVA IL PASSAGGIO, SENZA SAPERE DOVE STA L'OSTACOLO
%   La posizione dell'ostacolo e' scritta dentro la sua mesh, non in cfg.
%   Invece di ricavarla dalla geometria, lo script la ricava dall'EVENTO: un
%   piede FERMO nel mondo (in appoggio) e piu' alto del pavimento di almeno
%   t5_soglia_su e' un piede sull'ostacolo.
%
%   [CORRETTO] L'APPOGGIO E' CINEMATICO, NON DAI SENSORI. La prima versione
%   usava run.contact, cioe' i sensori di forza: ma i sensori del modello
%   misurano SOLO il contatto col pavimento. Un piede sull'ostacolo e' carico
%   e il sensore segna zero - verificato: nel passaggio la forza misurata va a
%   zero per oltre un secondo mentre i piedi stanno fermi a 25-45 mm. Lo script
%   si fermava dicendo "ostacolo non incontrato" con il robot sopra.
%   Un piede in appoggio e' fermo rispetto al mondo, su qualunque superficie:
%   velocita' sotto t5_v_appoggio, per almeno un quarto dell'appoggio nominale.
%   Non dipende da quali contatti il modello misura.
%
%   Il passaggio e' definito dal piede, non dall'effetto sul corpo: se si
%   definisse la finestra da dove l'assetto si scompone, si misurerebbe il
%   beccheggio dove il beccheggio e' grande, e il risultato sarebbe circolare.
%
%   Due guardie vengono da qui:
%     - nessun piede fermo sopra il pavimento -> l'ostacolo non e' stato
%       incontrato (fuori traiettoria, o durata troppo corta): errore;
%     - ultimo piede sull'ostacolo a meno di un ciclo dalla fine -> la run e'
%       finita col robot ancora sopra: avviso, e superato = false.
%
% COLONNE DI CONTATTO: VUOTE SE I SENSORI NON VEDONO TUTTO IL PESO
%   Per la stessa ragione, metriche calcola appoggio_medio, sotto3_frac,
%   slip_tot, disp_carico... da forze che sull'ostacolo mancano. Lo script
%   controlla la chiusura sul peso (forza media misurata / peso) e, se si
%   scosta oltre il 5%, mette quelle colonne a NaN invece di scrivere numeri
%   sbagliati. L'assetto, che e' la metrica di T5, viene dalla posa del corpo
%   e non ne risente.
%
% FINESTRA
%   [primo appoggio sull'ostacolo - T/2,  ultimo appoggio + T]
%   Mezzo ciclo prima per prendere un eventuale urto dello swing contro il
%   bordo (un piede che sbatte di lato tocca in basso e non viene riconosciuto
%   come "sopra"), un ciclo dopo per l'assestamento. Scelta dichiarata qui,
%   prima di guardare i dati.
%
% ASSETTO: ESCURSIONE RISPETTO AL PIANO, NON VALORE ASSOLUTO
%   C2 ha un beccheggio di fondo di ~0.035 rad anche su terreno piano, quasi
%   costante (T2 e T3). Il massimo assoluto nella finestra lo conterrebbe, e C2
%   risulterebbe peggio di C1 per un offset che con l'ostacolo non c'entra. La
%   metrica principale e' quindi l'ESCURSIONE: max |angolo - media in piano|,
%   con la media presa sul tratto piano a regime FUORI dalla finestra - prima
%   o dopo l'ostacolo. I valori assoluti restano in tabella accanto.
%
%   [CORRETTO] La prima versione prendeva solo il tratto PRIMA dell'ostacolo.
%   Ma l'ostacolo di T5 sta a 16 cm dalla partenza e il robot ci arriva a
%   1.2 s, prima della fine del transitorio (2 s): quel tratto non esiste, e
%   l'escursione usciva NaN. Il riferimento serve a togliere l'offset di
%   assetto in piano, che non dipende da quando lo si misura: dopo l'ostacolo
%   ci sono undici secondi di marcia regolare.
%
% ATTENZIONE AI NOMI: init_gait e' uno script e sovrascrive variabili del
% workspace. Tutte le variabili qui sono prefissate t5_.
%
% Progetto FSR PhantomX - A. Russo

t5_cfg       = phantomx_config();
t5_mdl       = 'phantomx_sim_zero';
t5_c2        = true;       % false = C1, anello aperto. true = C2.
t5_dur       = 20;          % [s] a 1.0x sono ~2.8 m: abbondante per superarlo
t5_soglia_su = 0.015;       % [m] appoggio piu' alto del piano = sull'ostacolo
t5_v_appoggio = 0.05;       % [m/s] piede piu' lento di cosi' = fermo, in appoggio
                            %       (lo swing va a ~0.24 m/s di media)
t5_ctrl      = t5_nome_ctrl(t5_c2);

% Il terreno si fissa qui, non si eredita.
applica_terreno('T5', false, t5_mdl);
% [23/9] Inerzie corrette in memoria: vedi applica_inerzie e piano_confronto 9.
applica_inerzie(t5_mdl);

fprintf('\nT5: controllore %s, ostacolo singolo, %g s a velocita'' nominale\n', ...
        t5_ctrl, t5_dur);

OVERRIDE_C2 = t5_c2;                                           %#ok<NASGU>
clear OVERRIDE_GAIT                                            % andatura nominale
init_gait

t5_out = sim(t5_mdl, 'StopTime', num2str(t5_dur));
t5_run = adatta_simscape(t5_out, struct( ...
             'controller', t5_ctrl, 'task','T5', 'run',1, ...
             'condizione', 'ost1', 'vel_d', [t5_cfg.v_nom 0]));

clear OVERRIDE_C2
init_gait                                                      % ripristina cfg

t5_riga = metriche(t5_run, t5_cfg, struct('t_regime', 2*t5_cfg.T));

%% ---- il passaggio ----
t5_P = t5_passaggio(t5_run, t5_soglia_su, t5_v_appoggio, t5_cfg, 2*t5_cfg.T);

%% ---- le colonne di contatto valgono solo se i sensori vedono tutto il peso ----
t5_chiusura = NaN;
if isfield(t5_run,'Fc') && ~isempty(t5_run.Fc)
    t5_sel = t5_run.t >= 2*t5_cfg.T;
    t5_chiusura = mean(sum(t5_run.Fc(t5_sel,3:3:18),2)) / (t5_cfg.mass*t5_cfg.g);
end
t5_riga.chiusura_peso = t5_chiusura;
if isnan(t5_chiusura) || abs(t5_chiusura - 1) > 0.05
    t5_contatto = intersect({'appoggio_medio','sotto3_frac','slip_tot','slip_per_passo', ...
        'distacchi','frazione_persa','Fz_max_norm','disp_carico', ...
        'appoggi_scartati','appoggi_totali'}, t5_riga.Properties.VariableNames);
    for t5_k = 1:numel(t5_contatto), t5_riga.(t5_contatto{t5_k}) = NaN; end
    t5_riga.note = t5_riga.note + sprintf( ...
        "; sensori chiudono al %.0f%% del peso (vedono solo il pavimento): colonne di contatto a NaN", ...
        100*t5_chiusura);
    fprintf(2, ['\n  I sensori chiudono al %.0f%% del peso: non vedono i piedi sull''ostacolo.\n' ...
                '  Colonne di contatto messe a NaN. L''assetto non ne risente.\n'], 100*t5_chiusura);
end

t5_riga.t_ingresso      = t5_P.t_ini;
t5_riga.t_uscita        = t5_P.t_fin;
t5_riga.x_ingresso      = t5_P.x_ini;
t5_riga.x_uscita        = t5_P.x_fin;
t5_riga.appoggi_su_ost  = t5_P.n_appoggi;
t5_riga.zampe_su_ost    = string(t5_P.zampe);
t5_riga.alt_ost         = t5_P.altezza;
t5_riga.roll_esc        = t5_P.roll_esc;
t5_riga.pitch_esc       = t5_P.pitch_esc;
t5_riga.roll_max_ost    = t5_P.roll_max;
t5_riga.pitch_max_ost   = t5_P.pitch_max;
t5_riga.roll_ref_piano  = t5_P.roll_ref;
t5_riga.pitch_ref_piano = t5_P.pitch_ref;
t5_riga.corpoZ_pp_ost   = t5_P.z_pp;
t5_riga.superato        = t5_P.uscito && t5_riga.causa_fallimento ~= "ribaltamento";

if ~isfolder('results'), mkdir('results'); end
t5_file = fullfile('results', sprintf('T5_%s.csv', t5_ctrl));
writetable(t5_riga, t5_file);

%% ---- lettura ----
fprintf('\n=============== T5 - %s ===============\n', t5_ctrl);
fprintf('  ostacolo alto ~%.1f mm (dalla quota dei piedi in appoggio)\n', 1e3*t5_P.altezza);
fprintf('  passaggio  t = %.2f .. %.2f s,  x = %.3f .. %.3f m\n', ...
        t5_P.t_ini, t5_P.t_fin, t5_P.x_ini, t5_P.x_fin);
fprintf('  appoggi sull''ostacolo: %d  (zampe: %s)\n', t5_P.n_appoggi, t5_P.zampe);
fprintf('\n                  escursione    massimo assoluto    media in piano\n');
fprintf('  rollio         %6.2f deg      %6.2f deg          %+6.2f deg\n', ...
        rad2deg(t5_P.roll_esc), rad2deg(t5_P.roll_max), rad2deg(t5_P.roll_ref));
fprintf('  beccheggio     %6.2f deg      %6.2f deg          %+6.2f deg\n', ...
        rad2deg(t5_P.pitch_esc), rad2deg(t5_P.pitch_max), rad2deg(t5_P.pitch_ref));
fprintf('  corpo in z     %6.1f mm picco-picco nella finestra\n', 1e3*t5_P.z_pp);
fprintf('\n  superato: %s   frazione del task %.0f%%   %s\n', ...
        t5_si(t5_riga.superato), 100*t5_riga.frazione_task, char(t5_riga.causa_fallimento));
fprintf('\n  scritto  %s\n\n', t5_file);

%% ---- grafico ----
figure;
subplot(3,1,1); hold on; grid on
plot(t5_run.t, rad2deg(t5_run.rpy(:,1)), 'DisplayName','rollio');
plot(t5_run.t, rad2deg(t5_run.rpy(:,2)), 'DisplayName','beccheggio');
t5_ombra(t5_P);
ylabel('[deg]'); legend('Location','best');
title(sprintf('T5 - %s: assetto (in grigio il passaggio sull''ostacolo)', t5_ctrl))
subplot(3,1,2); hold on; grid on
plot(t5_run.t, 1e3*t5_run.p(:,3));
t5_ombra(t5_P);
ylabel('z corpo [mm]');
subplot(3,1,3); hold on; grid on
plot(t5_run.t, 1e3*(t5_run.pf(:,3:3:18) - t5_P.z_piano));
yline(1e3*t5_soglia_su, 'k:');
t5_ombra(t5_P);
xlabel('t [s]'); ylabel('z piedi - piano [mm]');
salva_grafico(sprintf('T5_%s', t5_ctrl));   % grafici/<tag>.fig, testi modificabili dopo

%% ================= helper =================
function P = t5_passaggio(r, soglia, v_app, cfg, t_regime)
%T5_PASSAGGIO  Finestra del passaggio sull'ostacolo, ricavata dai piedi.
%   L'appoggio e' CINEMATICO: piede fermo nel mondo. Non si usano i sensori,
%   che vedono solo il pavimento.
T = cfg.T;
if ~isfield(r,'pf') || isempty(r.pf)
    error('script_T5:dati', 'Serve la posizione dei piedi (run.pf).');
end
t  = r.t(:);
dt = median(diff(t));
zf = r.pf(:, 3:3:18);                  % [N x 6] quota dei piedi, ordine CAN
fermo = false(size(zf));
dmin  = round(0.25 * cfg.T_stance / dt);   % campioni: un quarto di appoggio
for i = 1:6
    c = 3*(i-1) + (1:3);
    v = vecnorm(gradient(r.pf(:,c).', dt).', 2, 2);
    g = v < v_app;
    % scarta i tratti fermi troppo brevi: agli estremi dello swing il piede
    % rallenta per un istante senza essere appoggiato
    ini = find(diff([false; g]) ==  1);
    fin = find(diff([g; false]) == -1);
    for k = 1:numel(ini)
        if fin(k) - ini(k) + 1 < dmin, g(ini(k):fin(k)) = false; end
    end
    fermo(:,i) = g;
end
giu = fermo;

z_piano = median(zf(giu));             % dominata dal pavimento
su = giu & (zf - z_piano > soglia);    % fermo E sopra il pavimento
qualcuno = any(su, 2);

if ~any(qualcuno)
    error('script_T5:mancato', ...
        ['Nessun piede fermo sopra il pavimento di %.0f mm: l''ostacolo non ' ...
         'e'' stato incontrato.\nO e'' fuori dalla traiettoria del robot, o la ' ...
         'durata e'' troppo corta. Guarda l''animazione prima di cambiare le soglie.'], ...
         1e3*soglia);
end

i1 = find(qualcuno, 1, 'first');
i2 = find(qualcuno, 1, 'last');
P.uscito = t(i2) < t(end) - T;
if ~P.uscito
    warning('script_T5:corta', ['La run finisce col robot ancora sull''ostacolo: ' ...
            'aumenta t5_dur. La riga e'' scritta, con superato = false.']);
end

P.t_ini = max(t(1),   t(i1) - T/2);
P.t_fin = min(t(end), t(i2) + T);
fin = t >= P.t_ini & t <= P.t_fin;
P.x_ini = r.p(find(fin,1,'first'), 1);
P.x_fin = r.p(find(fin,1,'last'),  1);
P.z_piano = z_piano;
P.altezza = median(zf(su) - z_piano);
P.z_pp    = max(r.p(fin,3)) - min(r.p(fin,3));

% appoggi sull'ostacolo per zampa, e quali. I nomi vengono da
% cfg.legNamesCAN: le colonne di pf sono in ordine CAN, che NON e' quello dei
% To Workspace (adatta_simscape riordina).
nomi = cfg.legNamesCAN;
n = zeros(1,6);
for i = 1:6
    n(i) = sum(diff([false; su(:,i)]) == 1);
end
P.n_appoggi = sum(n);
P.zampe = strjoin(nomi(n > 0), ' ');

% riferimento in piano: a regime, FUORI dalla finestra (prima o dopo)
ref = t >= t_regime & ~(t >= P.t_ini & t <= P.t_fin);
if sum(ref) < 2*T / max(median(diff(t)), eps)
    warning('script_T5:piano', ['Meno di due cicli in piano fuori dal passaggio: ' ...
            'il riferimento dell''assetto e'' debole.']);
end
if any(ref)
    P.roll_ref  = mean(r.rpy(ref,1));
    P.pitch_ref = mean(r.rpy(ref,2));
else
    P.roll_ref = NaN;  P.pitch_ref = NaN;
end
P.roll_esc  = max(abs(r.rpy(fin,1) - P.roll_ref));
P.pitch_esc = max(abs(r.rpy(fin,2) - P.pitch_ref));
P.roll_max  = max(abs(r.rpy(fin,1)));
P.pitch_max = max(abs(r.rpy(fin,2)));
end

function t5_ombra(P)
%T5_OMBRA  Fascia grigia sul passaggio. Va chiamata DOPO i plot: prima, ylim
% vale ancora [0 1] e la fascia coprirebbe l'intervallo sbagliato.
yl = ylim;
h = patch([P.t_ini P.t_fin P.t_fin P.t_ini], [yl(1) yl(1) yl(2) yl(2)], ...
          [0.85 0.85 0.85], 'EdgeColor','none', 'HandleVisibility','off');
uistack(h, 'bottom');
ylim(yl);
end

function s = t5_nome_ctrl(c2)
if c2, s = 'C2'; else, s = 'C1'; end
end

function s = t5_si(b)
if b, s = 'si'; else, s = 'NO'; end
end
