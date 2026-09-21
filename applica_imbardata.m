function info = applica_imbardata(yaw_d, verbose, mdl, opt)
%APPLICA_IMBARDATA  Traiettoria curva per il controllore cinematico.
%
%   info = applica_imbardata(0.1)     imbardata di 0.1 rad/s, avanzando
%   info = applica_imbardata(0)       ripristina la marcia rettilinea
%
% OPZIONI (quarto argomento, per la diagnostica)
%   .v          [m/s] velocita' di avanzamento. Default cfg.S/cfg.T_stance.
%               Con .v = 0 si ha la ROTAZIONE SUL POSTO, che e' il caso
%               ESATTO: tutte le anche stanno a r_offset, quindi i sei moduli
%               |omega x r_i| sono uguali e il passo condiviso va bene cosi'
%               com'e'. Se la rotazione sul posto funziona e l'arco no, la
%               causa e' l'approssimazione sul modulo; se non funziona
%               nemmeno quella, e' aderenza.
%   .segno      +1 (default) o -1: inverte il delta di TUTTE le zampe.
%               +1 e' il valore geometrico, confermato da una misura in
%               sessione pulita. Va lasciato li': vedi la nota sul default.
%   .specchia   false (default). Se true, inverte il delta solo delle zampe
%               DESTRE. Serve a verificare se il parametro "side" di inv_kyn
%               specchia anche la rotazione del passo: in quel caso i
%               contributi dei due lati si cancellerebbero e l'imbardata
%               risulterebbe quasi nulla pur con i segni giusti.
%
% USO
%   info = applica_imbardata(cfg.yaw_d);
%   OVERRIDE_GAIT = struct('S', info.S);
%   init_gait
%   out = sim('phantomx_sim_zero','StopTime','15');
%
% IL PROBLEMA
%   tripod_trajectory ha  y = 0  cablato: genera solo marcia rettilinea. E i
%   blocchi di traiettoria nel modello sono DUE, non sei: le tre zampe di un
%   tripode condividono lo stesso (x, y, z). Un vettore di passo per zampa
%   sembrerebbe quindi impossibile senza riscrivere lo schema.
%
% LA SOLUZIONE, SENZA TOCCARE IL MODELLO
%   inv_kyn riceve alpha PER ZAMPA e lo usa solo per portare il comando dal
%   frame corpo al frame zampa (righe 42-44):
%       x_loc = r_offset + x*cos(alpha) + y*sin(alpha)
%       y_loc = (-x*sin(alpha) + y*cos(alpha)) * side
%   cioe' applica R_z(-alpha) al comando. Il montaggio fisico della zampa sta
%   in Simscape, non qui: alpha e' soltanto l'angolo con cui il comando viene
%   interpretato.
%
%   Passando alpha_i + delta_i, la posizione del piede nel frame corpo diventa
%       piede = R_z(-delta_i) * comando  +  r_offset * [cos alpha_i; sin alpha_i]
%   il secondo termine - la posa nominale della zampa - resta INVARIATO,
%   mentre la direzione del passo ruota di -delta_i. Un angolo per zampa, ed e'
%   esattamente la libertà che serve.
%
%   I sei alpha sono Constant cablati nel modello (deg2rad(45), ...). Qui
%   vengono riscritti con set_param SENZA salvare il modello, come fa
%   applica_terreno: il .slx sul disco non cambia.
%
% LA MATEMATICA
%   Perche' il piede non strisci, durante l'appoggio deve muoversi nel frame
%   corpo con velocita' opposta a quella del terreno visto dall'anca:
%       v_piede,i = -( v_d + omega x r_i ),   r_i = r_offset*[cos a_i; sin a_i]
%                 = -[ v - omega*r*sin(a_i) ;  omega*r*cos(a_i) ]
%   Il generatore muove il piede lungo -x del corpo. Imponendo che la
%   direzione coincida:
%       delta_i = atan2( -omega*r*cos(a_i),  v - omega*r*sin(a_i) )
%
% L'APPROSSIMAZIONE, DICHIARATA
%   La DIREZIONE del passo e' esatta per ogni zampa. Il MODULO no: S e' uno
%   solo, condiviso, mentre servirebbe |v_piede,i|*T_stance, diverso fra zampa
%   interna ed esterna alla curva. Qui S viene scelto sulla MEDIA dei sei
%   moduli, cosi' l'errore e' centrato invece di essere tutto da un lato.
%   L'errore residuo si scarica in scivolamento, ed e' misurato da
%   D.slip_tot: non e' un difetto nascosto, e' una grandezza in tabella.
%
%   Il confronto con la rotazione sul posto (v = 0) misura quanto costa questa
%   approssimazione: in imbardata pura tutti i moduli sono uguali a
%   omega*r_offset, quindi S condiviso e' esatto e lo scivolamento residuo e'
%   solo quello del controllore.
%
% ATTENZIONE
%   L'imbardata attesa va passata anche alle metriche, altrimenti
%   A.yaw_err_fin la confronta con zero:
%       run.meta.yaw_d = yaw_d;
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(yaw_d),   yaw_d = 0;    end
if nargin < 2 || isempty(verbose), verbose = true; end
if nargin < 3 || isempty(mdl),     mdl = 'phantomx_sim_zero'; end
if nargin < 4, opt = struct(); end

cfg = phantomx_config();
if ~bdIsLoaded(mdl), load_system(mdl); end

% [MISURATO] segno = +1, il valore geometrico. E' stato a -1 per tre giorni, e
% la storia va tenuta perche' e' un errore facile da ripetere.
%
% Venerdi' e' stato portato a -1 perche' script_T3 misurava l'imbardata di
% segno opposto al comando. Ma quella misura era fatta in una sessione MATLAB
% con stato residuo nel workspace: il robot aveva mezzo piede a terra (0.455)
% e beccheggiava di 4 gradi, quindi il segno era RUMORE. Invertirlo non
% cambiava niente, e la cosa andava letta come "la misura non e' valida", non
% come "il segno e' giusto comunque".
%
% In sessione pulita, con -1: arco 0.1 -> -14.7%, arco 0.4 -> -12.1%.
% Verso opposto, modulo riproducibile e lineare, imbardata parassita tremila
% volte piu' piccola: un'inversione vera, causata da quel -1.
%
% Riscontro indipendente: con specchia = true l'imbardata va a ~0, esattamente
% quello che il segno sbagliato predice - sinistre al contrario, destre giuste.
%
% Il segno si decide QUI e una volta sola, non nei sei Constant e non nella
% formula: la formula da' il valore geometrico, l'opzione porta quello
% misurato, e due posti che decidono lo stesso segno sono un posto di troppo.
def = struct('v', cfg.S/cfg.T_stance, 'segno', +1, 'specchia', false);
fo = fieldnames(def);
for q = 1:numel(fo)
    if ~isfield(opt,fo{q}), opt.(fo{q}) = def.(fo{q}); end
end

%% ---- i sei Constant di alpha, letti dal modello ----
% zampa, blocco, alpha nominale [gradi]
mappa = {
    'FL'  'Constant27'   45
    'ML'  'Constant26'   90
    'RL'  'Constant25'  135
    'FR'  'Constant21'  -45
    'MR'  'Constant20'  -90
    'RR'  'Constant19' -135
    };

r = cfg.r_offset;
v = opt.v;                         % velocita' di avanzamento comandata
w = yaw_d;

n     = size(mappa,1);
delta = zeros(1,n);
modulo = zeros(1,n);
mancanti = {};

for i = 1:n
    a = deg2rad(mappa{i,3});
    vx = v - w*r*sin(a);
    vy =   - w*r*cos(a);
    delta(i)  = atan2(vy, vx);
    modulo(i) = hypot(vx, vy);

    % varianti diagnostiche
    delta(i) = opt.segno * delta(i);
    if opt.specchia && mappa{i,3} < 0        % zampe destre: alpha negativo
        delta(i) = -delta(i);
    end
end

% S sulla media dei moduli: errore centrato, non tutto da un lato
S_nuovo = mean(modulo) * cfg.T_stance;

%% ---- scrittura dei Constant ----
for i = 1:n
    a_cmd = deg2rad(mappa{i,3}) + delta(i);
    blocco = sprintf('%s/%s', mdl, mappa{i,2});
    try
        if yaw_d == 0
            % ripristino: si rimette l'espressione originale, non il numero,
            % cosi' il modello resta leggibile a chi lo apre
            set_param(blocco, 'Value', sprintf('deg2rad(%d)', mappa{i,3}));
        else
            set_param(blocco, 'Value', sprintf('%.10g', a_cmd));
        end
    catch
        mancanti{end+1} = mappa{i,2};                                 %#ok<AGROW>
    end
end

info = struct('yaw_d',yaw_d, 'S',S_nuovo, 'S_nom',cfg.S, ...
              'delta_deg',rad2deg(delta), 'moduli',modulo, ...
              'zampe',{mappa(:,1).'}, 'modello',mdl, ...
              'v',opt.v, 'segno',opt.segno, 'specchia',opt.specchia);
info.errore_modulo = (max(modulo) - min(modulo)) / max(mean(modulo), eps);
info.slip_atteso   = info.errore_modulo * S_nuovo / 2;   % [m] per passo, ordine di grandezza

%% ---- riepilogo ----
if verbose
    fprintf('\n--- IMBARDATA: %.4g rad/s ---\n', yaw_d);
    if yaw_d == 0
        fprintf('  marcia rettilinea ripristinata (alpha ai valori nominali)\n');
    else
        fprintf('  %-4s %10s %12s\n', 'zampa', 'delta [deg]', '|v| [m/s]');
        for i = 1:n
            fprintf('  %-4s %10.3f %12.5f\n', mappa{i,1}, rad2deg(delta(i)), modulo(i));
        end
        fprintf('  raggio di curvatura   %.3f m\n', abs(v/max(abs(w),eps)));
        fprintf('  passo  %.5f -> %.5f m   (media dei moduli)\n', cfg.S, S_nuovo);
        fprintf('  dispersione dei moduli %.1f%%  -> scivolamento atteso ~%.1f mm/passo\n', ...
                100*info.errore_modulo, 1e3*info.slip_atteso);
        fprintf('\n  OVERRIDE_GAIT = struct(''S'', %.10g);\n', S_nuovo);
    end
    if ~isempty(mancanti)
        fprintf(2,'  blocchi non trovati: %s\n', strjoin(unique(mancanti), ', '));
    end
    fprintf('  (il modello NON e'' stato salvato)\n\n');
end

end
