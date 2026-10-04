function info = applica_imbardata(yaw_d, verbose, mdl, opt)
% Traiettoria curva per il controllore cinematico.
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

if nargin < 1 || isempty(yaw_d),   yaw_d = 0;    end
if nargin < 2 || isempty(verbose), verbose = true; end
if nargin < 3 || isempty(mdl),     mdl = 'phantomx_sim_zero'; end
if nargin < 4, opt = struct(); end

cfg = phantomx_config();
if ~bdIsLoaded(mdl), load_system(mdl); end

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
