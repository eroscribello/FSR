function S = soglia_deriva_T6(verbose)
%SOGLIA_DERIVA_T6  Quanta deriva laterale basta per mancare un ostacolo di T6.
%
%   S = soglia_deriva_T6
%   S.soglia     [m] la soglia su dev_lat_max
%   S.per_ost    tabella ostacolo per ostacolo
%
% LA DOMANDA
%   [1/10] Su T6 C2 risulta "superato" deviando di 0.30 m di lato, mentre C1
%   e C3 deviano di 0.07 m e non lo superano. Nell'animazione le zampe destre
%   di C2 non toccano l'ultimo ostacolo: gli passa di fianco. Allora
%   "superato" non dice se il robot e' passato SOPRA gli ostacoli, e va letto
%   insieme alla deriva. Serve una soglia su dev_lat_max oltre la quale
%   "superato" non vale.
%
% PERCHE' DALLA GEOMETRIA
%   La soglia non si sceglie guardando i valori dei controllori: si ricava
%   da quanto bisogna spostarsi di lato perche' nessun piede tocchi un
%   ostacolo. Dipende dalle mesh e dalla carreggiata, non da C1, C2 o C3.
%
% LA DEFINIZIONE, SCRITTA PRIMA DI CALCOLARE
%   - piedi: posa nominale cfg.pf_nom (y nel corpo), raggio cfg.contact.foot_r.
%     Si trascurano l'imbardata e l'oscillazione del passo: in marcia dritta
%     la y dei piedi resta quella nominale;
%   - ostacolo k: estensione in y dalla sua mesh. Un piede lo puo' toccare
%     se la sua y cade in [y_min - r, y_max + r];
%   - d_manca(k) = il piu' piccolo spostamento laterale del corpo |d|, in
%     una qualunque delle due direzioni, per cui NESSUNO dei sei piedi cade
%     nella fascia dell'ostacolo k. Un ostacolo che copre tutta la
%     carreggiata non si manca spostandosi poco: d_manca grande;
%   - soglia = min_k d_manca(k): con una deriva cosi' il robot puo' aver
%     mancato per intero almeno un ostacolo.
%
% COME SI USA, E COSA NON DICE
%   dev_lat_max >= soglia  ->  "superato" su T6 NON VALE: il robot puo' essere
%   passato di fianco a un ostacolo invece che sopra. dev_lat_max e' il
%   massimo nel tempo, non la deriva nel punto dell'ostacolo: la regola e'
%   prudente, puo' invalidare un superamento vero, non convalidarne uno falso.
%   Sotto la soglia "superato" si legge come prima.
%
% CONCLUSIONI POSSIBILI
%   - soglia fra le derive di C1/C3 (~0.07 m) e quella di C2 (~0.30 m): il
%     "superato" di C2 non vale, gli altri due si leggono come sono;
%   - soglia sotto ~0.07 m: anche C1 e C3 possono aver mancato un ostacolo,
%     e su T6 nessun esito binario e' leggibile. Va detto cosi';
%   - soglia sopra ~0.30 m: la deriva di C2 non basta a mancare un ostacolo
%     intero, e il suo superato resta valido. Anche questo va detto.
%
% SOLO LETTURA: legge le mesh e la configurazione, non simula.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(verbose), verbose = true; end

cfg  = phantomx_config();
dirp = fullfile(fileparts(mfilename('fullpath')), 'simscape', 'props');
yp   = cfg.pf_nom(2,:);                 % y nominale dei sei piedi nel corpo
r    = cfg.contact.foot_r;

n = cfg.terreno.n_prop;
k_ = (1:n)';  x_min = NaN(n,1);  x_max = x_min;  y_min = x_min;  y_max = x_min;
h = x_min;  d_manca = x_min;  verso = strings(n,1);
for k = 1:n
    if k == 1, f = 'ProvaPianoImperfettoOstacolo.stl';
    else,      f = sprintf('ProvaPianoImperfettoOstacolo%d.stl', k); end
    m = stlread(fullfile(dirp, f));
    lo = min(m.Points);  hi = max(m.Points);
    x_min(k) = lo(1);  x_max(k) = hi(1);  y_min(k) = lo(2);  y_max(k) = hi(2);
    h(k) = hi(3) - lo(3);

    a = y_min(k) - r;  b = y_max(k) + r;          % fascia in cui un piede tocca
    % Se a d = 0 nessun piede cade nella fascia (ostacolo stretto fra due
    % piedi), il robot lo manca anche andando dritto.
    if ~any(yp >= a & yp <= b)
        d_manca(k) = 0;  verso(k) = "gia' fuori";
        continue
    end
    % Spostandosi di d tutti i piedi traslano insieme. Il piu' piccolo d per
    % cui l'intera fila dei piedi [min(yp), max(yp)] esce dalla fascia:
    %   verso +y:  min(yp) + d > b   ->  d_su  = b - min(yp)
    %   verso -y:  max(yp) - d < a   ->  d_giu = max(yp) - a
    % (nel tragitto la fila puo' attraversare la fascia con l'altro lato: e'
    % il motivo per cui conta l'uscita di TUTTA la fila, non del lato vicino)
    d_su  = b - min(yp);
    d_giu = max(yp) - a;
    [d_manca(k), i] = min([d_su, d_giu]);
    nomi = ["verso +y (sinistra)", "verso -y (destra)"];
    verso(k) = nomi(i);
end

S.per_ost = table(k_, x_min, x_max, y_min, y_max, 1e3*h, d_manca, verso, ...
    'VariableNames', {'ostacolo','x_min','x_max','y_min','y_max','h_mm','d_manca','verso'});
[S.soglia, S.k_min] = min(d_manca);
S.piedi_y = yp;  S.foot_r = r;

if verbose
    fprintf('\n  piedi nominali, y nel corpo [m]: %s   (raggio %.3f)\n', mat2str(yp, 3), r);
    disp(S.per_ost)
    fprintf(['  SOGLIA su dev_lat_max = %.3f m (ostacolo %d, %s)\n' ...
             '  oltre questa deriva "superato" su T6 non vale.\n\n'], ...
            S.soglia, S.k_min, S.per_ost.verso(S.k_min));
end
end
