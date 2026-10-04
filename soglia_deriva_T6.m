function S = soglia_deriva_T6(verbose)
%SOGLIA_DERIVA_T6  Quanta deriva laterale basta per mancare un ostacolo di T6.
%
%   S = soglia_deriva_T6
%   S.soglia     [m] la soglia su dev_lat_max
%   S.per_ost    tabella ostacolo per ostacolo


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
    if ~any(yp >= a & yp <= b)
        d_manca(k) = 0;  verso(k) = "gia' fuori";
        continue
    end
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
