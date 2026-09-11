function r = analizza_forze(v, etichetta)
%ANALIZZA_FORZE  Quattro numeri sulla qualita' dell'appoggio.
%
%   r = analizza_forze(simout, 'baseline')
%   r = analizza_forze(simout, 'T = 2.0 s')
%
%   v          uscita del blocco To Workspace (timeseries, struct o matrice)
%   etichetta  nome della prova, per la tabella di confronto
%
% BERSAGLI
%   media          = peso (15.55 N)   -> massa e appoggio corretti
%   dev. standard  < 15% della media  -> carico stabile
%   massimo        < 2x il peso       -> niente urti
%   tempo in volo  = 0%               -> il robot non salta
%
% Le statistiche sono calcolate su un tempo RICAMPIONATO uniforme: il
% solutore e' a passo variabile e accorcia il passo proprio durante gli
% urti, quindi senza ricampionare le percentuali sarebbero gonfiate.
%
% Il primo secondo viene scartato: e' transitorio di avvio.
%
% Progetto FSR PhantomX

if nargin < 2, etichetta = ''; end

%% ---- lettura, qualunque sia il formato ----
if isa(v,'timeseries')
    t = v.Time;   F = v.Data;
elseif isstruct(v) && isfield(v,'signals')
    t = v.time;   F = v.signals.values;
else
    F = v;        t = (0:size(F,1)-1)';
end
F = squeeze(F);
if size(F,1) < size(F,2), F = F.'; end

%% ---- regime + ricampionamento ----
i0 = find(t >= 1, 1);  if isempty(i0), i0 = 1; end
tt = t(i0:end);   FF = F(i0:end,:);

tu = linspace(tt(1), tt(end), 20000)';
Fu = interp1(tt, FF, tu);

if size(Fu,2) == 1, S = Fu; else, S = sum(Fu,2); end

%% ---- massa dal config, non scritta a mano ----
try
    cfg = phantomx_config();
    mg  = cfg.mass * cfg.g;
catch
    mg = 15.55;
end

%% ---- risultati ----
r.etichetta = etichetta;
r.media     = mean(S);
r.std       = std(S);
r.std_perc  = 100*std(S)/mean(S);
r.max       = max(S);
r.max_g     = max(S)/mg;
r.volo      = 100*mean(S < 1);

fprintf('\n=== %s ===\n', etichetta);
fprintf('  media           %7.2f N    (peso %.2f N)   %s\n', ...
        r.media, mg, esito(abs(r.media-mg) < 0.05*mg));
fprintf('  dev. standard   %7.2f N    (%.0f%%)          %s\n', ...
        r.std, r.std_perc, esito(r.std_perc < 15));
fprintf('  massimo         %7.2f N    (%.1fx peso)     %s\n', ...
        r.max, r.max_g, esito(r.max_g < 2));
fprintf('  tempo in volo   %7.1f%%                    %s\n', ...
        r.volo, esito(r.volo < 0.1));

%% ---- per zampa ----
if size(Fu,2) == 6
    n = sum(Fu > 0.5, 2);
    r.duty    = mean(Fu > 0.5, 1);
    r.n_medio = mean(n);
    r.p0      = 100*mean(n==0);
    r.p3      = 100*mean(n==3);
    r.p6      = 100*mean(n==6);

    fprintf('\n  %-6s %8s %10s %10s\n','ZAMPA','duty','F media','F max');
    for k = 1:6
        f = Fu(:,k);  inC = f > 0.5;
        fprintf('  %-6d %8.2f %10.2f %10.2f\n', k, mean(inC), mean(f(inC)), max(f));
    end
    fprintf('\n  piedi a terra   media %.2f   0:%.0f%%  3:%.0f%%  6:%.0f%%\n', ...
            r.n_medio, r.p0, r.p3, r.p6);
end
fprintf('\n');

end

function s = esito(ok)
if ok, s = 'OK'; else, s = '<--'; end
end