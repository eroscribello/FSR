function fermo = appoggio_cinematico(r, cfg, v_app)
% Piedi in appoggio riconosciuti dal moto, non dai sensori.
%
%   fermo = appoggio_cinematico(run, cfg, 0.05)     [N x 6] logico, ordine CAN
%
% Un piede e' in appoggio se e' fermo nel mondo: velocita' sotto v_app per
% almeno un quarto dell'appoggio nominale. 
%
% I tratti fermi piu' brevi di un quarto d'appoggio si scartano

if nargin < 3 || isempty(v_app), v_app = 0.05; end
if ~isfield(r,'pf') || isempty(r.pf)
    error('appoggio_cinematico:dati', 'Serve la posizione dei piedi (run.pf).');
end
t  = r.t(:);
dt = median(diff(t));
dmin  = round(0.25 * cfg.T_stance / dt);
fermo = false(numel(t), 6);
for i = 1:6
    c = 3*(i-1) + (1:3);
    v = vecnorm(gradient(r.pf(:,c).', dt).', 2, 2);
    g = v < v_app;
    ini = find(diff([false; g]) ==  1);
    fin = find(diff([g; false]) == -1);
    for k = 1:numel(ini)
        if fin(k) - ini(k) + 1 < dmin, g(ini(k):fin(k)) = false; end
    end
    fermo(:,i) = g;
end
end
