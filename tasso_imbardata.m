function [w, tot, info] = tasso_imbardata(t, yaw, wz)
% Velocita' di imbardata, dalla sorgente migliore disponibile.
%
%   [w, tot] = tasso_imbardata(t, yaw)        dall'angolo, srotolandolo
%   [w, tot] = tasso_imbardata(t, yaw, wz)    dalla velocita' angolare
%
%   w     [rad/s] tasso medio di imbardata
%   tot   [rad]   rotazione totale accumulata, segno incluso
%   info  struttura: sorgente usata, unita' rilevata, giri, scarto fra le due
%
t   = t(:);
yaw = yaw(:);

if numel(t) ~= numel(yaw)
    error('tasso_imbardata:lunghezza','t e yaw devono avere la stessa lunghezza.');
end
if numel(t) < 3
    w = NaN;  tot = NaN;  info = struct('unita','?','giri',NaN,'residuo',NaN);
    return
end

%% ---- unita' ----
if max(abs(yaw)) > 2*pi
    unita = 'deg';
    periodo = 360;
    warning('tasso_imbardata:unita', ...
        ['L''angolo supera 2*pi in valore assoluto: lo interpreto in GRADI\n' ...
         '(massimo %.1f). Se e'' davvero in radianti, il robot ha ruotato di\n' ...
         '%.1f giri e la conversione qui sotto e'' sbagliata.'], ...
        max(abs(yaw)), max(abs(yaw))/(2*pi));
else
    unita = 'rad';
    periodo = 2*pi;
end

%% ---- srotolamento ----
d = diff(yaw);
d = mod(d + periodo/2, periodo) - periodo/2;   % ogni passo in (-meta', meta']
srotolato = [0; cumsum(d)];

if strcmp(unita,'deg')
    srotolato = deg2rad(srotolato);
end
tot = srotolato(end);

%% ---- pendenza ai minimi quadrati ----
tt = t - t(1);
p  = polyfit(tt, srotolato, 1);
w  = p(1);

res = srotolato - polyval(p, tt);

info = struct('sorgente', 'angolo srotolato', ...
              'unita', unita, ...
              'giri', tot/(2*pi), ...
              'residuo', sqrt(mean(res.^2)), ...
              'tot', tot, ...
              'w_angolo', w, ...
              'w_velocita', NaN, ...
              'scarto', NaN, ...
              'w_ingenuo', (yaw(end)-yaw(1))/(t(end)-t(1)));

%% ---- se c'e' la velocita' angolare, e' lei la misura ----
if nargin >= 3 && ~isempty(wz)
    wz = wz(:);
    if numel(wz) == numel(t)
        w_vel = mean(wz);
        info.w_velocita = w_vel;
        info.scarto     = w_vel - info.w_angolo;

        rif = max(abs([w_vel, info.w_angolo]));
        if rif > 1e-3 && abs(info.scarto)/rif > 0.2
            warning('tasso_imbardata:discordanza', ...
                ['Le due stime dell''imbardata non coincidono:\n' ...
                 '  dalla velocita'' angolare  %+.5f rad/s\n' ...
                 '  dall''angolo srotolato     %+.5f rad/s   (scarto %.0f%%)\n' ...
                 'Se la run ruota meno di mezzo giro dovrebbero coincidere.\n' ...
                 'Probabile: la terza componente di S.w non e'' l''imbardata,\n' ...
                 'oppure e'' risolta in un frame diverso da quello del mondo.\n' ...
                 'Uso la velocita'' angolare, ma il numero va verificato.'], ...
                w_vel, info.w_angolo, 100*abs(info.scarto)/rif);
        end

        w = w_vel;
        info.sorgente = 'velocita'' angolare del corpo';
        tot = trapz(t, wz);
        info.tot  = tot;
        info.giri = tot/(2*pi);
    else
        warning('tasso_imbardata:lunghezzaW', ...
            'wz ha %d campioni, t ne ha %d: ignoro la velocita'' angolare.', ...
            numel(wz), numel(t));
    end
end

end
