function [w, tot, info] = tasso_imbardata(t, yaw, wz)
%TASSO_IMBARDATA  Velocita' di imbardata, dalla sorgente migliore disponibile.
%
%   [w, tot] = tasso_imbardata(t, yaw)        dall'angolo, srotolandolo
%   [w, tot] = tasso_imbardata(t, yaw, wz)    dalla velocita' angolare
%
%   w     [rad/s] tasso medio di imbardata
%   tot   [rad]   rotazione totale accumulata, segno incluso
%   info  struttura: sorgente usata, unita' rilevata, giri, scarto fra le due
%
% LA SORGENTE MIGLIORE E' LA VELOCITA' ANGOLARE
%   Il log di Simscape contiene x6_DOF_Joint.S.w, la velocita' angolare del
%   corpo, che adatta_simscape mette in run.w in rad/s. Quella non si
%   avvolge: non serve srotolare niente, non serve stimare pendenze.
%       w = tasso_imbardata(run.t, run.rpy(:,3), run.w(:,3))
%
%   L'angolo resta comunque utile come CONTROLLO INCROCIATO: le due stime
%   devono coincidere, e su una run che ruota meno di mezzo giro coincidono
%   anche in valore assoluto. Se divergono, la convenzione degli assi di
%   S.w non e' quella che si assume (terza componente = imbardata) e info
%   lo segnala. E' un controllo che costa zero e che avrebbe evitato le
%   conclusioni sbagliate di oggi.
%
% L'ERRORE CHE QUESTA FUNZIONE ESISTE PER NON RIFARE
%   Scrivere
%       w = (yaw(end) - yaw(1)) / (t(end) - t(1))
%   e' sbagliato appena il robot ruota di piu' di mezzo giro. L'angolo di
%   imbardata e' definito in (-pi, pi]: superato il mezzo giro riparte
%   dall'altro estremo, e la differenza fra primo e ultimo campione non
%   misura la rotazione ma il resto della divisione.
%
%   Conseguenza pratica, misurata su questo progetto: con una run da 15 s
%   quella formula non puo' restituire nulla fuori da +-pi/15 = +-0.209
%   rad/s, QUALUNQUE sia la rotazione vera. Cinque celle con comandi da 0.21
%   a 1.71 rad/s hanno dato +0.124, -0.172, +0.062, -0.149, +0.059: tutti
%   dentro quella banda, con segni alterni. Sembrava un controllore senza
%   autorita' di imbardata che saturava; era il modulo dell'angolo.
%
% COME SI FA
%   Si sommano le differenze fra campioni consecutivi, ognuna riportata
%   nell'intervallo (-pi, pi]. Finche' fra due campioni il robot ruota di
%   meno di mezzo giro - sempre, a passo di campionamento ragionevole - la
%   somma e' la rotazione vera, giri inclusi. Poi si stima la pendenza ai
%   minimi quadrati sull'angolo srotolato, che e' piu' robusta del rapporto
%   fra estremi.
%
% UNITA'
%   La funzione rileva da sola se yaw e' in gradi o in radianti: se supera
%   2*pi in valore assoluto sono gradi. Avvisa quando lo fa, perche' in
%   questo progetto le unita' del log di Simscape hanno gia' prodotto un
%   errore da un fattore 57 (gli angoli di giunto letti come radianti
%   davano CoT 159 ed energia 3463 J).
%
% Progetto FSR PhantomX - A. Russo

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

        % Lo scarto relativo ha senso solo se c'e' qualcosa da misurare:
        % a comando nullo entrambe le stime sono rumore e il rapporto
        % esplode senza significare niente.
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
        % la rotazione totale diventa l'integrale, coerente con la stima
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
