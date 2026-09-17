function [z_out, z_hold, z_ext, t_prev] = ricerca_terreno(z, c, t, z_hold, z_ext, t_prev, par)
%RICERCA_TERRENO  Rilevazione del contatto dalla coppia: la logica di C2.
%#codegen
%
%   [z_out, z_hold, z_ext, t_prev] = ricerca_terreno(z, c, t, ...
%                                        z_hold, z_ext, t_prev, par)
%
% COS'E'
%   E' il contributo di Arrigoni et al. §5, nella forma semplificata che il
%   simulatore permette: in Simscape la coppia ai giunti si misura, quindi non
%   serve il modello dinamico di stima. Durante l'abbassamento del piede, se
%   la zampa non risulta a terra, il comando di profondita' viene esteso verso
%   il basso a velocita' costante; quando il contatto viene rilevato, la
%   zampa si blocca alla quota raggiunta.
%
% PERCHE' STA IN UN FILE E NON DENTRO IL BLOCCO
%   Era scritta due volte dentro due blocchi MATLAB Function, con le costanti
%   cablate (0.14, 0.06, 0.03, 0.002, 0.5). Conseguenze:
%     - il .slx e' binario, quindi quella logica era invisibile a git: nessun
%       diff, nessuna storia, nessuna revisione possibile;
%     - le due copie potevano divergere senza che nulla lo segnalasse;
%     - z_nominal = 0.14 duplicava cfg.z0 senza saperlo: ritarare z0 per la
%       campagna T2 avrebbe sfasato la soglia di ricerca.
%   Adesso i due blocchi sono una riga sola, come tripod_gait e leg_ik.
%
% L'INTERRUTTORE C1 / C2 E' par(1), E NON E' c2_soglia
%   Questo va detto chiaramente perche' la versione precedente della
%   documentazione sbagliava. Portare la soglia di coppia a infinito NON da'
%   l'anello aperto: rende il flag di contatto sempre falso, e con il flag
%   falso la ricerca entra nel suo ramo di DISCESA a ogni appoggio, scendendo
%   di 3 cm a 0.06 m/s. E' il contrario di quello che serve.
%   Con par(1) = 0 il comando passa invariato: quello e' l'anello aperto.
%
% INGRESSI
%   z        [m]      profondita' comandata dal generatore di traiettoria,
%                     comune alle tre zampe del tripode (POSITIVA = in basso)
%   c        [1 x 3]  flag di contatto delle tre zampe (0 = non a terra)
%   t        [s]      tempo di simulazione
%   z_hold   [1 x 3]  stato: quota a cui la zampa si e' fermata
%   z_ext    [1 x 3]  stato: estensione accumulata in questa fase di ricerca
%   t_prev   [s]      stato: istante della chiamata precedente, < 0 = prima
%   par      [1 x 6]  [attiva, z_nom, v_search, z_ext_max, t_reset, tol]
%
% USCITE
%   z_out    [1 x 3]  profondita' comandata per ciascuna delle tre zampe
%   gli altri tre     lo stato aggiornato, da riconsegnare alla chiamata dopo
%
% LO STATO STA NEL BLOCCO, NON QUI
%   Niente variabili persistent in questa funzione: i due blocchi la chiamano
%   entrambi e lo stato dei due tripodi deve restare separato. Ogni blocco
%   tiene le sue persistent e le passa avanti e indietro.
%
% Progetto FSR PhantomX - A. Russo

attiva    = par(1);
z_nom     = par(2);
v_search  = par(3);
z_ext_max = par(4);
t_reset   = par(5);
tol       = par(6);

z_out = [z, z, z];

%% ---- C1: anello aperto ----
% Lo stato viene comunque azzerato, cosi' riaccendendo C2 a meta' sessione
% non si eredita una ricerca rimasta a meta'.
if attiva == 0
    z_hold = [z, z, z];
    z_ext  = [0, 0, 0];
    t_prev = t;
    return
end

%% ---- C2 ----
% Reset all'avvio e per tutto il transitorio iniziale: prima che il robot si
% sia assestato, una ricerca partita per un contatto mancante e' rumore.
if t_prev < 0 || t < t_reset
    z_hold = [z, z, z];
    z_ext  = [0, 0, 0];
    t_prev = t;
end

dt = t - t_prev;
if dt <= 0 || dt > 0.1
    dt = 0.001;
end
t_prev = t;

for i = 1:3
    if c(i) == 0
        % zampa che non trasmette forza
        if z >= (z_nom - tol)
            % ... e il comando la vuole a terra: e' la fase di abbassamento,
            % quindi si cerca il terreno scendendo.
            z_ext(i)  = min(z_ext(i) + v_search*dt, z_ext_max);
            z_hold(i) = z + z_ext(i);
            z_out(i)  = z_hold(i);
        else
            % ... ma il comando la vuole in volo: nessuna ricerca, e lo stato
            % si azzera in vista del prossimo appoggio.
            z_ext(i)  = 0;
            z_hold(i) = z;
            z_out(i)  = z;
        end
    else
        % contatto rilevato: la zampa non scende oltre la quota raggiunta
        z_out(i) = min(z + z_ext(i), z_hold(i));
    end
end

end