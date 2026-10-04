function [z_out, z_hold, z_ext, t_prev] = ricerca_terreno(z, c, t, z_hold, z_ext, t_prev, par)
% Rilevazione del contatto dalla coppia: la logica di C2.
%#codegen
%
%   [z_out, z_hold, z_ext, t_prev] = ricerca_terreno(z, c, t, ...
%                                        z_hold, z_ext, t_prev, par)
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

attiva    = par(1);
z_nom     = par(2);
v_search  = par(3);
z_ext_max = par(4);
t_reset   = par(5);
tol       = par(6);

z_out = [z, z, z];

%% ---- C1: anello aperto ----
if attiva == 0
    z_hold = [z, z, z];
    z_ext  = [0, 0, 0];
    t_prev = t;
    return
end

%% ---- C2 ----
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
        if z >= (z_nom - tol)
            z_ext(i)  = min(z_ext(i) + v_search*dt, z_ext_max);
            z_hold(i) = z + z_ext(i);
            z_out(i)  = z_hold(i);
        else
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