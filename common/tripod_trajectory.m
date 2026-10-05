function [x, y, z] = tripod_trajectory(t_in, T, S, H, z0, duty)
% Traiettoria del piede per un ciclo di andatura a tripode.
%#codegen
%
%   t_in [s] tempo. Per il secondo tripode passare  t + T/2
%   T    [s] periodo del ciclo completo (volo + appoggio)
%   S    [m] lunghezza del passo
%   H    [m] altezza di sollevamento del piede in volo
%   z0   [m] profondita' di appoggio, POSITIVA VERSO IL BASSO
%
%   x, y, z  posizione comandata del piede nel frame CORPO, da passare
%            a inv_kyn. Stessa convenzione: +z = verso il basso.


    T_swing  = duty * T;
    T_stance = T - T_swing;
    t_phase  = mod(t_in, T);

    if t_phase < T_swing
        % ---------------- FASE DI VOLO ----------------
        n = t_phase / T_swing;                       % 0 -> 1

        % Orizzontale: smoothstep quintico raccordato alla velocita'
        % della fase di appoggio (posizione, velocita' e accelerazione
        % continue ai due estremi).
        v0 = -S * T_swing / T_stance;
        K  = S - v0;
        x  = -S/2 + v0*n + K*(10*n^3 - 15*n^4 + 6*n^5);

        % Verticale: campana ottenuta componendo due smoothstep quintici,
        % sale su [0, 0.5] e ridiscende su [0.5, 1].
        % Rispetto a H*sin(pi*n)^2 (che e' solo C1) azzera anche il salto
        % di accelerazione: 0.03 invece di 0.93 m/s^2 a stacco e contatto.
        if n < 0.5
            u = 2*n;
        else
            u = 2*(1 - n);
        end
        bump = u*u*u * (10 - 15*u + 6*u*u);          % 0 -> 1 -> 0
        z = z0 - H*bump;
    else
        % ---------------- FASE DI APPOGGIO ----------------
        n = (t_phase - T_swing) / T_stance;
        x = S/2 - S*n;                               % velocita' costante
        z = z0;                                      % piede fermo a terra
    end

    y = 0;   % avanzamento rettilineo; per curvare, imporre y ~= 0
end