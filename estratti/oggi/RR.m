% ==== estratto da phantomx_sim_zero ====
% blocco : phantomx_sim_zero/RR
% SID    : 554
% quando : 2026-09-25 13:27
% Generato da estrai_funzioni.m - NON e' un file del progetto:
% serve solo a poter fare un diff su codice che vive nel .slx.

function [theta, phi, psi] = leg_ik(x, y, z, side, alpha)
    [theta, phi, psi] = inv_kyn(x, y, z, side, alpha);
end