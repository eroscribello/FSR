% ==== estratto da phantomx_sim_zero ====
% blocco : phantomx_sim_zero/MATLAB Function1
% SID    : 675
% quando : 2026-09-25 13:27
% Generato da estrai_funzioni.m - NON e' un file del progetto:
% serve solo a poter fare un diff su codice che vive nel .slx.

function [x, y, z] = tripod_gait(t_in, T, S, H, z0, duty)
%#codegen
    [x, y, z] = tripod_trajectory(t_in, T, S, H, z0, duty);
end