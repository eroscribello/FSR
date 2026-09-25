% ==== estratto da phantomx_sim_zero ====
% blocco : phantomx_sim_zero/MATLAB Function3
% SID    : 1159
% quando : 2026-09-25 13:27
% Generato da estrai_funzioni.m - NON e' un file del progetto:
% serve solo a poter fare un diff su codice che vive nel .slx.

function [z_rf, z_rr, z_lm] = fcn(z, c_rf, c_rr, c_lm, t, par)
%#codegen
persistent z_hold z_ext t_prev
if isempty(t_prev)
    z_hold = [0 0 0];
    z_ext  = [0 0 0];
    t_prev = -1;
end

[z_out, z_hold, z_ext, t_prev] = ricerca_terreno( ...
        z, [c_rf, c_rr, c_lm], t, z_hold, z_ext, t_prev, par);

z_rf = z_out(1);
z_rr = z_out(2);
z_lm = z_out(3);
end