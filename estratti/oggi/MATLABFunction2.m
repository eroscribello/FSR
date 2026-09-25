% ==== estratto da phantomx_sim_zero ====
% blocco : phantomx_sim_zero/MATLAB Function2
% SID    : 1134
% quando : 2026-09-25 13:27
% Generato da estrai_funzioni.m - NON e' un file del progetto:
% serve solo a poter fare un diff su codice che vive nel .slx.

function [z_lf, z_lr, z_rm] = fcn(z, c_lf, c_lr, c_rm, t, par)
%#codegen
% La logica sta in ricerca_terreno.m, sul path: il .slx e' binario e quello
% che ci sta dentro e' invisibile a git.
% par = c2_par, dal Constant collegato alla porta 6. L'interruttore C1/C2
% e' par(1), lo assembla init_gait.
persistent z_hold z_ext t_prev
if isempty(t_prev)
    z_hold = [0 0 0];
    z_ext  = [0 0 0];
    t_prev = -1;
end

[z_out, z_hold, z_ext, t_prev] = ricerca_terreno( ...
        z, [c_lf, c_lr, c_rm], t, z_hold, z_ext, t_prev, par);

z_lf = z_out(1);
z_lr = z_out(2);
z_rm = z_out(3);
end