% ==== estratto da phantomx_sim_zero ====
% blocco : phantomx_sim_zero/MATLAB Function2
% SID    : 1134
% quando : 2026-09-25 15:50
% Generato da estrai_funzioni.m - NON e' un file del progetto:
% serve solo a poter fare un diff su codice che vive nel .slx.

function [z_lf, z_lr, z_rm] = fcn(z, c_lf, c_lr, c_rm, t)

z_nominal  = 0.14;   
v_search   = 0.06; 
z_ext_max  = 0.03;    

persistent z_hold_lf z_hold_lr z_hold_rm
persistent z_ext_lf  z_ext_lr  z_ext_rm
persistent t_prev

if isempty(z_hold_lf) || t < 0.5
    z_hold_lf = z;
    z_hold_lr = z;
    z_hold_rm = z;
    
    z_ext_lf  = 0;
    z_ext_lr  = 0;
    z_ext_rm  = 0;
    
    t_prev    = t;
end

dt = t - t_prev;
if dt <= 0 || dt > 0.1
    dt = 0.001; 
end
t_prev = t;

if c_lf == 0
    if z >= (z_nominal - 0.002)
        z_ext_lf  = min(z_ext_lf + v_search * dt, z_ext_max);
        z_hold_lf = z + z_ext_lf;
        z_lf      = z_hold_lf;
    else
        z_ext_lf  = 0;
        z_hold_lf = z;
        z_lf      = z;
    end
else
    z_lf = min(z + z_ext_lf, z_hold_lf);
end


if c_lr == 0
    if z >= (z_nominal - 0.002)
        z_ext_lr  = min(z_ext_lr + v_search * dt, z_ext_max);
        z_hold_lr = z + z_ext_lr;
        z_lr      = z_hold_lr;
    else
        z_ext_lr  = 0;
        z_hold_lr = z;
        z_lr      = z;
    end
else
    z_lr = min(z + z_ext_lr, z_hold_lr);
end


if c_rm == 0
    if z >= (z_nominal - 0.002)
        z_ext_rm  = min(z_ext_rm + v_search * dt, z_ext_max);
        z_hold_rm = z + z_ext_rm;
        z_rm      = z_hold_rm;
    else
        z_ext_rm  = 0;
        z_hold_rm = z;
        z_rm      = z;
    end
else
    z_rm = min(z + z_ext_rm, z_hold_rm);
end