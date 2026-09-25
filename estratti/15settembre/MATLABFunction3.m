% ==== estratto da phantomx_sim_zero ====
% blocco : phantomx_sim_zero/MATLAB Function3
% SID    : 1159
% quando : 2026-09-25 15:50
% Generato da estrai_funzioni.m - NON e' un file del progetto:
% serve solo a poter fare un diff su codice che vive nel .slx.

function [z_rf, z_rr, z_lm] = fcn(z, c_rf, c_rr, c_lm, t)

z_nominal  = 0.14; 
v_search   = 0.06;
z_ext_max  = 0.03;  

persistent z_hold_rf z_hold_rr z_hold_lm
persistent z_ext_rf  z_ext_rr  z_ext_lm
persistent t_prev

if isempty(z_hold_rf) || t < 0.5
    z_hold_rf = z;
    z_hold_rr = z;
    z_hold_lm = z;
    
    z_ext_rf  = 0;
    z_ext_rr  = 0;
    z_ext_lm  = 0;
    
    t_prev    = t;
end

dt = t - t_prev;
if dt <= 0 || dt > 0.1
    dt = 0.001; 
end
t_prev = t;


if c_rf == 0
    if z >= (z_nominal - 0.002)
        z_ext_rf  = min(z_ext_rf + v_search * dt, z_ext_max);
        z_hold_rf = z + z_ext_rf;
        z_rf      = z_hold_rf;
    else
        z_ext_rf  = 0;
        z_hold_rf = z;
        z_rf      = z;
    end
else
    z_rf = min(z + z_ext_rf, z_hold_rf);
end

if c_rr == 0
    if z >= (z_nominal - 0.002)
        z_ext_rr  = min(z_ext_rr + v_search * dt, z_ext_max);
        z_hold_rr = z + z_ext_rr;
        z_rr      = z_hold_rr;
    else
        z_ext_rr  = 0;
        z_hold_rr = z;
        z_rr      = z;
    end
else
    z_rr = min(z + z_ext_rr, z_hold_rr);
end


if c_lm == 0
    if z >= (z_nominal - 0.002)
        z_ext_lm  = min(z_ext_lm + v_search * dt, z_ext_max);
        z_hold_lm = z + z_ext_lm;
        z_lm      = z_hold_lm;
    else
        z_ext_lm  = 0;
        z_hold_lm = z;
        z_lm      = z;
    end
else
    z_lm = min(z + z_ext_lm, z_hold_lm);
end