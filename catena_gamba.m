function catena_gamba()
%CATENA_GAMBA  La catena cinematica vera di una zampa, letta dall'URDF.
%
%   catena_gamba
%
% PERCHE'
%   lc, lf e lt in cfg sono stati dedotti da fonti indirette - l'URDF letto a
%   mano, la mesh, i Rigid Transform del modello - e lt e' gia' cambiata tre
%   volte. La catena vera non l'abbiamo mai letta.
%
%   Qui viene estratta dal rigidBodyTree: le trasformazioni fisse fra giunti
%   consecutivi, alla configurazione zero, con assi e origini. Da queste si
%   riscrive inv_kyn senza piu' ipotesi.
%
% COSA GUARDARE
%   - le traslazioni fra giunti consecutivi danno lc, lf e la geometria vera
%   - le componenti FUORI dal piano della gamba dicono se la catena e'
%     davvero planare (l'IK 2R lo assume)
%   - gli assi di rotazione dicono quale piano e' quello della gamba
%
% PRIMA
%   startup_phantomx ; init_gait
%
% Progetto FSR PhantomX

if ~evalin('base','exist(''robotModel'',''var'')')
    fprintf(2,'\n  robotModel non c''e''. Lancia init_gait.\n\n'); return
end
rbt = evalin('base','robotModel');
cfg = phantomx_config();

corpi = {'c1_lf','c2_lf','thigh_lf','tibia_lf'};

fprintf('\n========= CATENA DELLA ZAMPA FL =========\n');
fprintf('  base: %s\n', rbt.BaseName);

home = homeConfiguration(rbt);

%% ---- assi dei giunti ----
fprintf('\n--- ASSI DEI GIUNTI (nel frame del corpo padre) ---\n');
for k = 1:numel(corpi)
    b = getBody(rbt, corpi{k});
    j = b.Joint;
    fprintf('  %-10s giunto %-14s tipo %-9s asse [%+.3f %+.3f %+.3f]\n', ...
            corpi{k}, j.Name, j.Type, j.JointAxis);
end

%% ---- trasformazioni fra giunti consecutivi ----
fprintf('\n--- TRASFORMAZIONI FRA CORPI CONSECUTIVI (configurazione zero) ---\n');
prec = rbt.BaseName;
for k = 1:numel(corpi)
    T = getTransform(rbt, home, corpi{k}, prec);
    t = T(1:3,4);
    fprintf('\n  %s -> %s\n', prec, corpi{k});
    fprintf('    traslazione  [%+.6f %+.6f %+.6f]   norma %.6f\n', t, norm(t));
    fprintf('    rotazione    [%+.3f %+.3f %+.3f]\n', T(1,1:3));
    fprintf('                 [%+.3f %+.3f %+.3f]\n', T(2,1:3));
    fprintf('                 [%+.3f %+.3f %+.3f]\n', T(3,1:3));
    prec = corpi{k};
end

%% ---- lunghezze a confronto con cfg ----
fprintf('\n--- LUNGHEZZE ---\n');
T12 = getTransform(rbt, home, 'c2_lf',    'c1_lf');
T23 = getTransform(rbt, home, 'thigh_lf', 'c2_lf');
T34 = getTransform(rbt, home, 'tibia_lf', 'thigh_lf');

fprintf('%-28s %12s %12s\n','TRATTO','misurato','cfg');
fprintf('%-28s %12.6f %12.6f\n','c1 -> c2   (coxa)',    norm(T12(1:3,4)), cfg.lc);
fprintf('%-28s %12.6f %12s\n','c2 -> thigh',           norm(T23(1:3,4)), '-');
fprintf('%-28s %12.6f %12.6f\n','thigh -> tibia (femore)', norm(T34(1:3,4)), cfg.lf);

%% ---- il piede ----
fprintf('\n--- IL PIEDE ---\n');
fprintf('  cfg.foot_xyz              [%+.5f %+.5f %+.5f]  norma %.5f\n', ...
        cfg.foot_xyz, norm(cfg.foot_xyz));
fprintf('  cfg.lt                    %.6f\n', cfg.lt);
fprintf(['  offset richiesto (misurato da offset_piede, a x = 0)\n' ...
         '                            [-0.00004 +0.13809 -0.00955]  norma 0.13842\n']);

fprintf(['\n--- COSA CERCARE ---\n' ...
         '  Se una traslazione ha una componente non trascurabile lungo l''asse\n' ...
         '  del giunto, la catena NON e'' planare e l''IK 2R non puo'' essere\n' ...
         '  esatta: quella componente va gestita esplicitamente.\n\n' ...
         '  Confronta la norma c1->c2 con lc e thigh->tibia con lf: se non\n' ...
         '  coincidono, l''IK sta usando link di lunghezza sbagliata, ed e''\n' ...
         '  quello il motivo per cui l''errore cresce con l''escursione.\n\n']);
end