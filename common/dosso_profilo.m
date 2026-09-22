function G = dosso_profilo(d, z0)
%DOSSO_PROFILO  Geometria del dosso di T4D: salita, cima piana, discesa.
%
%   G = dosso_profilo(cfg.terreno.dosso, cfg.floor_top)
%
%   G.x      [x0 x1 x2 x3]  piede della salita, inizio cima, fine cima,
%                           fondo della discesa  [m, mondo]
%   G.zg     @(x) quota della superficie sotto x (pavimento fuori dal dosso)
%   G.pend   @(x) pendenza della superficie sotto x [rad], + in salita
%   G.P      profilo [x z] del prisma (per l'STL)
%
% Unica fonte della geometria: applica_terreno ne scrive l'STL, script_T4D ne
% ricava le finestre. Se cambia qui, cambia in entrambi.
%
% Il prisma e' convesso (salita, cima, discesa, fondo): serve, perche' il
% contatto di Simscape usa l'inviluppo convesso del File Solid.
%
% Progetto FSR PhantomX - A. Russo

x0 = d.x_inizio;
x1 = x0 + d.H / tand(d.gradi_su);
x2 = x1 + d.L_cima;
x3 = x2 + d.H / tand(d.gradi_giu);
G.x = [x0 x1 x2 x3];
G.H = d.H;
G.P = [x0 z0-d.spessore;  x0 z0;  x1 z0+d.H;  x2 z0+d.H;  x3 z0;  x3 z0-d.spessore];
G.zg = @(x) z0 + d.H * min(1, max(0, min((x - x0)/(x1 - x0), (x3 - x)/(x3 - x2))));
G.pend = @(x) atan(d.H/(x1 - x0) * (x >= x0 & x < x1) - d.H/(x3 - x2) * (x > x2 & x <= x3));
end
