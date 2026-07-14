%% ============================================================
%  Parametri PhantomX AX Metal Hexapod Mark II
%  Estratti da phantomx.urdf — da integrare in get_params.m
%
%  Ordine canonico gambe (usare IDENTICO in tutti i file):
%     1 = lf (ant-sx)   2 = rf (ant-dx)
%     3 = lm (med-sx)   4 = rm (med-dx)
%     5 = lr (post-sx)  6 = rr (post-dx)
%  Tripode A = {1,4,5} (lf, rm, lr)
%  Tripode B = {2,3,6} (rf, lm, rr)
%% ============================================================

p.nLeg = 6;

%% ---- Massa ----
% Massa TOTALE del robot (corpo 0.976 kg + 24 link gamba * 0.0244 kg).
% Il modello SRB rappresenta l'intero robot come corpo rigido -> massa totale.
p.mass = 1.56;                 % [kg]

%% ---- Inerzia del corpo  ***DA CONFERMARE*** ----
% ATTENZIONE: le inerzie nell'URDF NON sono utilizzabili:
%   - stesso identico blocco inerziale copiato su tutti i link gamba;
%   - valori ~1000x troppo grandi (raggio di girazione fisicamente impossibile).
% NON copiare i valori dell'URDF.
% Il valore qui sotto e' una STIMA a parallelepipedo (box ~0.26x0.24x0.08 m,
% massa totale) — fisicamente sensata, sufficiente per far partire l'MPC.
% -> Sostituirlo con l'inerzia AGGREGATA che Simscape riporta sul modello
%    assemblato (Mechanics Explorer / model report), che la calcola dalle mesh.
p.J = diag([0.008, 0.010, 0.016]);   % [kg m^2]  stima -> confermare in Simscape

%% ---- Posizioni delle 6 anche (giunti j_c1_*) rispetto al centro corpo ----
% [x y z] in metri, colonna per gamba, ordine canonico.
% NB: layout ESAGONALE, non rettangolare: le gambe centrali (3,4) sporgono
% in y (0.1034) piu delle anteriori/posteriori (0.0616). Usare QUESTE posizioni,
% non una parametrizzazione L/W.  (z anca reale ~0.0011 m, trascurato)
p.p_hip = [ 0.1248,  0.1248,  0.0000,  0.0000, -0.1248, -0.1248;
            0.0616, -0.0616,  0.1034, -0.1034,  0.0616, -0.0616;
            0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000];  % 3x6

%% ---- Stance neutra ----
% Altezza nominale del corpo. RIDOTTA rispetto al quadrupede (era 0.2 m):
% allungo massimo gamba ~0.14 m, quindi 0.08 m e' una stance comoda. Da rifinire.
p.z0 = 0.08;                   % [m]

% Piedi neutri: sotto le anche, a terra (z=0), col corpo ad altezza z0.
% Default sicuro e stabile. Per un poligono d'appoggio piu largo, allarga in y.
p.pf36 = p.p_hip;              % 3x6, z gia' a 0

%% ---- Lunghezze link ----
% Servono alla FK di DISEGNO (animazione) e a verificare la fattibilita' della
% stance. NON entrano nel modello SRB dell'MPC.
p.d  = 0.054;                  % coxa: offset laterale c1 -> c2
p.l1 = 0.066;                  % femore: giunto thigh -> giunto tibia
p.l2 = 0.075;                  % tibia: stima da mesh -> confermare

%% ---- Ingombro corpo (solo per il disegno del parallelepipedo) ----
% Le posizioni vere delle anche stanno in p.p_hip: questi sono solo bounding.
p.L = 0.250;                   % span anche in x (2*0.1248)
p.W = 0.207;                   % span anche in y (2*0.1034, gambe centrali)
p.h = 0.05;                    % altezza corpo (disegno)