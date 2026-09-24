%% calcola_J.m - l'inerzia del robot intero, quella che usa l'MPC
%
% PERCHE'
%   Il modello di predizione dell'MPC e' un corpo rigido singolo: tutta la
%   dinamica angolare passa da p.J = cfg.J. Era marcata "[TARATO] stimata,
%   non misurata" ed e' l'ultima inerzia del progetto mai verificata - dopo
%   aver scoperto che quelle dei 25 solidi erano sbagliate di mille volte,
%   lasciarla cosi' non si puo'.
%
% PERCHE' NON SI USA L'URDF
%   Il piano diceva "cfg.J dall'URDF con importrobot". NON si puo': le
%   inerzie dell'URDF sono esattamente quelle sbagliate (vedi
%   applica_inerzie). L'URDF resta valido per la cinematica, non per le
%   inerzie. Qui si compone J dai valori corretti di phantomx_config.
%
% COME
%   Teorema degli assi paralleli sul robot in posa nominale:
%       J = I_corpo + m_corpo*(|d|^2 E - d d')  +  somma sui 24 link
%   con d la distanza dal baricentro dell'insieme. Il contributo che domina
%   NON sono le inerzie proprie dei link (~1e-5) ma i termini m*d^2 delle
%   zampe: 0.58 kg distribuiti fino a 20 cm dall'asse.
%
% L'APPROSSIMAZIONE, DICHIARATA
%   Non si ricostruisce la catena cinematica link per link: i quattro
%   baricentri di ogni zampa sono messi lungo il segmento retto anca-piede.
%   La zampa vera e' piegata (il ginocchio sta piu' in alto e piu' in fuori),
%   quindi la dispersione orizzontale reale e' un po' maggiore e J e'
%   verosimilmente >= di questa stima. Per dare la misura dell'incertezza si
%   calcolano anche i due casi limite: masse tutte all'anca (minimo) e tutte
%   al piede (massimo). Il valore vero sta fra i due, per costruzione.
%
% COSA FARNE
%   Se cfg.J cade dentro la forchetta, e' verificata e si tiene: un errore
%   del 15-20% su J e' piccolo accanto all'errore di modello che l'MPC ha
%   comunque, cioe' le zampe considerate SENZA MASSA quando pesano il 37%
%   del totale. Se cade fuori, si sostituisce.
%
% USO
%   calcola_J
%
% Progetto FSR PhantomX - A. Russo

cfg = phantomx_config();

cj_mod = { 'masse all''anca (minimo)',      [0.0  0.0   0.0   0.0 ]
           'equispaziate sul segmento',     [0.125 0.375 0.625 0.875]
           'masse a meta'' segmento',       [0.5  0.5   0.5   0.5 ]
           'masse al piede (massimo)',      [1.0  1.0   1.0   1.0 ] };

cj_I = [cfg.I_c1; cfg.I_c2; cfg.I_thigh; cfg.I_tibia];   % 4 x 3

fprintf('\n=============== INERZIA DEL ROBOT INTERO ===============\n');
fprintf('  posa nominale: r_offset %.3f m, z0 %.3f m\n', cfg.r_offset, cfg.z0);
fprintf('  corpo %.4f kg + 24 link x %.5f kg = %.4f kg\n\n', ...
        cfg.m_body, cfg.m_link, cfg.m_body + 24*cfg.m_link);

fprintf('  %-28s %9s %9s %9s %9s\n', 'modello', 'Jxx', 'Jyy', 'Jzz', 'cm_z [mm]');
cj_J = cell(size(cj_mod,1),1);
for cj_k = 1:size(cj_mod,1)
    [cj_J{cj_k}, cj_cm] = cj_componi(cfg, cj_I, cj_mod{cj_k,2});
    fprintf('  %-28s %9.5f %9.5f %9.5f %9.1f\n', cj_mod{cj_k,1}, ...
            cj_J{cj_k}(1,1), cj_J{cj_k}(2,2), cj_J{cj_k}(3,3), 1e3*cj_cm(3));
end

cj_min = cj_J{1};  cj_max = cj_J{4};   % i due casi limite
fprintf('\n  %-28s %9.5f %9.5f %9.5f\n', 'cfg.J (in uso)', ...
        cfg.J(1,1), cfg.J(2,2), cfg.J(3,3));

fprintf('\n  verifica: cfg.J dentro la forchetta [minimo, massimo]?\n');
cj_ok = true;
for cj_a = 1:3
    cj_dentro = cfg.J(cj_a,cj_a) >= cj_min(cj_a,cj_a) && ...
                cfg.J(cj_a,cj_a) <= cj_max(cj_a,cj_a);
    cj_ok = cj_ok && cj_dentro;
    fprintf('    asse %d: %.5f in [%.5f, %.5f]   %s\n', cj_a, cfg.J(cj_a,cj_a), ...
            cj_min(cj_a,cj_a), cj_max(cj_a,cj_a), cj_si(cj_dentro));
end

cj_rif = cj_J{2};                       % il modello equispaziato, il piu' sensato
fprintf('\n  scarto rispetto al modello equispaziato: %+.0f%%  %+.0f%%  %+.0f%%\n', ...
        100*(cfg.J(1,1)/cj_rif(1,1)-1), 100*(cfg.J(2,2)/cj_rif(2,2)-1), ...
        100*(cfg.J(3,3)/cj_rif(3,3)-1));

if cj_ok
    fprintf('\n  cfg.J e'' VERIFICATA come ordine di grandezza. Si tiene.\n');
else
    fprintf(2, '\n  cfg.J e'' FUORI dalla forchetta: va sostituita.\n');
end
fprintf(['\n  Da ricordare comunque: il modello SRB dell''MPC considera le zampe\n' ...
         '  SENZA MASSA, e le zampe sono il %.0f%% della massa totale. Quello e''\n' ...
         '  l''errore di modello che conta, non il 15%% su J.\n\n'], ...
        100*(24*cfg.m_link + 6*cfg.m_foot)/cfg.mass);

%% ================= helper =================
function [J, cm] = cj_componi(cfg, I_link, frazioni)
%CJ_COMPONI  Inerzia composta attorno al baricentro, per una disposizione.
m = [cfg.m_link cfg.m_link cfg.m_link cfg.m_link];
P = zeros(3, 0);  Il = zeros(3, 3, 0);  mv = [];
for i = 1:6
    anca  = cfg.p_hip(:,i);
    piede = cfg.pf_nom(:,i);
    for k = 1:4
        P(:,end+1)    = anca + frazioni(k) * (piede - anca);   %#ok<AGROW>
        Il(:,:,end+1) = diag(I_link(k,:));                     %#ok<AGROW>
        mv(end+1)     = m(k);                                  %#ok<AGROW>
    end
end

M  = cfg.m_body + sum(mv);
cm = (cfg.m_body * [0;0;0] + P * mv(:)) / M;      % il corpo ha il baricentro nell'origine

J = diag(cfg.I_body) + cj_steiner(cfg.m_body, [0;0;0] - cm);
for k = 1:size(P,2)
    J = J + Il(:,:,k) + cj_steiner(mv(k), P(:,k) - cm);
end
end

function S = cj_steiner(m, d)
S = m * ((d.' * d) * eye(3) - d * d.');
end

function s = cj_si(tf)
if tf, s = 'si'; else, s = 'NO'; end
end
