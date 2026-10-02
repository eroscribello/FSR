function R = stato_pacco(out, task, cfg)
%STATO_PACCO  Il pacco e' rimasto sul robot? Dalla posa del suo giunto nel log.
%
%   script_T4;  stato_pacco(t4_out, 'T4')      % nella stessa sessione, con C3P
%
%   Aggiorna results/diagnostica/pacco_C3P.csv: una riga per task (una riga
%   gia' presente per lo stesso task viene sostituita).
%
% LA DOMANDA                                    [scritta il 2/10, prima dei numeri]
%   Con C3P (C3 con il pacco attivo) il task si porta a termine. Ma "a
%   termine" vale solo se il pacco e' ancora SOPRA il robot alla fine: un
%   robot che arriva in fondo dopo aver perso il carico non ha portato niente.
%
% COM'E' FATTO IL PACCO (letto dall'XML di phantomx_sim_attitude, 2/10)
%   Brick Solid    cubo 0.2 m, 1 kg, libero: 6-DOF Joint1 lo lega al corpo
%   Brick Solid1   vassoio 0.3 x 0.3 x 0.1 m, 0.1 kg, rigido sul corpo
%   Spatial Contact Force fra i due: k 1200 N/m, c 50, attrito 0.8 / 0.75
%   Il vassoio e' piu' largo del cubo di 0.1 m: 5 cm di gioco per lato SE
%   il cubo parte centrato (non verificato).
%
% COSA SI MISURA
%   La posizione del 6-DOF Joint1 (Px, Py, Pz dal log di Simscape) e' la posa
%   relativa fra cubo e corpo. Riferimento: la mediana su [1.5, 2] s, dopo
%   l'assestamento iniziale e prima del regime (2T). Si misura lo spostamento
%   dal riferimento, |P(t) - P_ref|. E' nel frame del giunto, che ruota con
%   il cubo: per rotazioni piccole e' lo spostamento relativo; con il cubo
%   ribaltato non lo e' piu', ma a quel punto l'esito e' gia' "caduto".
%
% SOGLIE, DICHIARATE PRIMA
%   spostamento massimo < 0.02 m          'fermo'
%   0.02 <= ... < 0.05 m                  'scivola, resta sul vassoio'
%   >= 0.05 m                             'al bordo o caduto' (il gioco di 5 cm
%                                          e' esaurito: si guarda l'animazione)
%
% SOLO LETTURA. Non simula, non tocca il modello.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 3 || isempty(cfg), cfg = phantomx_config(); end
task = upper(char(task));

%% ---- il nodo del giunto nel log ----
lg = [];
for nome = {'simlog','simlog_phantomx_sim_attitude'}
    try, lg = out.(nome{1}); break; catch, end
end
if isempty(lg)
    error('stato_pacco:log', 'Nessun log di Simscape nell''uscita della simulazione.');
end
ids = lg.childIds;
k = find(~cellfun(@isempty, regexp(ids, '6_DOF_Joint1$', 'once')), 1);
if isempty(k)
    error('stato_pacco:giunto', ['Nel log non c''e'' 6-DOF Joint1. Nodi di primo ' ...
        'livello: %s.\nIl carico e'' stato commentato, o il blocco rinominato.'], ...
        strjoin(ids, ', '));
end
g = lg.(ids{k});

P = [];
for c = {'Px','Py','Pz'}
    s = g.(c{1}).p.series;
    t = s.time;   t = t(:);        % s.time(:) e' letto come chiamata di metodo
    v = s.values('m');
    P = [P, v(:)];                                                %#ok<AGROW>
end

%% ---- spostamento dal riferimento ----
ref = t >= 1.5 & t <= 2.0;
if ~any(ref)
    error('stato_pacco:corta', 'La run e'' piu'' corta di 2 s: niente riferimento.');
end
P0 = median(P(ref,:), 1);
d  = vecnorm(P - P0, 2, 2);
reg = t >= 2*cfg.T;

R = struct();
R.task        = string(task);
R.controller  = "C3P";
R.spost_max   = max(d(reg));
R.spost_fin   = d(end);
[~, im]       = max(d .* reg);
R.t_max       = t(im);
R.dz_max      = max(abs(P(reg,3) - P0(3)));
if R.spost_max < 0.02,     R.esito = "fermo";
elseif R.spost_max < 0.05, R.esito = "scivola, resta sul vassoio";
else,                      R.esito = "al bordo o caduto";
end
R.data = string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm'));

fprintf('\n=============== PACCO - %s, C3P ===============\n', task);
fprintf('  spostamento dal riferimento   max %.1f mm (t = %.2f s)   finale %.1f mm\n', ...
        1e3*R.spost_max, R.t_max, 1e3*R.spost_fin);
fprintf('  di cui in Pz                  max %.1f mm\n', 1e3*R.dz_max);
if R.esito == "al bordo o caduto"
    fprintf(2, '  -> %s: guardare l''animazione prima di dichiarare il task.\n', R.esito);
else
    fprintf('  -> %s\n', R.esito);
end

%% ---- CSV ----
dest = fullfile('results', 'diagnostica');
if ~isfolder(dest), mkdir(dest); end
f = fullfile(dest, 'pacco_C3P.csv');
T = struct2table(R);
if isfile(f)
    o = detectImportOptions(f, 'TextType', 'string');
    o = setvartype(o, {'task','controller','esito','data'}, 'string');
    V = readtable(f, o);
    V = V(V.task ~= R.task, T.Properties.VariableNames);
    T = [V; T];
end
writetable(sortrows(T, 'task'), f);
fprintf('  scritto  %s\n\n', f);
end
