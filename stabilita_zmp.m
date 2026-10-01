function info = stabilita_zmp(r, cfg, opt)
%STABILITA_ZMP  ZMP, baricentro e margine sul poligono d'appoggio.
%
%   stabilita_zmp(t4_run)
%   info = stabilita_zmp(r, cfg, struct('grafico',false))
%
% LA DOMANDA
%   [30/9, dal ricevimento] In anello aperto la condizione di ZMP regge?
%
%   Il paper di Arrigoni risponde a monte: la stabilita' non e' controllata,
%   e' IMPOSTA dal generatore di andatura, che accetta solo andature in cui
%   il triangolo d'appoggio contiene sempre la proiezione del baricentro
%   (sez. 3, "arithmetically enforces the robot COG to always stay within
%   bounds"). Il tripode e' la meno ridondante di quelle andature.
%
%   Ma quella garanzia vale sulla geometria NOMINALE. Il triangolo usato nel
%   calcolo e' quello comandato, non quello reale: su rampa e ostacoli i
%   piedi non sono dove la cinematica crede, e nessuno verifica. Questa
%   funzione misura la condizione sullo stato EFFETTIVO della run.
%
% COSA CALCOLA
%   baricentro   CoM completo dell'albero URDF (corpo + 24 link) portato nel
%                mondo con la posa del corpo. NON il solo tronco: il corpo e'
%                il 61.5% della massa, le zampe il 36.9%, e le zampe si
%                muovono - approssimare il CoM col tronco sposterebbe proprio
%                la quantita' che si vuole misurare.
%   ZMP          approssimazione a massa puntiforme:
%                    x_zmp = x_c - (z_c - z_terreno) * ax / (az + g)
%                Trascura la derivata del momento angolare attorno al CoM.
%   poligono     inviluppo convesso dei piedi in APPOGGIO, riconosciuti con
%                appoggio_cinematico - la stessa funzione di script_T4/T4D,
%                non una copia. Niente sensori di forza: su rampa e ostacoli
%                vedono solo il pavimento.
%   margine      distanza con segno dal bordo del poligono. Positivo dentro.
%                Si riporta sia sul baricentro (statico) sia sullo ZMP
%                (dinamico): la loro differenza dice se l'ipotesi
%                quasi-statica regge, invece di assumerlo.
%   CoP          quando le forze ai piedi chiudono sul peso, si calcola anche
%                il centro di pressione MISURATO. E' lo ZMP vero, senza
%                approssimazioni: serve come controprova dove e' disponibile.
%
% TRE LIMITI, DA DICHIARARE IN RELAZIONE
%   1. Lo ZMP classico e' definito per contatto PIANO e complanare. Su rampa
%      e ostacoli i piedi non sono complanari: il calcolo resta nel piano
%      orizzontale e la funzione RIPORTA l'inclinazione del piano d'appoggio,
%      cosi' si vede quando l'approssimazione si indebolisce invece di
%      scoprirlo dopo.
%   2. Il termine dinamico richiede la derivata seconda della posizione, che
%      amplifica il rumore. Si liscia prima di derivare, con una finestra
%      dichiarata (opt.liscia) che finisce nell'output.
%   3. Il CoM e' quello dell'albero URDF: i piedi (1.6% della massa) sono
%      sfere di contatto in Simscape e non corpi dell'URDF, quindi non ci
%      sono. Le MASSE dell'URDF sono giuste - era sbagliato solo il blocco
%      <inertia> - e l'inerzia qui non entra.
%
% SOLO LETTURA. Non simula, non tocca il modello.
%
% OPZIONI
%   .v_app    [m/s] soglia di appoggio cinematico            (0.05)
%   .liscia   [s]   finestra di lisciatura prima di derivare (0.05)
%   .cop            calcola anche il CoP misurato            (true)
%   .grafico                                                 (true)
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(r)
    error('stabilita_zmp:run', ...
        'Serve una run di adatta_simscape (es. t4_run dal workspace).');
end
if nargin < 2 || isempty(cfg), cfg = phantomx_config(); end
if nargin < 3, opt = struct(); end
def = struct('v_app',0.05, 'liscia',0.05, 'cop',true, 'grafico',true);
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt, f{k}) || isempty(opt.(f{k})), opt.(f{k}) = def.(f{k}); end
end

for c = {'t','p','rpy','q','pf'}
    if ~isfield(r, c{1}) || isempty(r.(c{1}))
        error('stabilita_zmp:campo', 'Manca run.%s.', c{1});
    end
end

t  = r.t(:);
dt = median(diff(t));
N  = numel(t);
g  = cfg.g;

%% ================= 1. baricentro nel mondo =================
rbt = sz_albero();
com_b = sz_com_base(rbt, r.q, cfg);        % [N x 3] nel frame del corpo
com   = nan(N,3);
for n = 1:N
    R = sz_rotZYX(r.rpy(n,:));
    com(n,:) = (r.p(n,:).' + R*com_b(n,:).').';
end

%% ================= 2. appoggio e poligono =================
fermo = appoggio_cinematico(r, cfg, opt.v_app);
nApp  = sum(fermo, 2);
xf = r.pf(:, 1:3:18);  yf = r.pf(:, 2:3:18);  zf = r.pf(:, 3:3:18);

%% ================= 3. ZMP =================
% Si liscia PRIMA di derivare: la derivata seconda di un segnale a 200 Hz
% amplifica tutto quello che non e' moto del corpo.
w    = max(3, round(opt.liscia/dt));
coms = movmean(com, w, 1);
acc  = [gradient(gradient(coms(:,1), dt), dt), ...
        gradient(gradient(coms(:,2), dt), dt), ...
        gradient(gradient(coms(:,3), dt), dt)];

z_terr = nan(N,1);
for n = 1:N
    if nApp(n) > 0, z_terr(n) = median(zf(n, fermo(n,:))); end
end
h   = com(:,3) - z_terr;                    % altezza del CoM sul terreno
den = acc(:,3) + g;
den(abs(den) < 0.1*g) = NaN;                % caduta libera: lo ZMP non esiste
zmp = [com(:,1) - h .* acc(:,1) ./ den, ...
       com(:,2) - h .* acc(:,2) ./ den];

%% ================= 4. margini =================
m_stat = nan(N,1);  m_din = nan(N,1);  incl = nan(N,1);
for n = 1:N
    i = find(fermo(n,:));
    if numel(i) < 3, continue; end
    P = [xf(n,i).', yf(n,i).'];
    [m_stat(n), K] = sz_margine(com(n,1:2), P);
    m_din(n)       = sz_margine(zmp(n,:),   P);
    incl(n)        = sz_inclinazione([P, zf(n,i).'], K);
end

def_poly = ~isnan(m_stat);
scarto   = vecnorm(zmp - com(:,1:2), 2, 2);

%% ================= stampa =================
fprintf('\n=============================================================\n');
fprintf('  STABILITA'' / ZMP - %s\n', sz_meta(r));
fprintf('  CoM: corpo %.1f%% + zampe %.1f%% della massa (%.3f kg)\n', ...
        100*cfg.m_body/cfg.mass, 100*24*cfg.m_link/cfg.mass, cfg.mass);
fprintf('  lisciatura prima di derivare: %.3f s (%d campioni)\n', w*dt, w);
fprintf('=============================================================\n\n');

fprintf('  APPOGGIO\n');
fprintf('    piedi in appoggio, mediana        %d\n', median(nApp));
fprintf('    istanti con almeno 3 piedi        %.1f%%\n', 100*mean(nApp >= 3));
if mean(nApp >= 3) < 0.5
    fprintf(2, ['    Meno della meta'' degli istanti ha un poligono: il margine\n' ...
                '    non e'' rappresentativo. Prima si chiude diagnosi_appoggio.\n']);
end

fprintf('\n  MARGINE sul poligono d''appoggio  [mm]\n');
fprintf('    %-24s %8s %8s %8s\n', '', 'mediana', 'minimo', 'negativo');
fprintf('    %-24s %8.1f %8.1f %7.1f%%\n', 'baricentro (statico)', ...
        1e3*median(m_stat,'omitnan'), 1e3*min(m_stat), ...
        100*mean(m_stat(def_poly) < 0));
fprintf('    %-24s %8.1f %8.1f %7.1f%%\n', 'ZMP (dinamico)', ...
        1e3*median(m_din,'omitnan'), 1e3*min(m_din), ...
        100*mean(m_din(def_poly) < 0));

fprintf('\n  QUANTO PESA IL TERMINE DINAMICO\n');
fprintf('    |ZMP - proiezione CoM|   mediana %.1f mm   massimo %.1f mm\n', ...
        1e3*median(scarto,'omitnan'), 1e3*max(scarto));
mm = median(abs(m_stat),'omitnan');
if median(scarto,'omitnan') < 0.1*mm
    fprintf(['    Sotto il 10%% del margine tipico: il regime e'' quasi-statico\n' ...
             '    e il criterio statico del paper e'' quello giusto.\n']);
else
    fprintf(2, ['    NON trascurabile rispetto al margine tipico (%.1f mm): qui il\n' ...
                '    criterio statico non basta, va citato lo ZMP.\n'], 1e3*mm);
end

fprintf('\n  PIANO D''APPOGGIO\n');
fprintf('    inclinazione   mediana %.1f deg   massima %.1f deg\n', ...
        median(incl,'omitnan'), max(incl));
if max(incl) > 5
    fprintf(['    Sopra i 5 gradi: lo ZMP classico e'' definito per contatto\n' ...
             '    piano. Su questo task il calcolo nel piano orizzontale e''\n' ...
             '    un''approssimazione, e va dichiarata.\n']);
end

%% ================= 5. CoP misurato, dove si puo' =================
cop = nan(N,2);  chiusura = NaN;
if opt.cop && isfield(r,'Fc') && ~isempty(r.Fc)
    sel = t >= 2*cfg.T;
    Fz  = r.Fc(:, 3:3:18);
    chiusura = mean(sum(Fz(sel,:),2)) / (cfg.mass*g);
    fprintf('\n  CoP MISURATO\n');
    fprintf('    chiusura sul peso   %.0f%%\n', 100*chiusura);
    if abs(chiusura - 1) > 0.05
        fprintf(2, ['    Fuori dal 5%%: i sensori non vedono tutto il contatto\n' ...
                    '    (su rampa e ostacoli vedono solo il pavimento). Il CoP\n' ...
                    '    non e'' calcolabile su questo task, per costruzione.\n']);
    else
        tot = sum(Fz, 2);
        ok  = tot > 0.1*cfg.mass*g;
        cop(ok,1) = sum(xf(ok,:).*Fz(ok,:), 2) ./ tot(ok);
        cop(ok,2) = sum(yf(ok,:).*Fz(ok,:), 2) ./ tot(ok);
        d = vecnorm(cop - zmp, 2, 2);
        fprintf('    |CoP - ZMP|   mediana %.1f mm   massimo %.1f mm\n', ...
                1e3*median(d,'omitnan'), 1e3*max(d));
        fprintf(['    E'' la controprova: il CoP e'' misurato e non approssima\n' ...
                 '    niente. Uno scarto piccolo valida il modello di ZMP.\n']);
    end
end

%% ================= verdetto =================
fprintf('\n  -----------------------------------------------------------\n');
if ~any(def_poly)
    esito = 'non calcolabile';
    fprintf('  Nessun istante con tre piedi in appoggio: non c''e'' poligono.\n');
elseif min(m_din) > 0
    esito = 'condizione sempre rispettata';
    fprintf('  Lo ZMP resta DENTRO il poligono per tutta la run.\n');
    fprintf('  Margine minimo %.1f mm.\n', 1e3*min(m_din));
else
    esito = 'condizione violata';
    [~, kmin] = min(m_din);
    fprintf(2, '  Lo ZMP ESCE dal poligono nel %.1f%% degli istanti.\n', ...
            100*mean(m_din(def_poly) < 0));
    fprintf(2, '  Peggio a t = %.2f s, x = %.3f m, margine %.1f mm.\n', ...
            t(kmin), com(kmin,1), 1e3*m_din(kmin));
    fprintf(['  In anello aperto non c''e'' niente che lo riporti dentro: la\n' ...
             '  condizione e'' imposta a priori sulla geometria nominale.\n']);
end
fprintf('  -----------------------------------------------------------\n\n');

%% ================= grafici =================
if opt.grafico
    figure;
    subplot(2,1,1); hold on; grid on; axis equal
    k = find(def_poly);
    for n = k(round(linspace(1, numel(k), min(12, numel(k)))))
        i = find(fermo(n,:));
        P = [xf(n,i).', yf(n,i).'];
        K = convhull(P(:,1), P(:,2));
        plot(P(K,1), P(K,2), '-', 'Color', [.8 .8 .8]);
    end
    plot(com(:,1), com(:,2), 'b-', 'DisplayName','baricentro');
    plot(zmp(:,1), zmp(:,2), 'r-', 'DisplayName','ZMP');
    if any(~isnan(cop(:,1)))
        plot(cop(:,1), cop(:,2), 'g-', 'DisplayName','CoP misurato');
    end
    xlabel('x [m]'); ylabel('y [m]'); legend('Location','best');
    title(sprintf('%s - vista dall''alto (grigio: poligoni d''appoggio)', sz_meta(r)));

    subplot(2,1,2); hold on; grid on
    plot(t, 1e3*m_stat, 'b-', 'DisplayName','margine sul baricentro');
    plot(t, 1e3*m_din,  'r-', 'DisplayName','margine sullo ZMP');
    yline(0, 'k--');
    xlabel('t [s]'); ylabel('margine [mm]'); legend('Location','best');
end

info = struct('esito', esito, 'com', com, 'zmp', zmp, 'cop', cop, ...
              'margine_statico', m_stat, 'margine_dinamico', m_din, ...
              'n_appoggio', nApp, 'inclinazione', incl, ...
              'scarto_zmp_com', scarto, 'chiusura_peso', chiusura, ...
              'liscia_s', w*dt, 'v_app', opt.v_app);
end

%% ================= helper =================
function rbt = sz_albero()
%SZ_ALBERO  L'albero URDF dal base workspace, o costruito se manca.
if ~evalin('base', 'exist(''robotModel'',''var'')')
    fprintf('  robotModel non c''e'': lo costruisco con Setup_robot_object...\n');
    evalin('base', 'Setup_robot_object');
end
rbt = evalin('base', 'robotModel');
end

function com = sz_com_base(rbt, q, cfg)
%SZ_COM_BASE  CoM dell'albero nel frame del corpo, istante per istante.
%   L'indicizzazione dei giunti e' la stessa di adatta_simscape>piedi: per
%   NOME, non per posizione nella lista. Una sola copia della convenzione.
suff  = struct('FL','lf','FR','rf','ML','lm','MR','rm','RL','lr','RR','rr');
conf0 = homeConfiguration(rbt);
strutt = isstruct(conf0);
if strutt
    nomi = {conf0.JointName};
else
    nomi = {};
    for b = 1:numel(rbt.Bodies)
        if ~strcmp(rbt.Bodies{b}.Joint.Type, 'fixed')
            nomi{end+1} = rbt.Bodies{b}.Joint.Name; %#ok<AGROW>
        end
    end
end

ix = nan(6,3);
for i = 1:6
    u = suff.(cfg.legNamesCAN{i});
    g = {['j_c1_' u], ['j_thigh_' u], ['j_tibia_' u]};
    for j = 1:3
        k = find(strcmp(nomi, g{j}), 1);
        if ~isempty(k), ix(i,j) = k; end
    end
end
if any(isnan(ix(:)))
    error('stabilita_zmp:giunti', ...
        ['Non trovo tutti e 18 i giunti nell''albero per nome. Se il modello\n' ...
         'e'' cambiato, guarda showdetails(robotModel) prima di fidarti.']);
end

N = size(q,1);
com = nan(N,3);
for n = 1:N
    c = conf0;
    for i = 1:6
        for j = 1:3
            if strutt, c(ix(i,j)).JointPosition = q(n, 3*(i-1)+j);
            else,      c(ix(i,j))               = q(n, 3*(i-1)+j);
            end
        end
    end
    com(n,:) = centerOfMass(rbt, c).';
end
end

function [m, K] = sz_margine(pt, P)
%SZ_MARGINE  Distanza con segno dal bordo dell'inviluppo convesso di P.
%   Positiva dentro, negativa fuori. Nessun toolbox.
m = NaN;  K = [];
if size(P,1) < 3 || any(isnan(pt)), return; end
try
    K = convhull(P(:,1), P(:,2));
catch
    return          % piedi allineati: il poligono degenera, margine indefinito
end
V = P(K(1:end-1), :);                  % vertici, senza ripetere il primo
if size(V,1) < 3, return; end
% orientamento antiorario, cosi' "dentro" e' sempre a sinistra di ogni lato
if sz_area(V) < 0, V = flipud(V); end
n = size(V,1);
d = inf;
for k = 1:n
    a = V(k,:);  b = V(mod(k,n)+1, :);
    e = b - a;   L = norm(e);
    if L < eps, continue; end
    d = min(d, ((b(1)-a(1))*(pt(2)-a(2)) - (b(2)-a(2))*(pt(1)-a(1))) / L);
end
m = d;
end

function A = sz_area(V)
A = 0.5 * sum(V(:,1).*V([2:end 1],2) - V([2:end 1],1).*V(:,2));
end

function a = sz_inclinazione(P3, K)
%SZ_INCLINAZIONE  Angolo fra il piano dei piedi in appoggio e l'orizzontale.
a = NaN;
if size(P3,1) < 3, return; end
c = mean(P3, 1);
[~, ~, V] = svd(P3 - c, 0);
nz = V(:,3);                            % normale al piano dei minimi quadrati
a = rad2deg(acos(min(1, abs(nz(3)))));
end

function R = sz_rotZYX(a)
%SZ_ROTZYX  Stessa convenzione di adatta_simscape.
cr = cos(a(1)); sr = sin(a(1));
cp = cos(a(2)); sp = sin(a(2));
cy = cos(a(3)); sy = sin(a(3));
R = [ cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr ; ...
      sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr ; ...
        -sp,            cp*sr,            cp*cr ];
end

function s = sz_meta(r)
s = 'run';
if isfield(r,'meta') && isstruct(r.meta)
    c = ''; k = '';
    if isfield(r.meta,'controller'), c = char(string(r.meta.controller)); end
    if isfield(r.meta,'task'),       k = char(string(r.meta.task));       end
    if ~isempty(c) || ~isempty(k), s = strtrim([k ' ' c]); end
end
end
