function complanarita(n)
%COMPLANARITA  I sei piedi arrivano alla stessa quota? E se no, di quanto.
%
%   complanarita        7 punti lungo il passo
%   complanarita(15)    piu' fitto
%
% PERCHE' ESISTE (e perche' verifica_ik non bastava)
%   verifica_ik misura due cose, e nessuna delle due e' quella che rompe il
%   tripode:
%     - l'ESCURSIONE della quota di UNA zampa lungo il passo
%     - solo su FL e ML, due zampe su sei
%   Un errore di quota COSTANTE ma DIVERSO da zampa a zampa passa
%   inosservato: verifica_ik lo vede come costante (e quindi innocuo) su
%   ciascuna zampa presa da sola. Ma e' esattamente cio' che mette i piedi
%   su piani diversi.
%
%   Sintomo osservato: i tripodi partono corretti (FL-MR-RL, poi FR-ML-RR)
%   ma dentro ogni gruppo le tre zampe toccano scaglionate di ~0.17 s, e
%   ciascuna porta da sola tutto il peso. Il profilo di volo C2 amplifica
%   il difetto: a fine volo la velocita' verticale del piede e' nulla per
%   costruzione, quindi la zampa piu' alta non scende piu' da sola e tocca
%   solo quando il corpo si abbassa.
%
% COSA MISURA
%   La quota ASSOLUTA dei sei piedi nel frame dell'URDF, lungo tutto il
%   passo. Conta lo scarto max-min DENTRO ogni tripode: sono quelle tre
%   zampe che devono toccare insieme.
%
% COSA PROPONE
%   1. una correzione di quota per zampa, tradotta in correzioni di giunto
%      da incollare in init_gait al posto di  q_corr = zeros(18,1).
%      Usa il vettore "offset" gia' cablato: il modello Simulink non si tocca.
%   2. in alternativa, la rigidezza di contatto che assorbirebbe lo scarto
%      senza correggere niente. Non e' barare: un terreno reale e' piu'
%      cedevole di 5000 N/m. Va dichiarato fra le assunzioni del modello.
%
% CRITERIO (fissato prima di misurare, per non inseguire la perfezione)
%   Lo scarto dentro il tripode deve stare sotto il 30% della penetrazione
%   statica disponibile. Sotto quella soglia le tre zampe si caricano tutte;
%   non serve che sia zero.
%
% PRIMA
%   startup_phantomx ; init_gait
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1, n = 7; end

cfg = phantomx_config();
if ~evalin('base','exist(''robotModel'',''var'')')
    fprintf(2,'\n  robotModel non c''e''. Lancia init_gait.\n\n'); return
end
rbt = evalin('base','robotModel');

suff  = {'lf','rf','lm','rm','lr','rr'};          % ordine CAN
tripA = [1 4 5];   nomeA = 'FL-MR-RL';           % fase 0
tripB = [2 3 6];   nomeB = 'FR-ML-RR';           % fase 1/2
off   = [cfg.foot_xyz(1); -cfg.foot_xyz(2); cfg.foot_xyz(3); 1];
xs    = linspace(-cfg.S/2, +cfg.S/2, n);
pen   = cfg.mass*cfg.g/(3*cfg.contact.k);

fprintf('\n=================== COMPLANARITA'' ===================\n');
fprintf('  quota assoluta del piede, frame URDF, %d punti del passo\n', n);
fprintf('  penetrazione statica disponibile: %.2f mm\n', 1000*pen);

%% ---- quote ----
Z = quote(rbt, suff, cfg, xs, off, cfg.z0*ones(1,6));
if any(isnan(Z(:)))
    fprintf(2,'\n  La cinematica diretta non ha risposto su qualche zampa.\n\n'); return
end

fprintf('\n--- QUOTA DEL PIEDE [mm] ---\n');
fprintf('%-8s', 'x[mm]');
for i = 1:6, fprintf(' %8s', cfg.legNamesCAN{i}); end
fprintf(' %10s\n', 'max-min');
for k = 1:n
    fprintf('%-8.0f', 1000*xs(k));
    for i = 1:6, fprintf(' %8.2f', 1000*Z(k,i)); end
    fprintf(' %10.2f\n', 1000*(max(Z(k,:)) - min(Z(k,:))));
end

%% ---- scarto dentro i tripodi ----
sA = max(max(Z(:,tripA),[],2) - min(Z(:,tripA),[],2));
sB = max(max(Z(:,tripB),[],2) - min(Z(:,tripB),[],2));
sG = max(max(Z,[],2) - min(Z,[],2));
esc = max(max(Z,[],1) - min(Z,[],1));             % escursione peggiore, una zampa

fprintf('\n--- SCARTO ---\n');
fprintf('  tripode A  %-10s  %6.2f mm\n', nomeA, 1000*sA);
fprintf('  tripode B  %-10s  %6.2f mm\n', nomeB, 1000*sB);
fprintf('  fra tutte e sei             %6.2f mm\n', 1000*sG);
fprintf('  escursione di una zampa     %6.2f mm   (quella che vedeva verifica_ik)\n', 1000*esc);

scarto = max(sA, sB);

%% ---- verdetto ----
fprintf('\n--- VERDETTO ---\n');
if scarto < 0.3*pen
    fprintf('  PASSA. Lo scarto dentro il tripode e'' il %.0f%% della penetrazione\n', ...
            100*scarto/pen);
    fprintf('  disponibile: le tre zampe si caricano insieme. La causa della\n');
    fprintf('  staffetta e'' ALTROVE (body_z0, parametri di contatto, traiettoria).\n');
    fprintf('\n=====================================================\n\n');
    return
end

fprintf(2,'  NON PASSA: %.2f mm di scarto contro %.2f mm di penetrazione.\n', ...
        1000*scarto, 1000*pen);
fprintf(2,'  La zampa piu'' alta tocca solo dopo che il corpo e'' sceso, e nel\n');
fprintf(2,'  frattempo la piu'' bassa porta tutto il peso da sola.\n');

%% ==================== STRADA 1: correzione per zampa ====================
z_med = mean(Z, 1);                 % quota media di ogni zampa lungo il passo
z_rif = mean(z_med);                % piano di riferimento
dz    = z_med - z_rif;              % positivo = piede troppo ALTO, deve scendere

z0_i  = cfg.z0 + dz;                % profondita' comandata, per zampa

Zc = quote(rbt, suff, cfg, xs, off, z0_i);
sAc = max(max(Zc(:,tripA),[],2) - min(Zc(:,tripA),[],2));
sBc = max(max(Zc(:,tripB),[],2) - min(Zc(:,tripB),[],2));
res = max(sAc, sBc);

fprintf('\n--- STRADA 1: correzione di quota per zampa ---\n');
fprintf('%-6s %12s %14s\n', 'zampa', 'dz [mm]', 'z0 corretta [m]');
for i = 1:6
    fprintf('%-6s %12.3f %14.6f\n', cfg.legNamesCAN{i}, 1000*dz(i), z0_i(i));
end
fprintf('  scarto residuo dentro il tripode: %.3f mm  (era %.2f)\n', 1000*res, 1000*scarto);

if res < 0.3*pen
    fprintf('  BASTA: sotto la soglia. Incolla il blocco qui sotto.\n');
else
    fprintf(2,'  NON basta da sola: resta %.0f%% della penetrazione.\n', 100*res/pen);
    fprintf(2,'  La parte residua VARIA lungo il passo, e una correzione\n');
    fprintf(2,'  costante non puo'' assorbirla. Vai alla strada 2.\n');
end

%% ---- correzioni di giunto, ordine Mux ----
% init_gait vuole q_corr in ordine Mux (RR MR FR RL ML FL), terne coxa-femore-tibia.
Q = zeros(6,3);
for i = 1:6
    [t0,f0,s0] = inv_kyn(0, 0, cfg.z0,  cfg.side(i), cfg.alpha(i));
    [t1,f1,s1] = inv_kyn(0, 0, z0_i(i), cfg.side(i), cfg.alpha(i));
    Q(i,:) = [t1-t0, f1-f0, s1-s0];
end

fprintf('\n--- IN init_gait.m, al posto di  q_corr = zeros(18,1) ---\n\n');
fprintf('    %% complanarita'': scarto fra i piedi di uno stesso tripode.\n');
fprintf('    %% Calcolato da complanarita.m sulla FK esatta dell''URDF.\n');
fprintf('    q_corr = [ ...\n');
for k = 1:6
    i = cfg.mux2can(k);                         % zampa CAN nello slot Mux k
    fprintf('        %+.6f; %+.6f; %+.6f;   %% %s\n', ...
            Q(i,1), Q(i,2), Q(i,3), cfg.legNamesCAN{i});
end
fprintf('    ];\n\n');
fprintf('  Poi:  init_gait ; out = sim(''phantomx_sim_zero'',''StopTime'',''10'');\n');
fprintf('        analizza_forze(out.Fleg, ''con correzione'')\n');

%% ==================== STRADA 2: cedevolezza del contatto ====================
k_nec = 0.3 * cfg.mass*cfg.g / (3*res);
k_now = cfg.contact.k;

fprintf('\n--- STRADA 2: contatto piu'' cedevole ---\n');
fprintf('  rigidezza attuale          %8.0f N/m   -> penetrazione %.2f mm\n', ...
        k_now, 1000*pen);
fprintf('  rigidezza che assorbirebbe %8.0f N/m   -> penetrazione %.2f mm\n', ...
        k_nec, 1000*cfg.mass*cfg.g/(3*k_nec));
if k_nec < k_now
    fprintf('  cfg.contact.k = %.0f;   e ricalibra lo smorzamento:\n', round(k_nec, -1));
    fprintf('  cfg.contact.c = %.0f;   (zeta = 1 su un terzo della massa)\n', ...
            round(2*sqrt(k_nec*cfg.mass/3)));
    fprintf('  Da dichiarare nel report accanto a mu_plant: e'' un''assunzione\n');
    fprintf('  sul terreno, non un aggiustamento nascosto.\n');
else
    fprintf('  Non serve: la rigidezza attuale e'' gia'' sufficiente una volta\n');
    fprintf('  applicata la strada 1.\n');
end

fprintf('\n  Criterio di accettazione (fissato prima di misurare):\n');
fprintf('    piedi a terra >= 2.7    tre piedi >= 70%%\n');
fprintf('    carico max per zampa < 10 N    picco < 2x peso\n');
fprintf('  Raggiunte queste, si chiude: non serve arrivare a 3.00 e 100%%.\n');
fprintf('\n=====================================================\n\n');

end

%% ================================================================
function Z = quote(rbt, suff, cfg, xs, off, z0_i)
%QUOTE  Quota assoluta dei sei piedi, per ogni punto del passo.
Z = nan(numel(xs), 6);
for i = 1:6
    for k = 1:numel(xs)
        [th,ph,ps] = inv_kyn(xs(k), 0, z0_i(i), cfg.side(i), cfg.alpha(i));
        v = fkGamba(rbt, suff{i}, [th ph ps], off);
        if isempty(v), return; end
        Z(k,i) = v(3);
    end
end
end

function v = fkGamba(rbt, u, q, off)
v = [];
g = {['j_c1_' u], ['j_thigh_' u], ['j_tibia_' u]};
try
    c = homeConfiguration(rbt);
    if isstruct(c)
        for j = 1:3
            k = find(strcmp({c.JointName}, g{j}), 1);
            if isempty(k), return; end
            c(k).JointPosition = q(j);
        end
    else
        nomi = {};
        for b = 1:numel(rbt.Bodies)
            if ~strcmp(rbt.Bodies{b}.Joint.Type,'fixed')
                nomi{end+1} = rbt.Bodies{b}.Joint.Name; %#ok<AGROW>
            end
        end
        for j = 1:3
            k = find(strcmp(nomi, g{j}), 1);
            if isempty(k), return; end
            c(k) = q(j);
        end
    end
    T = getTransform(rbt, c, ['tibia_' u], rbt.BaseName) * off;
    v = T(1:3);
catch
end
end