%% asimmetria_T6.m - il corpo resta in piano quando un lato e' piu' alto?
%
% PERCHE' NON BASTANO LE METRICHE DI T6
%   [26/9] Gli ostacoli di T6 sono a mezza larghezza apposta: servono a
%   mettere il lato destro piu' alto del sinistro e poi viceversa. E' quello
%   che devono mostrare.
%   Ma alt_ost, appoggi_su_ost e superato contano QUALI ostacoli il robot ha
%   incontrato, e quelli dipendono dalla traiettoria. Misurato oggi: C1
%   sbanda di 57 gradi e si ferma a 2.45 m, C2 tiene la direzione (0.4 gradi)
%   ma trasla di 24 cm su 3.7 m e manca gli ostacoli locali. Due percorsi
%   diversi non calpestano gli stessi ostacoli, quindi quelle colonne
%   confrontano cose diverse e non sono attribuibili al controllore.
%   Allargare gli ostacoli a tutto il pavimento toglierebbe il problema e
%   anche il motivo per cui esistono. Quindi non si cambia la pista: si
%   cambia la misura.
%
% COSA MISURA
%   A ogni istante con i piedi in appoggio:
%       dz   = (quota media piedi SINISTRI) - (quota media piedi DESTRI)
%       roll = rollio del corpo
%   e si stima la PENDENZA della retta  roll = a*dz + b.
%
%       a ~ 1/carreggiata   il corpo SEGUE il terreno, nessuna compensazione
%       a ~ 0               il corpo resta IN PIANO, compensazione piena
%
%   Indice di compensazione:  1 - a*w   con w = carreggiata misurata dai dati
%       0 = segue il terreno      1 = resta in piano
%
%   E' adimensionale e NON dipende da quanti ostacoli sono stati calpestati
%   ne' da dove il robot e' passato: usa solo i campioni in cui l'asimmetria
%   c'e' davvero. Se un controllore ne incontra meno, ha meno campioni ma la
%   pendenza si stima lo stesso - sotto la soglia di campioni dichiarata qui
%   sotto la stima non si fa e si dice perche'.
%
% L'APPOGGIO SI RICONOSCE DALLA VELOCITA', NON DALLA FORZA
%   Su T6 i sensori di forza chiudono al 41-65% del peso: vedono il pavimento
%   e non i piedi sugli ostacoli, cioe' proprio quelli che qui servono.
%   run.contact e' quindi inutilizzabile. Si usa lo stesso criterio di
%   t6_passaggio in script_T6: piede piu' lento di v_appoggio = fermo, in
%   appoggio. Lo swing va a ~0.24 m/s di media, la soglia e' 0.05 m/s.
%
% I CRITERI, SCRITTI PRIMA DI LANCIARE
%   1. La stima vale solo con almeno 200 campioni ad asimmetria significativa
%      (|dz| > 5 mm). Sotto, si dichiara "campioni insufficienti" e basta.
%   2. UN CONTROLLORE COMPENSA se il suo indice supera di almeno 0.15 quello
%      dell'altro. Meno di cosi' non lo si distingue con una run per parte.
%   3. La traslazione laterale NON entra in questo indice: si riporta a
%      parte, come fatto separato. Un robot puo' restare in piano e
%      contemporaneamente sbandare, e sono due difetti diversi.
%
% COSA NON TOCCA
%   Non usa script_T6 e non scrive nessun CSV di campagna: simula qui, con la
%   stessa preparazione (terreno, inerzie, stimatore) e lo stesso taglio al
%   bordo del pavimento. Scrive results/diagnostica/asimmetria_T6.csv e una
%   figura. Nessun save_system.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   asimmetria_T6
%
% Progetto FSR PhantomX - A. Russo

% SCRIPT e non funzione: OVERRIDE_C2 deve stare nel workspace BASE, che e'
% dove l'InitFcn del modello lo legge. Variabili prefissate am_.

am_cfg   = phantomx_config();
am_mdl   = 'phantomx_sim_zero';
am_dur   = 30;                     % come script_T6
am_xbordo = 4 - 0.05;              % il pavimento e' il cubo 8 x 8
am_vapp  = 0.05;                   % [m/s] piede piu' lento = in appoggio
am_dzmin = 0.005;                  % [m] sotto questa asimmetria non si impara niente
am_nmin  = 200;                    % campioni minimi per stimare

% ordine CAN delle zampe: FL FR ML MR RL RR
am_sin = [1 3 5];                  % FL ML RL
am_des = [2 4 6];                  % FR MR RR

if ~bdIsLoaded(am_mdl), load_system(am_mdl); end
applica_terreno('T6', false, am_mdl);
applica_inerzie(am_mdl);
allinea_stimatore(am_mdl);

AM  = table();
am_R = {};
am_crono = tic;

try
    for am_c2 = [false true]
        am_ctrl = 'C1';  if am_c2, am_ctrl = 'C2'; end
        fprintf('\n\n============ T6, %s ============\n', am_ctrl);

        OVERRIDE_C2 = am_c2;                                   %#ok<NASGU>
        clear OVERRIDE_GAIT OVERRIDE_C2_SOGLIA                 % andatura e soglia di cfg
        init_gait

        am_out = sim(am_mdl, 'StopTime', num2str(am_dur));
        am_run = adatta_simscape(am_out, struct('controller', am_ctrl, ...
                     'task','T6', 'run',1, 'condizione','asimmetria', ...
                     'vel_d', [am_cfg.v_nom 0]));

        % stesso taglio al bordo di script_T6: la caduta dal pavimento non e'
        % un comportamento del controllore
        [am_run, am_tb] = am_taglia(am_run, am_xbordo);
        if ~isnan(am_tb)
            fprintf(2,'  un piede supera il bordo a t = %.2f s: run troncata li''\n', am_tb);
        end

        am_R{end+1} = am_run;                                  %#ok<AGROW>
        am_riga = am_analisi(am_run, am_cfg, am_sin, am_des, ...
                             am_vapp, am_dzmin, am_nmin);
        am_riga.controller = string(am_ctrl);
        am_q = metriche(am_run, am_cfg, struct('t_regime', 2*am_cfg.T));
        for am_c = {'dev_lat_max','yaw_err_fin','distanza','frazione_task','roll_max'}
            if ismember(am_c{1}, am_q.Properties.VariableNames)
                am_riga.(am_c{1}) = am_q.(am_c{1})(1);
            end
        end
        AM = [AM; am_riga];                                    %#ok<AGROW>
    end
    clear OVERRIDE_C2
    init_gait
catch am_err
    clear OVERRIDE_C2
    fprintf(2,'\n  Errore: modello non salvato, nessun CSV di campagna toccato.\n');
    rethrow(am_err);
end

if ~isfolder(fullfile('results','diagnostica')), mkdir(fullfile('results','diagnostica')); end
writetable(AM, fullfile('results','diagnostica','asimmetria_T6.csv'));

%% ---- lettura ----
fprintf('\n\n========== T6: IL CORPO RESTA IN PIANO? ==========\n');
fprintf('  carreggiata misurata: %.3f m\n', mean(AM.carreggiata));
fprintf('  pendenza "segue il terreno" attesa: %.2f rad/m\n\n', 1/mean(AM.carreggiata));
fprintf('  %-12s %10s %10s %12s %10s %10s\n', 'controllore', 'campioni', ...
        'pendenza', 'indice', 'R2', 'roll_rms*');
for am_i = 1:height(AM)
    fprintf('  %-12s %10d %10.3f %12.3f %10.3f %10.4f\n', AM.controller(am_i), ...
            AM.n_campioni(am_i), AM.pendenza(am_i), AM.indice(am_i), ...
            AM.R2(am_i), AM.roll_rms_asim(am_i));
end
fprintf('  * roll_rms calcolato SOLO sui campioni asimmetrici\n');
fprintf('  indice: 0 = segue il terreno, 1 = resta in piano\n');

fprintf('\n  --- fatti separati, non entrano nell''indice ---\n');
fprintf('  %-12s %12s %12s %12s\n', 'controllore', 'dev_lat[m]', 'yaw_fin[deg]', 'distanza[m]');
for am_i = 1:height(AM)
    fprintf('  %-12s %12.3f %12.1f %12.3f\n', AM.controller(am_i), ...
            AM.dev_lat_max(am_i), rad2deg(AM.yaw_err_fin(am_i)), AM.distanza(am_i));
end

%% ---- i criteri ----
if any(AM.n_campioni < am_nmin)
    fprintf(2,['\n  => CAMPIONI INSUFFICIENTI per almeno un controllore ' ...
               '(minimo %d).\n     La stima non si fa.\n'], am_nmin);
elseif height(AM) == 2
    am_d = AM.indice(2) - AM.indice(1);
    fprintf('\n  differenza di indice C2 - C1: %+.3f\n', am_d);
    if abs(am_d) < 0.15
        fprintf('  => NON DISTINGUIBILI su questo indice con una run per parte.\n');
        fprintf('     Su T6 nessuno dei due compensa l''asimmetria piu'' dell''altro.\n');
    elseif am_d > 0
        fprintf('  => C2 COMPENSA di piu'': tiene il corpo piu'' in piano di C1\n');
        fprintf('     quando un lato e'' piu'' alto dell''altro. E'' quello che gli\n');
        fprintf('     ostacoli a mezza larghezza erano stati messi li'' a mostrare.\n');
    else
        fprintf('  => C1 COMPENSA di piu''. Da spiegare, perche'' e'' ad anello\n');
        fprintf('     aperto: probabilmente non compensa, e'' C2 che peggiora.\n');
    end
end

%% ---- figura ----
am_fig = figure('Name','asimmetria_T6','Position',[80 80 980 420]);
tiledlayout(am_fig, 1, 2, 'TileSpacing','compact', 'Padding','compact');
for am_i = 1:numel(am_R)
    nexttile;
    [am_dz, am_rl] = am_serie(am_R{am_i}, am_sin, am_des, am_vapp);
    am_s = abs(am_dz) > am_dzmin;
    plot(1e3*am_dz(~am_s), rad2deg(am_rl(~am_s)), '.', 'Color',[.8 .8 .8]); hold on
    plot(1e3*am_dz(am_s),  rad2deg(am_rl(am_s)),  '.', 'MarkerSize',4);
    am_x = linspace(min(1e3*am_dz), max(1e3*am_dz), 20);
    plot(am_x, rad2deg(AM.pendenza(am_i)*am_x*1e-3 + AM.offset(am_i)), 'LineWidth',1.8);
    plot(am_x, rad2deg(am_x*1e-3 / AM.carreggiata(am_i)), 'k--', 'LineWidth',1.2);
    grid on; xlabel('dz sinistra - destra [mm]'); ylabel('rollio [deg]');
    title(sprintf('%s   indice %.2f', AM.controller(am_i), AM.indice(am_i)));
    if am_i == 1
        legend({'|dz| piccolo','usati','retta stimata','segue il terreno'}, 'Location','best');
    end
end
salva_grafico('asimmetria_T6', am_fig);

fprintf('\n  scritto  %s   (%.0f s)\n', ...
        fullfile('results','diagnostica','asimmetria_T6.csv'), toc(am_crono));
fprintf('  Nessun CSV di campagna toccato, modello non salvato.\n\n');

%% ================= helper =================
function [r, t_bordo] = am_taglia(r, x_bordo)
%AM_TAGLIA  Tronca la run al primo campione con un piede oltre x_bordo.
%
% [CORRETTO 26/9] LA VERSIONE INLINE ERA SBAGLIATA, E IN MODO ISTRUTTIVO.
%   Troncava i campi confrontando ogni lunghezza con numel(r.t) DENTRO il
%   ciclo. Ma t e' il primo campo: appena troncato, numel(r.t) cambia e
%   nessun altro campo corrisponde piu'. Risultato: t accorciato, p e rpy
%   interi, e metriche si ferma su un assert.
%   La lunghezza di riferimento va presa PRIMA del ciclo - che e' esattamente
%   quello che fanno t4d_taglia_bordo, sp_taglia e ps_taglia. Avevo riscritto
%   a mano una funzione che esisteva gia' in tre copie giuste.
t_bordo = NaN;
if ~isfield(r,'pf') || isempty(r.pf), return; end
N = numel(r.t);                     % <-- prima del ciclo
k = find(any(r.pf(:, 1:3:18) > x_bordo, 2), 1, 'first');
if isempty(k) || k < 2, return; end
t_bordo = r.t(k);
f = fieldnames(r);
for i = 1:numel(f)
    v = r.(f{i});
    if (isnumeric(v) || islogical(v)) && size(v,1) == N && N > 1
        r.(f{i}) = v(1:k-1, :);
    end
end
end

function [dz, roll] = am_serie(r, sin_i, des_i, vapp)
%AM_SERIE  dz fra i lati e rollio, sui campioni con entrambi i lati in appoggio.
z  = r.pf(:, 3:3:18);
x  = r.pf(:, 1:3:18);
y  = r.pf(:, 2:3:18);
dt = median(diff(r.t));

% velocita' del piede: in appoggio e' quasi ferma rispetto al mondo
v = sqrt(diff([x(1,:); x]).^2 + diff([y(1,:); y]).^2 + diff([z(1,:); z]).^2) / dt;
app = v < vapp;

ok = any(app(:, sin_i), 2) & any(app(:, des_i), 2);
dz   = nan(size(ok));
roll = r.rpy(:,1);
for k = find(ok).'
    dz(k) = mean(z(k, sin_i(app(k, sin_i)))) - mean(z(k, des_i(app(k, des_i))));
end
sel  = ~isnan(dz);
dz   = dz(sel);
roll = roll(sel);
end

function T = am_analisi(r, cfg, sin_i, des_i, vapp, dzmin, nmin)
%AM_ANALISI  Pendenza di roll contro dz, e indice di compensazione.
[dz, roll] = am_serie(r, sin_i, des_i, vapp);

% carreggiata: distanza laterale media fra i piedi dei due lati, dai dati
y = r.pf(:, 2:3:18);
w = abs(mean(mean(y(:, sin_i), 2) - mean(y(:, des_i), 2)));

s = abs(dz) > dzmin;
n = sum(s);

T = table();
T.n_campioni   = n;
T.carreggiata  = w;
T.pendenza     = NaN;
T.offset       = 0;
T.R2           = NaN;
T.indice       = NaN;
T.roll_rms_asim = NaN;

if n < nmin, return; end

% minimi quadrati con offset: un rollio medio non nullo non deve entrare
% nella pendenza
A = [dz(s), ones(n,1)];
p = A \ roll(s);
res = roll(s) - A*p;
T.pendenza = p(1);
T.offset   = p(2);
T.R2       = 1 - sum(res.^2) / sum((roll(s) - mean(roll(s))).^2);
T.indice   = 1 - p(1)*w;          % 0 = segue il terreno, 1 = resta in piano
T.roll_rms_asim = sqrt(mean(roll(s).^2));
end
