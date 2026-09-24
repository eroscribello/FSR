%% prova_S0.m - il gradino S0: giunti in coppia, niente MPC, robot fermo
%
% COSA VERIFICA
%   Che la mappa forze -> coppie sia giusta: si applica su tutte e sei le
%   zampe la forza verticale m*g/6 e si guarda se il robot resta dov'e'.
%   Se il segno o lo Jacobiano fossero sbagliati, da S1 in poi si vedrebbe
%   "un robot che si comporta male" senza sapere perche'.
%
% [RITIRATO 24/9] LA RAMPA DI COPPIA
%   La versione precedente faceva salire la coppia da 0 in 0.5 s. Sbagliata:
%   la gravita' resta piena da t = 0, quindi durante la rampa il robot e'
%   sotto-coppia per costruzione e DEVE scendere. Misurato: scende e il
%   solutore molla a 0.2 s, cioe' al 40% della rampa. Non era il transitorio
%   del contatto: era la rampa stessa a togliere il sostegno.
%   La rampa resta solo come raccordo numerico (t_rampa = 0.05 s) per non
%   dare una discontinuita' al solutore al primo passo.
%
% [DA MISURARE 24/9] COSA MANCA, L'IPOTESI DA VERIFICARE: SMORZAMENTO
%   Nel modello base i 18 giunti sono comandati in POSIZIONE: l'attrito del
%   riduttore non serve, perche' il moto e' imposto. Liberandoli in coppia
%   (copia per C3) restano 18 giunti con attrito ESATTAMENTE ZERO. Le coppie
%   statiche sono un equilibrio, ma un equilibrio senza dissipazione: ogni
%   errore cresce invece di spegnersi. E' coerente con i due sintomi opposti
%   gia' visti, gradino che schizza in alto e rampa che cade.
%   Il valore NON e' un numero di comodo: e' la retta coppia-velocita'
%   dell'AX-12A, che e' gia' in phantomx_config,
%       b = cfg.tau_max / cfg.qd_max = 1.5 / 5.655 = 0.265 N*m*s/rad
%   cioe' coppia di stallo a velocita' zero, coppia nulla a vuoto. Un servo
%   con riduttore 254:1 si comporta cosi'.
%   Importante: a velocita' nulla lo smorzamento da' coppia nulla, quindi
%   NON sposta l'equilibrio statico e NON invalida tau_statico. Cambia solo
%   il transitorio. Se e' questa la causa, resta nel modello anche per
%   S1..S4 e per C3, perche' e' una proprieta' fisica del servo.
%
% COSA TOCCA E COSA NO
%   - smorzamento: scritto IN MEMORIA sui 18 giunti, come applica_inerzie.
%     Nessun save_system, il .slx non cambia.
%   - sorgente della coppia: il Constant tau_S0 viene sostituito UNA VOLTA
%     da un From Workspace che legge tau_ts. Questa e' una modifica di
%     struttura, quindi viene salvata - ma solo sulla COPIA
%     phantomx_sim_mpc. Il modello del collega non viene mai aperto.
%
% IL CRITERIO, DICHIARATO PRIMA (piano MPC, gradino S0)
%   Sull'ULTIMO SECONDO, cioe' a transitorio esaurito:
%     - deriva di quota      < 5 mm
%     - rollio e beccheggio  < 1 grado
%
% USO
%   clear all; bdclose all; startup_phantomx
%   prova_S0
%   prova_S0(struct('smorz',1.0))      % smorzamento piu' alto: se cambia il
%                                      % comportamento, la causa e' quella
%   prova_S0(struct('smorz',0))        % senza, per il confronto
%   prova_S0(struct('segno',-1))       % per rimisurare il segno opposto
%   prova_S0(struct('sorgente','analitico'))   % col vettore di tau_statico
%
% PRIMA DI LANCIARLO: tau_misurato
%   Le 18 coppie di default NON sono piu' quelle analitiche di tau_statico,
%   ma quelle MISURATE sul modello base (results/diagnostica/
%   tau_statico_misurato.csv). Motivo in tau_misurato: dopo tre S0 falliti
%   sullo stesso ingresso, l'ingresso va verificato prima di continuare a
%   cambiare il resto.
%
% Progetto FSR PhantomX - A. Russo

function info = prova_S0(opt)

if nargin < 1, opt = struct(); end
cfg = phantomx_config();

if ~isfield(opt,'segno'),   opt.segno   = +1;   end   % +1 = tau_statico cosi' com'e'
if ~isfield(opt,'t_rampa'), opt.t_rampa = 0.05; end   % [s] solo raccordo numerico
if ~isfield(opt,'durata'),  opt.durata  = 3.0;  end   % [s]
if ~isfield(opt,'smorz'),   opt.smorz   = cfg.tau_max / cfg.qd_max; end  % [N*m*s/rad]
if ~isfield(opt,'sorgente'),opt.sorgente= 'auto'; end  % 'auto'|'misurato'|'analitico'

ps_mdl = 'phantomx_sim_mpc';

%% ---- da dove vengono le 18 coppie ----
% [24/9] Di default si usa il vettore MISURATO sul modello base (tau_misurato),
% non quello analitico: il modello base tiene il robot fermo davvero, quindi
% le sue coppie sono l'unico riferimento verificato. L'analitico resta come
% confronto, non come ingresso.
ps_csv = fullfile('results','diagnostica','tau_statico_misurato.csv');
switch lower(opt.sorgente)
    case 'analitico'
        tau = tau_statico(false);  ps_da = 'tau_statico (analitico)';
    case 'misurato'
        if ~isfile(ps_csv)
            error('prova_S0:misurato', 'Manca %s: lancia prima tau_misurato.', ps_csv);
        end
        tau = readtable(ps_csv).misurato;  ps_da = ps_csv;
    otherwise
        if isfile(ps_csv)
            tau = readtable(ps_csv).misurato;  ps_da = ps_csv;
        else
            tau = tau_statico(false);  ps_da = 'tau_statico (analitico: manca il misurato)';
        end
end
tau = opt.segno * tau(:);
t   = (0 : 0.002 : opt.durata).';
k   = min(t / max(opt.t_rampa, 1e-9), 1);
tau_ts = timeseries(k * tau.', t);
assignin('base', 'tau_ts', tau_ts);

fprintf('\nS0: coppie da %s%s\n', ps_da, ternario(opt.segno > 0, '', '  [SEGNO INVERTITO]'));
fprintf('    raccordo %.3f s, smorzamento %.4f N*m*s/rad, durata %.1f s\n', ...
        opt.t_rampa, opt.smorz, opt.durata);

if ~bdIsLoaded(ps_mdl), load_system(ps_mdl); end

%% ---- smorzamento ai giunti, in memoria ----
ps_smorzamento(ps_mdl, opt.smorz);

%% ---- il blocco sorgente: Constant -> From Workspace, una volta sola ----
ps_goto = find_system(ps_mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                      'BlockType','Goto', 'GotoTag','ref_joints');
if numel(ps_goto) ~= 1
    error('prova_S0:refJoints', 'Goto ref_joints trovati: %d.', numel(ps_goto));
end
ps_padre = get_param(ps_goto{1}, 'Parent');
ps_fw    = [ps_padre '/tau_ts'];

if getSimulinkBlockHandle(ps_fw) < 0
    ps_cost = [ps_padre '/tau_S0'];
    ps_pos  = get_param(ps_goto{1}, 'Position');
    if getSimulinkBlockHandle(ps_cost) >= 0
        ps_pos = get_param(ps_cost, 'Position');
        ps_l   = get_param(get_param(ps_goto{1},'PortHandles').Inport(1), 'Line');
        if ps_l >= 0, delete_line(ps_l); end
        delete_block(ps_cost);
    end
    add_block('simulink/Sources/From Workspace', ps_fw, ...
              'VariableName', 'tau_ts', 'Position', ps_pos, ...
              'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value');
    add_line(ps_padre, 'tau_ts/1', [get_param(ps_goto{1},'Name') '/1'], 'autorouting','on');
    save_system(ps_mdl);
    fprintf('  sostituito il Constant con un From Workspace (una volta sola).\n');
end

%% ---- la run ----
applica_terreno('T1', false, ps_mdl);
applica_inerzie(ps_mdl);

ps_out = sim(ps_mdl, 'StopTime', num2str(opt.durata));
r = adatta_simscape(ps_out, struct('controller','C3', 'task','S0', 'run',1, ...
                                   'condizione','statico', 'vel_d',[0 0]));

%% ---- lettura, sull'ultimo secondo ----
finita = r.t(end) >= opt.durata - 1e-6;
sel = r.t >= r.t(end) - 1;
dz  = 1e3 * (r.p(end,3) - r.p(find(sel,1),3));
roll  = max(abs(rad2deg(r.rpy(sel,1))));
pitch = max(abs(rad2deg(r.rpy(sel,2))));
Fz1 = NaN;
if isfield(r,'Fc') && ~isempty(r.Fc)
    Fz1 = mean(sum(r.Fc(sel, 3:3:18), 2));
end

fprintf('\n  durata simulata   : %.3f s   (attese %.1f)\n', r.t(end), opt.durata);
fprintf('  Fz a regime       : %.2f N        (peso %.2f)\n', Fz1, cfg.mass*cfg.g);
fprintf('  quota corpo       : %.1f -> %.1f mm   (nominale %.1f)\n', ...
        1e3*r.p(1,3), 1e3*r.p(end,3), 1e3*cfg.body_z0);
fprintf('  deriva ultimo s   : %+.2f mm      (criterio: < 5)\n', dz);
fprintf('  rollio / beccheggio max ultimo s : %.2f / %.2f deg   (criterio: < 1)\n', roll, pitch);

ok = finita && abs(dz) < 5 && roll < 1 && pitch < 1;
info = struct('ok', ok, 'dz_mm', dz, 'roll', roll, 'pitch', pitch, ...
              'durata', r.t(end), 'Fz_regime', Fz1, 'segno', opt.segno, ...
              'smorz', opt.smorz, 'run', r);

if ok
    fprintf('\n  S0 PASSA. Jacobiano, segno e ordine dei giunti sono giusti: avanti con S1.\n\n');
    return
end

%% ---- se non passa: la traccia, per non tirare a indovinare ----
fprintf(2, '\n  S0 NON passa. Traccia del corpo (11 punti sulla run):\n');
fprintf('    %8s %10s %10s %10s\n', 't [s]', 'z [mm]', 'roll [deg]', 'pitch [deg]');
idx = unique(round(linspace(1, numel(r.t), 11)));
for i = idx
    fprintf('    %8.3f %10.2f %10.2f %10.2f\n', r.t(i), 1e3*r.p(i,3), ...
            rad2deg(r.rpy(i,1)), rad2deg(r.rpy(i,2)));
end

if ~finita
    fprintf(2, ['\n  Il solutore si e'' fermato prima della fine. Come si legge:\n' ...
                '    - z che SCENDE fino a fermarsi -> coppia insufficiente:\n' ...
                '      le zampe cedono e finiscono in singolarita''.\n' ...
                '    - z che SALE -> coppia in eccesso o segno sbagliato.\n' ...
                '    - z quasi ferma e assetto che oscilla sempre di piu'' ->\n' ...
                '      manca ancora dissipazione: rilancia con smorz = 1.0.\n\n']);
else
    fprintf(2, '\n  Il robot regge per tutta la run ma si muove piu'' del criterio.\n\n');
end
end

%% ================= helper =================
function ps_smorzamento(mdl, b)
%PS_SMORZAMENTO  Scrive lo smorzamento viscoso sui giunti rotoidali, in memoria.
%   Nessun save_system: come applica_inerzie, il .slx non viene toccato.
g = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                'MaskType','Revolute Joint');
if isempty(g)
    error('prova_S0:giunti', 'Nessun Revolute Joint in %s.', mdl);
end

% Il nome del parametro non si indovina: si cerca. Prima il nome ESATTO, poi
% la ricerca larga - stessa regola di cm_nome_par in crea_modello_mpc, che era
% nata proprio da questo (tre candidati, e il primo non era per forza giusto).
% [24/9] Qui i candidati larghi erano tre: DampingCoefficient e i due dei
% FINECORSA (LowerLimitDamping, UpperLimitDamping), che smorzano l'urto contro
% il limite di giunto e non hanno niente a che fare con l'attrito del riduttore.
n = fieldnames(get_param(g{1}, 'DialogParameters'));
p_b = '';
if any(strcmp(n, 'DampingCoefficient'))
    p_b = 'DampingCoefficient';
else
    cand = n(contains(n, 'Damping', 'IgnoreCase', true) & ...
             ~contains(n, 'Limit',  'IgnoreCase', true) & ...
             ~contains(n, 'Unit',   'IgnoreCase', true) & ~endsWith(n, '_conf'));
    if numel(cand) ~= 1
        fprintf(2, '  Parametri con "Damping": %s\n', ...
                strjoin(n(contains(n,'Damping','IgnoreCase',true)), ', '));
        error('prova_S0:smorzamento', ...
              'Parametro di smorzamento non identificato (%d candidati).', numel(cand));
    end
    p_b = cand{1};
end
u = n(strcmp(n, [p_b 'Units']));

for k = 1:numel(g)
    set_param(g{k}, p_b, num2str(b, '%.10g'));
    if numel(u) == 1
        try, set_param(g{k}, u{1}, 'N*m/(rad/s)'); end %#ok<TRYNC>
    end
end
fprintf('  smorzamento %s = %.4f su %d giunti (solo in memoria).\n', p_b, b, numel(g));
end

function s = ternario(c, a, b)
if c, s = a; else, s = b; end
end
