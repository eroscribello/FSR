function T = esegui_misure(task, opt)
%ESEGUI_MISURE  Esegue la matrice task x controllore x ripetizione e produce
%               la tabella delle metriche.
%
%   T = esegui_misure                        % C3 su T1, 5 ripetizioni
%   T = esegui_misure({'T1','T2'})           % T2 si espande in 5 velocita'
%   T = esegui_misure({'T1'}, opt)
%
% OPZIONI
%   .controllori   cell, default {'C3'}. 'C1' e 'C2' richiedono l'adattatore
%                  Simscape e il logging dei segnali nel modello.
%   .nRun          ripetizioni per cella, default 5
%   .durata        [s] durata di ogni run, default 10
%   .salva         true per scrivere la tabella su disco, default true
%   .cartella      dove salvare, default 'results'
%   .seed0         seme di partenza, default 20260904
%   .override      struct di campi da sovrascrivere in p dopo get_params.
%                  Serve per le prove diagnostiche e per la famiglia E:
%                     struct('Tmpc',0.04)          passo del QP dimezzato
%                     struct('mass',0.9*cfg.mass)  massa sottostimata del 10%
%                     struct('mu',0.4)             attrito ridotto
%                  I campi sovrascritti finiscono nell'etichetta della
%                  condizione, cosi' non si confondono con le run nominali.
%
% TASK DISPONIBILI
%   T1    piano, rettilineo, velocita' nominale
%   T2    piano, curva di velocita': si ESPANDE nelle celle di cfg.t2_fattori
%         (oggi 0.5x 1.0x 1.20x 1.5x 2.0x), una per fattore, tutte con
%         task = 'T2' e condizione = 'v0.50x' ... 'v2.00x'.
%         1.20x e' il LIMITE misurato di C1, e le ultime due celle sono
%         FUORI dall'inviluppo: vedi cfg.t2_confronto e cfg.t2_fuori
%   T3    imbardata costante
%   T7    disturbo impulsivo laterale   [richiede fcn_get_disturbance riscritta]
%
% LA DEFINIZIONE DI T2 STA IN cfg.t2_fattori, NON QUI
%   Prima era scritta due volte: qui tre fattori (T2A/T2B/T2C) variando v, in
%   script_T2 cinque variando T. Due griglie diverse per lo stesso task
%   producono due CSV che non si possono mettere nella stessa tabella. Ora la
%   griglia e' una sola e la variabile indipendente e' la velocita' comandata,
%   che e' l'unica grandezza comune ai tre controllori.
%
%   Le vecchie etichette T2A, T2B, T2C sono ancora accettate e mappate sui
%   fattori 0.5, 1.0, 2.0, per non rompere le chiamate salvate negli script.
%
%   T4, T5, T6 richiedono terreno non piano: non sono simulabili sul modello
%   a corpo rigido singolo, arriveranno con l'MPC in Simscape.
%
% RIPETIBILITA'
%   Ogni ripetizione ha il suo seme e parte da una posa leggermente diversa
%   (3 mm di posizione, mezzo grado di assetto). Cinque run che partono
%   identiche sono una run contata cinque volte: la deviazione standard che
%   ne uscirebbe sarebbe zero e non significherebbe niente.
%
%   Sull'SRB la dispersione resta comunque piccola, perche' il simulatore e'
%   deterministico e non c'e' modello di contatto che possa comportarsi in
%   modo diverso fra una run e l'altra. Non e' un difetto della misura: e'
%   una proprieta' di quell'impianto, e va detta nella relazione. Su Simscape
%   la dispersione sara' maggiore e piu' informativa.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(task), task = {'T1'}; end
if ischar(task) || isstring(task), task = cellstr(task); end
if nargin < 2, opt = struct(); end

def = struct('controllori',{{'C3'}}, 'nRun',5, 'durata',10, ...
             'salva',true, 'cartella','results', 'seed0',20260904, ...
             'override',struct());
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt,f{k}), opt.(f{k}) = def.(f{k}); end
end

% Il richiamo su body_z0 riguarda il baseline Simscape e verrebbe ripetuto a
% ogni get_params: in una campagna da decine di run e' solo rumore.
statoWarn = warning('off','phantomx:config:quota');
ripristina = onCleanup(@() warning(statoWarn));

cfg = phantomx_config();
T = table();

% --- espansione dei task: 'T2' diventa una cella per ogni fattore ---
% La griglia sta in cfg, non qui: vedi l'intestazione.
celle = {};      % ogni elemento: {nome, fattore}
for it = 1:numel(task)
    nome = task{it};
    if strcmpi(nome,'T2')
        for f = cfg.t2_fattori(:).'
            celle{end+1} = {'T2', f};                          %#ok<AGROW>
        end
    else
        celle{end+1} = {nome, 1};                              %#ok<AGROW>
    end
end

nTot = numel(opt.controllori) * numel(celle) * opt.nRun;
n = 0;

fprintf('\n=== MISURE ===\n');
fprintf('controllori: %s\n', strjoin(opt.controllori,', '));
fprintf('task       : %s\n', strjoin(task,', '));
if numel(celle) > numel(task)
    fprintf('             T2 espanso in %s\n', ...
            strjoin(arrayfun(@(f) sprintf('%.2fx',f), cfg.t2_fattori, ...
                             'UniformOutput',false), ' '));
end
fprintf('ripetizioni: %d   -> %d run in totale\n\n', opt.nRun, nTot);

cronometro = tic;

for ic = 1:numel(opt.controllori)
    ctrl = opt.controllori{ic};

    for it = 1:numel(celle)
        nome = celle{it}{1};
        fatt = celle{it}{2};
        d = definisci_task(nome, cfg, fatt);

        % l'override entra nell'etichetta: una run con Tmpc dimezzato non e'
        % una run nominale e non deve finire nella stessa cella
        if ~isempty(fieldnames(opt.override))
            fo = fieldnames(opt.override);
            et = cell(1,numel(fo));
            for q = 1:numel(fo)
                et{q} = sprintf('%s=%g', fo{q}, opt.override.(fo{q}));
            end
            d.condizione = strjoin(et, ',');
        end

        for ir = 1:opt.nRun
            n = n + 1;
            seed = opt.seed0 + 1000*it + ir;
            rng(seed);

            fprintf('[%2d/%2d] %s %s %s run %d (seed %d) ... ', ...
                    n, nTot, ctrl, nome, d.condizione, ir, seed);

            try
                sim_info = struct('interrotta',false, 'motivo',"");
                switch ctrl
                    case 'C3'
                        [r, sim_info] = una_run_c3(d, opt.durata, ir, ...
                                                   opt.nRun, opt.override);
                    case {'C1','C2'}
                        error('esegui_misure:nonPronto', ...
                            ['%s richiede l''adattatore Simscape e il logging\n' ...
                             'dei segnali nel modello: vedi docs/todo-modello.md'], ctrl);
                    otherwise
                        error('esegui_misure:controllore', ...
                              'Controllore sconosciuto: %s', ctrl);
                end

                r.meta.controller = ctrl;
                r.meta.task       = d.nome;   % non 'nome': le etichette vecchie
                                              % (T2A..T2C) si normalizzano in 'T2'
                r.meta.run        = ir;
                r.meta.seed       = seed;
                r.meta.condizione = d.condizione;
                r.meta.vel_d      = d.vel_d;
                r.meta.yaw_d      = d.yaw_d;

                riga = metriche(r, cfg);

                % Una run interrotta NON e' un successo, anche se nel tratto
                % percorso il robot sembrava andare bene: metriche.m non sa
                % che la simulazione si e' fermata prima del tempo.
                if sim_info.interrotta
                    riga.successo = false;
                    riga.causa_fallimento = string(sim_info.motivo);
                elseif riga.durata < 0.95 * opt.durata
                    riga.successo = false;
                    riga.causa_fallimento = sprintf("durata %.1f s su %.1f richiesti", ...
                                                    riga.durata, opt.durata);
                end

                T = [T; riga];                                   %#ok<AGROW>

                if riga.successo
                    fprintf('v=%.3f m/s  d=%.2f m  t=%.1f s  ok\n', ...
                            riga.vel_media, riga.distanza, riga.durata);
                else
                    fprintf(2,'v=%.3f m/s  d=%.2f m  t=%.1f s  FALLITA (%s)\n', ...
                            riga.vel_media, riga.distanza, riga.durata, ...
                            riga.causa_fallimento);
                end

            catch ME
                fprintf(2,'ERRORE: %s\n', ME.message);
                T = [T; riga_fallita(ctrl, d.nome, ir, seed, d, ME, cfg)];  %#ok<AGROW>
            end
        end
    end
end

fprintf('\nCompletate %d run in %.1f s.\n', height(T), toc(cronometro));

%% --- riepilogo per cella ---
if height(T) > 0
    riassumi(T);
end

%% --- salvataggio ---
if opt.salva && height(T) > 0
    if ~isfolder(opt.cartella), mkdir(opt.cartella); end
    stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
    base  = fullfile(opt.cartella, sprintf('misure_%s', stamp));
    writetable(T, [base '.csv']);
    save([base '.mat'], 'T', 'opt', 'cfg');
    fprintf('Salvato: %s.csv  e  .mat\n\n', base);
end

end

%% ====================================================================
function d = definisci_task(nome, cfg, fatt)
%DEFINISCI_TASK  Parametri del task. Un posto solo, cosi' i task sono
%                riproducibili e citabili nella relazione.
%
%   fatt  fattore di velocita' della cella, usato solo da T2. Default 1.
%         La griglia dei fattori NON sta qui: sta in cfg.t2_fattori, perche'
%         la legge anche script_T2 per l'impianto Simscape.

if nargin < 3 || isempty(fatt), fatt = 1; end

d = struct('nome',nome, 'v',cfg.v_nom, 'yaw_d',0, ...
           'disturbo',false, 'condizione','nominale');

switch upper(nome)
    case 'T1',  % nominale, tutto di default
    case 'T2'
        % La variabile indipendente e' la VELOCITA' COMANDATA: e' rispetto a
        % lei che sono definite le metriche della famiglia A, ed e' l'unica
        % grandezza comune ai tre controllori (C1/C2 accettano un periodo,
        % C3 una velocita'). L'etichetta ha lo stesso formato di script_T2.
        d.v          = fatt * cfg.v_nom;
        d.condizione = sprintf('v%.2fx', fatt);
    case {'T2A','T2B','T2C'}
        % vecchie etichette, tenute per non rompere le chiamate salvate
        vecchi = struct('T2A',0.5, 'T2B',1.0, 'T2C',2.0);
        f = vecchi.(upper(nome));
        d.nome       = 'T2';
        d.v          = f * cfg.v_nom;
        d.condizione = sprintf('v%.2fx', f);
        warning('esegui_misure:etichettaVecchia', ...
            ['%s e'' un''etichetta superata: usa ''T2'', che si espande su\n' ...
             'tutti i fattori di cfg.t2_fattori. Questa cella vale %.2fx.'], ...
            nome, f);
    case 'T3',  d.yaw_d = 0.1;                 % [rad/s]
    case 'T7',  d.disturbo = true;  d.condizione = 'disturbo-laterale';
    case {'T4','T5','T6'}
        error('esegui_misure:taskNonPiano', ...
            ['%s richiede terreno non piano: non e'' simulabile sul modello\n' ...
             'a corpo rigido singolo. Arrivera'' con l''MPC in Simscape.'], nome);
    otherwise
        error('esegui_misure:taskIgnoto', 'Task sconosciuto: %s', nome);
end

d.vel_d = [d.v, 0];
end

%% ====================================================================
function [r, info] = una_run_c3(d, durata, ir, nRun, override) %#ok<INUSD>
%UNA_RUN_C3  Una simulazione dell'MPC sul simulatore ridotto.

if nargin < 5, override = struct(); end

p = get_params(0);                       % 0 = tripode
p = imposta_velocita(p, d.v);
p.yaw_d = d.yaw_d;

% override dopo imposta_velocita, cosi' si puo' sovrascrivere anche la
% temporizzazione se serve
fo = fieldnames(override);
for q = 1:numel(fo)
    p.(fo{q}) = override.(fo{q});
end

% La fase resta 0: fcn_gen_XdUd costruisce lo stato iniziale con tutti i
% piedi a terra nelle posizioni nominali, quindi partire a fase diversa da 0
% metterebbe le zampe in volo a terra dove non dovrebbero stare. Le
% ripetizioni si differenziano perturbando la posa di partenza.
o = struct('waitbar',false, 'verbose',false, 'disturbo',d.disturbo, ...
           'fase', 0, ...
           'perturba_pos', 3e-3, ...     % 3 mm sulla posizione
           'perturba_rpy', deg2rad(0.5));% mezzo grado sull'assetto

[out, info] = mpc_loop(p, durata, o);

% evalc silenzia il riepilogo che adatta_mpc stampa: utile a mano, rumore
% in una campagna da decine di run
[~, r] = evalc('adatta_mpc(out, p)');
r.meta.impianto = 'SRB';
r.meta.note = sprintf('QP falliti: %d', info.qp_fallimenti);
if info.interrotta
    r.meta.note = sprintf('%s | INTERROTTA: %s', r.meta.note, info.motivo);
end
end

%% ====================================================================
function p = imposta_velocita(p, v)
%IMPOSTA_VELOCITA  Cambia la velocita' tenendo FISSA la lunghezza del passo.
%
% La velocita' di un'andatura e' S / T_stance: si puo' variare allungando il
% passo o accorciando il periodo. Qui si accorcia il periodo, perche'
% allungare il passo cambia anche quanto la gamba si estende e mescolerebbe
% due effetti nella stessa curva.

assert(v > 0, 'esegui_misure:velocita', 'La velocita'' deve essere positiva.');

p.v_nom = v;
p.Tst   = p.S / v;
p.T     = p.Tst / p.beta;
p.Tsw   = p.T - p.Tst;
p.vel_d = [v; 0];

p.stepLenRef = p.S;
p.stepClamp  = p.S;
end

%% ====================================================================
function riga = riga_fallita(ctrl, nome, ir, seed, d, ME, cfg)
%RIGA_FALLITA  Una run che va in errore produce comunque la sua riga: in una
%              campagna serve sapere QUALI celle sono fallite, non trovarsi
%              una tabella con dei buchi.
%
% PERCHE' LE COLONNE SI CHIEDONO A metriche
%   L'elenco era scritto a mano e si era disallineato: mancavano
%   'avanzamento', 'frazione_task' e 'disp_carico', aggiunte a metriche.m
%   dopo. Conseguenza: [T; riga_fallita(...)] fallisce la concatenazione, e la
%   campagna perde TUTTA la tabella alla prima run andata in errore - proprio
%   quando la riga di fallimento serve.
%
%   Adesso le colonne vengono da metriche.m stessa, chiamata su una run
%   minima sintetica. Qualunque metrica aggiunta in futuro compare qui senza
%   che nessuno debba ricordarsene. L'elenco a mano resta solo come ripiego,
%   se quella chiamata non riuscisse.

if nargin < 7 || isempty(cfg), cfg = phantomx_config(); end

riga = [];
try
    N  = 20;
    r0 = struct();
    r0.t   = (0:N-1).' * 0.01;
    r0.p   = zeros(N,3);
    r0.rpy = zeros(N,3);
    r0.Fc  = zeros(N,18);
    r0.meta = struct('controller',ctrl, 'task',nome, 'run',ir, 'seed',seed, ...
                     'condizione',d.condizione, 'note','');

    % evalc silenzia l'elenco dei campi assenti che metriche stampa
    [~, riga] = evalc('metriche(r0, cfg, struct(''t_regime'',0))');

    % tutto cio' che e' numerico non e' stato misurato: NaN
    for v = riga.Properties.VariableNames
        if isnumeric(riga.(v{1})), riga.(v{1}) = NaN; end
    end
catch
    riga = [];
end

if isempty(riga)
    % ripiego: elenco a mano, tenuto allineato a metriche.m
    riga = table(string(ctrl), string(nome), ir, seed, string(d.condizione), NaN, ...
        'VariableNames', {'controller','task','ripetizione','seed','condizione','durata'});

    vuoti = {'err_vx_rms','err_vy_rms','dev_lat_rms','dev_lat_max','yaw_err_fin', ...
             'distanza','distanza_regime','vel_media','perdita_avvio', ...
             'avanzamento','frazione_task', ...
             'roll_rms','pitch_rms','roll_max','pitch_max','z_rms','z_max','z_media', ...
             'tau_rms','tau_max','tau_rms_giunto_peggiore','frazione_saturo', ...
             'energia','cot','potenza_max', ...
             'slip_tot','slip_per_passo','distacchi','frazione_persa', ...
             'Fz_max_norm','appoggio_medio','disp_carico'};
    for k = 1:numel(vuoti), riga.(vuoti{k}) = NaN; end
    riga.note = "";
end

% i metadati sono noti anche quando la run fallisce
riga.controller       = string(ctrl);
riga.task             = string(nome);
riga.ripetizione      = ir;
riga.seed             = seed;
riga.condizione       = string(d.condizione);
riga.successo         = false;
riga.causa_fallimento = "errore di esecuzione";
riga.note             = string(ME.message);
end

%% ====================================================================
function riassumi(T)
%RIASSUMI  Media e deviazione standard per cella. E' la forma in cui i numeri
%          vanno nella relazione: un run singolo e' un aneddoto, cinque sono
%          una misura.

% Il raggruppamento include la CONDIZIONE, non solo controllore e task: le
% cinque velocita' di T2 hanno tutte task = 'T2' e collassavano in un'unica
% riga di riepilogo, che e' esattamente la media che non si vuole vedere.
fprintf('\n--- riepilogo per cella ---\n');
[gr, ctrl, tsk, cnd] = findgroups(T.controller, T.task, T.condizione);

fprintf('%-5s %-5s %-10s %5s %10s %10s %10s %10s\n', ...
        'ctrl','task','condizione','succ','v [m/s]','roll [mrad]','pitch [mrad]','dev [mm]');

for k = 1:max(gr)
    s = T(gr==k, :);
    ok = s.successo;
    fprintf('%-5s %-5s %-10s %2d/%-2d %6.4f±%.4f %5.2f±%.2f %5.2f±%.2f %5.1f±%.1f\n', ...
        ctrl(k), tsk(k), cnd(k), nnz(ok), height(s), ...
        mean(s.vel_media(ok),'omitnan'),  std(s.vel_media(ok),'omitnan'), ...
        1e3*mean(s.roll_rms(ok),'omitnan'),  1e3*std(s.roll_rms(ok),'omitnan'), ...
        1e3*mean(s.pitch_rms(ok),'omitnan'), 1e3*std(s.pitch_rms(ok),'omitnan'), ...
        1e3*mean(s.dev_lat_max(ok),'omitnan'), 1e3*std(s.dev_lat_max(ok),'omitnan'));
end
fprintf('\n');
end