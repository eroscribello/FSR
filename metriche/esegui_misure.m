function T = esegui_misure(task, opt)
%ESEGUI_MISURE  Esegue la matrice task x controllore x ripetizione e produce
%               la tabella delle metriche.
%
%   T = esegui_misure                        % C3 su T1, 5 ripetizioni
%   T = esegui_misure({'T1','T2a','T2c'})
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
%   T2a   piano, 0.5x la nominale
%   T2b   piano, 1.0x  (identico a T1, tenuto per completezza della curva)
%   T2c   piano, 2.0x
%   T3    imbardata costante
%   T7    disturbo impulsivo laterale   [richiede fcn_get_disturbance riscritta]
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
nTot = numel(opt.controllori) * numel(task) * opt.nRun;
n = 0;

fprintf('\n=== MISURE ===\n');
fprintf('controllori: %s\n', strjoin(opt.controllori,', '));
fprintf('task       : %s\n', strjoin(task,', '));
fprintf('ripetizioni: %d   -> %d run in totale\n\n', opt.nRun, nTot);

cronometro = tic;

for ic = 1:numel(opt.controllori)
    ctrl = opt.controllori{ic};

    for it = 1:numel(task)
        nome = task{it};
        d = definisci_task(nome, cfg);

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

            fprintf('[%2d/%2d] %s %s run %d (seed %d) ... ', ...
                    n, nTot, ctrl, nome, ir, seed);

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
                r.meta.task       = nome;
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
                T = [T; riga_fallita(ctrl, nome, ir, seed, d, ME)];  %#ok<AGROW>
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
function d = definisci_task(nome, cfg)
%DEFINISCI_TASK  Parametri del task. Un posto solo, cosi' i task sono
%                riproducibili e citabili nella relazione.

d = struct('nome',nome, 'v',cfg.v_nom, 'yaw_d',0, ...
           'disturbo',false, 'condizione','nominale');

switch upper(nome)
    case 'T1',  % nominale, tutto di default
    case 'T2A', d.v = 0.5 * cfg.v_nom;
    case 'T2B', d.v = 1.0 * cfg.v_nom;
    case 'T2C', d.v = 2.0 * cfg.v_nom;
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
function riga = riga_fallita(ctrl, nome, ir, seed, d, ME)
%RIGA_FALLITA  Una run che va in errore produce comunque la sua riga: in una
%              campagna serve sapere QUALI celle sono fallite, non trovarsi
%              una tabella con dei buchi.

riga = table(string(ctrl), string(nome), ir, seed, string(d.condizione), NaN, ...
    'VariableNames', {'controller','task','ripetizione','seed','condizione','durata'});

vuoti = {'err_vx_rms','err_vy_rms','dev_lat_rms','dev_lat_max','yaw_err_fin', ...
         'distanza','distanza_regime','perdita_avvio','vel_media','roll_rms','pitch_rms','roll_max','pitch_max', ...
         'z_rms','z_max','z_media','tau_rms','tau_max','tau_rms_giunto_peggiore', ...
         'frazione_saturo','energia','cot','potenza_max','slip_tot', ...
         'slip_per_passo','distacchi','frazione_persa','Fz_max_norm','appoggio_medio'};
for k = 1:numel(vuoti), riga.(vuoti{k}) = NaN; end

riga.successo         = false;
riga.causa_fallimento = "errore di esecuzione";
riga.note             = string(ME.message);
end

%% ====================================================================
function riassumi(T)
%RIASSUMI  Media e deviazione standard per cella. E' la forma in cui i numeri
%          vanno nella relazione: un run singolo e' un aneddoto, cinque sono
%          una misura.

fprintf('\n--- riepilogo per cella ---\n');
[gr, ctrl, tsk] = findgroups(T.controller, T.task);

fprintf('%-5s %-5s %5s %10s %10s %10s %10s\n', ...
        'ctrl','task','succ','v [m/s]','roll [mrad]','pitch [mrad]','dev [mm]');

for k = 1:max(gr)
    s = T(gr==k, :);
    ok = s.successo;
    fprintf('%-5s %-5s %2d/%-2d %6.4f±%.4f %5.2f±%.2f %5.2f±%.2f %5.1f±%.1f\n', ...
        ctrl(k), tsk(k), nnz(ok), height(s), ...
        mean(s.vel_media(ok),'omitnan'),  std(s.vel_media(ok),'omitnan'), ...
        1e3*mean(s.roll_rms(ok),'omitnan'),  1e3*std(s.roll_rms(ok),'omitnan'), ...
        1e3*mean(s.pitch_rms(ok),'omitnan'), 1e3*std(s.pitch_rms(ok),'omitnan'), ...
        1e3*mean(s.dev_lat_max(ok),'omitnan'), 1e3*std(s.dev_lat_max(ok),'omitnan'));
end
fprintf('\n');
end