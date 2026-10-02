function run = adatta_simscape(out, meta, opt)
%ADATTA_SIMSCAPE  Converte una run di phantomx_sim_zero nella struttura run.
%
%   run = adatta_simscape                  % legge 'out' dal base workspace
%   run = adatta_simscape(out)
%   run = adatta_simscape(out, meta)
%   run = adatta_simscape(out, meta, opt)
%
% USO TIPICO
%   set_param('phantomx_sim_zero','SimscapeLogType','all');
%   out = sim('phantomx_sim_zero','StopTime','10');
%   run = adatta_simscape(out, struct('controller','C1','task','T1'));
%   riga = metriche(run)
%
% DA DOVE VIENE OGNI CAMPO
%   t, p, v        log di Simscape, primitive Px/Py/Pz del giunto 6-DOF
%   rpy, w         log di Simscape, primitiva sferica (Q) o Rx/Ry/Rz
%   q, qd          log di Simscape, giunti j_c1_* j_thigh_* j_tibia_*
%   tau            To Workspace  torque_sens  (o torque_estim, vedi opt.tau)
%   Fc, contact    To Workspace  Fleg
%   pf             cinematica diretta dell'URDF (robotModel) + posa del corpo
%   contact_sched  ricostruito dal ciclo di andatura e verificato sul contatto
%
% TRE COSE CHE QUESTO FILE FA E CHE NON SONO OVVIE
%
%   1. RICAMPIONA SU GRIGLIA UNIFORME. Il solutore e' a passo variabile e
%      infittisce i campioni attorno agli impatti. Tutte le metriche che sono
%      medie su campioni (zampe a terra, frazione di saturazione, frazione
%      persa) risulterebbero sbilanciate verso gli istanti di urto. E' lo
%      stesso errore gia' corretto in analizza_forze.
%
%   2. ORDINA LE ZAMPE. Il modello lavora in ordine Mux (RR MR FR RL ML FL),
%      run_vuoto impone l'ordine CAN (FL FR ML MR RL RR). I vettori a 18
%      componenti che arrivano dai To Workspace vengono permutati qui, una
%      volta sola. q e qd no: arrivano dal log per NOME di giunto, quindi
%      l'ordine e' corretto per costruzione e non passa da nessuna tabella.
%
%   3. SI RIFIUTA DI PRODURRE UNA RIGA SE L'IK NON E' COERENTE. Se
%      l'escursione verticale del piede lungo il passo supera la
%      penetrazione disponibile, i piedi non possono toccare insieme e ogni
%      numero della famiglia D descrive un artefatto. Con opt.ignora_ik si
%      forza, ma la nota finisce in run.meta.note.
%
% OPZIONI
%   .fs         [Hz]  frequenza della griglia uniforme          (200)
%   .soglia_F   [N]   forza sopra cui la zampa e' in appoggio   (0.5)
%   .pf         calcola le posizioni dei piedi                  (true)
%   .tau        'sens' | 'estim' | 'nessuno'                    ('sens')
%   .ordine     ordine dei 18-vettori dei To Workspace:
%               'mux' -> vengono permutati   'can' -> gia' a posto  ('mux')
%   .ignora_ik  produce la run anche se verifica_ik non passa    (false)
%   .verbose    stampa il riepilogo                             (true)
%
% Progetto FSR PhantomX - A. Russo

%% ==================== argomenti ====================
if nargin < 1 || isempty(out)
    if evalin('base','exist(''out'',''var'')')
        out = evalin('base','out');
    else
        error('adatta_simscape:manca', ...
          ['Non c''e'' nessuna simulazione da convertire.\n' ...
           'Lancia prima, con il log di Simscape attivo:\n' ...
           '   set_param(''phantomx_sim_zero'',''SimscapeLogType'',''all'');\n' ...
           '   out = sim(''phantomx_sim_zero'',''StopTime'',''10'');']);
    end
end
if nargin < 2 || isempty(meta), meta = struct(); end
if nargin < 3, opt = struct(); end

def = struct('fs',200, 'soglia_F',0.5, 'pf',true, 'tau','sens', ...
             'ordine','mux', 'ignora_ik',false, 'verbose',true);
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt,f{k}), opt.(f{k}) = def.(f{k}); end
end

cfg   = phantomx_config();
manca = {};          % elenco di cosa non si e' potuto riempire
note  = {};          % avvertenze che finiscono in run.meta.note

%% ==================== guardia sulla coerenza dell'IK ====================
% Non e' una formalita': tutta la famiglia D presuppone che i sei piedi
% possano stare a terra insieme. Se l'IK non lo consente, la riga di tabella
% misura l'errore di cinematica, non il controllore.
[ik_ok, ik_esc, ik_pen] = guardia_ik(cfg);
if ~isnan(ik_esc)
    if ~ik_ok
        msg = sprintf(['la guardia IK non passa: escursione %.2f mm contro %.2f mm\n' ...
                       'di penetrazione disponibile. I sei piedi non possono\n' ...
                       'toccare insieme, quindi le metriche di contatto\n' ...
                       'descriverebbero un difetto di cinematica.\n' ...
                       'Se sai quello che fai:  adatta_simscape(out, meta, struct(''ignora_ik'',true))'], ...
                       1000*ik_esc, 1000*ik_pen);
        if opt.ignora_ik
            note{end+1} = sprintf('IK incoerente: escursione %.2f mm > penetrazione %.2f mm', ...
                                  1000*ik_esc, 1000*ik_pen);
            fprintf(2,'\n[adatta_simscape] %s\n\n', note{end});
        else
            error('adatta_simscape:ik', '%s', msg);
        end
    end
else
    note{end+1} = 'guardia IK non eseguita: robotModel assente';
end

%% ==================== log di Simscape ====================
nomeLog = 'simlog';
try, nomeLog = get_param('phantomx_sim_zero','SimscapeLogName'); catch, end

simlog = [];
if isprop(out,nomeLog) || (isstruct(out) && isfield(out,nomeLog))
    simlog = out.(nomeLog);
end
if isempty(simlog)
    error('adatta_simscape:log', ...
      ['Il log di Simscape (%s) non e'' nell''uscita della simulazione.\n' ...
       'Senza di esso mancano posizione e assetto del corpo, che sono\n' ...
       'campi OBBLIGATORI per metriche.m. Attiva il log e rilancia:\n' ...
       '   set_param(''phantomx_sim_zero'',''SimscapeLogType'',''all'')'], nomeLog);
end

L = raccogli(simlog, '', struct('nome',{},'percorso',{},'nodo',{}));
if opt.verbose
    fprintf('\n[adatta_simscape] log di Simscape: %d nodi\n', numel(L));
end

%% ==================== posa del corpo ====================
[t_px, x_b] = primitiva(L, 'Px', 'p', 'm');
[~,    y_b] = primitiva(L, 'Py', 'p', 'm');
[~,    z_b] = primitiva(L, 'Pz', 'p', 'm');

if isempty(x_b) || isempty(y_b) || isempty(z_b)
    error('adatta_simscape:corpo', ...
      ['Non trovo le primitive Px/Py/Pz del giunto del corpo nel log.\n' ...
       'Guarda l''albero con   simlog.print   e dimmi come si chiamano:\n' ...
       'senza posizione del corpo metriche.m non parte.']);
end

%% ---------- il log copre tutta la simulazione? ----------
% Simscape logga con un buffer circolare: con SimscapeLogLimitData = 'on'
% conserva solo gli ULTIMI SimscapeLogDataHistory punti. Su una run lunga il
% log parte a meta' strada e l'inizio del task sparisce, in silenzio. E' il
% motivo per cui verifica_marcia riportava "regime 5.0 s" su una run da 10.
t_vero = [];
for nm = {'Fleg','Fsum','torque_sens','torque_estim','tout'}
    [tv, ~] = daWorkspace(out, nm{1});
    if ~isempty(tv), t_vero = tv;  break; end
end
if ~isempty(t_vero)
    perso = t_px(1) - t_vero(1);
    if perso > 0.05*(t_vero(end) - t_vero(1))
        msg = sprintf([ ...
            'Il log di Simscape copre %.2f-%.2f s, la simulazione %.2f-%.2f s:\n' ...
            'mancano i primi %.2f s. Il buffer del log e'' limitato e tiene\n' ...
            'solo gli ultimi punti, quindi l''avvio del task non c''e''.\n' ...
            'Conseguenza: distanza, perdita_avvio e tutta la famiglia A\n' ...
            'misurerebbero il regime spacciandolo per il task intero.\n\n' ...
            'Rimedio:\n' ...
            '   set_param(''phantomx_sim_zero'',''SimscapeLogLimitData'',''off'');\n' ...
            '   save_system(''phantomx_sim_zero'');\n\n' ...
            'Se vuoi la riga lo stesso: opt.ignora_log = true'], ...
            t_px(1), t_px(end), t_vero(1), t_vero(end), perso);
        if isfield(opt,'ignora_log') && opt.ignora_log
            note{end+1} = sprintf('log troncato: mancano i primi %.2f s', perso);
            fprintf(2,'\n[adatta_simscape] %s\n\n', note{end});
        else
            error('adatta_simscape:logTroncato', '%s', msg);
        end
    end
end

% griglia uniforme: e' la base temporale di tutto il resto
t0 = t_px(1);  t1 = t_px(end);
N  = max(10, round((t1 - t0)*opt.fs) + 1);
t  = linspace(t0, t1, N).';

p = [interp1(t_px, x_b, t, 'linear','extrap'), ...
     interp1(t_px, y_b, t, 'linear','extrap'), ...
     interp1(t_px, z_b, t, 'linear','extrap')];

% velocita': dalle primitive se ci sono, altrimenti derivata della posizione
[t_vx, vx] = primitiva(L, 'Px', 'v', 'm/s');
[t_vy, vy] = primitiva(L, 'Py', 'v', 'm/s');
[t_vz, vz] = primitiva(L, 'Pz', 'v', 'm/s');
if ~isempty(vx) && ~isempty(vy) && ~isempty(vz)
    v = [interp1(t_vx, vx, t, 'linear','extrap'), ...
         interp1(t_vy, vy, t, 'linear','extrap'), ...
         interp1(t_vz, vz, t, 'linear','extrap')];
else
    v = [gradient(p(:,1), t), gradient(p(:,2), t), gradient(p(:,3), t)];
    note{end+1} = 'v derivata numericamente da p';
end

%% ==================== assetto ====================
[rpy, w, fonte_att] = assetto(L, t);
if isempty(rpy)
    error('adatta_simscape:assetto', ...
      ['Non trovo l''assetto del corpo nel log: ne'' una primitiva sferica\n' ...
       'con quaternione Q, ne'' le tre rotazionali Rx/Ry/Rz.\n' ...
       'rpy e'' un campo OBBLIGATORIO. Guarda  simlog.print  e dimmi\n' ...
       'che primitive ha il giunto 6-DOF.']);
end
if isempty(w), manca{end+1} = 'w'; end

%% ==================== giunti ====================
% Presi per NOME dall'URDF: niente tabelle di permutazione, niente ordine
% Mux. Se un nome non c'e', la colonna resta NaN e si vede.
[q, qd, nq, diagG] = giunti(simlog, t, cfg);
if nq == 0
    q = [];  qd = [];  manca{end+1} = 'q, qd';
elseif nq < 18
    note{end+1} = sprintf('solo %d giunti su 18 trovati nel log', nq);
end
if nq < 18 && ~isempty(diagG)
    fprintf(2,'[adatta_simscape] giunti non letti:\n');
    fprintf(2,'    %s\n', diagG{:});
end

% Guardia sulle unita': se gli angoli superano i limiti meccanici del giunto
% non e' il robot ad essere rotto, sono i numeri ad essere in gradi. E' il
% controllo che avrebbe smascherato subito i 215 m di scivolamento.
if ~isempty(q)
    lim  = max(abs([cfg.q_min cfg.q_max]));
    qmax = max(abs(q(:)));
    if qmax > 1.05*lim
        error('adatta_simscape:unitaGiunti', ...
          ['Gli angoli di giunto arrivano a %.2f, oltre il limite meccanico\n' ...
           'di %.2f rad. Non e'' il robot: sono gradi letti come radianti.\n' ...
           '%.2f / %.2f = %.1f, cioe'' 180/pi.\n' ...
           'Il log va letto con l''unita'' esplicita: q.series.values(''rad'').'], ...
           qmax, lim, qmax, qmax*pi/180, 180/pi);
    end
end

%% ==================== To Workspace ====================
perm = permutazione(cfg, opt.ordine);      % da ordine del modello a ordine CAN

% --- coppie ---
tau = [];
switch lower(opt.tau)
    case 'sens',  nomeTau = 'torque_sens';
    case 'estim', nomeTau = 'torque_estim';
    otherwise,    nomeTau = '';
end
if ~isempty(nomeTau)
    [tt, Y] = daWorkspace(out, nomeTau);
    if isempty(Y)
        manca{end+1} = sprintf('tau (%s assente)', nomeTau);
    elseif size(Y,2) ~= 18
        manca{end+1} = sprintf('tau (%s ha %d colonne, ne servono 18)', nomeTau, size(Y,2));
    else
        tau = interp1(tt, Y(:,perm), t, 'linear','extrap');
        note{end+1} = sprintf('tau da %s', nomeTau);
    end
else
    manca{end+1} = 'tau (escluso da opt.tau)';
end

%% ==================== posizione dei piedi ====================
% Calcolata PRIMA delle forze, perche' quando Fleg non e' disponibile le
% forze si ricostruiscono da qui.
pf = [];
if opt.pf
    if ~evalin('base','exist(''robotModel'',''var'')')
        manca{end+1} = 'pf (robotModel assente: lancia init_gait)';
    elseif isempty(q)
        manca{end+1} = 'pf (servono gli angoli di giunto)';
    else
        if opt.verbose
            fprintf('  calcolo pf con la cinematica diretta: %d campioni x 6 zampe...\n', numel(t));
        end
        pf = piedi(evalin('base','robotModel'), q, p, rpy, cfg);
        note{end+1} = ['pf dalla FK dell''URDF: l''origine puo'' essere sfalsata di qualche mm ' ...
                       'rispetto al frame Simscape, lo scivolamento (moto relativo) non ne risente'];
    end
else
    manca{end+1} = 'pf (disattivato da opt.pf)';
end

% --- forze di contatto ---
Fc = [];  inContatto = [];
[tf, F] = daWorkspace(out, 'Fleg');
if isempty(F) || numel(tf) < 3
    % ------------------------------------------------------------------
    % RICOSTRUZIONE DALLA PENETRAZIONE
    %
    % Nel modello le forze di contatto NON sono disponibili, in nessuna
    % delle tre forme in cui le abbiamo cercate:
    %   - il ramo Fleg/Fsum e' un abbozzo mai finito: i dodici From cercano
    %     le etichette Force_sens_lf..Force_sens_rr e nel modello non
    %     esiste alcun Goto che le produca. Un From senza Goto e' risolto
    %     come costante, da cui l'unico campione che Fleg restituiva;
    %   - il log di Simscape non contiene i blocchi di contatto: 107 nodi,
    %     tutti giunti piu' il 6-DOF Joint;
    %   - LogSimulationData sui blocchi di contatto non si puo' accendere,
    %     Simulink risponde "does not support logging".
    %
    % Si ricostruisce allora la forza normale dalla PENETRAZIONE, con la
    % stessa legge costitutiva che usa il solutore - il blocco dichiara
    % NormalForceType = SmoothSpringDamper, NormalStiffness = contact_k,
    % NormalDamping = contact_c, NormalTransitionRegionWidth = contact_w:
    %
    %     delta = max(0, z_terreno - z_piede)
    %     Fz    = (k*delta + c*d(delta)/dt) * rampa(delta/w)
    %
    % La quota del terreno NON viene assunta da cfg: viene RICAVATA
    % imponendo che la somma delle sei forze valga in media il peso del
    % robot. Cosi' un eventuale sfasamento costante della catena
    % cinematica - che la nota su pf dichiara possibile, qualche mm - viene
    % assorbito invece di propagarsi su tutte le forze.
    %
    % COSA QUESTA RICOSTRUZIONE DA' E COSA NO
    %   da':   Fz per zampa, quindi appoggio_medio, disp_carico,
    %          Fz_max_norm, e il contatto per slip_tot
    %   non da': le componenti tangenziali. Fc(:,1:3:18) e Fc(:,2:3:18)
    %          restano zero, quindi nessuna metrica di attrito.
    %   assume: equilibrio quasi statico per la taratura della quota. In
    %          una run con fase di volo prolungata la somma non e' il peso
    %          e la stima della quota peggiora: il diagnostico sotto lo
    %          segnala confrontando la forza totale media con il peso.
    % ------------------------------------------------------------------
    if isempty(pf)
        manca{end+1} = 'Fc, contact (ne'' Fleg ne'' pf: servono gli angoli di giunto e robotModel)';
    else
        [Fz, delta, z_terr, dFz] = forzeDaPenetrazione(pf, t, cfg);
        Fc = zeros(numel(t), 18);
        Fc(:, 3:3:18) = Fz;
        inContatto = Fz > opt.soglia_F;
        note{end+1} = sprintf( ...
            ['Fc RICOSTRUITA dalla penetrazione (k=%g, c=%g, w=%g), quota del ' ...
             'terreno stimata a z=%.5f m dall''equilibrio dei pesi: solo la ' ...
             'componente normale, nessun attrito'], ...
            cfg.contact.k, cfg.contact.c, cfg.contact.w, z_terr);
        if opt.verbose
            fprintf('  forze ricostruite: quota terreno %.5f m, penetrazione media %.2f mm\n', ...
                    z_terr, 1e3*mean(delta(delta>0)));
            fprintf('    forza totale media %.3f N contro un peso di %.3f N (scarto %.1f%%)\n', ...
                    dFz.F_media, dFz.peso, 100*(dFz.F_media/dFz.peso - 1));
            fprintf('    piedi in appoggio in media %.2f\n', mean(sum(inContatto,2)));
        end
        if abs(dFz.F_media/dFz.peso - 1) > 0.05
            note{end+1} = sprintf( ...
                ['la forza totale ricostruita si scosta del %.0f%% dal peso: la run ' ...
                 'ha fasi di volo o la quota del terreno non e'' costante, Fz e'' ' ...
                 'quantitativamente inaffidabile (il contatto resta valido)'], ...
                100*abs(dFz.F_media/dFz.peso - 1));
        end
    end
elseif size(F,2) == 18
    Fc = interp1(tf, F(:,perm), t, 'linear','extrap');
    inContatto = Fc(:,3:3:18) > opt.soglia_F;
elseif size(F,2) == 6
    % Fleg logga il MODULO della forza, non le tre componenti. Su terreno
    % piatto la normale e' quasi tutta la forza, quindi il modulo e' una
    % stima per ECCESSO di Fz: buona per il contatto (soglia a 0.5 N),
    % ottimistica su Fz_max_norm. Va detto, non nascosto.
    if strcmpi(opt.ordine,'can'), col = 1:6; else, col = slotUscita(cfg).'; end
    M  = interp1(tf, F(:,col), t, 'linear','extrap');
    Fc = zeros(numel(t),18);
    Fc(:,3:3:18) = M;
    inContatto = M > opt.soglia_F;
    note{end+1} = 'Fleg e'' il modulo: Fc ha solo la componente z, Fz_max_norm e'' un limite superiore';
else
    manca{end+1} = sprintf('Fc, contact (Fleg ha %d colonne, attese 6 o 18)', size(F,2));
end

%% ==================== contatto schedulato ====================
sched = [];
if ~isempty(inContatto)
    % L'andatura effettiva puo' differire da cfg: la campagna T2 cambia il
    % periodo con OVERRIDE_GAIT e init_gait lascia il valore vero in gait.
    % Usare cfg.T qui produceva uno schedule sfasato e quindi distacchi e
    % frazione_persa senza significato in tutte le celle diverse da 1x.
    andatura = struct('T', cfg.T, 'beta_stance', cfg.beta_stance, 'phase', cfg.phase);
    if evalin('base','exist(''gait'',''var'')')
        g = evalin('base','gait');
        if isfield(g,'T') && ~isempty(g.T),    andatura.T = g.T; end
        if isfield(g,'duty') && ~isempty(g.duty), andatura.beta_stance = 1 - g.duty; end
    end
    if abs(andatura.T - cfg.T) > 1e-9
        note{end+1} = sprintf('andatura a T = %.3f s invece di cfg.T = %.3f s', ...
                              andatura.T, cfg.T);
    end
    [sched, accordo, sfas] = schedula(t, andatura, inContatto);
    if opt.verbose
        fprintf('  schedule di contatto: accordo %.0f%% (sfasamento %.2f del ciclo)\n', ...
                100*accordo, sfas);
    end
    if accordo < 0.7
        note{end+1} = sprintf('contact_sched concorda col contatto reale solo al %.0f%%', 100*accordo);
        fprintf(2,['[adatta_simscape] %s.\n' ...
                   '    Lo sfasamento e'' gia'' ottimizzato, quindi non e'' un disallineamento\n' ...
                   '    di calcolo: il robot non sta seguendo lo schedule comandato.\n' ...
                   '    distacchi e frazione_persa vanno letti come sintomo, non come misura.\n'], ...
                   note{end});
    end
end

%% ==================== struttura ====================
run = run_vuoto(0);
run.t   = t;
run.p   = p;
run.v   = v;
run.rpy = rpy;
if ~isempty(w),          run.w   = w;   end
if ~isempty(q),          run.q   = q;   run.qd = qd; end
if ~isempty(tau),        run.tau = tau; end
if ~isempty(pf),         run.pf  = pf;  end
if ~isempty(Fc),         run.Fc  = Fc;  end
if ~isempty(inContatto), run.contact = inContatto; end
if ~isempty(sched),      run.contact_sched = sched; end

% L'etichetta del controllore si LEGGE dallo stato del modello, non si
% assume. Era cablata a 'C1': una run fatta senza impostare OVERRIDE_C2
% girava in C2 - cfg.c2.attiva ha default true - e finiva in tabella
% marcata C1. Un'intera campagna puo' essere attribuita al controllore
% sbagliato senza che nulla lo segnali.
run.meta.controller = nomeControllore();
run.meta.terreno    = terrenoAttivo();
run.meta.task       = 'T1';
run.meta.run        = 1;
run.meta.condizione = 'nominale';
run.meta.impianto   = 'Simscape';
run.meta.vel_d      = [cfg.v_nom 0];
run.meta.yaw_d      = 0;

f = fieldnames(meta);
for k = 1:numel(f), run.meta.(f{k}) = meta.(f{k}); end
if ~isfield(meta,'note')
    run.meta.note = strjoin(note, '; ');
end

%% ==================== riepilogo ====================
if opt.verbose
    fprintf('\n--- RUN CONVERTITA ---\n');
    fprintf('  %d campioni uniformi a %g Hz, da %.2f a %.2f s\n', numel(t), opt.fs, t(1), t(end));
    fprintf('  assetto da            %s\n', fonte_att);
    fprintf('  giunti trovati        %d su 18\n', nq);
    if isfield(cfg,'legNamesOUT')
        fprintf('  ordine dei To Workspace: %s -> CAN\n', strjoin(cfg.legNamesOUT,' '));
    else
        fprintf('  ordine dei To Workspace: %s -> CAN\n', upper(opt.ordine));
    end
    fprintf('  distanza percorsa     %.4f m\n', norm(p(end,1:2)-p(1,1:2)));
    fprintf('  quota media           %.4f m   escursione %.2f mm\n', ...
            mean(p(:,3)), 1000*(max(p(:,3))-min(p(:,3))));
    if ~isempty(inContatto)
        fprintf('  zampe a terra medie   %.2f su 6\n', mean(sum(inContatto,2)));
        fprintf('  istanti con 3 a terra %.0f%%\n', 100*mean(sum(inContatto,2)==3));
    end
    if ~isnan(ik_esc)
        fprintf('  escursione IK         %.2f mm  (penetrazione %.2f mm)\n', ...
                1000*ik_esc, 1000*ik_pen);
    end
    if isempty(manca)
        fprintf('  campi assenti:        nessuno\n');
    else
        fprintf(2,'  campi assenti:        %s\n', strjoin(manca, ', '));
    end
    fprintf('\n  riga = metriche(run)\n\n');
end

end

%% ================================================================
%  LOG DI SIMSCAPE
%% ================================================================
function L = raccogli(nodo, percorso, L)
%RACCOGLI  Appiattisce l'albero del log in un elenco nome/percorso/nodo.
try, ids = nodo.childIds; catch, return; end
for k = 1:numel(ids)
    id = ids{k};
    try, c = nodo.(id); catch, continue; end
    p = [percorso '.' id];
    L(end+1) = struct('nome',id, 'percorso',p, 'nodo',c);   %#ok<AGROW>
    L = raccogli(c, p, L);
end
end

function [t, y] = primitiva(L, prim, var, unita)
%PRIMITIVA  Serie della variabile 'var' dentro la primitiva 'prim'.
%   Se ce n'e' piu' di una si tiene quella con l'escursione maggiore: fra i
%   giunti che hanno una Pz, quello del corpo e' l'unico che si muove molto.
t = [];  y = [];
for k = 1:numel(L)
    if ~strcmp(L(k).nome, prim), continue; end
    try
        [yk, tk] = serieSI(L(k).nodo.(var).series, unita);
    catch
        continue
    end
    if isempty(yk) || size(yk,2) > 1, continue; end
    if isempty(y) || (max(yk)-min(yk)) > (max(y)-min(y)), y = yk; t = tk; end
end
end

function [v, t] = serieSI(s, unita)
%SERIESI  Valori di una serie del log NELL'UNITA' RICHIESTA.
%
%   Due trappole, entrambe gia' costate un giro di debug:
%
%   1. values e time sono METODI, non proprieta'. Scrivere s.values(:) non
%      rimodella: passa ':' come unita' e solleva "Invalid unit". Prima si
%      assegna, poi si rimodella.
%
%   2. values senza argomento restituisce l'unita' di VISUALIZZAZIONE della
%      variabile, non quella SI. Gli angoli dei giunti escono in GRADI: letti
%      come radianti davano energia 3463 J, CoT 159 e 215 m di scivolamento
%      su 1.4 m percorsi. Numeri sbagliati di un fattore 57.3, tutti
%      plausibili a prima vista.
%
%   Chiedere l'unita' esplicitamente non e' solo una conversione: e' un
%   controllo. Se la variabile non fosse un angolo, 'rad' solleva un errore
%   invece di restituire un numero credibile e falso. E l'unita' di default
%   e' una proprieta' del blocco, che chiunque apra il modello puo' cambiare.
v = [];  t = [];
try
    v = s.values(unita);
catch ME
    if nargin > 1 && ~isempty(unita)
        warning('adatta_simscape:unita', ...
            ['Il log non accetta l''unita'' ''%s'' per questa variabile (%s).\n' ...
             'Uso l''unita'' di default: se non e'' SI i numeri saranno\n' ...
             'sbagliati di un fattore costante, e plausibili.'], unita, ME.message);
    end
    v = s.values;
end
v = v(:);
t = s.time;  t = t(:);
end

function [rpy, w, fonte] = assetto(L, t)
%ASSETTO  Rollio/beccheggio/imbardata del corpo, da quaternione o da Rx/Ry/Rz.
rpy = [];  w = [];  fonte = 'non trovato';

% --- ipotesi 1: primitiva sferica con quaternione ---
for k = 1:numel(L)
    if ~strcmp(L(k).nome,'S'), continue; end
    try
        s  = L(k).nodo.Q.series;
        Q  = s.values;  tq = s.time;
    catch
        continue
    end
    if size(Q,2) ~= 4, continue; end
    Qi = interp1(tq, Q, t, 'linear','extrap');
    Qi = Qi ./ vecnorm(Qi, 2, 2);
    rpy = quat2rpy(Qi);
    fonte = 'primitiva sferica (quaternione)';
    % la velocita' angolare e' a 3 componenti: niente serieSI (che
    % rimodella a colonna), ma l'unita' va chiesta lo stesso.
    try
        sw = L(k).nodo.w.series;
        try, wv = sw.values('rad/s'); catch, wv = sw.values; end
        w  = interp1(sw.time, wv, t, 'linear','extrap');
    catch
    end
    return
end

% --- ipotesi 2: tre primitive rotazionali ---
a = zeros(numel(t),3);  trovate = 0;
for j = 1:3
    nomi = {'Rx','Ry','Rz'};
    [tk, qk] = primitiva(L, nomi{j}, 'q', 'rad');
    if isempty(qk), continue; end
    a(:,j) = interp1(tk, qk, t, 'linear','extrap');
    trovate = trovate + 1;
end
if trovate == 3
    rpy = a;
    fonte = 'primitive Rx/Ry/Rz (assunte rollio-beccheggio-imbardata)';
    w = [gradient(a(:,1),t), gradient(a(:,2),t), gradient(a(:,3),t)];
end
end

function rpy = quat2rpy(Q)
%QUAT2RPY  Convenzione ZYX. Q = [w x y z], come Simscape.
qw = Q(:,1);  qx = Q(:,2);  qy = Q(:,3);  qz = Q(:,4);
R11 = 1 - 2*(qy.^2 + qz.^2);
R21 = 2*(qx.*qy + qw.*qz);
R31 = 2*(qx.*qz - qw.*qy);
R32 = 2*(qy.*qz + qw.*qx);
R33 = 1 - 2*(qx.^2 + qy.^2);
rpy = [atan2(R32, R33), atan2(-R31, hypot(R32,R33)), atan2(R21, R11)];
end

function [q, qd, n, diag] = giunti(simlog, t, cfg)
%GIUNTI  Angoli e velocita' dei 18 giunti, presi per nome dall'URDF.
%   L'ordine delle colonne e' quello CAN, tre per zampa: coxa, femore, tibia.
%   Non passa dalla lista appiattita dei percorsi: scende nell'albero per
%   nome. La versione precedente cercava i percorsi e restituiva zero giunti
%   senza dire perche'; qui ogni fallimento finisce in diag.
suff = struct('FL','lf','FR','rf','ML','lm','MR','rm','RL','lr','RR','rr');
q  = nan(numel(t), 18);
qd = nan(numel(t), 18);
n  = 0;  diag = {};
for i = 1:6
    u = suff.(cfg.legNamesCAN{i});
    g = {['j_c1_' u], ['j_thigh_' u], ['j_tibia_' u]};
    for j = 1:3
        [tk, qk, tw, wk, msg] = leggiGiunto(simlog, g{j});
        if isempty(qk)
            diag{end+1} = sprintf('%-16s %s', g{j}, msg);  %#ok<AGROW>
            continue
        end
        c = 3*(i-1) + j;
        q(:,c) = interp1(tk, qk, t, 'linear','extrap');
        if ~isempty(wk)
            qd(:,c) = interp1(tw, wk, t, 'linear','extrap');
        else
            qd(:,c) = gradient(q(:,c), t);
        end
        n = n + 1;
    end
end
end

function [tq, qv, tw, wv, msg] = leggiGiunto(simlog, blocco)
%LEGGIGIUNTO  q e w di un giunto, provando le due strutture possibili:
%   blocco.Rz.q   (primitiva rotazionale, il caso di smimport)
%   blocco.q      (variabile appesa direttamente al blocco)
tq = [];  qv = [];  tw = [];  wv = [];  msg = '';
try, ids = simlog.childIds; catch, msg = 'log senza figli'; return; end
if ~any(strcmp(ids, blocco))
    msg = 'blocco assente al primo livello del log';  return
end
b = simlog.(blocco);

cand = {};
try
    fi = b.childIds;
    for k = 1:numel(fi)
        if any(strcmp(fi{k}, {'Rz','Rx','Ry','R'})), cand{end+1} = b.(fi{k}); end %#ok<AGROW>
    end
catch
end
cand{end+1} = b;

for k = 1:numel(cand)
    try
        s  = cand{k}.q.series;
        [qv, tq] = serieSI(s, 'rad');     % unita' esplicita: vedi serieSI
    catch
        continue
    end
    try
        sw = cand{k}.w.series;
        [wv, tw] = serieSI(sw, 'rad/s');
    catch
    end
    return
end
msg = 'blocco trovato ma nessuna variabile q leggibile';
end

%% ================================================================
%  TO WORKSPACE
%% ================================================================
function terr = terrenoAttivo()
%TERRENOATTIVO  Il terreno con cui ha girato il modello, dichiarato da
%               applica_terreno nel base workspace.
%
%   Se manca, la run e' stata fatta su un terreno IGNOTO: probabilmente
%   quello rimasto da una chiamata precedente. E' gia' successo - una
%   campagna T2 partita su T5, con il gradino - e nei risultati non si
%   vedeva. Qui si dichiara l'ignoranza invece di lasciarla implicita.
terr = '?';
try
    if evalin('base','exist(''TERRENO_ATTIVO'',''var'')')
        v = evalin('base','TERRENO_ATTIVO');
        if ~isempty(v), terr = char(v); return; end
    end
catch
end
warning('adatta_simscape:terreno', ...
    ['Terreno non dichiarato: TERRENO_ATTIVO non c''e'' nel base workspace.\n' ...
     'La run ha girato sul terreno lasciato dall''ultima chiamata, che\n' ...
     'potrebbe non essere quello del task. Lancia applica_terreno prima di\n' ...
     'simulare. La colonna terreno restera'' ''?''.']);
end

function nome = nomeControllore()
%NOMECONTROLLORE  C1 o C2, letto dallo stato con cui ha girato il modello.
%
%   La sorgente piu' attendibile e' c2_par, il vettore che init_gait
%   assembla e che i due blocchi MATLAB Function leggono davvero:
%   c2_par(1) = 0 significa ricerca del terreno disattivata, cioe' anello
%   aperto. Se non c'e', si ripiega su cfg.c2.attiva. Se non c'e' nemmeno
%   quello, si dichiara l'incertezza invece di inventare un'etichetta.
nome = 'C?';
try
    if evalin('base','exist(''c2_par'',''var'')')
        v = evalin('base','c2_par');
        if ~isempty(v)
            if v(1) == 0, nome = 'C1'; else, nome = 'C2'; end
            return
        end
    end
catch
end
try
    c = phantomx_config();
    if isfield(c,'c2') && isfield(c.c2,'attiva')
        if c.c2.attiva, nome = 'C2'; else, nome = 'C1'; end
    end
catch
end
if strcmp(nome,'C?')
    warning('adatta_simscape:controllore', ...
        ['Non riesco a stabilire se la run e'' C1 o C2: manca c2_par nel base\n' ...
         'workspace e cfg.c2 non e'' leggibile. L''etichetta resta ''C?'':\n' ...
         'non metterla in tabella senza risolverla.']);
end
end

function [Fz, delta, z_terr, diag] = forzeDaPenetrazione(pf, t, cfg)
%FORZEDAPENETRAZIONE  Forza normale per zampa dalla quota dei piedi.
%
%   La legge e' quella dichiarata dai blocchi Spatial Contact Force del
%   modello: molla-smorzatore con regione di transizione smussata.
%
%   La quota del terreno non e' un dato in ingresso: viene ricavata
%   imponendo che la somma delle sei forze valga IN MEDIA il peso. E' un
%   vincolo fisico, non una taratura, e assorbe lo sfasamento costante fra
%   il frame della cinematica diretta e quello di Simscape.

z = pf(:, 3:3:18);                 % [N x 6] quota dei sei piedi
k = cfg.contact.k;
c = cfg.contact.c;
w = max(cfg.contact.w, eps);
peso = cfg.mass * cfg.g;

% --- quota del terreno dall'equilibrio ---
% somma(zt) e' monotona crescente in zt: nulla quando il terreno sta sotto
% il piede piu' basso, massima quando sta sopra il piu' alto. Quindi la
% radice esiste ed e' unica.
somma = @(zt) mean(sum(max(0, k*(zt - z)), 2)) - peso;
lo = min(z(:));
hi = max(z(:)) + peso/k;           % margine: garantisce somma(hi) > 0
try
    z_terr = fzero(somma, [lo hi]);
catch
    % ripiego: la quota che rende la penetrazione media pari a quella
    % statica nominale, peso/(3k) in tripode
    z_terr = median(min(z, [], 2)) + peso/(3*k);
end

% --- penetrazione e forza ---
delta = max(0, z_terr - z);

ddelta = zeros(size(delta));
for i = 1:size(delta,2)
    ddelta(:,i) = gradient(delta(:,i), t);
end

% rampa della regione di transizione: la forza non parte a gradino sui
% primi w metri di penetrazione, come nel blocco
s    = min(1, delta / w);
ramp = s.^2 .* (3 - 2*s);

Fz = max(0, (k*delta + c*ddelta) .* ramp);

diag = struct('peso', peso, ...
              'F_media', mean(sum(Fz,2)), ...
              'pen_media', mean(delta(delta>0)), ...
              'z_terr', z_terr);
end

function [t, Y] = daWorkspace(out, nome)
%DAWORKSPACE  Legge un To Workspace qualunque sia il formato di salvataggio.
t = [];  Y = [];
d = [];
if isprop(out,nome) || (isstruct(out) && isfield(out,nome))
    d = out.(nome);
elseif evalin('base', sprintf('exist(''%s'',''var'')', nome))
    d = evalin('base', nome);
end
if isempty(d), return; end

if isa(d,'timeseries')
    t = d.Time(:);  Y = squeeze(d.Data);
elseif isstruct(d) && isfield(d,'time') && isfield(d,'signals')
    t = d.time(:);  Y = d.signals.values;
elseif isnumeric(d)
    Y = d;
    if isprop(out,'tout') || (isstruct(out) && isfield(out,'tout'))
        t = out.tout(:);
    else
        t = (0:size(Y,1)-1).';
    end
end
if ~isempty(Y) && size(Y,1) ~= numel(t) && size(Y,2) == numel(t), Y = Y.'; end

% ---- lunghezze incoerenti: un To Workspace in formato Array ----
% Un To Workspace salvato come 'Array' non porta con se' il tempo, e se ha un
% suo SampleTime o una Decimation la sua lunghezza NON e' quella di tout.
% Prima questo caso arrivava fino a interp1, che si fermava con
%   "X and V must be of the same length"
% senza dire quale segnale fosse. Qui si ricostruisce una griglia uniforme
% sull'intervallo della simulazione - che e' esattamente cio' che un
% To Workspace a passo fisso produce - e si avvisa, perche' se il blocco
% avesse invece una Decimation non uniforme la ricostruzione sarebbe
% sbagliata e va messo in formato Timeseries.
if ~isempty(Y) && size(Y,1) ~= numel(t)
    n = size(Y,1);
    if numel(t) >= 2 && n >= 2
        warning('adatta_simscape:lunghezzaSegnale', ...
            ['%s ha %d campioni, il tempo della simulazione ne ha %d.\n' ...
             'E'' un To Workspace in formato Array con un passo proprio: il\n' ...
             'tempo viene ricostruito uniforme su [%.4f, %.4f].\n' ...
             'Per avere il tempo vero, metti il blocco in formato Timeseries\n' ...
             '(lo fa abilita_log).'], nome, n, numel(t), t(1), t(end));
        t = linspace(t(1), t(end), n).';
    else
        warning('adatta_simscape:lunghezzaSegnale', ...
            '%s ha %d campioni e non e'' associabile a un tempo: lo scarto.', nome, n);
        t = [];  Y = [];
    end
end
end

function perm = permutazione(cfg, ordine)
%PERMUTAZIONE  Indici che portano un 18-vettore del modello in ordine CAN.
%
%   I To Workspace stanno a valle dei Mux di USCITA, che nel modello sono
%   cablati in ordine  lf lm lr rf rm rr  (FL ML RL FR MR RR) - DIVERSO da
%   quello del Mux di ingresso (RR MR FR RL ML FL). Qui serve il primo.
if strcmpi(ordine,'can'), perm = 1:18; return; end
k    = slotUscita(cfg);
perm = reshape((3*(k-1) + [1 2 3]).', 1, []);
end

function k = slotUscita(cfg)
%SLOTUSCITA  Per ogni zampa CAN, la posizione che occupa nei Mux di uscita.
if isfield(cfg,'can2out')
    k = cfg.can2out(:);
else
    warning('adatta_simscape:ordineUscita', ...
      ['cfg.can2out non c''e'': uso can2mux, che descrive il Mux di INGRESSO.\n' ...
       'Nel modello i due Mux hanno ordini diversi (ingresso RR MR FR RL ML FL,\n' ...
       'uscita FL ML RL FR MR RR), quindi le grandezze per zampa finirebbero\n' ...
       'sulla zampa sbagliata. Aggiorna phantomx_config.']);
    k = cfg.can2mux(:);
end
end

%% ================================================================
%  ANDATURA E CINEMATICA
%% ================================================================
function [sched, accordo, sfas] = schedula(t, and, reale)
%SCHEDULA  Appoggio previsto dal ciclo di andatura, allineato sul reale.
%
%   'and' porta T, beta_stance e phase EFFETTIVI della run, non quelli di
%   cfg: con OVERRIDE_GAIT i due possono differire.
%
%   L'istante in cui comincia il ciclo rispetto a t = 0 non e' documentato,
%   e la versione precedente provava solo due ipotesi (appoggio per primo,
%   volo per primo). Non basta: lo sfasamento reale puo' essere qualunque.
%   Qui si cerca sulla griglia lo sfasamento che massimizza l'accordo.
%
%   ATTENZIONE: si cerca SOLO lo sfasamento, che e' un'incognita di
%   allineamento. T e beta_stance restano quelli NOMINALI, perche'
%   contact_sched deve dire cosa il controllore ha COMANDATO. Adattando
%   anche il duty, frazione_persa verrebbe azzerata per costruzione e la
%   metrica non misurerebbe piu' nulla.
%
%   Se anche con il miglior allineamento l'accordo resta basso, allora non
%   e' un problema di calcolo: e' il robot che non sta seguendo il comando.
ns    = 200;
prove = linspace(0, 1, ns+1);  prove(end) = [];
best  = -inf;  sched = [];  sfas = 0;
for s = prove
    u  = mod(t/and.T - and.phase(:).' - s, 1);   % fase di ogni zampa, N x 6
    ip = u < and.beta_stance;                    % appoggio nella prima parte
    a  = mean(ip(:) == reale(:));
    if a > best, best = a;  sched = ip;  sfas = s; end
end
accordo = best;
end

function pf = piedi(rbt, q, p, rpy, cfg)
%PIEDI  Posizione dei sei piedi nel frame mondo, ordine CAN.
suff  = struct('FL','lf','FR','rf','ML','lm','MR','rm','RL','lr','RR','rr');
conf0 = homeConfiguration(rbt);
strutt = isstruct(conf0);
if strutt
    nomi = {conf0.JointName};
else
    nomi = {};
    for b = 1:numel(rbt.Bodies)
        if ~strcmp(rbt.Bodies{b}.Joint.Type,'fixed')
            nomi{end+1} = rbt.Bodies{b}.Joint.Name; %#ok<AGROW>
        end
    end
end
off  = [cfg.foot_xyz(1); -cfg.foot_xyz(2); cfg.foot_xyz(3); 1];   % frame URDF
base = rbt.BaseName;
N    = size(q,1);
pf   = nan(N,18);

for i = 1:6
    u  = suff.(cfg.legNamesCAN{i});
    g  = {['j_c1_' u], ['j_thigh_' u], ['j_tibia_' u]};
    ix = cellfun(@(x) find(strcmp(nomi,x),1), g, 'UniformOutput',false);
    if any(cellfun(@isempty, ix)), continue; end
    ix = cell2mat(ix);
    for n = 1:N
        c = conf0;
        for j = 1:3
            if strutt, c(ix(j)).JointPosition = q(n,3*(i-1)+j);
            else,      c(ix(j))               = q(n,3*(i-1)+j);
            end
        end
        vb = getTransform(rbt, c, ['tibia_' u], base) * off;
        R  = rotZYX(rpy(n,:));
        pf(n, 3*(i-1)+(1:3)) = (p(n,:).' + R*vb(1:3)).';
    end
end
end

function R = rotZYX(a)
cr = cos(a(1)); sr = sin(a(1));
cp = cos(a(2)); sp = sin(a(2));
cy = cos(a(3)); sy = sin(a(3));
R = [ cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr ; ...
      sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr ; ...
        -sp,            cp*sr,            cp*cr ];
end

function [ok, esc, pen] = guardia_ik(cfg)
%GUARDIA_IK  Escursione verticale del piede lungo il passo, come verifica_ik.
ok = false;  esc = NaN;  pen = cfg.mass*cfg.g/(3*cfg.contact.k);
if ~evalin('base','exist(''robotModel'',''var'')'), return; end
rbt  = evalin('base','robotModel');
xs   = linspace(-cfg.S/2, +cfg.S/2, 7);
off  = [cfg.foot_xyz(1); -cfg.foot_xyz(2); cfg.foot_xyz(3); 1];
prove = {1,'lf'; 3,'lm'};
esc = 0;
for pr = 1:size(prove,1)
    i = prove{pr,1};  u = prove{pr,2};
    z = nan(size(xs));
    for m = 1:numel(xs)
        [th,ph,ps] = inv_kyn(xs(m), 0, cfg.z0, cfg.side(i), cfg.alpha(i));
        v = fkGamba(rbt, u, [th ph ps], off);
        if isempty(v), esc = NaN; return; end
        z(m) = cfg.p_hip(3,i) - v(3);
    end
    esc = max(esc, max(z)-min(z));
end
ok = esc < 0.3*pen;
end

function v = fkGamba(rbt, u, q, off)
v = [];
g = {['j_c1_' u], ['j_thigh_' u], ['j_tibia_' u]};
try
    c = homeConfiguration(rbt);
    if isstruct(c)
        for j = 1:3
            k = find(strcmp({c.JointName}, g{j}),1);
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
            k = find(strcmp(nomi, g{j}),1);
            if isempty(k), return; end
            c(k) = q(j);
        end
    end
    T = getTransform(rbt, c, ['tibia_' u], rbt.BaseName) * off;
    v = T(1:3);
catch
end
end