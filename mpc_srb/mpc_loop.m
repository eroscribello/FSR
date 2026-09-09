function [out, info] = mpc_loop(p, durata, opt)
%MPC_LOOP  Ciclo di simulazione dell'MPC sul modello a corpo rigido singolo.
%
%   [out, info] = mpc_loop(p, durata)
%   [out, info] = mpc_loop(p, durata, opt)
%
% E' il ciclo che stava dentro MAIN.m, estratto in funzione. MAIN lo chiama
% per l'uso interattivo (e poi disegna), esegui_misure lo chiama in batch.
% Una sola copia del ciclo: se un giorno cambia, cambia per entrambi.
%
% INGRESSO
%   p        struttura di get_params
%   durata   [s] durata simulata
%   opt      opzioni, tutte facoltative:
%     .waitbar        barra di avanzamento (default true; in batch metti false)
%     .verbose        messaggi a schermo (default true)
%     .fase           [0,1) sfasamento iniziale del ciclo di andatura
%                     (default 0). ATTENZIONE: fcn_gen_XdUd costruisce lo
%                     stato iniziale con TUTTI i piedi nelle posizioni
%                     nominali di appoggio, a prescindere dalla fase. Con
%                     fase diversa da 0 le zampe che dovrebbero essere in
%                     volo partono a terra, e la simulazione parte da una
%                     configurazione incoerente. Lasciare 0 finche'
%                     fcn_gen_XdUd non sapra' generare lo stato alla fase
%                     richiesta.
%     .perturba_pos   [m] deviazione standard del rumore sulla posizione
%                     iniziale del corpo (default 0)
%     .perturba_rpy   [rad] deviazione standard del rumore sull'assetto
%                     iniziale (default 0)
%     .disturbo       true per attivare fcn_get_disturbance (default false)
%     .z_minima       [m] sotto questa quota la run si ferma: il robot e'
%                     caduto (default -0.05)
%
% USCITA
%   out    campi tout, Xout, Uout, Xdout, Udout, Uext, FSMout — gli stessi
%          nomi che MAIN lasciava nel workspace
%   info   diagnostica: fallimenti del QP, ExitFlag, iterazioni, tempo di
%          calcolo, motivo dell'eventuale arresto anticipato
%
% NOTA SULLA RIPETIBILITA'
%   La prima cosa che fa e' 'clear fcn_FSM'. Quella funzione usa variabili
%   persistent: senza azzerarle, la run n+1 riparte dallo stato lasciato
%   dalla n e due run identiche danno risultati diversi. Metterlo qui dentro
%   significa che non ci si puo' dimenticare di farlo.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 3, opt = struct(); end
def = struct('waitbar',true, 'verbose',true, 'fase',0, ...
             'perturba_pos',0, 'perturba_rpy',0, ...
             'disturbo',false, 'z_minima',-0.05);
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt,f{k}), opt.(f{k}) = def.(f{k}); end
end

clear fcn_FSM                      % <-- azzera lo stato persistent

nU     = 3 * p.nLeg;
dt_sim = p.simTimeStep;
N_IT   = floor(durata / dt_sim);

%% --- condizione iniziale ---
[Xt, Ut] = fcn_gen_XdUd(0, [], true(p.nLeg,1), p);

if opt.perturba_pos > 0
    Xt(1:3) = Xt(1:3) + opt.perturba_pos * randn(3,1);
end

if opt.perturba_rpy > 0
    dR = expm(hatMap(opt.perturba_rpy * randn(3,1)));
    R0 = reshape(Xt(7:15), [3,3]);
    Xt(7:15) = reshape(dR * R0, [9,1]);
end

if opt.fase ~= 0
    warning('mpc_loop:faseIncoerente', ...
        ['fase = %.2f ma lo stato iniziale ha tutti i piedi a terra nelle\n' ...
         'posizioni nominali: le zampe in volo partono da dove non dovrebbero.\n' ...
         'Vedi la nota su opt.fase nell''intestazione.'], opt.fase);
end

t_off = opt.fase * p.T;            % sfasamento del ciclo di andatura

%% --- logging ---
tstart = 0;
tend   = dt_sim;
[tout, Xout, Uout, Xdout, Udout, Uext, FSMout] = deal([]);

info = struct('qp_fallimenti',0, 'qp_exitflag',nan(N_IT,1), ...
              'qp_iter',nan(N_IT,1), 't_calcolo',0, ...
              'interrotta',false, 'motivo',"", 'iterazioni',0);

%% --- ciclo ---
if opt.waitbar, h = waitbar(0,'Calcolo...'); end
cronometro = tic;

for ii = 1:N_IT
    t_ = t_off + dt_sim*(ii-1) + p.Tmpc * (0:p.predHorizon-1);

    % --- macchina a stati dell'andatura ---
    [FSM, Xd, Ud, Xt] = fcn_FSM(t_, Xt, p);

    % --- QP ---
    [H, g, Aineq, bineq, Aeq, beq] = fcn_get_QP_form_eta(Xt, Ut, Xd, Ud, p);
    [sol, basic_info, ~] = qpSWIFT(sparse(H), g, sparse(Aeq), beq, ...
                                   sparse(Aineq), bineq);

    % --- diagnostica del solutore ---
    % Nel vecchio MAIN basic_info veniva catturato e mai letto: un QP che
    % fallisce restituisce forze qualunque e si vede solo un robot che si
    % comporta male, senza sapere perche'.
    if isstruct(basic_info)
        if isfield(basic_info,'ExitFlag'),   info.qp_exitflag(ii) = basic_info.ExitFlag; end
        if isfield(basic_info,'Iterations'), info.qp_iter(ii)     = basic_info.Iterations; end
    end
    ef = info.qp_exitflag(ii);
    if ~isnan(ef) && ef ~= 0
        info.qp_fallimenti = info.qp_fallimenti + 1;
        if info.qp_fallimenti == 1 && opt.verbose
            fprintf(2,'[mpc_loop] QP non risolto a t = %.3f s (ExitFlag %d).\n', ...
                    tstart, ef);
        end
    end

    Ut = Ut + sol(1:nU);

    % --- disturbo esterno ---
    [u_ext, p_ext] = fcn_get_disturbance(tstart, p);
    p.p_ext = p_ext;
    if ~opt.disturbo
        u_ext = 0*u_ext;           % comportamento storico: disturbo spento
    end

    % --- integrazione ---
    [t, X] = ode45(@(t,X)dynamics_SRB(t,X,Ut,Xd,u_ext,p), [tstart,tend], Xt);

    Xt     = X(end,:)';
    tstart = tend;
    tend   = tstart + dt_sim;

    % --- log ---
    lent   = numel(t) - 1;
    tout   = [tout;   t(2:end)];                        %#ok<AGROW>
    Xout   = [Xout;   X(2:end,:)];                      %#ok<AGROW>
    Uout   = [Uout;   repmat(Ut',      [lent,1])];      %#ok<AGROW>
    Xdout  = [Xdout;  repmat(Xd(:,1)', [lent,1])];      %#ok<AGROW>
    Udout  = [Udout;  repmat(Ud(:,1)', [lent,1])];      %#ok<AGROW>
    Uext   = [Uext;   repmat(u_ext',   [lent,1])];      %#ok<AGROW>
    FSMout = [FSMout; repmat(FSM',     [lent,1])];      %#ok<AGROW>

    info.iterazioni = ii;

    % --- arresto anticipato ---
    % In batch una run divergente non deve occupare la macchina per minuti.
    if ~all(isfinite(Xt))
        info.interrotta = true;
        info.motivo = sprintf("divergenza numerica a t = %.2f s", tstart);
        break
    end
    if Xt(3) < opt.z_minima
        info.interrotta = true;
        info.motivo = sprintf("caduta a t = %.2f s (z = %.3f m)", tstart, Xt(3));
        break
    end

    if opt.waitbar, waitbar(ii/N_IT, h, 'Calcolo...'); end
end

info.t_calcolo = toc(cronometro);
if opt.waitbar && ishandle(h), close(h); end

info.qp_exitflag = info.qp_exitflag(1:info.iterazioni);
info.qp_iter     = info.qp_iter(1:info.iterazioni);

%% --- uscita ---
out = struct('tout',tout, 'Xout',Xout, 'Uout',Uout, 'Xdout',Xdout, ...
             'Udout',Udout, 'Uext',Uext, 'FSMout',FSMout);

% Un'interruzione NON e' chiacchiera da modalita' verbosa: e' un fallimento,
% e va detta sempre. Sopprimerla in batch e' come non averla vista.
if info.interrotta
    fprintf(2, '[mpc_loop] INTERROTTA dopo %d/%d iterazioni: %s\n', ...
            info.iterazioni, N_IT, info.motivo);
end

if opt.verbose
    fprintf('[mpc_loop] %d iterazioni in %.1f s', info.iterazioni, info.t_calcolo);
    if info.qp_fallimenti > 0
        fprintf(2, '  |  %d QP falliti', info.qp_fallimenti);
    end
    fprintf('\n');
end

end