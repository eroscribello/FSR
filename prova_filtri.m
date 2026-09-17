function P = prova_filtri(opt)
%PROVA_FILTRI  I filtri sui comandi di giunto spiegano il fallimento a 2x?
%
%   P = prova_filtri
%   P = prova_filtri(struct('tau',[0.05 0.01], 'fattori',2.0))
%
% LA DOMANDA
%   A 2x il robot cammina INDIETRO: frazione_task da -26% a -69% in tutte le
%   celle della griglia di taratura, con contatto eccellente (3.46 piedi a
%   terra, dispersione del carico 1.3%). Non e' perdita di appoggio e nessun
%   valore di z0 o H la recupera.
%
%   Resta un sospetto: sys_filter, diciotto filtri del primo ordine con
%   tau = 0.05 s sui comandi di giunto, nell'InitFcn del modello. A 2x lo
%   swing dura 250 ms, quindi il filtro ne ritarda un quinto: il piede tocca
%   in ritardo e in una posizione diversa da quella comandata, e la corsa di
%   appoggio puo' risultare invertita.
%
% PERCHE' LA RISPOSTA CAMBIA LA RELAZIONE
%   Se abbassando tau la cella 2x torna a camminare, il fallimento e' del
%   BANCO DI PROVA, non del controllore cinematico. Riportarlo come limite di
%   C1 attribuirebbe all'MPC un vantaggio che non ha - ed e' esattamente il
%   tipo di rilievo che arriva in discussione.
%
%   Se invece nessun tau la recupera, il limite e' del controllore e il
%   risultato regge.
%
% LA CELLA DI CONTROLLO
%   Si prova anche 1.0x, che oggi funziona. Serve a escludere che abbassare
%   tau rompa quello che va: un filtro piu' rapido passa piu' rumore ai
%   giunti, e se anche 1x peggiora la conclusione non e' "tau troppo alto" ma
%   "il banco e' sensibile a tau", che e' un'altra cosa.
%
% PRIMA
%   Serve la modifica all'InitFcn del modello: tau deve leggere tau_filtro
%   dal base workspace se esiste. Questa funzione VERIFICA che sia stata
%   fatta, e si ferma se non lo e' - senza quella verifica si sweeperebbe una
%   variabile che nessuno legge, concludendo il contrario del vero.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1, opt = struct(); end
cfg = phantomx_config();

def = struct('tau',      [0.05 0.02 0.01 0.005 0.002], ...
             'fattori',  [2.0 1.0], ...
             'H',        cfg.H, ...
             'nCicli',   10, ...
             'mdl',      'phantomx_sim_zero');
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt,f{k}), opt.(f{k}) = def.(f{k}); end
end

fprintf('\n============== PROVA DEI FILTRI ==============\n');
fprintf('  tau      : %s s\n', mat2str(opt.tau,4));
fprintf('  velocita'': %s\n', mat2str(opt.fattori,3));
fprintf('  H        : %.3f m (invariato)\n', opt.H);
fprintf('  -> %d run\n', numel(opt.tau)*numel(opt.fattori));

applica_terreno('T2', false, opt.mdl);

%% ---- verifica che l'InitFcn legga tau_filtro ----
% Senza questa verifica lo sweep sarebbe cieco: tau resterebbe 0.05 in ogni
% cella e la conclusione ("i filtri non c'entrano") sarebbe falsa.
fprintf('\n--- verifica del collegamento ---\n');
tau_prova = 0.007;
assignin('base','tau_filtro', tau_prova);
assignin('base','OVERRIDE_C2', false);
evalin('base','clear OVERRIDE_GAIT');
evalin('base','init_gait');
evalin('base', sprintf('evalc(''sim(''''%s'''',''''StopTime'''',''''0.2'''')'')', opt.mdl));

tau_letto = NaN;
try
    if evalin('base','exist(''tau'',''var'')')
        v = evalin('base','tau');
        tau_letto = v(1);
    end
catch
end

fprintf('  tau_filtro impostato a %.4f, l''InitFcn ha messo tau = %.4f\n', ...
        tau_prova, tau_letto);
if isnan(tau_letto) || abs(tau_letto - tau_prova) > 1e-9
    evalin('base','clear tau_filtro OVERRIDE_C2');
    evalin('base','init_gait');
    error('prova_filtri:nonCollegato', ...
      ['L''InitFcn NON legge tau_filtro: dopo la simulazione tau vale %.4f\n' ...
       'invece di %.4f. Lo sweep sarebbe cieco - ogni cella girerebbe con\n' ...
       'tau = 0.05 e la conclusione sarebbe l''opposto del vero.\n\n' ...
       'Nelle proprieta'' del modello -> Callbacks -> InitFcn, la riga che\n' ...
       'assegna tau deve diventare:\n' ...
       '    if evalin(''base'',''exist(''''tau_filtro'''',''''var'''')'')\n' ...
       '        tau = evalin(''base'',''tau_filtro'');\n' ...
       '    else\n' ...
       '        tau = 0.05;\n' ...
       '    end\n' ...
       'e deve stare PRIMA della riga che costruisce sys_filter.'], ...
       tau_letto, tau_prova);
end
fprintf('  collegamento OK\n\n');

%% ---- lo sweep ----
P = table();
n = 0;
for iF = 1:numel(opt.fattori)
    fatt  = opt.fattori(iF);
    v_cmd = fatt * cfg.v_nom;
    T_i   = cfg.S / (cfg.beta_stance * v_cmd);
    T_sw  = (1 - cfg.beta_stance) * T_i;
    stop  = opt.nCicli * T_i;

    fprintf('--- %.2fx :  T = %.3f s,  swing = %.0f ms ---\n', fatt, T_i, 1e3*T_sw);

    for iT = 1:numel(opt.tau)
        n = n + 1;
        tau_i = opt.tau(iT);
        fprintf('  [%2d] tau = %.4f s (%.0f%% dello swing) ... ', ...
                n, tau_i, 100*tau_i/T_sw);
        try
            assignin('base','tau_filtro', tau_i);
            assignin('base','OVERRIDE_C2', false);
            assignin('base','OVERRIDE_GAIT', struct('T',T_i, 'H',opt.H));
            evalin('base','init_gait');

            out = sim(opt.mdl, 'StopTime', num2str(stop));
            r = adatta_simscape(out, struct('task','T2', 'run',1, ...
                    'condizione', sprintf('v%.2fx tau%.4f', fatt, tau_i), ...
                    'vel_d', [v_cmd 0]), struct('verbose',false));

            cfgLoc = cfg;  cfgLoc.T = T_i;
            riga = metriche(r, cfgLoc, struct('t_regime', 2*T_i));

            riga.fattore   = fatt;
            riga.tau       = tau_i;
            riga.T_swing   = T_sw;
            riga.tau_su_sw = tau_i / T_sw;
            P = [P; riga];                                            %#ok<AGROW>

            fprintf('task %+6.1f%%  piedi %.2f  slip %5.1f mm  %s\n', ...
                100*riga.frazione_task, riga.appoggio_medio, ...
                1e3*riga.slip_tot, ternario(riga.successo,'ok','FALLITA'));
        catch ME
            fprintf(2,'ERRORE: %s\n', ME.message);
        end
    end
    fprintf('\n');
end

%% ---- ripristino ----
evalin('base','clear tau_filtro OVERRIDE_GAIT OVERRIDE_C2');
evalin('base','init_gait');

if height(P) == 0, return; end
disp(P(:, {'condizione','fattore','tau','tau_su_sw','frazione_task', ...
           'appoggio_medio','slip_tot','cot','successo'}))

%% ---- lettura ----
fprintf('\n=============== COME SI LEGGE ===============\n');
for iF = 1:numel(opt.fattori)
    fatt = opt.fattori(iF);
    s = P(P.fattore == fatt, :);
    if isempty(s), continue; end
    [~, ix] = max(s.frazione_task);
    fprintf('\n%.2fx: frazione del task da %+.0f%% (tau %.4f) a %+.0f%% (tau %.4f)\n', ...
            fatt, 100*min(s.frazione_task), s.tau(s.frazione_task == min(s.frazione_task)), ...
            100*max(s.frazione_task), s.tau(ix));

    if fatt >= 1.9
        if max(s.frazione_task) > 0.5 && min(s.frazione_task) < 0
            fprintf(['  RECUPERATA. Il fallimento a 2x era dei FILTRI, non del\n' ...
                     '  controllore. Va scritto in relazione: riportarlo come\n' ...
                     '  limite di C1 darebbe all''MPC un vantaggio che non ha.\n' ...
                     '  tau va portato a %.4f s per tutta la campagna, e la\n' ...
                     '  scelta dichiarata insieme al suo effetto su 1x.\n'], s.tau(ix));
        elseif max(s.frazione_task) < 0
            fprintf(['  NON recuperata da nessun tau: il robot cammina indietro\n' ...
                     '  comunque. Il limite e'' del controllore e il risultato\n' ...
                     '  regge. I filtri escono dalla lista dei sospetti.\n']);
        else
            fprintf(['  Migliora ma non basta: tau e'' UNA delle cause, non\n' ...
                     '  l''unica. Va riportato che la cella 2x e'' al limite del\n' ...
                     '  banco, e il confronto con C3 a quella velocita'' va\n' ...
                     '  dichiarato debole.\n']);
        end
    else
        peggio = min(s.frazione_task) < 0.9 * s.frazione_task(s.tau == max(s.tau));
        if peggio
            fprintf(['  ATTENZIONE: abbassare tau peggiora anche 1x. Allora non\n' ...
                     '  e'' "tau troppo alto" ma "il banco e'' sensibile a tau", e\n' ...
                     '  il valore va scelto come compromesso, non come correzione.\n']);
        else
            fprintf('  1x resta stabile: abbassare tau non rompe quello che funziona.\n');
        end
    end
end

figure; hold on; grid on
for iF = 1:numel(opt.fattori)
    s = P(P.fattore == opt.fattori(iF), :);
    if isempty(s), continue; end
    plot(s.tau, 100*s.frazione_task, 'o-');
end
yline(0,'k--');  yline(100,'k:');
set(gca,'XScale','log');
xlabel('tau dei filtri [s]'); ylabel('frazione del task [%]');
legend(arrayfun(@(x) sprintf('%.2fx',x), opt.fattori, 'UniformOutput',false), ...
       'Location','best');
title('Effetto dei filtri sui comandi di giunto')
fprintf('\n');

end

%% ================================================================
function v = ternario(c,a,b)
if c, v = a; else, v = b; end
end
