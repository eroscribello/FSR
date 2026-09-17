function C = curva_imbardata(modo, omega, mdl)
%CURVA_IMBARDATA  Imbardata eseguita in funzione di quella comandata.
%
%   C = curva_imbardata                      rotazione sul posto, griglia di default
%   C = curva_imbardata('arco')              arco di cerchio
%   C = curva_imbardata('posto', [0.4 0.857 1.5 3])
%
% PERCHE' UNA CURVA E NON UN VALORE
%   "Alzare l'imbardata finche' la curva si vede" produce un numero
%   arbitrario, che in relazione non si puo' giustificare. La stessa fatica
%   - cinque o sei run - produce invece una misura: quanta imbardata il
%   controllore ESEGUE in funzione di quanta gliene CHIEDI.
%
%   E' un risultato che riguarda la tesi del progetto, non un dettaglio di
%   taratura: il cinematico comanda posizioni in anello aperto e non sa se il
%   corpo stia ruotando davvero.
%
% [CORRETTO] LA PRIMA VERSIONE DI QUESTA FUNZIONE MISURAVA MALE
%   Stimava il tasso di imbardata come (yaw(end) - yaw(1))/durata. L'angolo
%   di imbardata si AVVOLGE a +-pi, quindi con una run da 15 s quel rapporto
%   non puo' uscire da +-pi/15 = +-0.209 rad/s, qualunque sia la rotazione
%   vera. I cinque valori usciti - +0.124, -0.172, +0.062, -0.149, +0.059 -
%   stavano tutti dentro quella banda con segni alterni: sembrava un
%   controllore senza autorita' che saturava, era il modulo dell'angolo.
%   Ora la stima passa da tasso_imbardata, che srotola l'angolo. Le
%   conclusioni tratte prima di questa correzione non valgono.
%
% LA SOGLIA DI LEGGIBILITA'
%   A comando nullo il robot deriva comunque in imbardata. Ogni cella il cui
%   effetto sia dello stesso ordine NON e' una misura. La funzione misura la
%   deriva come prima cella, la sottrae, e marca come non leggibile ogni riga
%   il cui segnale non la superi di almeno tre volte.
%
% ATTENZIONE AL PASSO, IN MODO 'posto'
%   Con v = 0 il passo vale  omega*r_offset*T_stance:  cresce con omega e a
%   un certo punto esce dalla portata della gamba. La funzione lo calcola,
%   confronta con cfg.S e con la portata massima, e rifiuta le celle
%   impossibili invece di simularle: una zampa che arriva a fine corsa
%   produce un numero che sembra saturazione del controllore e non lo e'.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(modo),  modo = 'posto'; end
if nargin < 3 || isempty(mdl),   mdl = 'phantomx_sim_zero'; end
modo = lower(strtrim(char(modo)));
if ~ismember(modo, {'posto','arco'})
    error('curva_imbardata:modo','Usa ''posto'' oppure ''arco''.');
end

cfg   = phantomx_config();
v_nom = cfg.S / cfg.T_stance;
w_rif = v_nom / cfg.r_offset;        % omega che da' il passo nominale

if nargin < 2 || isempty(omega)
    if strcmp(modo,'posto')
        omega = w_rif * [0.25 0.5 1.0 1.5 2.0];
    else
        omega = [0.2 0.4 0.8 1.2 1.6];
    end
end
omega = omega(:).';
dur = 15;

fprintf('\n========= CURVA DI AUTORITA'' DELL''IMBARDATA =========\n');
fprintf('  modo %s   v = %.4f m/s   r_offset = %.3f m\n', ...
        upper(modo), ternario(strcmp(modo,'posto'), 0, v_nom), cfg.r_offset);
fprintf('  omega di riferimento (passo nominale): %.4f rad/s\n', w_rif);
fprintf('  comandi: %s rad/s\n', mat2str(omega,4));
fprintf('  controllore C1, %g s per cella\n\n', dur);

%% ---- cella 0: la deriva ----
fprintf('[deriva] comando nullo ... ');
r0 = una_run(0, 0, cfg.S, mdl, dur, cfg);
deriva = r0.w_mis;
fprintf('%+.5f rad/s   (angolo in %s)\n', deriva, r0.info_yaw.unita);
soglia = 3 * abs(deriva);
fprintf('         soglia di leggibilita'' (3x la deriva): %.5f rad/s\n\n', soglia);

%% ---- la griglia ----
C = table();
for k = 1:numel(omega)
    w = omega(k);

    if strcmp(modo,'posto')
        v_cella = 0;
        S_cella = abs(w) * cfg.r_offset * cfg.T_stance;
    else
        v_cella = v_nom;
        S_cella = [];          % lo decide applica_imbardata sulla media dei moduli
    end

    % --- rifiuto delle celle fuori portata ---
    if ~isempty(S_cella)
        sweep = S_cella / cfg.r_offset;                 % [rad] arco di coxa
        if S_cella > 3*cfg.S
            fprintf(2,'[%d/%d] omega %.4f SALTATA: passo %.4f m, %.1f volte il nominale.\n', ...
                    k, numel(omega), w, S_cella, S_cella/cfg.S);
            fprintf(2,'       Non e'' saturazione del controllore, e'' fine corsa della gamba.\n');
            continue
        end
        if abs(sweep) > 0.8*min(abs(cfg.q_min), abs(cfg.q_max))
            fprintf(2,'[%d/%d] omega %.4f SALTATA: coxa dovrebbe spazzare %.2f rad.\n', ...
                    k, numel(omega), w, sweep);
            continue
        end
    end

    fprintf('[%d/%d] omega %+.4f ... ', k, numel(omega), w);
    try
        r = una_run(w, v_cella, S_cella, mdl, dur, cfg);

        netto = r.w_mis - deriva;
        riga = table(string(modo), w, r.w_mis, deriva, netto, netto/w, ...
                     r.giro_tot, r.giro_tot/(2*pi), ...
                     r.S, r.arco, r.vel, r.appoggio, r.slip, ...
                     abs(netto) > soglia, ...
                     'VariableNames', {'modo','yaw_cmd','yaw_mis','deriva', ...
                                       'yaw_netto','autorita','rot_tot','giri', ...
                                       'S','arco','vel','appoggio_medio', ...
                                       'slip_tot','leggibile'});
        C = [C; riga];                                                %#ok<AGROW>

        fprintf('misurata %+.5f  netta %+.5f  autorita'' %5.1f%%  giri %+.2f  S %.4f %s\n', ...
                r.w_mis, netto, 100*netto/w, r.giro_tot/(2*pi), r.S, ...
                ternario(abs(netto) > soglia, '', '  <- NON leggibile'));
    catch ME
        fprintf(2,'ERRORE: %s\n', ME.message);
    end
end

%% ---- ripristino ----
applica_imbardata(0, false, mdl);
evalin('base','clear OVERRIDE_GAIT OVERRIDE_C2');
evalin('base','init_gait');

if height(C) == 0, return; end

fprintf('\n');
disp(C)

%% ---- lettura ----
L = C(C.leggibile, :);
fprintf('\n=============== COME SI LEGGE ===============\n');
if isempty(L)
    fprintf(2,['\nNessuna cella sopra la soglia di leggibilita''. La deriva a\n' ...
               'comando nullo (%.5f rad/s) domina tutto: prima di misurare\n' ...
               'l''imbardata va capito perche'' il robot deriva camminando dritto.\n\n'], deriva);
    return
end

fprintf('\nCelle leggibili: %d su %d\n', height(L), height(C));
fprintf('Autorita'': da %.1f%% a %.1f%%\n', 100*min(L.autorita), 100*max(L.autorita));

if max(L.autorita) - min(L.autorita) < 0.05
    fprintf(['\nCOSTANTE. L''imbardata eseguita e'' una frazione fissa di quella\n' ...
             'comandata: e'' un guadagno, non una saturazione. In relazione si\n' ...
             'riporta come un fattore, e la causa va cercata nello scivolamento\n' ...
             '(colonna slip_tot), non nel comando.\n']);
else
    [~, imax] = max(L.autorita);
    fprintf(['\nVARIABILE, massimo %.1f%% a omega = %.4f rad/s.\n' ...
             'C''e'' un punto di lavoro migliore degli altri: e'' quello da usare\n' ...
             'per T3, e la curva intera e'' il risultato da riportare.\n'], ...
            100*L.autorita(imax), L.yaw_cmd(imax));
end

if any(~isnan(L.slip_tot))
    fprintf('\nScivolamento: da %.1f a %.1f mm sull''arco percorso.\n', ...
            1e3*min(L.slip_tot), 1e3*max(L.slip_tot));
    fprintf('Appoggio medio: da %.2f a %.2f piedi a terra.\n', ...
            min(L.appoggio_medio), max(L.appoggio_medio));
else
    fprintf(2,'\nslip_tot e appoggio_medio sono NaN: lancia abilita_log(''on'').\n');
end

figure; hold on; grid on
plot(C.yaw_cmd, C.yaw_netto, 'o-');
plot(C.yaw_cmd, C.yaw_cmd, 'k--');
xlabel('imbardata comandata [rad/s]'); ylabel('imbardata eseguita [rad/s]');
legend('misurata (deriva sottratta)','esecuzione perfetta','Location','best');
title(sprintf('Autorita'' dell''imbardata - C1, modo %s', modo));
fprintf('\n');

end

%% ================================================================
function r = una_run(w, v, S_forzato, mdl, dur, cfg)
%UNA_RUN  Una simulazione, e le grandezze che servono.

if w == 0
    info = applica_imbardata(0, false, mdl);
    S_use = cfg.S;
else
    info = applica_imbardata(w, false, mdl, struct('v', v));
    if isempty(S_forzato), S_use = info.S; else, S_use = S_forzato; end
end

assignin('base','OVERRIDE_C2',   false);
assignin('base','OVERRIDE_GAIT', struct('S', S_use));
evalin('base','init_gait');

out = sim(mdl, 'StopTime', num2str(dur));
run_ = adatta_simscape(out, struct( ...
           'controller','C1', 'task','T3curva', 'run',1, ...
           'condizione', sprintf('w%+.4f', w), ...
           'vel_d', [max(v, eps) 0], 'yaw_d', w));

sel = run_.t >= 2*cfg.T;
p   = run_.p(sel,1:2);
dt  = run_.t(end) - run_.t(find(sel,1,'first'));

m = metriche(run_, cfg, struct('t_regime', 2*cfg.T));

r = struct();
% NIENTE differenza fra primo e ultimo campione: l'angolo di imbardata si
% avvolge a +-pi e quel rapporto non puo' uscire da +-pi/durata, qualunque
% sia la rotazione vera. Vedi tasso_imbardata.
wz = [];
if isfield(run_,'w') && ~isempty(run_.w) && size(run_.w,2) >= 3
    wz = run_.w(:,3);       % velocita' angolare: non si avvolge
end
[r.w_mis, r.giro_tot, r.info_yaw] = tasso_imbardata(run_.t, run_.rpy(:,3), wz);
r.S        = S_use;
r.arco     = sum(vecnorm(diff(p),2,2));
r.vel      = r.arco / max(dt, eps);
r.appoggio = m.appoggio_medio;
r.slip     = m.slip_tot;
end

function v = ternario(c,a,b)
if c, v = a; else, v = b; end
end
