function [v_lim, L] = limite_velocita(opt)
%LIMITE_VELOCITA  La velocita' massima a cui C1 regge l'andatura a tripode.
%
%   [v_lim, L] = limite_velocita
%   [v_lim, L] = limite_velocita(struct('fattori', 1.0:0.1:1.5))
%
% A COSA SERVE
%   La terza velocita' della campagna T2 e' il LIMITE del cinematico, e va
%   misurato. Interpolare "circa 1.2x" fra 1.0x e 1.5x e metterlo in
%   relazione come soglia non e' difendibile: la soglia e' un risultato, e
%   un risultato si misura.
%
% IL CRITERIO, DICHIARATO PRIMA DI GUARDARE I DATI
%   L'andatura a tripode e' valida finche' tre piedi stanno a terra. Quindi:
%
%     il limite e' il FATTORE PIU' ALTO che soddisfa tutte e tre:
%       1. meno di tre piedi a terra per meno di soglia_sotto3 del tempo
%       2. rimbalzo del corpo sotto soglia_rimbalzo
%       3. frazione del task positiva (il robot va avanti)
%
%   Le tre condizioni misurano cose diverse e servono tutte: la prima e' la
%   definizione di tripode, la seconda coglie il rimbalzo prima che diventi
%   perdita di appoggio, la terza esclude che una cella "quieta" ma immobile
%   passi il filtro.
%
%   Il criterio e' scritto qui e non scelto dopo, perche' scegliere la soglia
%   guardando dove cade la curva e' come tarare H minimizzando la dispersione
%   del carico: si ottiene il numero che si voleva.
%
% PERCHE' LE SOGLIE VALGONO QUESTO
%   soglia_sotto3 = 0.05  il tripode c'e' il 95% del tempo. A 1.0x il valore
%                         misurato e' 0.004, a 1.5x e' 0.34: la soglia sta
%                         in un intervallo dove i dati non sono ambigui, e
%                         spostarla fra 0.02 e 0.15 non cambia l'esito.
%   soglia_rimbalzo = 0.010 m  un centimetro su 154 mm di altezza di
%                         appoggio, e piu' del doppio della penetrazione
%                         statica (4.3 mm): sotto questo valore il corpo
%                         non sta saltando, sta cedendo sul contatto.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1, opt = struct(); end
cfg = phantomx_config();

def = struct('fattori',         1.0:0.1:1.5, ...
             'soglia_sotto3',   0.05, ...
             'soglia_rimbalzo', 0.010, ...
             'nCicli',          10, ...
             'mdl',             'phantomx_sim_zero', ...
             'verbose',         true);
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt,f{k}), opt.(f{k}) = def.(f{k}); end
end

fprintf('\n========== LIMITE DI VELOCITA'' DI C1 ==========\n');
fprintf('  fattori provati : %s\n', mat2str(opt.fattori,3));
fprintf('  criterio        : sotto3 < %.0f%%, rimbalzo < %.0f mm, task > 0\n', ...
        100*opt.soglia_sotto3, 1e3*opt.soglia_rimbalzo);

applica_terreno('T2', false, opt.mdl);

L = table();
for iF = 1:numel(opt.fattori)
    fatt  = opt.fattori(iF);
    v_cmd = fatt * cfg.v_nom;
    T_i   = cfg.S / (cfg.beta_stance * v_cmd);
    stop  = opt.nCicli * T_i;

    fprintf('\n  %.2fx (T = %.3f s) ... ', fatt, T_i);
    try
        assignin('base','OVERRIDE_C2', false);
        assignin('base','OVERRIDE_GAIT', struct('T',T_i, 'H',cfg.H));
        evalin('base','init_gait');
        out = sim(opt.mdl, 'StopTime', num2str(stop));
        r = adatta_simscape(out, struct('task','T2', 'run',1, ...
                'condizione', sprintf('v%.2fx', fatt), ...
                'vel_d', [v_cmd 0]), struct('verbose',false));
        cfgLoc = cfg;  cfgLoc.T = T_i;
        riga = metriche(r, cfgLoc, struct('t_regime', 2*T_i));
    catch ME
        fprintf(2,'ERRORE: %s\n', ME.message);
        continue
    end

    if ~ismember('sotto3_frac', riga.Properties.VariableNames)
        fprintf(2,['\n  metriche non ha la colonna sotto3_frac: e'' la versione\n' ...
                   '  vecchia. Aggiornala, senza quella il criterio non si\n' ...
                   '  puo'' applicare.\n']);
        return
    end

    ok1 = riga.sotto3_frac < opt.soglia_sotto3;
    ok2 = riga.corpoZ_pp   < opt.soglia_rimbalzo;
    ok3 = riga.frazione_task > 0;
    ok  = ok1 && ok2 && ok3;

    L = [L; table(fatt, v_cmd, T_i, riga.frazione_task, riga.appoggio_medio, ...
            riga.sotto3_frac, riga.corpoZ_pp, ok1, ok2, ok3, ok, ...
        'VariableNames', {'fattore','v_cmd','T','frazione_task','appoggio_medio', ...
                          'sotto3_frac','corpoZ_pp','c1_tripode','c2_rimbalzo', ...
                          'c3_avanza','valido'})];                    %#ok<AGROW>

    fprintf('task %+6.1f%%  piedi %.2f  sotto3 %5.1f%%  rimbalzo %5.1f mm  %s\n', ...
            100*riga.frazione_task, riga.appoggio_medio, ...
            100*riga.sotto3_frac, 1e3*riga.corpoZ_pp, ...
            ternario(ok,'VALIDO','fuori'));
end

evalin('base','clear OVERRIDE_GAIT OVERRIDE_C2');
evalin('base','init_gait');

if height(L) == 0
    fprintf(2,'\nnessuna run completata\n');  v_lim = NaN;  return
end

fprintf('\n=============== QUADRO ===============\n');
disp(L)

%% ---- il limite ----
validi = L.fattore(L.valido);
if isempty(validi)
    v_lim = NaN;
    fprintf(2,'\n  NESSUN fattore provato e'' valido.\n');
    fprintf(['  Anche %.2fx non passa il criterio, quindi il limite sta SOTTO\n' ...
             '  l''intervallo provato. Rilancia con fattori piu'' bassi:\n' ...
             '    limite_velocita(struct(''fattori'', 0.8:0.05:1.1))\n'], ...
             min(L.fattore));
    return
end

v_lim = max(validi);

% Il limite e' credibile solo se la transizione e' MONOTONA: se un fattore
% alto passa e uno piu' basso no, la soglia non sta separando niente e il
% valore massimo sarebbe un caso fortunato.
primoNo = L.fattore(~L.valido);
if ~isempty(primoNo) && any(primoNo < v_lim)
    fprintf(2,'\n  LA TRANSIZIONE NON E'' MONOTONA.\n');
    fprintf(['  %.2fx passa il criterio ma %.2fx no: il limite non e'' una\n' ...
             '  soglia, e prendere il massimo dei validi sarebbe arbitrario.\n' ...
             '  Va riportato l''intervallo, non un valore, e la dispersione fra\n' ...
             '  ripetizioni va misurata prima di concludere.\n'], ...
             v_lim, max(primoNo(primoNo < v_lim)));
end

fprintf('\n=============== ESITO ===============\n');
fprintf('  LIMITE = %.2fx  (v = %.4f m/s)\n\n', v_lim, v_lim*cfg.v_nom);

iLim = find(L.fattore == v_lim, 1);
fprintf(['  A questo fattore: task %+.1f%%, piedi a terra %.2f, sotto tre il\n' ...
         '  %.1f%% del tempo, rimbalzo %.1f mm.\n'], ...
         100*L.frazione_task(iLim), L.appoggio_medio(iLim), ...
         100*L.sotto3_frac(iLim), 1e3*L.corpoZ_pp(iLim));

fuori = L(~L.valido, :);
if ~isempty(fuori)
    iF1 = 1;
    fprintf(['\n  Il primo fattore che cade e'' %.2fx: sotto tre il %.1f%% del\n' ...
             '  tempo, rimbalzo %.1f mm. Quale delle tre condizioni cede:\n' ...
             '    tripode  %s\n    rimbalzo %s\n    avanza   %s\n'], ...
             fuori.fattore(iF1), 100*fuori.sotto3_frac(iF1), ...
             1e3*fuori.corpoZ_pp(iF1), ...
             ternario(fuori.c1_tripode(iF1),'ok','CEDE'), ...
             ternario(fuori.c2_rimbalzo(iF1),'ok','CEDE'), ...
             ternario(fuori.c3_avanza(iF1),'ok','CEDE'));
end

fprintf(['\n  Da mettere in configurazione:\n' ...
         '    cfg.t2_fattori = [0.5 1.0 %.2f 1.5 2.0];\n' ...
         '  dove i primi tre sono punti di confronto e gli ultimi due casi\n' ...
         '  non funzionanti, da riportare con sotto3_frac e corpoZ_pp e non\n' ...
         '  con err_vx_rms - a quelle velocita'' non e'' un errore di\n' ...
         '  inseguimento, e'' il robot che non cammina.\n\n'], v_lim);

if opt.verbose
    figure;
    subplot(2,1,1); hold on; grid on
    plot(L.fattore, 100*L.sotto3_frac, 'o-', 'DisplayName','sotto 3 piedi [%]');
    yline(100*opt.soglia_sotto3,'r:','DisplayName','soglia');
    xline(v_lim,'k--','DisplayName','limite');
    ylabel('[%]'); legend('Location','best');
    title('Condizione 1: il tripode c''e''?');
    subplot(2,1,2); hold on; grid on
    plot(L.fattore, 1e3*L.corpoZ_pp, 's-', 'DisplayName','rimbalzo [mm]');
    yline(1e3*opt.soglia_rimbalzo,'r:','DisplayName','soglia');
    xline(v_lim,'k--','DisplayName','limite');
    xlabel('fattore di velocita'' [x]'); ylabel('[mm]'); legend('Location','best');
    title('Condizione 2: il corpo salta?');
end

end

%% ================================================================
function v = ternario(c,a,b)
if c, v = a; else, v = b; end
end
