function [best, G] = taratura_T2(opt)
%TARATURA_T2  Ritara l'andatura a ciascuna velocita' di T2, con un criterio
%             dichiarato e la griglia completa in chiaro.
%
%   [best, G] = taratura_T2
%   [best, G] = taratura_T2(struct('fattori',[0.5 1.0 2.0]))
%   [best, G] = taratura_T2(struct('dz0',[0 2e-3], 'H',[0.04 0.05]))
%
% USCITE
%   best  table, una riga per velocita': la taratura scelta
%   G     table, TUTTA la griglia: serve piu' di best, vedi sotto
%
% PERCHE' LA GRIGLIA COMPLETA CONTA PIU' DEL MASSIMO
%   Un "meglio" senza il paesaggio intorno non dice se l'ottimo e' largo o
%   stretto - ed e' esattamente il risultato che T2 deve produrre. Due
%   tarature con lo stesso punteggio, una su un plateau e una su una punta,
%   non sono la stessa cosa: la seconda non sopravvive a un cambio di
%   terreno. G va guardata, non solo best.
%
% COSA SI RITARA, E COSA NO
%   Si ritarano i parametri dell'ANDATURA, cioe' del controllore:
%       gait.z0   profondita' di appoggio
%       gait.H    altezza di sollevamento in volo
%   NON si toccano i parametri del CONTATTO (cfg.contact.k, c, w). Quelli
%   descrivono il terreno, non il controllore: sono uguali per C1, C2 e C3, e
%   ritararli per velocita' significherebbe cambiare il banco di prova per
%   far funzionare il controllore. Sarebbe un risultato falso.
%
%   Non si tocca nemmeno la lunghezza del passo S: e' fissa per costruzione
%   in T2 (la velocita' si ottiene variando la cadenza), altrimenti si
%   mescolerebbero due effetti nella stessa curva.
%
%   FUORI PORTATA DA SCRIPT: i filtri sys_filter (tau = 0.05 s) stanno
%   nell'InitFcn del modello, non in cfg, quindi non sono sweepabili da qui.
%   A 2x lo swing dura 200 ms e il filtro ne taglia il 25%: se la cella 2x
%   resta inammissibile per tutta la griglia, il sospetto e' quello e va
%   verificato abbassando tau nell'InitFcn a mano.
%
% IL CRITERIO, IN ORDINE LESSICOGRAFICO
%   1. successo == true          (no ribaltamento, no arresto, frazione del
%                                 task sopra opt.frac_min di metriche)
%   2. appoggio_medio >= 2.7     il tripode di CARICO esiste. 2.7 su 3 e' la
%                                 soglia usata quando il contatto e' stato
%                                 ritarato: sotto, il robot cammina a meno di
%                                 tre zampe e le metriche di coppia non
%                                 descrivono un'andatura a tripode
%   3. min disp_carico           fra le ammissibili, quella che distribuisce
%                                 meglio il carico fra le sei zampe
%   4. min cot                   spareggio
%   Se nessuna cella e' ammissibile, la funzione lo dice e riporta comunque
%   la migliore per appoggio_medio, marcandola come NON ammissibile. Una
%   velocita' che non ha tarature ammissibili e' un risultato, non un errore.
%
% ANELLO APERTO
%   La taratura gira in C1 (OVERRIDE_C2 = false). Ritarare in C2 misurerebbe
%   il meglio ottenibile dalla retroazione, che e' un'altra domanda: la
%   si rifa' dopo, con opt.c2 = true, e il confronto fra le due tabelle e'
%   esso stesso una misura.
%
% Progetto FSR PhantomX - A. Russo

%% ---- opzioni ----
if nargin < 1, opt = struct(); end
cfg0 = phantomx_config();

def = struct( ...
    'fattori',   cfg0.t2_fattori, ...
    'dz0',       [-2e-3 0 +2e-3 +4e-3], ...   % scostamenti da cfg.z0 [m]
    'H',         [0.035 0.05 0.065], ...      % altezze di volo [m]
    'nCicli',    10, ...
    'c2',        false, ...
    'soglia_app',2.7, ...
    'salva',     true, ...
    'cartella',  'results', ...
    'mdl',       'phantomx_sim_zero');
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt,f{k}), opt.(f{k}) = def.(f{k}); end
end

nF = numel(opt.fattori);
nZ = numel(opt.dz0);
nH = numel(opt.H);
nTot = nF * nZ * nH;

fprintf('\n=========== TARATURA T2 ===========\n');
fprintf('  velocita'' : %s\n', strjoin(arrayfun(@(x) sprintf('%.2fx',x), ...
        opt.fattori, 'UniformOutput',false), ' '));
fprintf('  z0        : %s  (nominale %.4f)\n', ...
        mat2str(cfg0.z0 + opt.dz0, 4), cfg0.z0);
fprintf('  H         : %s\n', mat2str(opt.H, 4));
fprintf('  contatto  : NON toccato (k=%g, c=%g, w=%g)\n', ...
        cfg0.contact.k, cfg0.contact.c, cfg0.contact.w);
fprintf('  modo      : %s\n', ternario(opt.c2,'C2','C1 anello aperto'));
fprintf('  -> %d run\n\n', nTot);

G = table();
cronometro = tic;
n = 0;

%% ---- griglia ----
for iF = 1:nF
    fatt  = opt.fattori(iF);
    v_cmd = fatt * cfg0.v_nom;
    T_i   = cfg0.S / (cfg0.beta_stance * v_cmd);
    stop  = opt.nCicli * T_i;
    etich = sprintf('v%.2fx', fatt);

    for iZ = 1:nZ
        for iH = 1:nH
            n = n + 1;
            z0_i = cfg0.z0 + opt.dz0(iZ);
            H_i  = opt.H(iH);

            fprintf('[%2d/%2d] %s  z0=%.4f  H=%.3f ... ', n, nTot, etich, z0_i, H_i);

            try
                % Gli override vanno nel BASE workspace: init_gait e' l'InitFcn
                % del modello e gira la', non qui dentro.
                assignin('base','OVERRIDE_C2',   opt.c2);
                assignin('base','OVERRIDE_GAIT', ...
                         struct('T',T_i, 'z0',z0_i, 'H',H_i));
                evalin('base','init_gait');

                out = sim(opt.mdl, 'StopTime', num2str(stop));

                r = adatta_simscape(out, struct( ...
                        'controller', ternario(opt.c2,'C2','C1'), ...
                        'task','T2', 'run',1, ...
                        'condizione', etich, ...
                        'vel_d', [v_cmd 0]));

                cfgLoc   = cfg0;
                cfgLoc.T = T_i;
                riga = metriche(r, cfgLoc, struct('t_regime', 2*T_i));

                riga.fattore = fatt;
                riga.z0      = z0_i;
                riga.H       = H_i;
                riga.T_gait  = T_i;
                riga.v_cmd   = v_cmd;

                G = [G; riga];                                          %#ok<AGROW>

                fprintf('task %5.1f%%  piedi %.2f  disp %4.1f%%  CoT %6.2f  %s\n', ...
                    100*riga.frazione_task, riga.appoggio_medio, ...
                    100*riga.disp_carico, riga.cot, ...
                    ternario(riga.successo,'ok','FALLITA'));

            catch ME
                fprintf(2,'ERRORE: %s\n', ME.message);
            end
        end
    end
end

%% ---- ripristino ----
evalin('base','clear OVERRIDE_GAIT OVERRIDE_C2');
evalin('base','init_gait');

fprintf('\n%d run su %d completate in %.1f min.\n', height(G), nTot, toc(cronometro)/60);
if height(G) == 0
    best = table();
    return
end

%% ---- scelta, per velocita' ----
best = table();
fprintf('\n=========== PAESAGGIO E SCELTA ===========\n');

for iF = 1:nF
    fatt = opt.fattori(iF);
    s = G(G.fattore == fatt, :);
    if isempty(s), continue; end

    amm = s.successo & s.appoggio_medio >= opt.soglia_app;

    fprintf('\n--- %.2fx  (%d tarature, %d ammissibili) ---\n', ...
            fatt, height(s), nnz(amm));
    fprintf('  %8s %7s | %7s %7s %7s %8s %s\n', ...
            'z0','H','task%','piedi','disp%','CoT','');
    for k = 1:height(s)
        fprintf('  %8.4f %7.3f | %7.1f %7.2f %7.1f %8.2f %s\n', ...
            s.z0(k), s.H(k), 100*s.frazione_task(k), s.appoggio_medio(k), ...
            100*s.disp_carico(k), s.cot(k), ternario(amm(k),'  <- ammissibile',''));
    end

    if any(amm)
        c = s(amm, :);
        % criterio: min disp_carico, spareggio su cot
        [~, ord] = sortrows([c.disp_carico, c.cot]);
        scelta = c(ord(1), :);
        scelta.ammissibile = true;
        % quanto e' largo l'ottimo: dispersione dei punteggi ammissibili
        scelta.n_ammissibili = nnz(amm);
        scelta.larghezza = max(c.disp_carico) - min(c.disp_carico);
    else
        [~, ix] = max(s.appoggio_medio);
        scelta = s(ix, :);
        scelta.ammissibile = false;
        scelta.n_ammissibili = 0;
        scelta.larghezza = NaN;
        fprintf(2,['  NESSUNA taratura ammissibile a %.2fx. Riportata la migliore\n' ...
                   '  per appoggio_medio, marcata non ammissibile. Se il motivo e''\n' ...
                   '  perdita di appoggio ad alta velocita'', il prossimo sospetto\n' ...
                   '  sono i filtri sys_filter: tau = 0.05 s su uno swing da %.0f ms.\n'], ...
                fatt, 1000*(1-cfg0.beta_stance)*s.T_gait(1));
    end

    fprintf('  SCELTA: z0 = %.4f   H = %.3f   (%d ammissibili su %d)\n', ...
            scelta.z0, scelta.H, scelta.n_ammissibili, height(s));

    best = [best; scelta];                                              %#ok<AGROW>
end

%% ---- blocco da incollare in phantomx_config ----
fprintf('\n=========== DA INCOLLARE IN phantomx_config.m ===========\n');
fprintf('%% [TARATO] taratura_T2 del %s, modo %s.\n', ...
        char(datetime('now','Format','d MMMM yyyy','Locale','it_IT')), ...
        ternario(opt.c2,'C2','C1'));
fprintf('%% Criterio: successo, appoggio_medio >= %.1f, min disp_carico.\n', opt.soglia_app);
fprintf('%% Colonne: fattore, z0, H, ammissibile.\n');
fprintf('cfg.t2_taratura = [\n');
for k = 1:height(best)
    fprintf('    %.4f  %.5f  %.4f  %d\n', ...
            best.fattore(k), best.z0(k), best.H(k), best.ammissibile(k));
end
fprintf('    ];\n\n');

%% ---- salvataggio ----
if opt.salva
    if ~isfolder(opt.cartella), mkdir(opt.cartella); end
    stamp = char(datetime('now','Format','yyyyMMdd_HHmmss'));
    base = fullfile(opt.cartella, sprintf('taratura_T2_%s_%s', ...
                    ternario(opt.c2,'C2','C1'), stamp));
    writetable(G,    [base '_griglia.csv']);
    writetable(best, [base '_scelte.csv']);
    fprintf('Salvato: %s_griglia.csv  e  _scelte.csv\n\n', base);
end

end

%% ================================================================
function v = ternario(c,a,b)
if c, v = a; else, v = b; end
end
