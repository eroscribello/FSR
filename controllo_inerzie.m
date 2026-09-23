%% controllo_inerzie.m - il modello regge con le inerzie corrette?  [PRIMA DELLE CAMPAGNE]
%
% PERCHE'
%   Correggere le inerzie cambia i risultati (vedi applica_inerzie e
%   results/prova_inerzie*.csv). Prima di rifare mezza giornata di campagne
%   conviene verificare che le TARATURE fatte con le inerzie vecchie valgano
%   ancora. Se qualcosa non torna, si ritara prima, non dopo.
%
% LE TRE VERIFICHE (criteri scritti prima di lanciare)
%   1. SENSORI DI FORZA. In piano la somma delle forze verticali misurate deve
%      chiudere sul peso. Oggi chiude allo 0.0%.
%      Passa se |chiusura - 1| <= 5%.
%   2. RICERCA DEL TERRENO DI C2. C2 riconosce il contatto confrontando la
%      coppia attesa con quella misurata (soglia t_threshold). Con le inerzie
%      corrette le coppie in volo calano molto, quindi la soglia potrebbe non
%      valere piu'. Non avendo un segnale "ho rilevato il contatto" leggibile
%      dall'esterno, si guardano le FIRME di C2 misurate in T2 e T3, che
%      esistono solo se la ricerca interviene:
%        - corpo piu' alto di C1 (era +5..7 mm)
%        - cot piu' alto di C1 (era +14%)
%      Passa se il corpo resta piu' alto di almeno 2 mm e il cot resta maggiore.
%      Se le firme spariscono, la ricerca non scatta piu': soglia da ritarare.
%   3. LE RUN SONO SANE. Successo, frazione del task entro +-20%, nessun
%      ribaltamento, tempo di calcolo non esploso.
%
% COSA NON TOCCA
%   Come prova_inerzie: nessun save_system, modello chiuso senza salvare alla
%   fine, nessun CSV di campagna sovrascritto (scrive results/controllo_inerzie.csv).
%
% USO
%   clear all; bdclose all; startup_phantomx
%   controllo_inerzie
%
% Progetto FSR PhantomX - A. Russo

ci_cfg = phantomx_config();
ci_mdl = 'phantomx_sim_zero';

if bdIsLoaded(ci_mdl) && strcmp(get_param(ci_mdl,'Dirty'),'on')
    error('controllo_inerzie:dirty', ['Il modello e'' aperto con modifiche non ' ...
        'salvate: salvale o chiudilo (bdclose all) prima. Questo script lo chiude ' ...
        'senza salvare.']);
end
load_system(ci_mdl);

ci_T    = ci_cfg.S / (ci_cfg.beta_stance * ci_cfg.v_nom);   % T2 a 1x
ci_stop = 10 * ci_T;
ci_tab  = table();
ci_tempo = zeros(1,2);

try
    applica_terreno('T2', false, ci_mdl);
    applica_inerzie(ci_mdl, true);            % <-- la correzione, solo in memoria

    for ci_k = 1:2
        ci_c2   = (ci_k == 2);
        ci_ctrl = 'C1';  if ci_c2, ci_ctrl = 'C2'; end
        fprintf('\n---- T2 1x, %s, inerzie corrette ----\n', ci_ctrl);

        OVERRIDE_C2   = ci_c2;                                 %#ok<NASGU>
        OVERRIDE_GAIT = struct('T', ci_T);                     %#ok<NASGU>
        init_gait
        ci_cron = tic;
        ci_out  = sim(ci_mdl, 'StopTime', num2str(ci_stop));
        ci_tempo(ci_k) = toc(ci_cron);

        ci_run = adatta_simscape(ci_out, struct('controller',ci_ctrl, 'task','T2', ...
                     'run',1, 'condizione','v1.00x', 'vel_d',[ci_cfg.v_nom 0]));
        ci_cfgLoc = ci_cfg;  ci_cfgLoc.T = ci_T;
        ci_riga = metriche(ci_run, ci_cfgLoc, struct('t_regime', 2*ci_T));

        % chiusura sul peso, come negli script T4-T6
        ci_ch = NaN;
        if isfield(ci_run,'Fc') && ~isempty(ci_run.Fc)
            ci_sel = ci_run.t >= 2*ci_T;
            ci_ch = mean(sum(ci_run.Fc(ci_sel,3:3:18),2)) / (ci_cfg.mass*ci_cfg.g);
        end
        ci_riga.chiusura_peso = ci_ch;
        ci_riga.t_calcolo     = ci_tempo(ci_k);
        ci_tab = [ci_tab; ci_riga];                            %#ok<AGROW>
    end
    clear OVERRIDE_GAIT OVERRIDE_C2
    init_gait
catch ci_err
    clear OVERRIDE_GAIT OVERRIDE_C2
    if bdIsLoaded(ci_mdl), close_system(ci_mdl, 0); end
    fprintf(2, '\n  Errore: modello chiuso SENZA salvare (le inerzie originali sono nel file).\n');
    rethrow(ci_err);
end
if bdIsLoaded(ci_mdl), close_system(ci_mdl, 0); end   % 0 = non salvare

%% ---- esito delle tre verifiche ----
if ~isfolder('results'), mkdir('results'); end
writetable(ci_tab, fullfile('results','controllo_inerzie.csv'));

C1 = ci_tab(1,:);   C2 = ci_tab(2,:);
dz   = 1e3 * (C2.z_media - C1.z_media);        % [mm] corpo di C2 rispetto a C1
dcot = 100 * (C2.cot / C1.cot - 1);            % [%]

fprintf('\n=============== CONTROLLO INERZIE (T2 1x, inerzie corrette) ===============\n');
fprintf('  %-28s %12s %12s\n', '', 'C1', 'C2');
for c = {'chiusura_peso','vel_media','frazione_task','z_media','cot','tau_max', ...
         'appoggio_medio','sotto3_frac','t_calcolo'}
    fprintf('  %-28s %12.4g %12.4g\n', c{1}, C1.(c{1}), C2.(c{1}));
end

fprintf('\n  1. sensori di forza          ');
ci_ok1 = all(abs(ci_tab.chiusura_peso - 1) <= 0.05);
fprintf('%s  (C1 %.1f%%, C2 %.1f%% del peso)\n', ci_esito(ci_ok1), ...
        100*C1.chiusura_peso, 100*C2.chiusura_peso);

fprintf('  2. ricerca del terreno di C2  ');
ci_ok2 = (dz >= 2) && (C2.cot > C1.cot);
fprintf('%s  (corpo %+.1f mm, cot %+.1f%% rispetto a C1)\n', ci_esito(ci_ok2), dz, dcot);
if ~ci_ok2
    fprintf(2, ['     Le firme di C2 non ci sono piu'': con le inerzie corrette la\n' ...
                '     soglia di rilevazione del contatto (t_threshold) va ritarata\n' ...
                '     PRIMA di rifare le campagne.\n']);
end

fprintf('  3. run sane                   ');
ci_ok3 = all(ci_tab.successo) && all(abs(ci_tab.frazione_task - 1) <= 0.2) && ...
         all(ci_tab.causa_fallimento ~= "ribaltamento");
fprintf('%s  (frazione del task %.0f%% e %.0f%%, calcolo %.0f s e %.0f s)\n', ...
        ci_esito(ci_ok3), 100*C1.frazione_task, 100*C2.frazione_task, ci_tempo(1), ci_tempo(2));

fprintf('\n  %s\n', ci_verdetto(ci_ok1 && ci_ok2 && ci_ok3));
fprintf('  scritto  results/controllo_inerzie.csv\n');
fprintf('  modello chiuso senza salvare: nel .slx restano le inerzie originali.\n\n');

%% ================= helper =================
function s = ci_esito(ok)
if ok, s = 'PASSA '; else, s = 'NO    '; end
end

function s = ci_verdetto(ok)
if ok
    s = 'Tutto a posto: le campagne si possono rifare con applica_inerzie.';
else
    s = 'Qualcosa non torna: leggi sopra e ritara PRIMA di rifare le campagne.';
end
end
