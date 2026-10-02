%% prova_soglia_T6.m - la soglia di C2 conta sul percorso a ostacoli?
%
% LA DOMANDA
%   [25/9] Dopo aver allineato lo stimatore (allinea_stimatore) il flag di
%   contatto di C2 funziona. Resta da scegliere la terna di soglie:
%       A  [0.04  0.1    0.1  ]   quella che sta nel modello del collega
%       B  [0.385 0.136  0.105]   l'ottimo misurato da soglia_ottima
%   Su T2, terreno piano, sono equivalenti dove conta. L'unica differenza
%   grossa e' il RITARDO del flag: -220 ms con A, -10 ms con B. In
%   ricerca_terreno un flag che si alza prima del contatto congela la zampa
%   alla quota di quel momento: su un pavimento non cambia niente, su un
%   dislivello si'. Quindi T2 non puo' decidere.
%
% ATTENZIONE A COSA FA LA RICERCA, E A COSA NON FA
%   Estende il piede VERSO IL BASSO: serve dove il terreno e' piu' in basso
%   del previsto - il bordo in discesa di un ostacolo, un avvallamento. NON
%   aiuta a scavalcare uno scalino in salita, che dipende solo da
%   cfg.H = 50 mm, e la barra piu' alta di T6 e' 106 mm.
%
% [CORRETTO 25/9] DUE ERRORI DELLA PRIMA VERSIONE, ENTRAMBI MIEI
%
%   1. CONFRONTAVA RUN DI LUNGHEZZA DIVERSA. Con A il robot percorreva
%      3.09 m, con B 3.97 m. Ma gli ostacoli di T6 stanno a x fissi: due run
%      che arrivano a x diversi NON INCONTRANO GLI STESSI OSTACOLI, quindi
%      alt_ost, appoggi_su_ost e il rollio non sono confrontabili. Il primo
%      confronto dava alt_ost 39 mm contro 66 mm, e sembrava un effetto
%      della soglia: era solo il fatto che una delle due era arrivata piu'
%      in la'. Adesso le metriche si ricalcolano su una FINESTRA COMUNE in x.
%
%   2. IL PAVIMENTO FINISCE A x = 3.95 m. Con B la distanza era 3.971 m e
%      roll_max esattamente 3.141 rad, cioe' pi greco: il robot non si e'
%      ribaltato per colpa del controllo, e' CADUTO DAL BORDO del pavimento
%      8x8. E' lo stesso artefatto che script_T4D gia' toglie con
%      t4d_taglia_bordo, e che qui mancava. Adesso la run si tronca al primo
%      campione con un piede oltre il bordo, prima delle metriche.
%
%   Finche' questi due non erano corretti, il confronto misurava la
%   geometria del pavimento, non la soglia.
%
% I CRITERI, SCRITTI PRIMA DI LANCIARE
%   Si leggono NELL'ORDINE, e il primo che si applica decide. La prima
%   versione metteva per primo "nessuna delle due supera" e cosi' liquidava
%   come irrilevanti differenze enormi: l'ordine giusto guarda prima QUANTO
%   sono diverse, poi l'esito.
%   1. LA SOGLIA CONTA se, sulla finestra comune, roll_max o pitch_max
%      cambiano di piu' del 50%, oppure cambia l'esito di 'superato'.
%   2. LA SOGLIA NON CONTA se tutte le metriche della finestra comune
%      stanno entro il 20%. Allora si resta su A: e' il valore del modello,
%      e un parametro che non tariamo noi e' un grado di liberta' in meno da
%      difendere in relazione.
%   3. Fra il 20% e il 50% il dato non decide: si dichiara incerto e, se
%      serve deciderlo, si ripete su T5 e T4D.
%   In tutti i casi, se NESSUNA supera, si dice anche perche': la barra e'
%   106 mm contro un sollevamento di 50 mm, e nessuna soglia lo cambia.
%
% COSA NON TOCCA
%   Rilancia script_T6 due volte, quindi superato/alt_ost/appoggi_su_ost
%   sono esattamente quelli di campagna. Il CSV results/T6_C2.csv viene
%   messo da parte prima e rimesso a posto dopo, anche in caso di errore.
%   Nessun save_system.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   prova_soglia_T6
%
% Progetto FSR PhantomX - A. Russo

% SCRIPT e non funzione, apposta: script_T6 assegna OVERRIDE_C2 nel proprio
% workspace e l'InitFcn del modello lo legge dal workspace BASE. Da dentro
% una funzione l'override non arriverebbe e le due run girerebbero uguali
% senza che nulla lo segnali. Stesso motivo di spazzata_soglia.
% Variabili prefissate ps_ per non finire sotto quelle di script_T6 (t6_*) e
% di init_gait (k, j, L, cfg, alpha, side, offset, pen).

ps_cfg   = phantomx_config();
ps_terne = { 'A_modello',  reshape(ps_cfg.c2.soglia_tau, 1, [])
             'B_ottima',   [0.3849 0.1359 0.1052] };
ps_xbordo = 4 - 0.05;        % pavimento 8x8, x in [-4 4]: stesso valore di T4D

ps_file_camp = fullfile('results','T6_C2.csv');
ps_dest      = fullfile('results','diagnostica');
if ~isfolder(ps_dest), mkdir(ps_dest); end

% Il CSV di campagna si mette da parte PRIMA: se una run si rompe a meta',
% quello che resta sul disco non deve essere una riga di prova travestita da
% riga di campagna. E' gia' successo due volte con T6.
ps_backup = '';
if isfile(ps_file_camp)
    ps_backup = [tempname '.csv'];
    copyfile(ps_file_camp, ps_backup);
    fprintf('  results/T6_C2.csv messo da parte, verra'' rimesso a posto alla fine.\n');
end

PS    = table();          % righe di campagna, run intera
ps_rr = {};               % le run, per il ricalcolo sulla finestra comune
ps_crono = tic;

try
    for ps_i = 1:size(ps_terne,1)
        ps_nome = ps_terne{ps_i,1};
        ps_s    = ps_terne{ps_i,2};

        fprintf('\n\n============ T6, C2, soglia %s = %s ============\n', ...
                ps_nome, mat2str(ps_s,4));

        OVERRIDE_C2_SOGLIA = ps_s;                                 %#ok<NASGU>
        script_T6                     % <-- le metriche di campagna, non copie

        if exist('c2_soglia','var') && any(abs(reshape(c2_soglia,1,[]) - ps_s) > 1e-12)
            error('prova_soglia_T6:soglia', ...
                'Ho chiesto %s, il modello ha usato %s.', ...
                mat2str(ps_s,4), mat2str(c2_soglia,4));
        end

        copyfile(ps_file_camp, fullfile(ps_dest, sprintf('T6_C2_soglia%s.csv', ps_nome)));

        ps_r = t6_riga;
        ps_r.soglia    = string(mat2str(ps_s,4));
        ps_r.etichetta = string(ps_nome);
        PS = [PS; ps_r];                                           %#ok<AGROW>
        ps_rr{end+1} = t6_run;                                     %#ok<AGROW>
    end
catch ps_err
    clear OVERRIDE_C2_SOGLIA OVERRIDE_C2
    if ~isempty(ps_backup), copyfile(ps_backup, ps_file_camp); end
    fprintf(2,'\n  Errore: CSV di campagna rimesso a posto, modello non salvato.\n');
    rethrow(ps_err);
end

clear OVERRIDE_C2_SOGLIA OVERRIDE_C2
if ~isempty(ps_backup)
    copyfile(ps_backup, ps_file_camp);
    delete(ps_backup);
    fprintf('\n  results/T6_C2.csv rimesso com''era.\n');
end

%% ---- finestra comune, e taglio al bordo ----
% Il confronto onesto e' a PARI PERCORSO: stessi ostacoli incontrati, e
% niente caduta dal bordo dentro le metriche.
ps_n = numel(ps_rr);
ps_kb = zeros(1,ps_n);  ps_xb = zeros(1,ps_n);  ps_tb = nan(1,ps_n);
for ps_i = 1:ps_n
    r = ps_rr{ps_i};
    k = find(any(r.pf(:, 1:3:18) > ps_xbordo, 2), 1, 'first');
    if isempty(k), k = numel(r.t); else, ps_tb(ps_i) = r.t(k); end
    ps_kb(ps_i) = k;
    ps_xb(ps_i) = r.p(k,1);
end
ps_xcom = min(ps_xb);

fprintf('\n---- finestra di confronto ----\n');
for ps_i = 1:ps_n
    if isnan(ps_tb(ps_i))
        fprintf('  %-12s arriva a x = %.3f m, nessun piede oltre il bordo\n', ...
                ps_terne{ps_i,1}, ps_xb(ps_i));
    else
        fprintf(2,'  %-12s un piede supera il bordo (x = %.2f m) a t = %.2f s: da li'' cade\n', ...
                ps_terne{ps_i,1}, ps_xbordo, ps_tb(ps_i));
    end
end
fprintf('  finestra comune: fino a x = %.3f m\n', ps_xcom);

PSC = table();
for ps_i = 1:ps_n
    r = ps_rr{ps_i};
    k = find(r.p(:,1) >= ps_xcom, 1, 'first');
    if isempty(k), k = ps_kb(ps_i); end
    k = min(k, ps_kb(ps_i));
    r = ps_taglia(r, k);
    ps_q = metriche(r, ps_cfg, struct('t_regime', 2*ps_cfg.T));
    ps_q.etichetta = string(ps_terne{ps_i,1});
    ps_q.t_fine    = r.t(end);
    PSC = [PSC; ps_q];                                             %#ok<AGROW>
end

writetable(PS,  fullfile(ps_dest,'prova_soglia_T6.csv'));
writetable(PSC, fullfile(ps_dest,'prova_soglia_T6_finestra.csv'));

%% ---- lettura ----
fprintf('\n\n======== RUN INTERE (non confrontabili se i percorsi differiscono) ========\n');
ps_stampa(PS, {'superato','distanza','alt_ost','appoggi_su_ost','pitch_esc','roll_esc'});

fprintf('\n======== FINESTRA COMUNE, fino a x = %.3f m ========\n', ps_xcom);
ps_col = {'roll_max','pitch_max','roll_rms','pitch_rms','dev_lat_max', ...
          'yaw_err_fin','cot','energia','z_media','tau_max','t_fine'};
ps_stampa(PSC, ps_col);

%% ---- i criteri, nell'ordine dichiarato ----
ps_var = @(T,c) (ismember(c, T.Properties.VariableNames) && ...
                 min(abs(double(T.(c)))) > 0) * ...
                ((max(double(T.(c))) - min(double(T.(c)))) / ...
                 max(min(abs(double(T.(c)))), eps));

ps_dr = ps_var(PSC,'roll_max');
ps_dp = ps_var(PSC,'pitch_max');
ps_esito_diverso = numel(unique(double(PS.superato))) > 1;
ps_max_scarto = 0;
for ps_k = 1:numel(ps_col)
    ps_max_scarto = max(ps_max_scarto, ps_var(PSC, ps_col{ps_k}));
end

fprintf('\n  scarto su roll_max  %.0f%%\n', 100*ps_dr);
fprintf('  scarto su pitch_max %.0f%%\n', 100*ps_dp);
fprintf('  scarto massimo      %.0f%%\n', 100*ps_max_scarto);
fprintf('  esito di "superato" diverso: %s\n', ps_si(ps_esito_diverso));

fprintf('\n');
if ps_esito_diverso || ps_dr > 0.50 || ps_dp > 0.50
    fprintf('  => LA SOGLIA CONTA SU T6. Si adotta la terna migliore e si dichiara\n');
    fprintf('     in relazione che quel parametro l''abbiamo tarato noi, con il\n');
    fprintf('     criterio e la misura che lo hanno deciso.\n');
elseif ps_max_scarto <= 0.20
    fprintf('  => LA SOGLIA NON CONTA SU T6: si resta su A = %s.\n', ...
            mat2str(ps_terne{1,2},4));
    fprintf('     E'' il valore che sta nel modello: non tarando noi quel parametro,\n');
    fprintf('     il confronto C1/C2 resta sui controllori e non sulla nostra\n');
    fprintf('     taratura. L''ottimo misurato resta un risultato da scrivere.\n');
else
    fprintf('  => IL DATO NON DECIDE (scarto massimo %.0f%%, fra il 20%% e il 50%%).\n', ...
            100*ps_max_scarto);
    fprintf('     Se serve deciderlo, si ripete la stessa prova su T5 e T4D.\n');
end

if ~any(double(PS.superato))
    fprintf('\n  Nota: nessuna delle due supera il percorso, e non dipende dalla\n');
    fprintf('  soglia. La barra piu'' alta e'' 106 mm, il sollevamento e''\n');
    fprintf('  cfg.H = %.0f mm, e la ricerca del terreno estende il piede verso il\n', 1e3*ps_cfg.H);
    fprintf('  BASSO: non puo'' scavalcare niente. E'' un limite dell''andatura.\n');
end

fprintf('\n  scritto  %s\n', fullfile(ps_dest,'prova_soglia_T6.csv'));
fprintf('  scritto  %s   (%.0f s)\n', ...
        fullfile(ps_dest,'prova_soglia_T6_finestra.csv'), toc(ps_crono));
fprintf('  CSV di campagna intatti, modello non salvato.\n\n');

%% ================= helper =================
function r = ps_taglia(r, k)
%PS_TAGLIA  Tronca la run al campione k. Come t4d_taglia_bordo.
N = numel(r.t);
if k >= N || k < 2, return; end
f = fieldnames(r);
for i = 1:numel(f)
    v = r.(f{i});
    if (isnumeric(v) || islogical(v)) && size(v,1) == N
        r.(f{i}) = v(1:k, :);
    end
end
end

function ps_stampa(T, col)
fprintf('  %-18s', 'metrica');
for i = 1:height(T), fprintf('%16s', T.etichetta(i)); end
fprintf('%12s\n', 'scarto');
for k = 1:numel(col)
    c = col{k};
    if ~ismember(c, T.Properties.VariableNames), continue; end
    v = T.(c);
    if ~(isnumeric(v) || islogical(v)), continue; end
    v = double(v);
    fprintf('  %-18s', c);
    fprintf('%16.4g', v);
    if min(abs(v)) > 0
        fprintf('%11.0f%%\n', 100*(max(v)-min(v))/min(abs(v)));
    else
        fprintf('%12s\n', '-');
    end
end
end

function s = ps_si(tf)
if tf, s = 'SI'; else, s = 'NO'; end
end
