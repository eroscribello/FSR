%% script_T4_limite.m - T4: angolo limite di salita, C1 e C2  [impianto SIMSCAPE]
%
% COSA MISURA
%   La pendenza massima che ciascun controllore riesce a salire. "Riesce" e'
%   il criterio dichiarato in testa a script_T4 (tutte le zampe sulla rampa,
%   due cicli a regime, velocita' almeno meta' di quella in piano, assetto
%   entro 30 gradi rispetto alla rampa). Deciso prima di lanciare la ricerca.
%
% COME
%   1. griglia grossa: 10, 15, 20, ... gradi, fino al primo fallimento;
%   2. bisezione fra l'ultimo angolo riuscito e il primo fallito, fino a una
%      forchetta di lim_risoluzione gradi.
%   La bisezione ASSUME che sopra un fallimento si fallisca sempre. Se non
%   fosse cosi' (es. un angolo che fallisce per un caso e uno piu' ripido che
%   passa) la griglia grossa lo mostrerebbe solo in parte: per questo lo
%   script continua la griglia per UN passo oltre il primo fallimento, e se
%   quello passa lo dice.
%
% DUE LIMITI, COME PER LA VELOCITA' IN T2
%   - limite del BANCO: quello del criterio, con attuatori ideali;
%   - limite REALISTICO: la pendenza oltre cui tau_max supera il datasheet
%     dell'AX-12A (cfg.tau_max). Si legge dalla tabella, non richiede run in
%     piu'. Gia' a 8 gradi C1 e' a 2.7 N*m e C2 a 13.7, contro 1.5: e' probabile
%     che il limite realistico stia sotto la prima cella per entrambi, e va
%     scritto cosi'.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   script_T4_limite
%   Ogni run e' una normale run di script_T4 (20 s simulati). Le righe
%   singole finiscono in results/T4_limite/, la tabella riassuntiva in
%   results/T4_limite_C1.csv e results/T4_limite_C2.csv.
%
% Progetto FSR PhantomX - A. Russo

lim_controllori = [false true];      % C1, C2. Per uno solo: [false] o [true]
lim_griglia     = 10:5:45;           % [gradi] griglia grossa
lim_risoluzione = 1;                 % [gradi] ampiezza finale della forchetta
lim_cfg         = phantomx_config();

for lim_c2 = lim_controllori
    lim_nome = 'C1';  if lim_c2, lim_nome = 'C2'; end
    lim_tab  = table();
    lim_ok   = 0;          % ultimo angolo riuscito
    lim_ko   = inf;        % primo angolo fallito
    lim_oltre = NaN;       % un passo di griglia oltre il primo fallimento
    lim_oltre_ok = false;

    % ---- 1. griglia grossa ----
    for lim_g = lim_griglia
        [lim_riga, lim_esito] = lim_una_run(lim_c2, lim_g);
        lim_tab = [lim_tab; lim_riga];                                   %#ok<AGROW>
        if isinf(lim_ko)
            if lim_esito, lim_ok = lim_g; else, lim_ko = lim_g; end
        else
            lim_oltre = lim_g;  lim_oltre_ok = lim_esito;   % il passo di controllo
            break
        end
    end

    % ---- 2. bisezione ----
    lim_nonmono = lim_oltre_ok;
    if isinf(lim_ko)
        fprintf(2, '\n%s sale anche a %g gradi: allarga lim_griglia.\n', lim_nome, lim_griglia(end));
    elseif lim_nonmono
        fprintf(2, ['\n%s: fallisce a %g gradi ma sale a %g. Il limite non e'' monotono: ' ...
                    'niente bisezione, guarda le run una per una.\n'], lim_nome, lim_ko, lim_oltre);
    else
        while lim_ko - lim_ok > lim_risoluzione
            lim_g = round(2*(lim_ok + lim_ko)/2)/2;       % al mezzo grado
            if lim_g <= lim_ok || lim_g >= lim_ko, break; end
            [lim_riga, lim_esito] = lim_una_run(lim_c2, lim_g);
            lim_tab = [lim_tab; lim_riga];                               %#ok<AGROW>
            if lim_esito, lim_ok = lim_g; else, lim_ko = lim_g; end
        end
    end

    lim_tab = sortrows(lim_tab, 'gradi');
    writetable(lim_tab, fullfile('results', sprintf('T4_limite_%s.csv', lim_nome)));

    % ---- riepilogo ----
    fprintf('\n=============== T4 LIMITE - %s ===============\n', lim_nome);
    fprintf('  %6s  %-6s  %8s  %8s  %8s  %s\n', 'gradi', 'salito', 'v_rapp', 'tau_max', ...
            'roll_tr', 'causa');
    for lim_k = 1:height(lim_tab)
        fprintf('  %6.1f  %-6s  %8.2f  %8.2f  %8.2f  %s\n', lim_tab.gradi(lim_k), ...
            lim_si(lim_tab.salito(lim_k)), lim_tab.v_rapporto(lim_k), lim_tab.tau_max(lim_k), ...
            rad2deg(lim_tab.roll_esc_trans(lim_k)), char(lim_tab.causa_salita(lim_k)));
    end
    if ~isinf(lim_ko) && ~lim_nonmono
        fprintf('\n  limite del banco: sale a %g gradi, non a %g\n', lim_ok, lim_ko);
    end
    lim_real = lim_tab.gradi(lim_tab.salito & lim_tab.tau_max <= lim_cfg.tau_max);
    if isempty(lim_real)
        fprintf('  limite realistico (tau_max <= %.1f N*m): sotto la prima cella provata\n', ...
                lim_cfg.tau_max);
    else
        fprintf('  limite realistico (tau_max <= %.1f N*m): %g gradi\n', lim_cfg.tau_max, max(lim_real));
    end
end

%% ================= helper =================
function [riga, esito] = lim_una_run(c2, gradi)
%LIM_UNA_RUN  Una run di script_T4 nel workspace base, senza grafico.
% script_T4 e' uno script e usa il workspace base (init_gait, OVERRIDE_*):
% lo si lancia li', non dentro questa funzione.
assignin('base', 'OVERRIDE_T4', struct('c2', c2, 'gradi', gradi, 'grafico', false));
try
    evalin('base', 'script_T4');
    riga  = evalin('base', 't4_riga');
    esito = riga.salito;
catch ME
    % un errore del solutore su una rampa ripida e' un fallimento, ma diverso
    % da quelli del criterio: resta scritto com'e'
    evalin('base', 'clear OVERRIDE_T4 OVERRIDE_C2');
    riga = table(gradi, false, "errore di simulazione: " + string(ME.message), ...
                 NaN, NaN, NaN, 'VariableNames', ...
                 {'gradi','salito','causa_salita','v_rapporto','tau_max','roll_esc_trans'});
    esito = false;
    fprintf(2, '  %g gradi: errore di simulazione (%s)\n', gradi, ME.message);
end
% nella tabella riassuntiva bastano le colonne che servono a leggere il limite;
% le righe complete sono in results/T4_limite/
tieni = {'gradi','salito','causa_salita','v_rapporto','tau_max','roll_esc_trans'};
extra = {'pendenza_mis','incl_err','incl_pp','roll_esc_rampa','dh_corpo','frazione_saturo', ...
         'cot','t_tutti_su'};
for k = 1:numel(extra)
    if ~ismember(extra{k}, riga.Properties.VariableNames), riga.(extra{k}) = NaN; end
end
riga = riga(:, [tieni extra]);
end

function s = lim_si(b)
if b, s = 'si'; else, s = 'NO'; end
end
