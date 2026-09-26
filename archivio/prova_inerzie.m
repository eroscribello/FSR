%% prova_inerzie.m - quanto pesano le inerzie del modello sulle misure?  [SOLO PROVA]
%
% COSA FA
%   Per ogni prova in pi_prove (task + controllore), due run nella stessa
%   sessione:
%     A. modello COM'E'                (inerzie dell'URDF, ~1000x troppo grandi)
%     B. inerzie CORRETTE in memoria   (valori di phantomx_config)
%   e stampa il confronto delle metriche principali.
%
%   [22/9] Prima prova: T2 1x, C1 -> tau_max -5%, energia e cot -62%,
%   moto quasi identico (results/prova_inerzie.csv).
%   Seconda prova (default ora): T6, C2 e C1. Verifica se il tau_max di C2
%   sugli ostacoli (26.5 N*m, 18x datasheet) e il suo cot +35% rispetto a C1
%   reggono anche con le inerzie corrette.
%   ATTENZIONE nel leggere T6: con C1 deviazione e imbardata sono sensibili
%   alla singola run (visto in T4D). Le differenze A/B su quelle colonne non
%   si attribuiscono alle inerzie; le colonne da guardare sono tau, energia,
%   cot e assetto massimo.
%
% COSA NON FA - il lavoro fatto non si tocca
%   - NON salva il .slx: nessuna chiamata a save_system.
%   - Le inerzie originali vengono lette, messe da parte e RIMESSE alla fine,
%     anche se la run va in errore (try/catch). Poi il modello viene chiuso
%     SENZA salvare: alla prossima apertura e' quello del disco, identico a prima.
%   - Per poterlo chiudere senza salvare in sicurezza, all'avvio controlla che
%     il modello non abbia modifiche tue non salvate; se ne ha, si ferma.
%   - Non sovrascrive nessun CSV di campagna: scrive results/prova_inerzie_<task>.csv.
%     Le run di T6 NON passano da script_T6 (che scriverebbe results/T6_*.csv):
%     ne replicano solo la parte di simulazione, stesso terreno e durata.
%
% I VALORI CORRETTI
%   Sono quelli gia' presenti in phantomx_config (cfg.I_body, I_c1, I_c2,
%   I_thigh, I_tibia), prodotti d'inerzia a zero. Per il corpo tornano con il
%   conto a mano di un parallelepipedo da ~1 kg e 25 x 20 cm (I_zz ~ 8.6e-3).
%   Le masse NON cambiano.
%
% COSA MI ASPETTO (scritto prima di lanciare)
%   - movimento delle zampe IDENTICO: in C1 i giunti sono comandati in
%     posizione, il simulatore impone la traiettoria qualunque sia l'inerzia.
%     E' il motivo per cui l'andatura in animazione sembra giusta;
%   - coppie (tau_max, tau_rms) molto PIU' BASSE in B: quelle che il modello
%     registra sono le coppie necessarie a muovere zampe ~1000x piu' inerti;
%   - assetto del corpo (roll/pitch) DIVERSO, probabilmente piu' ampio in B:
%     un corpo meno inerte risponde di piu' alle spinte delle zampe;
%   - velocita' e distanza quasi uguali (le impone la cinematica).
%   Se le coppie non cambiano, l'ipotesi e' sbagliata e il modello va bene
%   com'e': lo si scrive e si va avanti.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   prova_inerzie
%
% Progetto FSR PhantomX - A. Russo

pi_cfg = phantomx_config();
pi_mdl = 'phantomx_sim_zero';

% prove: {task, c2}. task 'T2' = piano a 1x (10 cicli), 'T6' = sette ostacoli, 30 s
pi_prove = {'T6', true;  'T6', false};

%% ---- 0. sicurezza: niente modifiche non salvate ----
if bdIsLoaded(pi_mdl) && strcmp(get_param(pi_mdl,'Dirty'),'on')
    error('prova_inerzie:dirty', ['Il modello e'' aperto con modifiche non salvate.\n' ...
        'Salvale o chiudilo (bdclose all) prima: questa prova chiude il modello ' ...
        'senza salvare, e non deve portarsi via il tuo lavoro.']);
end
load_system(pi_mdl);

%% ---- 1. trova i 25 solidi con inerzia personalizzata ----
pi_S = pi_solidi(pi_mdl, pi_cfg);
if numel(pi_S) ~= 25
    error('prova_inerzie:solidi', ['Trovati %d solidi con inerzia, ne attendevo 25 ' ...
        '(corpo + 24 link). Mi fermo senza simulare.'], numel(pi_S));
end
fprintf('\n%d solidi trovati. Inerzie attuali -> corrette:\n', numel(pi_S));
for pi_k = [1 2 numel(pi_S)]
    fprintf('  %-26s %-32s -> %s\n', pi_S(pi_k).nome, pi_S(pi_k).I_orig, pi_S(pi_k).I_nuova);
end

%% ---- 2. le due run ----
% Tutto dentro try: se qualcosa va in errore, il catch rimette le inerzie
% originali e chiude senza salvare PRIMA di mostrare l'errore. (onCleanup non
% basta: in uno script non scatta sull'errore, perche' il workspace resta.)
try
pi_tab = table();
for pi_p = 1:size(pi_prove,1)
    pi_task = pi_prove{pi_p,1};   pi_c2 = pi_prove{pi_p,2};
    pi_ctrl = 'C1';  if pi_c2, pi_ctrl = 'C2'; end
    pi_cfgLoc = pi_cfg;
    if strcmp(pi_task,'T2')
        pi_T    = pi_cfg.S / (pi_cfg.beta_stance * pi_cfg.v_nom);   % come script_T2 a 1x
        pi_stop = 10 * pi_T;
        pi_ov   = struct('T', pi_T);
        pi_cond = 'v1.00x';
        pi_cfgLoc.T = pi_T;
    else                                                          % come script_T6
        pi_stop = 30;
        pi_ov   = [];
        pi_cond = 'ost1-7';
    end
    applica_terreno(pi_task, false, pi_mdl);

    for pi_caso = ["A_originali", "B_corrette"]
        for pi_k = 1:numel(pi_S)
            if pi_caso == "A_originali"
                set_param(pi_S(pi_k).blocco, 'MomentsOfInertia', pi_S(pi_k).I_orig, ...
                                             'ProductsOfInertia', pi_S(pi_k).P_orig);
            else
                set_param(pi_S(pi_k).blocco, 'MomentsOfInertia', pi_S(pi_k).I_nuova, ...
                                             'ProductsOfInertia', '[0 0 0]');
            end
        end
        fprintf('\n---- %s %s, run %s ----\n', pi_task, pi_ctrl, pi_caso);

        OVERRIDE_C2 = pi_c2;                                   %#ok<NASGU>
        if isempty(pi_ov), clear OVERRIDE_GAIT; else, OVERRIDE_GAIT = pi_ov; end %#ok<NASGU>
        init_gait
        pi_out = sim(pi_mdl, 'StopTime', num2str(pi_stop));
        pi_run = adatta_simscape(pi_out, struct('controller',pi_ctrl, 'task',pi_task, ...
                     'run',1, 'condizione', pi_cond, 'vel_d', [pi_cfg.v_nom 0]));
        pi_riga = metriche(pi_run, pi_cfgLoc, struct('t_regime', 2*pi_cfgLoc.T));
        pi_riga.caso  = pi_caso;
        pi_riga.prova = string(sprintf('%s_%s', pi_task, pi_ctrl));
        pi_tab = [pi_tab; pi_riga];                            %#ok<AGROW>
    end
end
clear OVERRIDE_GAIT OVERRIDE_C2
init_gait
catch pi_err
    pi_ripristina(pi_mdl, pi_S);
    clear OVERRIDE_GAIT OVERRIDE_C2
    fprintf(2, '\n  Errore: inerzie originali rimesse, modello chiuso SENZA salvare.\n');
    rethrow(pi_err);
end
pi_ripristina(pi_mdl, pi_S);

%% ---- 3. confronto ----
if ~isfolder('results'), mkdir('results'); end
pi_file = fullfile('results', sprintf('prova_inerzie_%s.csv', strjoin(unique(pi_prove(:,1)).', '_')));
writetable(pi_tab, pi_file);

pi_col = {'tau_max','N*m',1; 'tau_rms','N*m',1; 'frazione_saturo','%',100; ...
          'energia','J',1; 'cot','-',1; 'potenza_max','W',1; ...
          'roll_max','deg',180/pi; 'pitch_max','deg',180/pi; 'roll_rms','deg',180/pi; ...
          'pitch_rms','deg',180/pi; 'corpoZ_pp','mm',1e3; 'vel_media','m/s',1; ...
          'distanza','m',1; 'dev_lat_max','m',1; 'yaw_err_fin','deg',180/pi};
for pi_pr = unique(pi_tab.prova).'
    A = pi_tab(pi_tab.prova == pi_pr & pi_tab.caso == "A_originali", :);
    B = pi_tab(pi_tab.prova == pi_pr & pi_tab.caso == "B_corrette",  :);
    fprintf('\n=============== INERZIE: %s, originali contro corrette ===============\n', pi_pr);
    fprintf('  %-16s %6s %12s %12s %10s\n', 'metrica', '', 'originali', 'corrette', 'B / A');
    for pi_k = 1:size(pi_col,1)
        c = pi_col{pi_k,1};
        if ~ismember(c, pi_tab.Properties.VariableNames), continue; end
        a = A.(c) * pi_col{pi_k,3};   b = B.(c) * pi_col{pi_k,3};
        fprintf('  %-16s %6s %12.4g %12.4g %10.3g\n', c, pi_col{pi_k,2}, a, b, b/a);
    end
    fprintf('  esito: %s  ->  %s\n', string(A.causa_fallimento), string(B.causa_fallimento));
end
% il rapporto C2/C1 del cot, il numero che sta in relazione
if all(ismember(["T6_C1","T6_C2"], pi_tab.prova))
    g = @(pr,cs) pi_tab.cot(pi_tab.prova == pr & pi_tab.caso == cs);
    fprintf('\n  cot C2/C1 in T6:  originali %.2f   corrette %.2f\n', ...
        g("T6_C2","A_originali")/g("T6_C1","A_originali"), ...
        g("T6_C2","B_corrette")/g("T6_C1","B_corrette"));
end
fprintf('\n  scritto  %s\n', pi_file);
fprintf('  inerzie originali rimesse, modello chiuso SENZA salvare.\n\n');

%% ================= helper =================
function S = pi_solidi(mdl, cfg)
%PI_SOLIDI  I solidi con inerzia 'Custom' e massa > 0, con il valore corretto.
S = struct('blocco',{},'nome',{},'I_orig',{},'P_orig',{},'I_nuova',{});
bb = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Type','block');
for i = 1:numel(bb)
    try
        tipo = get_param(bb{i}, 'InertiaType');
        I    = get_param(bb{i}, 'MomentsOfInertia');
        m    = str2double(get_param(bb{i}, 'Mass'));
    catch
        continue
    end
    if ~strcmp(tipo,'Custom') || ~(m > 0), continue; end
    padre = regexprep(get_param(get_param(bb{i},'Parent'),'Name'), '\s+', ' ');
    if     startsWith(padre,'MP_BODY'), In = cfg.I_body;
    elseif startsWith(padre,'c1_'),     In = cfg.I_c1;
    elseif startsWith(padre,'c2_'),     In = cfg.I_c2;
    elseif startsWith(padre,'thigh_'),  In = cfg.I_thigh;
    elseif startsWith(padre,'tibia_'),  In = cfg.I_tibia;
    else
        continue                         % non e' un link del robot: non si tocca
    end
    S(end+1) = struct('blocco', bb{i}, 'nome', padre, 'I_orig', I, ...
        'P_orig', get_param(bb{i},'ProductsOfInertia'), ...
        'I_nuova', mat2str(In, 6));                          %#ok<AGROW>
end
% il corpo per primo, per la stampa
[~, o] = sort(~startsWith({S.nome}, 'MP_BODY'));  S = S(o);
end

function pi_ripristina(mdl, S)
%PI_RIPRISTINA  Rimette le inerzie originali e chiude il modello senza salvare.
if ~bdIsLoaded(mdl), return; end
for k = 1:numel(S)
    try
        set_param(S(k).blocco, 'MomentsOfInertia', S(k).I_orig, ...
                               'ProductsOfInertia', S(k).P_orig);
    catch
    end
end
close_system(mdl, 0);                    % 0 = NON salvare
end
