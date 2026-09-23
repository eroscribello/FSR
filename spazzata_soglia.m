%% spazzata_soglia.m - la soglia di contatto di C2 conta?   [DOPO LA CAMPAGNA]
%
% LA DOMANDA
%   Con le inerzie corrette C2 esce peggio di C1 su dosso (T4D) e ostacolo
%   (T5): rolla di piu' sugli spigoli e sbanda. Prima era il contrario.
%   Lo svantaggio e' di TARATURA o e' STRUTTURALE?
%
% PERCHE' PROPRIO LA SOGLIA
%   Dei sei parametri di C2, cinque non dipendono dalle inerzie: v_search,
%   z_ext_max, z_nom e tol sono comandi in posizione e geometria, t_reset e'
%   un tempo. L'unico che confronta COPPIE MISURATE con numeri fissi e' la
%   soglia del flag di contatto. Correggendo le inerzie le coppie sono
%   crollate, quindi quella soglia non significa piu' quello che significava.
%
% [23/9] LA SOGLIA E' UN VETTORE, E IL PRIMO GIRO DI QUESTA SPAZZATA ERA NULLO
%   La prima versione spazzava cfg.c2.soglia_tau = 0.5 e le tre run uscivano
%   IDENTICHE alla quindicesima cifra: c2_soglia era calcolata da init_gait e
%   nessun blocco la leggeva. Cercandola nel modello e' saltato fuori che il
%   confronto e' fra un Mux di tre coppie e il Constant [0.04 0.1 0.1], cioe'
%   una soglia PER GIUNTO, e che il flag e'
%       cont = OR( |tau_misurata - tau_attesa| > soglia )  sui tre giunti.
%   Adesso quel Constant vale c2_soglia e cfg porta il valore vero.
%   Si spazza quindi un MOLTIPLICATORE del vettore, non un valore in N*m:
%   spazzare tre soglie indipendenti richiederebbe 27 run per dire la stessa
%   cosa.
%
% I CRITERI, SCRITTI PRIMA DI LANCIARE
%   1. LA SOGLIA CONTA se fra 0.5x e 2x almeno una fra roll_max, dev_lat_max
%      e cot cambia di piu' del 20%. Se non cambia NULLA (righe identiche a
%      molte cifre) il collegamento e' di nuovo rotto: non e' un risultato,
%      e' un esperimento fallito.
%   2. LO SVANTAGGIO E' DI TARATURA se a qualche moltiplicatore C2 rientra
%      entro il 20% di C1 (riga di campagna, results/T4D_C1.csv e T5_C1.csv)
%      sia su dev_lat_max sia su roll_max.
%   3. Se la 1 e' falsa, oppure la 1 e' vera ma la 2 e' falsa a tutti e tre i
%      moltiplicatori, lo svantaggio di C2 e' STRUTTURALE: si dichiara e non
%      si ritara.
%
% COSA NON TOCCA
%   Non sovrascrive nessun CSV di campagna e nessuna figura: scrive solo
%   results/diagnostica/spazzata_soglia.csv. Nessun save_system: le inerzie
%   corrette e la soglia stanno in memoria, il .slx resta quello del collega.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   spazzata_soglia
%
% Progetto FSR PhantomX - A. Russo

sp_cfg  = phantomx_config();
sp_mdl  = 'phantomx_sim_zero';
sp_molt = [0.5 1 2];                   % moltiplicatori: 1 = valore di campagna
sp_task = {'T4D', 30                   % task, durata [s] (come script_T4D)
           'T5',  20};                 %                  (come script_T5)
sp_xbordo = 4 - 0.05;                  % bordo del pavimento 8x8, come in T4D

sp_base = reshape(sp_cfg.c2.soglia_tau, 1, []);
if isscalar(sp_base), sp_base = repmat(sp_base, 1, 3); end
fprintf('\nsoglia di riferimento (cfg.c2.soglia_tau) = %s N*m\n', mat2str(sp_base, 4));

if bdIsLoaded(sp_mdl) && strcmp(get_param(sp_mdl,'Dirty'),'on')
    error('spazzata_soglia:dirty', ['Il modello e'' aperto con modifiche non ' ...
        'salvate: salvale o chiudilo (bdclose all) prima.']);
end

SP = table();
sp_crono = tic;

try
    for sp_i = 1:size(sp_task,1)
        sp_t   = sp_task{sp_i,1};
        sp_dur = sp_task{sp_i,2};

        applica_terreno(sp_t, false, sp_mdl);
        applica_inerzie(sp_mdl);

        for sp_j = 1:numel(sp_molt)
            sp_s = sp_molt(sp_j) * sp_base;
            fprintf('\n---- %s, C2, soglia %.2fx = %s ----\n', sp_t, ...
                    sp_molt(sp_j), mat2str(sp_s, 4));

            OVERRIDE_C2        = true;                         %#ok<NASGU>
            OVERRIDE_C2_SOGLIA = sp_s;                         %#ok<NASGU>
            clear OVERRIDE_GAIT                                % andatura nominale
            init_gait

            sp_out = sim(sp_mdl, 'StopTime', num2str(sp_dur));
            sp_run = adatta_simscape(sp_out, struct( ...
                         'controller','C2', 'task',sp_t, 'run',1, ...
                         'condizione', sprintf('soglia%.2fx', sp_molt(sp_j)), ...
                         'vel_d', [sp_cfg.v_nom 0]));

            % T4D: la run si taglia al bordo del pavimento PRIMA delle
            % metriche, come in script_T4D. Senza, la caduta dal bordo entra
            % in tau_max e cot e la spazzata confronta cadute, non tarature.
            sp_bordo = NaN;
            if strcmp(sp_t,'T4D')
                [sp_run, sp_bordo] = sp_taglia(sp_run, sp_xbordo);
                if ~isnan(sp_bordo)
                    fprintf(2,'  un piede supera il bordo a t = %.2f s: run tagliata li''.\n', sp_bordo);
                end
            end

            sp_riga = metriche(sp_run, sp_cfg, struct('t_regime', 2*sp_cfg.T));
            sp_riga.molt       = sp_molt(sp_j);
            sp_riga.soglia_coxa = sp_s(1);
            sp_riga.soglia_fem  = sp_s(2);
            sp_riga.soglia_tib  = sp_s(3);
            sp_riga.t_bordo     = sp_bordo;
            SP = [SP; sp_riga];                                %#ok<AGROW>
        end
    end
    clear OVERRIDE_C2 OVERRIDE_C2_SOGLIA
    init_gait                                                  % ripristina cfg
catch sp_err
    clear OVERRIDE_C2 OVERRIDE_C2_SOGLIA
    fprintf(2,'\n  Errore: il modello NON e'' stato salvato, niente e'' andato perso.\n');
    rethrow(sp_err);
end

if ~isfolder(fullfile('results','diagnostica')), mkdir(fullfile('results','diagnostica')); end
writetable(SP, fullfile('results','diagnostica','spazzata_soglia.csv'));

%% ---- lettura ----
sp_col = {'roll_max','pitch_max','dev_lat_max','yaw_err_fin','cot','tau_max', ...
          'z_media','frazione_task'};

fprintf('\n\n================ SPAZZATA SULLA SOGLIA DI C2 ================\n');
for sp_i = 1:size(sp_task,1)
    sp_t  = sp_task{sp_i,1};
    sp_S  = SP(strcmp(string(SP.task), sp_t), :);
    sp_C1 = sp_leggi_c1(sp_t);

    fprintf('\n--- %s ---\n', sp_t);
    fprintf('  %-14s', 'moltiplicatore');
    fprintf('%12.2f', sp_S.molt);
    fprintf('%14s\n', 'C1 campagna');
    for sp_k = 1:numel(sp_col)
        sp_c = sp_col{sp_k};
        if ~ismember(sp_c, sp_S.Properties.VariableNames), continue; end
        fprintf('  %-14s', sp_c);
        fprintf('%12.4g', sp_S.(sp_c));
        if isempty(sp_C1) || ~ismember(sp_c, sp_C1.Properties.VariableNames)
            fprintf('%14s\n', '-');
        else
            fprintf('%14.4g\n', sp_C1.(sp_c)(1));
        end
    end

    % criterio 1: la soglia conta?
    sp_conta = false;
    for sp_cc = {'roll_max','dev_lat_max','cot'}
        sp_v = sp_S.(sp_cc{1});
        if min(abs(sp_v)) > 0 && (max(sp_v) - min(sp_v)) / min(abs(sp_v)) > 0.20
            sp_conta = true;
            fprintf('  -> %s varia del %.0f%% fra le soglie\n', sp_cc{1}, ...
                    100*(max(sp_v)-min(sp_v))/min(abs(sp_v)));
        end
    end
    fprintf('  1. la soglia conta:            %s\n', sp_si(sp_conta));

    % criterio 2: a qualche soglia C2 rientra su C1?
    sp_rec = false;  sp_qual = NaN;
    if ~isempty(sp_C1)
        for sp_j = 1:height(sp_S)
            sp_a = abs(sp_S.dev_lat_max(sp_j)) <= 1.2*abs(sp_C1.dev_lat_max(1));
            sp_b = abs(sp_S.roll_max(sp_j))    <= 1.2*abs(sp_C1.roll_max(1));
            if sp_a && sp_b, sp_rec = true; sp_qual = sp_S.molt(sp_j); end
        end
    end
    fprintf('  2. C2 rientra entro il 20%% di C1: %s', sp_si(sp_rec));
    if sp_rec, fprintf('  (a %.2fx la soglia di campagna)', sp_qual); end
    fprintf('\n');

    if sp_conta && sp_rec
        fprintf('  => svantaggio di C2 DI TARATURA su %s\n', sp_t);
    else
        fprintf('  => svantaggio di C2 STRUTTURALE su %s: si dichiara, non si ritara\n', sp_t);
    end
end

fprintf('\n  scritto  results\\diagnostica\\spazzata_soglia.csv   (%.0f s)\n', toc(sp_crono));
fprintf('  nessun CSV di campagna toccato, modello non salvato.\n\n');

%% ================= helper =================
function [r, t_bordo] = sp_taglia(r, x_bordo)
%SP_TAGLIA  Tronca la run al primo campione con un piede oltre x_bordo.
% Stessa logica di t4d_taglia_bordo in script_T4D: il pavimento e' il cubo
% 8 x 8 (x in [-4, 4]), cfg.floor_dim non lo descrive.
t_bordo = NaN;
N = numel(r.t);
k = find(any(r.pf(:, 1:3:18) > x_bordo, 2), 1, 'first');
if isempty(k), return; end
t_bordo = r.t(k);
f = fieldnames(r);
for i = 1:numel(f)
    v = r.(f{i});
    if (isnumeric(v) || islogical(v)) && size(v,1) == N && N > 1
        r.(f{i}) = v(1:k-1, :);
    end
end
end

function T = sp_leggi_c1(task)
%SP_LEGGI_C1  La riga di C1 della campagna, come termine di paragone.
T = [];
f = fullfile('results', sprintf('%s_C1.csv', task));
if isfile(f), T = readtable(f); end
end

function s = sp_si(tf)
if tf, s = 'SI'; else, s = 'NO'; end
end
