%% s0_vuoto.m - il modello e' rotto, o e' il robot che cade?
%
% LA DOMANDA
%   [24/9] "degenerate mass distribution on its follower side" del 6-DOF Joint
%   compare in OGNI tentativo di S0: coppia analitica, coppia misurata,
%   gradino, rampa, con e senza smorzamento, con tutti e due i segni. Cinque
%   ingressi diversi, sempre lo stesso errore. Allora l'errore non dipende
%   dall'ingresso, e continuare a cambiare l'ingresso e' tempo buttato.
%
%   Restano due spiegazioni, e sono opposte:
%     (A) la copia e' strutturalmente rotta - liberando i 18 giunti resta un
%         corpo o una catena con distribuzione di massa singolare, e il
%         solutore non puo' risolvere comunque, con qualunque forza;
%     (B) la copia e' sana, ma il robot in anello aperto CADE, le zampe
%         finiscono in configurazione singolare e la matrice di massa
%         degenera li'. In questo caso il messaggio non e' un difetto del
%         modello: e' la fotografia del robot che collassa.
%
% COME SI SEPARANO, IN UNA PROVA SOLA
%   Si toglie la gravita' e si mette coppia zero. Senza gravita' e senza
%   coppia non c'e' niente che possa muovere il robot: se il modello e' sano
%   deve restare immobile per tutta la run, e la simulazione deve arrivare
%   in fondo.
%     - arriva in fondo  -> (B): il modello e' sano. Il messaggio arriva
%       DOPO che il robot e' collassato, ed e' un sintomo, non la causa.
%       Conseguenza: S0 in anello aperto puro non e' un test che puo'
%       passare, e va ridefinito.
%     - muore lo stesso   -> (A): il modello e' degenere di suo, e nessuna
%       coppia lo salvera'. Si torna a crea_modello_mpc.
%   E' una prova che non puo' dare un risultato ambiguo: e' per questo che
%   viene prima di qualunque altra modifica.
%
% LA SECONDA PROVA
%   Rimessa la gravita', sempre a coppia zero: il robot deve accasciarsi.
%   Serve solo a leggere il tempo di morte "di riferimento", da confrontare
%   con quello delle prove con coppia. Se con la coppia statica il robot
%   sopravvive piu' a lungo che senza, la coppia sta facendo il suo lavoro
%   anche se non basta.
%
% SOLO IN MEMORIA
%   Gravita' e coppia vengono scritte con set_param e ripristinate all'uscita.
%   Nessun save_system: ne' sulla copia ne', ovviamente, sul modello base.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   s0_vuoto
%
% Progetto FSR PhantomX - A. Russo

function info = s0_vuoto(durata)

if nargin < 1 || isempty(durata), durata = 3.0; end

sv_mdl = 'phantomx_sim_mpc';
if ~bdIsLoaded(sv_mdl), load_system(sv_mdl); end

%% ---- il blocco della gravita' ----
sv_mc = find_system(sv_mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                    'MaskType','Mechanism Configuration');
if numel(sv_mc) ~= 1
    error('s0_vuoto:mc', 'Mechanism Configuration trovati: %d.', numel(sv_mc));
end
sv_mc = sv_mc{1};

sv_np = fieldnames(get_param(sv_mc, 'DialogParameters'));
if any(strcmp(sv_np, 'GravityVector'))
    sv_pg = 'GravityVector';
else
    c = sv_np(contains(sv_np,'Gravity','IgnoreCase',true) & ...
              ~contains(sv_np,'Unit','IgnoreCase',true) & ~endsWith(sv_np,'_conf'));
    if numel(c) ~= 1
        fprintf(2, '  Parametri con "Gravity": %s\n', ...
                strjoin(sv_np(contains(sv_np,'Gravity','IgnoreCase',true)), ', '));
        error('s0_vuoto:gravita', 'Parametro gravita'' non identificato (%d).', numel(c));
    end
    sv_pg = c{1};
end

sv_g0 = get_param(sv_mc, sv_pg);
sv_ripristina = onCleanup(@() set_param(sv_mc, sv_pg, sv_g0));
fprintf('\n=========== S0 A VUOTO ===========\n');
fprintf('  gravita'' originale (%s): %s\n', sv_pg, sv_g0);

%% ---- coppia zero, una volta per tutte ----
assignin('base', 'tau_ts', timeseries(zeros(2,18), [0; durata]));

%% ---- le due prove ----
esiti = cell(2,4);
for p = 1:2
    if p == 1
        set_param(sv_mc, sv_pg, '[0 0 0]');
        eti = 'senza gravita'', coppia zero';
    else
        set_param(sv_mc, sv_pg, sv_g0);
        eti = 'con gravita'',   coppia zero';
    end

    applica_terreno('T1', false, sv_mdl);
    applica_inerzie(sv_mdl);

    fprintf('\n--- prova %d: %s ---\n', p, eti);
    t_fin = NaN;  msg = '';
    try
        out = sim(sv_mdl, 'StopTime', num2str(durata));
        r   = adatta_simscape(out, struct('controller','C3', 'task','S0', 'run',p, ...
                                          'condizione','vuoto', 'vel_d',[0 0]));
        t_fin = r.t(end);
        fprintf('  arrivata a %.3f s su %.1f;  quota %.1f -> %.1f mm\n', ...
                t_fin, durata, 1e3*r.p(1,3), 1e3*r.p(end,3));
    catch ME
        msg = ME.message;
        fprintf(2, '  ERRORE: %s\n', regexprep(msg, '\s+', ' '));
    end
    esiti(p,:) = {p, eti, t_fin, msg};
end

%% ---- il verdetto ----
t1 = esiti{1,3};
fprintf('\n=========== VERDETTO ===========\n');
if ~isnan(t1) && t1 >= durata - 1e-6
    fprintf(['  (B) IL MODELLO E'' SANO.\n' ...
             '  Senza gravita'' e senza coppia la copia sta ferma per %.1f s\n' ...
             '  senza un solo messaggio. Quindi la distribuzione di massa non\n' ...
             '  e'' degenere di suo: lo diventa DOPO, quando il robot collassa\n' ...
             '  e una zampa raggiunge una configurazione singolare.\n' ...
             '  Conseguenza: S0 in anello aperto puro non e'' un test che puo''\n' ...
             '  passare. Tenere in piedi un esapode con sole coppie costanti e''\n' ...
             '  un equilibrio instabile: va aggiunta la retroazione, cioe'' si\n' ...
             '  passa a S1 invece di insistere su S0.\n'], durata);
else
    fprintf(2, ['  (A) IL MODELLO E'' DEGENERE.\n' ...
                '  Senza gravita'' e senza coppia non c''e'' niente che possa\n' ...
                '  muovere il robot, eppure la simulazione si ferma a %.3f s.\n' ...
                '  Non e'' il robot che cade: e'' la copia a essere costruita male.\n' ...
                '  Si torna a crea_modello_mpc, e la prossima prova e'' liberare\n' ...
                '  UNA zampa sola invece di sei per capire dove.\n'], t1);
end

fprintf('\n  riferimento: con gravita'' e coppia zero la run arriva a %.3f s\n', esiti{2,3});
fprintf('  (le prove con la coppia statica morivano a 0.07 - 0.22 s)\n\n');

info = cell2table(esiti, 'VariableNames', {'prova','condizione','t_finale','errore'});
end
