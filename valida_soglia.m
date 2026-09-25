function [VS, dati] = valida_soglia(opt)
%VALIDA_SOGLIA  La soglia di contatto di C2 legge davvero il contatto?
%
%   valida_soglia
%   VS = valida_soglia(struct('task','T2','molt',[0.25 0.5 1 2 4]))
%   [VS, dati] = valida_soglia    % dati.D, dati.vero, dati.appoggio, dati.volo
%                                 % per rifare i conti senza risimulare
%   valida_soglia(struct('inerzie',false))          % al punto di lavoro pre-22/9
%   valida_soglia(struct('soglia',[0.385 0.136 0.105]))  % prova una terna nuova
%
% LA DOMANDA, E PERCHE' NON E' QUELLA DI spazzata_soglia
%   cfg.c2.soglia_tau = [0.04 0.1 0.1] non e' mai stato validato: il valore
%   e' quello che sta nel Constant del modello, non il risultato di una
%   misura. Finche' resta cosi', ogni difetto di C2 - i piedi che restano per
%   aria sul gradino di T6, il rollio sugli spigoli di T5 - puo' essere del
%   controllore oppure soltanto di una soglia tarata male, e non sappiamo
%   distinguere.
%
%   spazzata_soglia.m risponde a "cambiando la soglia cambiano le METRICHE?"
%   e per farlo rifa' sei simulazioni. Qui la domanda e' a monte e diversa:
%   "il flag di contatto che quella soglia produce corrisponde al contatto
%   VERO?". Questa si risponde con UNA sola run, perche' nel log ci sono
%   insieme le coppie (torque_sens, torque_estim) e le forze ai piedi
%   (Fleg): il flag si ricalcola offline per qualunque soglia e si confronta
%   con la forza normale, che e' la verita'.
%
%   Le due prove sono complementari e vanno lette in quest'ordine: prima si
%   sa se la soglia legge il contatto, poi si guarda cosa ne consegue.
%
% IL LIMITE, DETTO PRIMA E NON DOPO
%   La ricostruzione e' ad ANELLO APERTO. Cambiando soglia offline cambia il
%   flag, ma NON cambia la traiettoria: le coppie restano quelle di questa
%   run. Quindi i numeri dei moltiplicatori diversi da 1 dicono quanto bene
%   quella soglia avrebbe separato contatto e volo SU QUESTI DATI, non cosa
%   sarebbe successo al robot. Per quello serve spazzata_soglia.
%   Al moltiplicatore 1 (la soglia di campagna) il limite non c'e': la run e'
%   girata proprio con quella soglia, quindi flag ricostruito e flag vero
%   coincidono e la misura e' esatta.
%
% LA REGOLA DEL FLAG: LA SI VA A LEGGERE NEL MODELLO, NON SI ASSUME
%   Il repo la descrive in due modi diversi, e non possono essere entrambi
%   giusti:
%     phantomx_config riga 376   cont = OR( |tau_mis - tau_att| > soglia )
%     abilita_log     riga 18    cont = OR( |tau|            > soglia )
%   La Parte 1 risale il collegamento del Constant c2_soglia dentro il .slx e
%   stampa la catena di blocchi che lo alimenta: se in mezzo c'e' una
%   differenza la regola e' la prima, altrimenti la seconda. Con
%   opt.regola = 'diff' | 'assoluta' si forza a mano.
%
% I CRITERI, SCRITTI PRIMA DI LANCIARE
%   1. IL SEGNALE SEPARA se a qualche moltiplicatore falsi positivi + falsi
%      negativi <= 10% dei campioni di APPOGGIO. Se non ci riesce nessuno, le
%      due distribuzioni di |dtau| si sovrappongono: la soglia non e'
%      tarabile, e il difetto non e' di taratura.
%   2. LA SOGLIA DI CAMPAGNA E' BEN TARATA se a 1x l'errore in appoggio sta
%      entro il 20% del minimo della spazzata E nessun appoggio resta non
%      rilevato.
%   3. [AGGIUNTO 25/9] IL FLAG STA BASSO IN VOLO: alto su non piu' del 5%
%      della fase di volo, e nessun volo attraversato senza che torni mai
%      basso. Questo criterio mancava nella prima versione e senza di lui il
%      verdetto era sbagliato per costruzione - vedi la nota sulla maschera.
%   Servono tutti e tre. Se passano 1 e 2 ma non il 3, la soglia legge
%   l'appoggio e sporca il volo: e' un difetto di C2 da misurare, non una
%   taratura da cambiare.
%
% IL RITARDO E' LA COSA CHE CONTA DAVVERO
%   Un falso negativo non e' un errore astratto: mentre il flag sta a zero la
%   ricerca di terreno continua a scendere a v_search = 0.06 m/s. Un ritardo
%   di 50 ms sono 3 mm di piede spinto dentro il terreno; oltre
%   z_ext_max / v_search = 0.5 s la zampa ha esaurito la corsa. Il ritardo si
%   misura a ogni appoggio e si converte in millimetri, che e' la forma in
%   cui il numero si puo' discutere.
%
% COSA NON TOCCA
%   Nessun save_system: terreno, inerzie e log si accendono a runtime come in
%   applica_terreno. Nessun CSV di campagna: scrive solo
%   results/diagnostica/valida_soglia.csv e la figura.
%   NOTA: salva_grafico e' stato modificato a mano durante l'indagine del
%   15/9 e scrive in grafici/inerzie_og/, non in grafici/. Finche' resta
%   cosi' ogni figura del progetto finisce in quella sottocartella.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   valida_soglia
%
% Progetto FSR PhantomX - A. Russo

%% ==================== opzioni ====================
if nargin < 1, opt = struct(); end
def = struct('task','T2', 'mdl','phantomx_sim_zero', 'durata',[], ...
             'nCicli',10, 'molt',[0.25 0.5 1 2 4], 'regola','auto', ...
             'soglia_F',0.5, 'fs',200, 'grafico',true, 'inerzie',true, ...
             'soglia',[], 'dirty','avvisa');
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt,f{k}) || isempty(opt.(f{k})), opt.(f{k}) = def.(f{k}); end
end

cfg = phantomx_config();
mdl = opt.mdl;

if isempty(opt.durata), opt.durata = opt.nCicli * cfg.T; end

% [CORRETTO 25/9] L'OVERRIDE PASSA DA QUI, NON DAL WORKSPACE.
%   Questa funzione fa clear OVERRIDE_C2_SOGLIA prima di init_gait, per
%   partire da una condizione pulita. Chi seguiva l'istruzione
%       OVERRIDE_C2_SOGLIA = s;  init_gait;  valida_soglia
%   se lo vedeva cancellare e otteneva una run con la soglia di cfg,
%   identica alla precedente - e niente lo segnalava. Adesso la terna da
%   provare si passa in opt.soglia, e la funzione la impone lei.
if ~isempty(opt.soglia)
    vs_base = reshape(opt.soglia, 1, []);
else
    vs_base = reshape(cfg.c2.soglia_tau, 1, []);
end
if isscalar(vs_base), vs_base = repmat(vs_base, 1, 3); end

fprintf('\n=================== VALIDAZIONE DELLA SOGLIA DI C2 ===================\n');
if isempty(opt.soglia)
    fprintf('  soglia di campagna   %s N*m   (coxa, femore, tibia)\n', mat2str(vs_base,4));
else
    fprintf(2,'  soglia IMPOSTA A MANO %s N*m  (cfg dice %s)\n', ...
            mat2str(vs_base,4), mat2str(reshape(cfg.c2.soglia_tau,1,[]),4));
end
fprintf('  task %s, C2, %.1f s (%d cicli da %.2f s)\n', opt.task, opt.durata, ...
        opt.nCicli, cfg.T);
fprintf('  verita'' = forza normale al piede > %.2f N\n', opt.soglia_F);

% [CORRETTO 25/9] LA GUARDIA SUL "DIRTY" SCATTAVA SEMPRE, E OBBLIGAVA A UN
% bdclose all A OGNI RUN.
%   Era copiata da spazzata_soglia, dove doveva proteggere le modifiche non
%   salvate di chi ha il modello aperto. Ma applica_terreno, applica_inerzie
%   e allinea_stimatore lavorano proprio con set_param in memoria: dopo la
%   prima chiamata il modello E' dirty per costruzione, e la guardia non
%   distingueva le nostre modifiche da quelle di una persona.
%   Il motivo vero della guardia - "non so in che stato sto simulando" - qui
%   non c'e': terreno, inerzie e stimatore vengono riportati allo stato
%   voluto sotto, in modo esplicito. Quindi si avvisa e si dice cosa c'e',
%   invece di fermarsi. Con opt.dirty = 'errore' torna il comportamento di
%   prima; questa funzione non salva mai, in nessun caso.
if ~bdIsLoaded(mdl), load_system(mdl); end
if strcmp(get_param(mdl,'Dirty'),'on')
    switch lower(opt.dirty)
        case 'errore'
            error('valida_soglia:dirty', ['Il modello ha modifiche non salvate ' ...
                'e opt.dirty = ''errore''.']);
        case 'ignora'
            % niente
        otherwise
            fprintf(2, ['\n  Il modello ha modifiche in memoria (normale: le fanno\n' ...
                        '  applica_terreno, applica_inerzie e allinea_stimatore).\n' ...
                        '  Terreno, inerzie e stimatore vengono reimpostati qui sotto.\n' ...
                        '  Questa funzione non salva il .slx in nessun caso.\n']);
    end
end

%% ==================== PARTE 1: che regola implementa il modello ====================
reg = vs_regola(mdl, opt.regola);
fprintf('\n  regola del flag usata da qui in poi:  %s\n', reg.descrizione);

%% ==================== PARTE 2: una run ====================
applica_terreno(opt.task, false, mdl);
% [25/9] L'interruttore serve a un esperimento preciso: se tau_attesa viene
% da un modello che porta ancora le inerzie dell'URDF, correggere solo il
% robot rende |tau_mis - tau_att| grande DOVUNQUE, contatto o no. Con
% opt.inerzie = false si rifa' la stessa misura al punto di lavoro vecchio:
% se il flag in volo crolla, il colpevole e' il disallineamento fra i due.
if opt.inerzie
    applica_inerzie(mdl);
    % [25/9] Le due vanno SEMPRE insieme. Correggere le inerzie del robot e
    % lasciare all'Inverse Dynamics quelle dell'URDF e' il disallineamento
    % che ha reso inutilizzabile il flag di contatto dal 22/9 al 25/9.
    allinea_stimatore(mdl);
else
    fprintf(2,'\n  [opt.inerzie = false] inerzie URDF su robot E stimatore:\n');
    fprintf(2,'  e'' il punto di lavoro precedente al 22/9, serve solo a confronto.\n');
end

% Fleg e Fsum sono commentati nel modello: senza non c'e' la verita' e tutto
% il resto e' inutile. Si riaccendono a runtime e si rispengono in uscita,
% cosi' il .slx resta esattamente com'e' committato.
logPrima = get_param(mdl,'SimscapeLogType');
set_param(mdl,'SimscapeLogType','all');
abilita_log('on', false, mdl);
ripristina = onCleanup(@() vs_ripristina(mdl, logPrima));           %#ok<NASGU>

assignin('base','OVERRIDE_C2', true);
evalin('base','clear OVERRIDE_GAIT');
if isempty(opt.soglia)
    evalin('base','clear OVERRIDE_C2_SOGLIA');
else
    assignin('base','OVERRIDE_C2_SOGLIA', vs_base);
end
evalin('base','init_gait');

% Si controlla che la soglia sia DAVVERO arrivata al modello: init_gait la
% stampa, ma una stampa non e' una verifica.
vs_c2s = [];
try, vs_c2s = reshape(evalin('base','c2_soglia'), 1, []); end %#ok<TRYNC>
if ~isempty(vs_c2s) && numel(vs_c2s) == 3 && any(abs(vs_c2s - vs_base) > 1e-12)
    error('valida_soglia:soglia', ...
        ['Ho chiesto %s ma il modello usa %s. La run direbbe una cosa\n' ...
         'diversa da quella che crede di misurare: mi fermo.'], ...
        mat2str(vs_base,4), mat2str(vs_c2s,4));
end

fprintf('\n  simulo %.1f s ...\n', opt.durata);
vs_crono = tic;
out = sim(mdl, 'StopTime', num2str(opt.durata));
fprintf('  fatta in %.0f s\n', toc(vs_crono));

evalin('base','clear OVERRIDE_C2');

o = struct('fs',opt.fs, 'soglia_F',opt.soglia_F, 'verbose',false, ...
           'ignora_ik',true, 'tau','sens');
run_s = adatta_simscape(out, struct('controller','C2','task',opt.task, ...
                                    'condizione','valida_soglia'), o);
o.tau = 'estim';  o.pf = false;
run_e = adatta_simscape(out, struct('controller','C2','task',opt.task), o);

if ~isfield(run_s,'tau') || isempty(run_s.tau) || all(isnan(run_s.tau(:)))
    error('valida_soglia:tau', ['torque_sens non e'' arrivato: senza le coppie ' ...
        'misurate non c''e'' niente da validare. Controlla il To Workspace.']);
end
if ~isfield(run_s,'Fc') || isempty(run_s.Fc) || all(isnan(run_s.Fc(:)))
    error('valida_soglia:forze', ['Non c''e'' la forza ai piedi: la verita'' manca ' ...
        'e il confronto non si puo'' fare. abilita_log(''on'') ha fallito?']);
end

t = run_s.t;

%% ==================== PARTE 3: il flag, ricostruito ====================
% D = la quantita' che viene confrontata con la soglia, 18 colonne in ordine
% CAN (FL FR ML MR RL RR), tre giunti per zampa.
switch reg.tipo
    case 'diff'
        if ~isfield(run_e,'tau') || isempty(run_e.tau)
            error('valida_soglia:estim', ['La regola e'' la differenza ma ' ...
                'torque_estim non e'' nel log: non posso ricostruire il flag. ' ...
                'Accendi quel To Workspace, oppure forza opt.regola = ''assoluta'' ' ...
                'sapendo che stai misurando un''altra cosa.']);
        end
        D = abs(run_s.tau - run_e.tau);
    otherwise
        D = abs(run_s.tau);
end

Fz   = run_s.Fc(:, 3:3:18);
vero = Fz > opt.soglia_F;                    % la verita', per zampa

% MASCHERA: dove il flag conta davvero.
%   Prima di t_reset la ricerca e' inibita per costruzione (ricerca_terreno
%   riga 77), quindi quei campioni non dicono nulla sulla soglia. Dove
%   disponibile si restringe anche alla fase di appoggio programmata: e'
%   li' che la ricerca e' attiva e il flag decide se fermare la zampa.
%
% [CORRETTO 25/9] LA PRIMA VERSIONE GUARDAVA SOLO L'APPOGGIO, ED ERA CIECA.
%   Restringendo tutto alla fase di appoggio programmata, i falsi positivi
%   della fase di VOLO finivano fuori dal conteggio: il flag poteva stare
%   alto per mezzo passo con la zampa per aria e la tabella diceva 97.6% di
%   accordo. Ed e' proprio il volo il caso che rompe C2, perche' in
%   ricerca_terreno il ramo c(i)==0 e' l'UNICO che azzera z_ext e z_hold
%   (righe 98-103): finche' il flag resta alto quei due stati non si
%   riazzerano e il comando di volo resta abbassato di z_ext.
%   Adesso le due fasi si misurano separate.
dopo   = repmat(t(:) > cfg.c2.t_reset, 1, 6);
if isfield(run_s,'contact_sched') && ~isempty(run_s.contact_sched) && ...
        size(run_s.contact_sched,2) == 6
    sched = logical(run_s.contact_sched);
    fonte_mask = 'appoggio e volo programmati dall''andatura';
else
    sched = vero;      % ripiego: la fase la da' la forza
    fonte_mask = 'ripiego sulla forza (contact_sched assente)';
end
mask = dopo & sched;           % fase di APPOGGIO: il flag deve dire 1
volo = dopo & ~sched;          % fase di VOLO:     il flag deve dire 0
fprintf('  campioni dopo t_reset: %d appoggio + %d volo   (%s)\n', ...
        sum(mask(:)), sum(volo(:)), fonte_mask);

%% ==================== spazzata offline ====================
VS = table();
for k = 1:numel(opt.molt)
    s    = opt.molt(k) * vs_base;
    flag = false(numel(t), 6);
    for i = 1:6
        flag(:,i) = any(D(:, 3*(i-1)+(1:3)) > s, 2);
    end

    fp = sum(flag & ~vero & mask, 'all') / sum(mask,'all');   % dice terra, e' in volo
    fn = sum(~flag & vero & mask, 'all') / sum(mask,'all');   % dice volo, e' a terra

    % IL NUMERO CHE MANCAVA: quanto del volo ha il flag alto, e per colpa di
    % quale giunto. Un flag alto in volo non e' solo un errore di lettura:
    % blocca il reset di z_ext in ricerca_terreno.
    fv = sum(flag & volo, 'all') / sum(volo,'all');
    fv_g = zeros(1,3);
    for j = 1:3
        sopra = D(:, j:3:18) > s(j);
        fv_g(j) = sum(sopra & volo, 'all') / sum(volo,'all');
    end

    % voli attraversati SENZA che il flag torni mai basso: li' z_ext e z_hold
    % non si azzerano e il passo successivo parte gia' abbassato
    sporchi = vs_voli_sporchi(volo, flag);

    rit = vs_ritardi(t, vero, flag, mask, cfg.T);

    r = table();
    r.molt        = opt.molt(k);
    r.soglia_coxa = s(1);
    r.soglia_fem  = s(2);
    r.soglia_tib  = s(3);
    r.falsi_pos   = fp;
    r.falsi_neg   = fn;
    r.errore_tot  = fp + fn;
    r.accordo     = sum(flag == vero & mask,'all') / sum(mask,'all');
    r.errore_banale    = sum(~vero & mask,'all') / sum(mask,'all');
    r.flag_in_volo     = fv;
    r.volo_coxa        = fv_g(1);
    r.volo_fem         = fv_g(2);
    r.volo_tib         = fv_g(3);
    r.voli             = sporchi.n;
    r.voli_senza_reset = sporchi.sporchi;
    r.appoggi          = rit.n;
    r.appoggi_mancati  = rit.mancati;
    r.gia_alto         = rit.gia_alto;
    r.ritardo_med_ms   = 1e3 * rit.mediano;
    r.affondo_med_mm   = 1e3 * cfg.c2.v_search * max(rit.mediano, 0);
    VS = [VS; r];                                                  %#ok<AGROW>
end

%% ==================== PARTE 4: lettura ====================
fprintf('\n---------------- FLAG RICOSTRUITO CONTRO CONTATTO VERO ----------------\n');
fprintf('  %-6s %9s %9s %9s %11s %10s %9s\n', 'molt', 'falsi+', 'falsi-', ...
        'errore', 'FLAG IN VOLO', 'voli sporchi', 'mancati');
for k = 1:height(VS)
    stella = ' ';
    if abs(VS.molt(k) - 1) < 1e-9, stella = '*'; end
    fprintf('%s %-5.2f %8.1f%% %8.1f%% %8.1f%% %10.1f%% %7d/%-4d %6d/%-4d\n', ...
        stella, VS.molt(k), 100*VS.falsi_pos(k), 100*VS.falsi_neg(k), ...
        100*VS.errore_tot(k), 100*VS.flag_in_volo(k), ...
        VS.voli_senza_reset(k), VS.voli(k), VS.appoggi_mancati(k), VS.appoggi(k));
end
fprintf('\n  quale giunto fa scattare l''OR in volo (frazione della fase di volo):\n');
fprintf('  %-6s %10s %10s %10s\n', 'molt', 'coxa', 'femore', 'tibia');
for k = 1:height(VS)
    fprintf('  %-6.2f %9.1f%% %9.1f%% %9.1f%%\n', VS.molt(k), ...
        100*VS.volo_coxa(k), 100*VS.volo_fem(k), 100*VS.volo_tib(k));
end
fprintf('  (* = soglia di campagna: quella con cui la run e'' davvero girata,\n');
fprintf('   l''unica riga che non risente della ricostruzione ad anello aperto)\n');

% --- separazione per giunto: la soglia sta fra le due distribuzioni? ---
nomi_g = {'coxa','femore','tibia'};
fprintf('\n---------------- |dtau| A TERRA E IN VOLO, PER GIUNTO ----------------\n');
fprintf('  %-8s %12s %12s %12s %12s %10s\n', 'giunto', 'terra p10', 'terra med', ...
        'volo med', 'volo p90', 'soglia');
sep = nan(1,3);
for j = 1:3
    col = j:3:18;
    Dj  = D(:, col);
    a_terra = Dj(vero & mask);
    in_volo = Dj(~vero & mask);
    if isempty(a_terra) || isempty(in_volo), continue; end
    q_terra = vs_perc(a_terra, [10 50]);
    q_volo  = vs_perc(in_volo,  [50 90]);
    sep(j)  = q_terra(1) - q_volo(2);       % > 0 = le due nuvole non si toccano
    fprintf('  %-8s %12.4f %12.4f %12.4f %12.4f %10.4f%s\n', nomi_g{j}, ...
            q_terra(1), q_terra(2), q_volo(1), q_volo(2), vs_base(j), ...
            vs_marca(vs_base(j), q_volo(2), q_terra(1)));
end
fprintf('  ok = la soglia cade fra il p90 in volo e il p10 a terra.\n');
fprintf('  alta/bassa = ci cade fuori: rilevazione tardiva / falsi contatti.\n');

% --- i tre criteri ---
[err_min, k_min] = min(VS.errore_tot);
k_camp = find(abs(VS.molt - 1) < 1e-9, 1);

% [CORRETTO 25/9] IL CRITERIO 1 ERA FALSATO DAL TASSO DI BASE.
%   Misurando l'errore solo in appoggio, un flag SEMPRE ALTO ottiene da solo
%   un errore pari alla frazione di appoggio senza forza - qui il 7% - e la
%   prima versione lo chiamava "separa". Un flag costante non separa niente.
%   Adesso il confronto e' contro quel predittore banale: per passare il
%   flag deve dimezzarne l'errore.
err_ban = VS.errore_banale(1);
separa  = err_min <= 0.5 * err_ban;
fprintf('\n  1. il segnale separa contatto e volo:      %s', vs_si(separa));
fprintf('   (errore minimo %.1f%% a %.2fx, contro il %.1f%% di un flag\n', ...
        100*err_min, VS.molt(k_min), 100*err_ban);
fprintf('     sempre alto: per separare deve stare sotto il %.1f%%)\n', 50*err_ban);

tarata = false;  pulita = false;
if ~isempty(k_camp)
    tarata = VS.errore_tot(k_camp) <= 1.2*err_min && VS.appoggi_mancati(k_camp) == 0;
    fprintf('  2. la soglia di campagna e'' ben tarata:    %s', vs_si(tarata));
    fprintf('   (errore %.1f%%, %d appoggi non rilevati)\n', ...
            100*VS.errore_tot(k_camp), VS.appoggi_mancati(k_camp));

    % [25/9] IL TERZO CRITERIO, CHE MANCAVA. Senza, il verdetto si puo'
    % dichiarare "validata" con il flag alto per meta' della fase di volo.
    pulita = VS.flag_in_volo(k_camp) <= 0.05 && VS.voli_senza_reset(k_camp) == 0;
    fprintf('  3. il flag sta basso in volo:              %s', vs_si(pulita));
    fprintf('   (alto sul %.1f%% del volo, %d voli su %d senza reset)\n', ...
            100*VS.flag_in_volo(k_camp), VS.voli_senza_reset(k_camp), VS.voli(k_camp));
end

fprintf('\n');
if ~separa
    fprintf('  => LE DUE DISTRIBUZIONI SI SOVRAPPONGONO: nessuna soglia su questa\n');
    fprintf('     quantita'' separa contatto e volo, e il flag non fa meglio di uno\n');
    fprintf('     costante. Ritarare la soglia non lo toglie: e'' la GRANDEZZA a non\n');
    fprintf('     portare l''informazione. Se la regola e'' la differenza, la prima\n');
    fprintf('     cosa da guardare e'' se tau_attesa e tau_misurata parlano dello\n');
    fprintf('     stesso robot:  valida_soglia(struct(''inerzie'',false))\n');
elseif tarata && pulita
    fprintf('  => LA SOGLIA E'' VALIDATA: %s N*m legge il contatto in appoggio e\n', ...
            mat2str(vs_base,4));
    fprintf('     sta basso in volo. Gli errori di C2 nelle tabelle non vengono\n');
    fprintf('     dalla rilevazione, e C3 si sceglie guardando altro.\n');
elseif tarata && ~pulita
    fprintf('  => LA SOGLIA LEGGE L''APPOGGIO MA SPORCA IL VOLO.\n');
    fprintf('     In appoggio l''errore e'' %.1f%%, ma il flag resta alto sul %.1f%%\n', ...
            100*VS.errore_tot(k_camp), 100*VS.flag_in_volo(k_camp));
    fprintf('     della fase di volo. In ricerca_terreno il ramo c(i)==0 e'' l''unico\n');
    fprintf('     che azzera z_ext e z_hold: con il flag alto quei due stati si\n');
    fprintf('     portano dietro l''estensione dell''appoggio precedente, e il passo\n');
    fprintf('     successivo parte abbassato di z_ext (fino a z_ext_max = %.0f mm\n', ...
            1e3*cfg.c2.z_ext_max);
    fprintf('     su un sollevamento H = %.0f mm). E'' un difetto di C2, non del\n', ...
            1e3*cfg.H);
    fprintf('     terreno: va misurato su T6 prima di scegliere C3.\n');
else
    fprintf('  => LA SOGLIA E'' MAL TARATA: a %.2fx l''errore scende da %.1f%% a %.1f%%.\n', ...
            VS.molt(k_min), 100*VS.errore_tot(k_camp), 100*err_min);
    fprintf('     Questo pero'' e'' offline. Il passo successivo e'' obbligato:\n');
    fprintf('       OVERRIDE_C2_SOGLIA = %.4g * cfg.c2.soglia_tau;  spazzata_soglia\n', ...
            VS.molt(k_min));
    fprintf('     e si guarda se le metriche seguono. Solo allora si ritara cfg.\n');
end

%% ==================== PARTE 5: CSV e figura ====================
if ~isfolder(fullfile('results','diagnostica')), mkdir(fullfile('results','diagnostica')); end
csv = fullfile('results','diagnostica','valida_soglia.csv');
writetable(VS, csv);
fprintf('\n  scritto  %s\n', csv);

dati = struct('t',t, 'D',D, 'vero',vero, 'appoggio',mask, 'volo',volo, ...
              'Fz',Fz, 'soglia',vs_base, 'regola',reg.tipo, 'cfg',cfg);

if opt.grafico
    fig = figure('Name','valida_soglia','Position',[80 80 1180 680]);
    tiledlayout(fig, 2, 3, 'TileSpacing','compact', 'Padding','compact');

    % 1 - errori contro moltiplicatore
    nexttile;
    semilogx(VS.molt, 100*VS.falsi_pos, 'o-', 'LineWidth',1.2); hold on
    semilogx(VS.molt, 100*VS.falsi_neg, 's-', 'LineWidth',1.2);
    semilogx(VS.molt, 100*VS.errore_tot,'k^-','LineWidth',1.4);
    xline(1, 'k--', 'campagna');
    grid on; xlabel('moltiplicatore della soglia'); ylabel('campioni [%]');
    title('errore del flag'); legend({'falsi positivi','falsi negativi','totale'}, ...
        'Location','best');

    % 2 - il flag in volo, per giunto: dove nasce il falso contatto
    nexttile;
    s_camp = vs_base;
    flag_c = false(numel(t), 6);
    for i = 1:6, flag_c(:,i) = any(D(:, 3*(i-1)+(1:3)) > s_camp, 2); end
    fv_g = zeros(1,3);
    for j = 1:3
        fv_g(j) = sum((D(:, j:3:18) > s_camp(j)) & volo, 'all') / sum(volo,'all');
    end
    bar(100*[fv_g, sum(flag_c & volo,'all')/sum(volo,'all')]); grid on
    set(gca,'XTickLabel',{'coxa','femore','tibia','OR'});
    ylabel('frazione della fase di volo [%]');
    title('flag alto con la zampa per aria (1x)');

    % 3 - una zampa, due periodi: verita' contro flag
    nexttile;
    k0 = find(t > cfg.c2.t_reset + cfg.T, 1);
    k1 = find(t > t(k0) + 2*cfg.T, 1);
    if isempty(k1), k1 = numel(t); end
    ii = k0:k1;
    plot(t(ii), double(vero(ii,1)), 'LineWidth',1.6); hold on
    plot(t(ii), 0.9*double(flag_c(ii,1)), '--', 'LineWidth',1.4);
    ylim([-0.2 1.3]); grid on
    xlabel('t [s]'); ylabel('contatto FL');
    title('zampa FL, due passi'); legend({'forza > soglia','flag di coppia'}, ...
        'Location','best');

    % 4..6 - distribuzioni per giunto
    for j = 1:3
        nexttile;
        col = j:3:18;  Dj = D(:, col);
        a_terra = Dj(vero & mask);  in_volo = Dj(~vero & mask);
        if isempty(a_terra) || isempty(in_volo), axis off; continue; end
        histogram(in_volo, 40, 'Normalization','probability'); hold on
        histogram(a_terra, 40, 'Normalization','probability');
        xline(vs_base(j), 'k-', 'LineWidth',1.6);
        set(gca,'XScale','log'); grid on
        xlabel('|dtau| [N*m]'); ylabel('frazione');
        title(nomi_g{j});
        if j == 1, legend({'in volo','a terra','soglia'}, 'Location','best'); end
    end

    salva_grafico(sprintf('valida_soglia_%s', opt.task), fig);
end

fprintf('  nessun CSV di campagna toccato, modello non salvato.\n\n');

end

%% ========================= helper =========================
function reg = vs_regola(mdl, forzata)
%VS_REGOLA  Che cosa viene confrontato con c2_soglia, letto dal modello.
%   Risale il collegamento a monte del Relational Operator alimentato dal
%   Constant c2_soglia e stampa la catena. Se in mezzo c'e' una somma con un
%   segno meno, la regola e' la differenza fra coppia misurata e attesa;
%   altrimenti e' il valore assoluto della coppia misurata.
%   SOLO LETTURA: nessun set_param.

reg = struct('tipo','assoluta', 'descrizione','', 'catena',{{}}, 'certa',false);

fprintf('\n---------------- PARTE 1: la regola del flag, dal .slx ----------------\n');

if ~strcmpi(forzata,'auto')
    reg.tipo = lower(char(forzata));
    reg.descrizione = sprintf('%s  [FORZATA da opt.regola, il modello non e'' stato letto]', reg.tipo);
    fprintf('  regola imposta a mano: %s\n', reg.tipo);
    return
end

try
    bb = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                     'BlockType','Constant');
    sorg = {};
    for k = 1:numel(bb)
        v = '';
        try, v = char(get_param(bb{k},'Value')); end %#ok<TRYNC>
        if contains(v, 'c2_soglia'), sorg{end+1} = bb{k}; end %#ok<AGROW>
    end

    if isempty(sorg)
        fprintf(2, '  Nessun Constant legge c2_soglia. O il collegamento e'' rotto\n');
        fprintf(2, '  (come il 23/9), o la soglia arriva per un''altra strada.\n');
        reg.descrizione = 'assoluta  [non verificata: c2_soglia non trovato]';
        return
    end

    fprintf('  %d Constant leggono c2_soglia:\n', numel(sorg));
    trovata_diff = false;
    for k = 1:numel(sorg)
        fprintf('    %s\n', strrep(sorg{k}, [mdl '/'], ''));
        dest = vs_valle(sorg{k});
        if isempty(dest)
            fprintf(2,'      il segnale non arriva a nessun blocco: collegamento rotto?\n');
        end
        fprintf('      %d consumatori del segnale (Goto/From e sottosistemi attraversati)\n', ...
                numel(dest));
        for d = 1:numel(dest)
            blkd = getfullname(dest(d).blocco);
            fprintf('      -> %-28s (%s), altro ingresso:\n', ...
                    strrep(blkd,[mdl '/'],''), get_param(dest(d).blocco,'BlockType'));
            cc = vs_monte(dest(d).altre_porte, 0, {});
            for c = 1:numel(cc)
                fprintf('         %s%s\n', repmat('  ',1,cc{c}.liv), cc{c}.testo);
                if cc{c}.diff, trovata_diff = true; end
            end
            reg.catena = [reg.catena; cc(:)];
        end
    end

    if isempty(reg.catena)
        fprintf(2,['\n  Nessun consumatore con un secondo ingresso: non ho visto il\n' ...
                   '  confronto, quindi NON dichiaro quale sia la regola. Apri a mano\n' ...
                   '  il blocco stampato qui sopra e guarda cosa entra nell''altra porta.\n']);
        reg.descrizione = 'assoluta  [NON VERIFICATA: il confronto non e'' stato raggiunto]';
        return
    end

    reg.certa = true;
    if trovata_diff
        reg.tipo = 'diff';
        reg.descrizione = '|tau_misurata - tau_attesa| > soglia   [letta nel modello]';
    else
        reg.tipo = 'assoluta';
        reg.descrizione = '|tau_misurata| > soglia   [letta nel modello: nessuna sottrazione a monte]';
    end
catch err
    % Un catch che nasconde dove si e' rotto costringe a indovinare: qui la
    % riga esatta si stampa, altrimenti si perde un giro a ogni errore.
    fprintf(2, '  Non sono riuscito a risalire il collegamento: %s\n', err.message);
    for q = 1:min(3,numel(err.stack))
        fprintf(2, '     in %s riga %d\n', err.stack(q).name, err.stack(q).line);
    end
    fprintf(2, '  NON dichiaro nessuna regola. La spazzata prosegue sul valore\n');
    fprintf(2, '  assoluto, ma il verdetto va letto sapendo che la grandezza\n');
    fprintf(2, '  confrontata non e'' stata verificata.\n');
    reg.descrizione = 'assoluta  [NON VERIFICATA: lettura del modello fallita]';
end
end

function dest = vs_valle(blk, prof, visti)
%VS_VALLE  I consumatori a valle dell'uscita di blk, con le loro ALTRE porte.
%
% [CORRETTO 25/9] LA PRIMA VERSIONE SI FERMAVA AL PRIMO BLOCCO.
%   c2_soglia esce dal Constant e finisce dentro un Goto: il confronto sta
%   dai From che portano lo stesso tag, una o piu' pagine piu' in la'.
%   Fermandosi li', lo script concludeva "nessuna sottrazione a monte"
%   senza aver mai visto il confronto. Adesso attraversa Goto/From, i
%   confini dei sottosistemi e gli Outport, e si ferma solo sul blocco che
%   il segnale consuma davvero.
if nargin < 2, prof = 0; end
if nargin < 3, visti = []; end
dest = struct('blocco',{}, 'altre_porte',{});
if prof > 10, return; end

h = get_param(blk,'Handle');
if any(visti == h), return; end
visti(end+1) = h;                                                  %#ok<AGROW>

ph = get_param(blk,'PortHandles');
if ~isfield(ph,'Outport') || isempty(ph.Outport), return; end

for u = 1:numel(ph.Outport)
    lh = get_param(ph.Outport(u), 'Line');
    if lh == -1, continue; end
    dh = get_param(lh, 'DstBlockHandle');
    dp = get_param(lh, 'DstPortHandle');
    for i = 1:numel(dh)
        if dh(i) == -1, continue; end
        tipo = get_param(dh(i),'BlockType');
        mdl  = bdroot(dh(i));

        switch tipo
            case 'Goto'
                % il segnale riparte da ogni From con lo stesso tag
                tag = get_param(dh(i),'GotoTag');
                ff = vs_cella(find_system(mdl,'LookUnderMasks','all', ...
                     'FollowLinks','on','BlockType','From','GotoTag',tag));
                for g = 1:numel(ff)
                    dest = [dest, vs_valle(ff{g}, prof+1, visti)];  %#ok<AGROW>
                end

            case 'SubSystem'
                % si entra: la porta d'ingresso k corrisponde all'Inport k
                k = get_param(dp(i),'PortNumber');
                ii = vs_cella(find_system(dh(i),'SearchDepth',1, ...
                     'LookUnderMasks','all','FollowLinks','on','BlockType','Inport'));
                for q = 1:numel(ii)
                    if str2double(get_param(ii{q},'Port')) == k
                        dest = [dest, vs_valle(ii{q}, prof+1, visti)]; %#ok<AGROW>
                    end
                end

            case 'Outport'
                % si esce: si continua dall'uscita k del sottosistema padre
                pad = get_param(get_param(dh(i),'Parent'),'Handle');
                if ~strcmp(get_param(pad,'Type'),'block'), continue; end
                dest = [dest, vs_valle(pad, prof+1, visti)];        %#ok<AGROW>

            otherwise
                php = get_param(dh(i),'PortHandles');
                altre = [];
                if isfield(php,'Inport'), altre = setdiff(php.Inport, dp(i)); end
                dest(end+1) = struct('blocco',dh(i), 'altre_porte',altre); %#ok<AGROW>
        end
    end
end
end

function c = vs_cella(x)
%VS_CELLA  find_system restituisce NOMI se lo chiami con un nome e HANDLE se
%   lo chiami con un handle. Mescolare le due cose e' costato un giro:
%   ff{g} su un vettore di handle da' "Brace indexing is not supported".
%   Qui si normalizza sempre a cella di nomi completi.
if iscell(x)
    c = x;
elseif isempty(x)
    c = {};
else
    c = cell(numel(x),1);
    for k = 1:numel(x), c{k} = getfullname(x(k)); end
end
end

function cc = vs_monte(porte, liv, cc)
%VS_MONTE  Risale ricorsivamente le sorgenti di un insieme di porte.
%   Si ferma a profondita' 6: serve a capire la forma del confronto, non a
%   ridisegnare il modello.
if liv > 6, return; end
for i = 1:numel(porte)
    lh = get_param(porte(i), 'Line');
    if lh == -1, continue; end
    sh = get_param(lh, 'SrcBlockHandle');
    if sh == -1, continue; end
    tipo = get_param(sh,'BlockType');
    nome = get_param(sh,'Name');

    segni = '';
    e_diff = false;
    if strcmp(tipo,'Sum')
        try, segni = char(get_param(sh,'Inputs')); end %#ok<TRYNC>
        e_diff = contains(segni,'-');
    end

    testo = sprintf('%s  [%s]%s', regexprep(nome,'\s+',' '), tipo, ...
                    vs_extra(sh, tipo, segni));
    cc{end+1} = struct('liv',liv, 'testo',testo, 'diff',e_diff); %#ok<AGROW>

    % si continua solo attraverso i blocchi di segnale, non oltre le sorgenti
    if any(strcmp(tipo, {'Sum','Abs','Mux','Gain','Product','Math','Demux', ...
                         'SignalConversion','RateTransition','UnitDelay'}))
        php = get_param(sh,'PortHandles');
        cc = vs_monte(php.Inport, liv+1, cc);
    elseif strcmp(tipo,'SubSystem')
        % il segnale nasce dall'Outport k dentro il sottosistema
        k = get_param(get_param(porte(i),'Line'),'SrcPortHandle');
        k = get_param(k,'PortNumber');
        oo = vs_cella(find_system(sh,'SearchDepth',1,'LookUnderMasks','all', ...
                      'FollowLinks','on','BlockType','Outport'));
        for q = 1:numel(oo)
            if str2double(get_param(oo{q},'Port')) == k
                php = get_param(oo{q},'PortHandles');
                cc = vs_monte(php.Inport, liv+1, cc);
            end
        end
    elseif strcmp(tipo,'Inport')
        % si esce dal sottosistema: si continua dalla porta k del padre
        pad = get_param(sh,'Parent');
        if strcmp(get_param(pad,'Type'),'block')
            k = str2double(get_param(sh,'Port'));
            php = get_param(pad,'PortHandles');
            if numel(php.Inport) >= k
                cc = vs_monte(php.Inport(k), liv+1, cc);
            end
        end
    elseif strcmp(tipo,'From')
        tag = get_param(sh,'GotoTag');
        mdl = bdroot(sh);
        gg = vs_cella(find_system(mdl,'LookUnderMasks','all','FollowLinks','on', ...
                      'BlockType','Goto','GotoTag',tag));
        for g = 1:numel(gg)
            php = get_param(gg{g},'PortHandles');
            cc{end+1} = struct('liv',liv+1, ...
                'testo',sprintf('(Goto %s) %s', tag, strrep(gg{g},[char(mdl) '/'],'')), ...
                'diff',false);                                  %#ok<AGROW>
            cc = vs_monte(php.Inport, liv+2, cc);
        end
    end
end
end

function s = vs_extra(sh, tipo, segni)
s = '';
switch tipo
    case 'Sum',      s = sprintf('  segni "%s"', segni);
    case 'Constant', try, s = sprintf('  = %s', char(get_param(sh,'Value'))); end %#ok<TRYNC>
    case 'From',     try, s = sprintf('  tag %s', char(get_param(sh,'GotoTag'))); end %#ok<TRYNC>
    case 'ToWorkspace', try, s = sprintf('  var %s', char(get_param(sh,'VariableName'))); end %#ok<TRYNC>
end
end

function r = vs_ritardi(t, vero, flag, mask, T)
%VS_RITARDI  Ritardo del flag su ogni appoggio, in secondi.
%
% [CORRETTO 25/9] LA PRIMA VERSIONE RESTITUIVA UN NUMERO CHE NON ESISTEVA.
%   Cercava il flag in una finestra che partiva un quarto di periodo PRIMA
%   del contatto, per ammettere un anticipo. Ma se il flag e' gia' alto
%   quando la finestra si apre - ed e' quello che succede: il flag resta
%   alto per meta' del volo - la ricerca trova sempre il primo campione
%   della finestra, e la mediana esce incollata al bordo (-250 ms su una
%   finestra di 250 ms). Non era un anticipo di rilevazione: era il flag
%   che non era mai tornato basso.
%   Adesso quei casi si contano a parte (gia_alto) e restano fuori dalla
%   mediana, che cosi' misura solo i ritardi veri.
r = struct('tutti',[], 'mediano',NaN, 'p90',NaN, 'n',0, 'mancati',0, 'gia_alto',0);
dt = median(diff(t));
if ~isfinite(dt) || dt <= 0, return; end
anticipo = round(0.25*T/dt);

d = [];
for i = 1:6
    v = vero(:,i) & mask(:,i);
    f = flag(:,i);
    su  = find(diff([false; v]) ==  1);
    giu = find(diff([v; false]) == -1);
    n = min(numel(su), numel(giu));
    for k = 1:n
        a = max(1, su(k) - anticipo);
        b = giu(k);
        r.n = r.n + 1;
        if f(a)
            % il flag era gia' alto all'apertura della finestra: non c'e'
            % nessun fronte da cronometrare
            r.gia_alto = r.gia_alto + 1;
            continue
        end
        j = find(f(a:b), 1, 'first');
        if isempty(j)
            r.mancati = r.mancati + 1;
        else
            d(end+1) = t(a + j - 1) - t(su(k));            %#ok<AGROW>
        end
    end
end
r.tutti = d;
if ~isempty(d)
    r.mediano = median(d);
    r.p90     = vs_perc(d, 90);
end
end

function sp = vs_voli_sporchi(volo, flag)
%VS_VOLI_SPORCHI  Voli attraversati senza che il flag torni mai basso.
%   E' la condizione che impedisce il reset di z_ext e z_hold in
%   ricerca_terreno: il ramo che li azzera e' dentro  if c(i) == 0.
sp = struct('n',0, 'sporchi',0);
for i = 1:6
    v = volo(:,i);
    su  = find(diff([false; v]) ==  1);
    giu = find(diff([v; false]) == -1);
    n = min(numel(su), numel(giu));
    for k = 1:n
        sp.n = sp.n + 1;
        if all(flag(su(k):giu(k), i)), sp.sporchi = sp.sporchi + 1; end
    end
end
end

function s = vs_marca(soglia, p90_volo, p10_terra)
if soglia > p90_volo && soglia < p10_terra
    s = '  ok';
elseif soglia <= p90_volo
    s = '  bassa';
else
    s = '  alta';
end
end

function y = vs_perc(x, p)
%VS_PERC  Percentili senza lo Statistics Toolbox.
%   Nessun file del progetto usa prctile: se lo introducessimo qui, questa
%   diagnostica girerebbe solo su chi ha il toolbox. Interpolazione lineare
%   sui campioni ordinati, la stessa convenzione di prctile.
x = sort(x(:));
n = numel(x);
if n == 0, y = nan(size(p)); return; end
if n == 1, y = repmat(x, size(p)); return; end
pos = (p(:)/100)*n + 0.5;
pos = min(max(pos, 1), n);
lo  = floor(pos);  hi = ceil(pos);  w = pos - lo;
y   = reshape((1-w).*x(lo) + w.*x(hi), size(p));
end

function s = vs_si(tf)
if tf, s = 'SI'; else, s = 'NO'; end
end

function vs_ripristina(mdl, logPrima)
try, abilita_log('off', false, mdl); end                 %#ok<TRYNC>
try, set_param(mdl,'SimscapeLogType',logPrima); end      %#ok<TRYNC>
try, evalin('base','clear OVERRIDE_C2 OVERRIDE_C2_SOGLIA'); end %#ok<TRYNC>
end
