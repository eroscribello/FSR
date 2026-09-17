function [riga, dettaglio] = metriche(run, cfg, opt)
%METRICHE  Calcola le cinque famiglie di metriche di una simulazione.
%
%   riga = metriche(run)                % cfg letta da phantomx_config
%   riga = metriche(run, cfg)
%   [riga, dettaglio] = metriche(run, cfg, opt)
%
% INGRESSO
%   run   struttura normalizzata, vedi run_vuoto.m
%   cfg   configurazione; se omessa viene letta da phantomx_config
%   opt   opzioni (tutte facoltative):
%           .t_regime      [s] istante da cui iniziare a misurare, per
%                          escludere il transitorio iniziale (default 1.0)
%           .soglia_rib    [rad] |rollio| o |beccheggio| oltre cui la run
%                          e' considerata fallita (default 0.52 = 30 deg)
%           .soglia_fermo  [m] distanza minima per non considerare il robot
%                          fermo (default 0.02)
%           .soglia_Fz     [N] forza normale sopra cui la zampa e' in
%                          appoggio, se run.contact non e' fornito
%                          (default 0.5)
%           .durata_min_appoggio
%                          [s] appoggi piu' brevi di questo vengono esclusi
%                          dal conteggio dello scivolamento: sono tocchi
%                          spuri, e il moto di volo che li accompagna
%                          falsava slip_tot di un fattore. NaN = un quarto
%                          della durata nominale dell'appoggio
%           .frac_min      frazione MINIMA del task da percorrere perche' la
%                          run conti come successo (default 0.5). E' un
%                          criterio RELATIVO al comando: quello assoluto
%                          ("almeno 2 cm") promuoveva una run che eseguiva il
%                          18% del task
%
% USCITA
%   riga        table 1 x M, una riga per run: e' quella che si impila per
%               fare la tabella della campagna
%   dettaglio   struttura con le serie temporali intermedie, per i grafici
%
% DUE COLONNE CHE DICONO *PERCHE'* UNA CELLA FALLISCE
%   sotto3_frac  frazione di tempo con meno di tre piedi a terra. Distingue
%                un tripode che tiene da un'andatura intermittente, cosa che
%                appoggio_medio NON fa: una media di 3.0 e' compatibile sia
%                con tre piedi sempre giu' sia con sei e zero alternati.
%   corpoZ_pp    [m] rimbalzo verticale del corpo, picco-picco per ciclo.
%                E' il modo in cui l'andatura cinematica cede: su C1 vale
%                0.003 a 1x, 0.033 a 1.5x, 0.081 a 2x, su un'altezza di
%                appoggio di 0.154.
%   Senza queste due, una cella che fallisce da' solo frazione_task negativa
%   e nessun modo di attribuirne la causa - ed e' esattamente il confronto
%   che serve fra il cinematico e l'MPC alle velocita' alte.
%
% CAMPI MANCANTI
%   Le metriche che dipendono da campi assenti in run valgono NaN, e la
%   funzione stampa una volta l'elenco di cosa manca. Non e' un errore:
%   all'inizio non tutto e' loggato, e una tabella con qualche NaN e' piu'
%   utile di una funzione che si rifiuta di partire.
%
% Progetto FSR PhantomX - A. Russo

%% ---------- argomenti ----------
if nargin < 2 || isempty(cfg), cfg = phantomx_config(); end
if nargin < 3, opt = struct(); end

def = struct('t_regime',1.0, 'soglia_rib',deg2rad(30), ...
             'soglia_fermo',0.02, 'soglia_Fz',0.5, 'frac_min',0.5, ...
             'durata_min_appoggio',NaN);
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt,f{k}), opt.(f{k}) = def.(f{k}); end
end

%% ---------- validazione ----------
obbligatori = {'t','p','rpy'};
for k = 1:numel(obbligatori)
    if ~isfield(run,obbligatori{k}) || isempty(run.(obbligatori{k}))
        error('metriche:campoMancante', ...
            'Il campo obbligatorio run.%s manca o e'' vuoto. Vedi run_vuoto.', ...
            obbligatori{k});
    end
end

t = run.t(:);
N = numel(t);
assert(size(run.p,1)==N && size(run.rpy,1)==N, 'metriche:lunghezza', ...
    'run.p e run.rpy devono avere %d righe come run.t.', N);

opzionali = {'v','w','q','qd','tau','pf','Fc','contact','contact_sched'};
manca = opzionali(cellfun(@(c) ~isfield(run,c) || isempty(run.(c)), opzionali));
if ~isempty(manca)
    fprintf(2,'[metriche] campi assenti, metriche relative a NaN: %s\n', ...
            strjoin(manca,', '));
end
ha = @(c) isfield(run,c) && ~isempty(run.(c));

%% ---------- finestra di regime ----------
% Il transitorio iniziale non e' rappresentativo del controllore: se il robot
% parte in aria e cade, i primi decimi di secondo raccontano la condizione
% iniziale, non le prestazioni.
sel = t >= opt.t_regime;
if nnz(sel) < 10
    warning('metriche:regimeCorto', ...
        ['Dopo t_regime = %.2f s restano %d campioni. La run e'' troppo corta\n' ...
         'oppure t_regime e'' troppo alto: le metriche saranno rumorose.'], ...
        opt.t_regime, nnz(sel));
    sel = true(N,1);
end
ts = t(sel);

%% ---------- velocita' ----------
if ha('v')
    v = run.v;
else
    v = gradiente(run.p, t);      % derivata numerica, fallback
end

%% ---------- A. esecuzione del task ----------
vel_d = [NaN NaN];
if isfield(run,'meta') && isfield(run.meta,'vel_d'), vel_d = run.meta.vel_d(:).'; end
if all(isnan(vel_d)), vel_d = [cfg.v_nom 0]; end

A.err_vx_rms   = rms_(v(sel,1) - vel_d(1));
A.err_vy_rms   = rms_(v(sel,2) - vel_d(2));
A.dev_lat_rms  = rms_(run.p(sel,2) - run.p(1,2));
A.dev_lat_max  = max(abs(run.p(sel,2) - run.p(1,2)));
A.yaw_err_fin  = wrapToPi_(run.rpy(end,3) - yaw_atteso(run, t));
A.distanza     = norm(run.p(end,1:2) - run.p(1,1:2));   % totale, avvio incluso

% Velocita' media misurata SOLO nella finestra di regime. Includendo l'avvio
% si misurerebbe anche la perdita di partenza, che e' un offset fisso in
% metri: diluita su durate diverse darebbe velocita' diverse per la stessa
% cella. Misurato: ~12.5 mm persi all'avvio, indipendenti dalla durata.
i0 = find(sel, 1, 'first');
A.distanza_regime = norm(run.p(end,1:2) - run.p(i0,1:2));
A.vel_media       = A.distanza_regime / (t(end) - t(i0));
% perdita di partenza: quanto il robot resta indietro rispetto al comando
% durante l'avvio. Sull'SRB vale ~12 mm ed e' indipendente dalla durata.
A.perdita_avvio   = norm(vel_d) * (t(i0) - t(1)) - norm(run.p(i0,1:2) - run.p(1,1:2));

% L'avanzamento va misurato lungo la direzione COMANDATA e CON SEGNO: la
% norma non distingue avanti da indietro, e a 2x il robot cammina
% all'indietro risultando comunque al 18% del task invece che a -18%.
dirc          = vel_d(:) / max(norm(vel_d), eps);
A.avanzamento = (run.p(end,1:2) - run.p(1,1:2)) * dirc(1:2);
att_tot       = norm(vel_d) * (t(end) - t(1));
A.frazione_task = A.avanzamento / max(att_tot, eps);

ribaltato     = any(abs(run.rpy(:,1)) > opt.soglia_rib) || ...
                any(abs(run.rpy(:,2)) > opt.soglia_rib);
fermo         = A.distanza < opt.soglia_fermo;
insufficiente = att_tot > 0 && A.avanzamento < opt.frac_min * att_tot;

A.successo = ~(ribaltato || fermo || insufficiente);
A.causa_fallimento = "";
if insufficiente, A.causa_fallimento = "avanzamento insufficiente"; end
if fermo,         A.causa_fallimento = "fermo"; end          % piu' grave
if ribaltato,     A.causa_fallimento = "ribaltamento"; end   % il piu' grave

%% ---------- B. planarita' del corpo ----------
B.roll_rms   = rms_(run.rpy(sel,1));
B.pitch_rms  = rms_(run.rpy(sel,2));
B.roll_max   = max(abs(run.rpy(sel,1)));
B.pitch_max  = max(abs(run.rpy(sel,2)));

z_rif = median(run.p(sel,3));       % quota di riferimento: la mediana a regime
B.z_rms      = rms_(run.p(sel,3) - z_rif);
B.z_max      = max(abs(run.p(sel,3) - z_rif));
B.z_media    = z_rif;

%% ---------- C. sforzo di attuazione ----------
C = struct('tau_rms',NaN, 'tau_max',NaN, 'tau_rms_giunto_peggiore',NaN, ...
           'frazione_saturo',NaN, 'energia',NaN, 'cot',NaN, 'potenza_max',NaN);
if ha('tau')
    tau = run.tau(sel,:);
    C.tau_rms = rms_(tau(:));
    C.tau_max = max(abs(tau(:)));
    C.tau_rms_giunto_peggiore = max(sqrt(mean(tau.^2,1)));
    C.frazione_saturo = mean(abs(tau(:)) > cfg.tau_max);

    if ha('qd')
        pot = sum(abs(tau .* run.qd(sel,:)), 2);     % potenza meccanica
        C.energia     = trapz(ts, pot);
        C.potenza_max = max(pot);
        d = A.distanza;
        if d > opt.soglia_fermo
            C.cot = C.energia / (cfg.mass * cfg.g * d);
        end
    end
end

%% ---------- D. qualita' del contatto ----------
D = struct('slip_tot',NaN, 'slip_per_passo',NaN, 'distacchi',NaN, ...
           'frazione_persa',NaN, 'Fz_max_norm',NaN, 'appoggio_medio',NaN, ...
           'disp_carico',NaN, 'appoggi_scartati',NaN, 'appoggi_totali',NaN, ...
           'sotto3_frac',NaN, 'corpoZ_pp',NaN);
inContatto = [];
if ha('contact')
    inContatto = logical(run.contact);
elseif ha('Fc')
    inContatto = run.Fc(:,3:3:18) > opt.soglia_Fz;
end

if ~isempty(inContatto)
    D.appoggio_medio = mean(sum(inContatto(sel,:),2));   % zampe a terra in media

    % SOTTO TRE PIEDI: la frazione di tempo in cui il tripode non c'e'.
    %
    % appoggio_medio non basta, ed e' un errore che ho gia' fatto: la media
    % nasconde l'intermittenza. Su C1 a 2x valeva 2.12, sotto tre, ma una
    % media di 3.0 e' compatibile sia con tre piedi sempre a terra sia con
    % sei e zero alternati - due andature completamente diverse. Questa e'
    % la metrica che le distingue, e a 1x vale 0.4% contro il 61% a 2x.
    D.sotto3_frac = mean(sum(inContatto(sel,:),2) < 3);
end

% RIMBALZO DEL CORPO: picco-picco di z, ciclo per ciclo.
%
% E' la misura dei "saltelli" visibili in animazione, e su C1 e' il modo in
% cui l'andatura cede: 3.2 mm a 1x, 33.3 a 1.5x, 80.9 a 2x su un'altezza di
% appoggio di 154 mm. Senza questa colonna una cella che fallisce da'
% frazione_task negativa e nessun modo di dire perche'.
%
% Per ciclo e non su tutta la run: su tutta la run il picco-picco include
% la deriva verticale e sovrastima. La mediana fra i cicli e' robusta ai
% cicli anomali, che alle velocita' alte ci sono.
if ha('p') && size(run.p,2) >= 3
    zc = run.p(sel,3);  tc = t(sel);
    if isfield(cfg,'T') && ~isempty(cfg.T) && cfg.T > 0 && numel(tc) > 3
        bordi = tc(1):cfg.T:tc(end);
        if numel(bordi) >= 2
            v = nan(numel(bordi)-1,1);
            for k = 1:numel(bordi)-1
                m = tc >= bordi(k) & tc < bordi(k+1);
                if sum(m) >= 3, v(k) = max(zc(m)) - min(zc(m)); end
            end
            D.corpoZ_pp = median(v,'omitnan');
        end
    end
    if isnan(D.corpoZ_pp) && ~isempty(zc)
        D.corpoZ_pp = max(zc) - min(zc);
    end
end

if ~isempty(inContatto) && ha('pf')
    % Scivolamento: spostamento orizzontale del piede mentre e' in appoggio.
    %
    % SI SCARTANO GLI APPOGGI TROPPO BREVI, e non e' cosmetica.
    % Misurato su una run da 10 s: 83 segmenti di appoggio invece dei 60
    % attesi (10 cicli x 6 zampe), mediana 0.435 s contro 0.500 nominali, e
    % il 28% sotto i 0.1 s. I segmenti principali sono corretti; i ventitre
    % in piu' sono tocchi spuri - il piede passa vicino al terreno durante
    % il volo e la forza ricostruita supera per pochi campioni la soglia.
    % Ogni tocco spurio veniva contato come un appoggio, e il moto di volo
    % che lo accompagna - a velocita' di volo, non di appoggio - finiva
    % nello scivolamento: slip_tot risultava 0.75 m su 1.37 m percorsi,
    % cioe' il 55%, mentre il robot andava PIU' VELOCE del comando. Le due
    % cose non possono stare insieme, ed era il conteggio a essere sbagliato.
    %
    % Non si alza la soglia di forza: allungherebbe il problema dall'altro
    % lato, accorciando l'appoggio vero (la mediana e' gia' sotto il
    % nominale perche' la soglia taglia inizio e fine, dove la penetrazione
    % e' piccola). Si scartano i segmenti brevi.
    dmin = opt.durata_min_appoggio;
    if isnan(dmin)
        if isfield(cfg,'T') && isfield(cfg,'beta_stance')
            dmin = 0.25 * cfg.T * cfg.beta_stance;
        else
            dmin = 0.1;
        end
    end

    slip = 0;  nScartati = 0;  nSegmenti = 0;
    for i = 1:6
        ix = 3*(i-1) + (1:3);
        pfi = run.pf(:,ix);
        dxy = [0 0; diff(pfi(:,1:2))];

        [giuLungo, ns, nt] = appoggiLunghi(inContatto(:,i), t, dmin);
        nScartati = nScartati + ns;
        nSegmenti = nSegmenti + nt;

        giu = giuLungo & [false; giuLungo(1:end-1)];   % appoggio continuo
        slip = slip + sum(vecnorm(dxy(giu & sel,:), 2, 2));
    end
    D.slip_tot = slip;
    nPassi = max(1, round((t(end)-t(1))/cfg.T) * 6);
    D.slip_per_passo = slip / nPassi;
    D.appoggi_scartati = nScartati;
    D.appoggi_totali   = nSegmenti;

    if nSegmenti > 0 && nScartati/nSegmenti > 0.5
        fprintf(2,['[metriche] scartati %d appoggi su %d perche'' piu'' brevi di\n' ...
                   '           %.3f s: piu'' della meta''. Il contatto e'' troppo\n' ...
                   '           frammentato perche'' lo scivolamento significhi\n' ...
                   '           qualcosa.\n'], nScartati, nSegmenti, dmin);
    end

    if isfield(run,'meta') && isfield(run.meta,'impianto') && ...
            strcmpi(run.meta.impianto,'SRB')
        fprintf(2,['[metriche] impianto SRB: lo scivolamento e'' nullo per\n' ...
                   '           costruzione, non per merito del controllore.\n' ...
                   '           Non riportarlo come risultato.\n']);
    end
end

if ~isempty(inContatto) && ha('Fc')
    Fz = run.Fc(:,3:3:18);
    D.Fz_max_norm = max(Fz(sel,:), [], 'all') / (cfg.mass*cfg.g);
end

% distacchi non previsti: zampa SCHEDULATA in appoggio che non trasmette forza.
if ha('contact_sched') && ~isempty(inContatto)
    sched = logical(run.contact_sched);
    mancato = sched(sel,:) & ~inContatto(sel,:);
    D.distacchi      = nnz(diff([false(1,6); mancato]) == 1);   % episodi, non campioni
    D.frazione_persa = mean(mancato(:));
elseif ~isempty(inContatto)
    % senza lo schedule nominale resta solo un criterio indiretto: transizioni
    % appoggio -> volo piu' frequenti di quelle previste dal ciclo di andatura
    trans  = sum(diff(inContatto(sel,:)) == -1, 1);
    attese = (t(end)-t(1)) / cfg.T;
    D.distacchi = sum(max(0, trans - ceil(attese)));
end

% Dispersione del carico fra le zampe. La complanarita' cinematica non
% garantisce quella di carico: a 1x le sei zampe stanno entro il 7%, a 0.5x
% arrivano al 17% perche' il corpo si assesta dentro la cedevolezza del
% contatto trovando un equilibrio asimmetrico.
%
% La guardia ha('Fc') non c'era, e queste due righe erano le uniche a leggere
% run.Fc fuori da una guardia: su una run senza forze di contatto la funzione
% moriva con un errore di campo inesistente, invece di restituire NaN come
% promette la sua stessa intestazione. Fc resta OPZIONALE.
if ha('Fc')
    Fm = mean(run.Fc(sel, 3:3:18), 1);
    D.disp_carico = std(Fm) / max(mean(Fm), eps);
end

%% ---------- riga di tabella ----------
meta = struct('controller','', 'task','', 'run',1, 'seed',NaN, ...
              'condizione','nominale', 'terreno','?', 'note','');
if isfield(run,'meta')
    f = fieldnames(run.meta);
    for k = 1:numel(f), meta.(f{k}) = run.meta.(f{k}); end
end

% La colonna terreno non e' un ornamento: una campagna T2 e' girata su T5
% senza che nulla nei risultati lo dicesse.
riga = table( ...
    string(meta.controller), string(meta.task), string(meta.terreno), ...
    meta.run, meta.seed, string(meta.condizione), t(end)-t(1), ...
    'VariableNames', {'controller','task','terreno','ripetizione','seed', ...
                      'condizione','durata'});

riga = [riga, struct2table(A,'AsArray',true), ...
              struct2table(B,'AsArray',true), ...
              struct2table(C,'AsArray',true), ...
              struct2table(D,'AsArray',true)];

riga.note = string(meta.note);

dettaglio = struct('t',t, 'sel',sel, 'v',v, 'inContatto',inContatto, ...
                   'z_rif',z_rif, 'opt',opt);

end

%% ==================== helper ====================
function [giu, nScartati, nSegmenti] = appoggiLunghi(g, t, dmin)
%APPOGGILUNGHI  Il contatto, con i segmenti piu' brevi di dmin azzerati.
g   = logical(g(:));
giu = g;
ini = find(diff([false; g]) == 1);
fin = find(diff([g; false]) == -1);
n   = min(numel(ini), numel(fin));
nSegmenti = n;  nScartati = 0;
for k = 1:n
    if t(fin(k)) - t(ini(k)) < dmin
        giu(ini(k):fin(k)) = false;
        nScartati = nScartati + 1;
    end
end
end

function r = rms_(x)
x = x(:);  x = x(~isnan(x));
if isempty(x), r = NaN; else, r = sqrt(mean(x.^2)); end
end

function d = gradiente(p, t)
d = zeros(size(p));
for k = 1:size(p,2)
    d(:,k) = gradient(p(:,k), t);
end
end

function a = wrapToPi_(a)
a = mod(a + pi, 2*pi) - pi;
end

function y = yaw_atteso(run, t)
% imbardata attesa alla fine: integrale del comando, zero se non specificato
y = 0;
if isfield(run,'meta') && isfield(run.meta,'yaw_d') && ~isnan(run.meta.yaw_d)
    y = run.meta.yaw_d * (t(end) - t(1));
end
end