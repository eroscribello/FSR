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
%
% USCITA
%   riga        table 1 x M, una riga per run: e' quella che si impila per
%               fare la tabella della campagna
%   dettaglio   struttura con le serie temporali intermedie, per i grafici
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
             'soglia_fermo',0.02, 'soglia_Fz',0.5);
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

ribaltato = any(abs(run.rpy(:,1)) > opt.soglia_rib) || ...
            any(abs(run.rpy(:,2)) > opt.soglia_rib);
fermo     = A.distanza < opt.soglia_fermo;
A.successo = ~(ribaltato || fermo);
A.causa_fallimento = "";
if ribaltato, A.causa_fallimento = "ribaltamento"; end
if fermo,     A.causa_fallimento = "fermo"; end

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
           'frazione_persa',NaN, 'Fz_max_norm',NaN, 'appoggio_medio',NaN);
inContatto = [];
if ha('contact')
    inContatto = logical(run.contact);
elseif ha('Fc')
    inContatto = run.Fc(:,3:3:18) > opt.soglia_Fz;
end

if ~isempty(inContatto)
    D.appoggio_medio = mean(sum(inContatto(sel,:),2));   % zampe a terra in media
end

if ~isempty(inContatto) && ha('pf')
    % scivolamento: spostamento orizzontale del piede mentre e' in appoggio
    slip = 0;
    for i = 1:6
        ix = 3*(i-1) + (1:3);
        pfi = run.pf(:,ix);
        dxy = [0 0; diff(pfi(:,1:2))];
        giu = inContatto(:,i) & [false; inContatto(1:end-1,i)];  % appoggio continuo
        slip = slip + sum(vecnorm(dxy(giu & sel,:), 2, 2));
    end
    D.slip_tot = slip;
    nPassi = max(1, round((t(end)-t(1))/cfg.T) * 6);
    D.slip_per_passo = slip / nPassi;

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

%% ---------- riga di tabella ----------
meta = struct('controller','', 'task','', 'run',1, 'seed',NaN, ...
              'condizione','nominale', 'note','');
if isfield(run,'meta')
    f = fieldnames(run.meta);
    for k = 1:numel(f), meta.(f{k}) = run.meta.(f{k}); end
end

riga = table( ...
    string(meta.controller), string(meta.task), meta.run, meta.seed, ...
    string(meta.condizione), t(end)-t(1), ...
    'VariableNames', {'controller','task','ripetizione','seed','condizione','durata'});

riga = [riga, struct2table(A,'AsArray',true), ...
              struct2table(B,'AsArray',true), ...
              struct2table(C,'AsArray',true), ...
              struct2table(D,'AsArray',true)];

riga.note = string(meta.note);

dettaglio = struct('t',t, 'sel',sel, 'v',v, 'inContatto',inContatto, ...
                   'z_rif',z_rif, 'opt',opt);

end

%% ==================== helper ====================
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