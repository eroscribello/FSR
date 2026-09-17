function I = inseguimento(opt)
%INSEGUIMENTO  I giunti inseguono il comando, o la finestra era sfasata?
%
%   I = inseguimento
%   I = inseguimento(struct('fattori',[1.0 1.5 2.0]))
%
% PERCHE' SERVE
%   causa_2x ha dato sweep = -1.98 mm a 2x contro -60 attesi, e ne ha
%   concluso che i giunti non inseguono. Quella conclusione NON e' sostenuta,
%   e il difetto e' mio: sweep e' misurato su una finestra, e a 2x la finestra
%   e' inaffidabile.
%
%   contact_sched e' agganciato da adatta_simscape cercando la fase che
%   massimizza l'accordo col contatto osservato. A 2x la finestra comandata e'
%   toccata solo al 37%, quindi quell'aggancio ha poco su cui appoggiarsi. Se
%   la finestra straddia la transizione appoggio/volo, sweep somma un pezzo di
%   corsa di appoggio (-S) e un pezzo di corsa di volo (+S), e la media tende
%   a ZERO. -1.98 mm e' indistinguibile da quello.
%
%   L'identita' Dcorpo = slip - sweep chiude al 2%, ma NON smentisce questo:
%   e' vera punto per punto, quindi torna per qualunque finestra. Valida il
%   cambio di frame, non la fase.
%
% LE DUE MISURE CHE NON DIPENDONO DALLA FINESTRA
%   1. ESCURSIONE PICCO-PICCO del piede nel frame corpo, su tutta la run.
%      Se i giunti inseguono, il piede spazza S in x e H in z QUALUNQUE sia
%      la fase, perche' il picco-picco non ha bisogno di sapere quando inizia
%      l'appoggio. Se l'escursione c'e', i giunti inseguono e il verdetto di
%      causa_2x era un artefatto della mia finestra. Se collassa, i giunti
%      davvero non inseguono.
%   2. MODO DI ATTUAZIONE dei giunti. Se sono attuati in MOTO (posizione
%      imposta), inseguono per costruzione e un ritardo e' impossibile: in
%      quel caso l'ipotesi A e' esclusa a priori e non serve nemmeno
%      misurarla. Se sono attuati in COPPIA, un ritardo e' possibile e va
%      quantificato.
%
% IN PIU'
%   - ampiezza dell'oscillazione verticale del corpo: i "saltelli" visti in
%     animazione a 1.5x sono una misura, non un'impressione;
%   - accordo fra contatto comandato e osservato, per dire quanto valeva la
%     fase che causa_2x ha usato.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1, opt = struct(); end
cfg = phantomx_config();

def = struct('fattori', [1.0 1.5 2.0], ...
             'nCicli',  10, ...
             'mdl',     'phantomx_sim_zero', ...
             'verbose', true);
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt,f{k}), opt.(f{k}) = def.(f{k}); end
end

fprintf('\n========== I GIUNTI INSEGUONO? ==========\n');
fprintf('  atteso: escursione %.1f mm in x, %.1f mm in z\n', 1e3*cfg.S, 1e3*cfg.H);

%% ---- 0. i giunti sono attuati in moto o in coppia? ----
% Questo viene PRIMA di ogni misura: se il moto e' imposto, i giunti
% inseguono per costruzione e l'ipotesi A e' esclusa senza simulare nulla.
fprintf('\n--- modo di attuazione dei giunti ---\n');
load_system(opt.mdl);
modo = modoAttuazione(opt.mdl);

%% ---- le run ----
applica_terreno('T2', false, opt.mdl);
zampe = {'rf','rm','rr','lf','lm','lr'};
I = table();

for iF = 1:numel(opt.fattori)
    fatt  = opt.fattori(iF);
    v_cmd = fatt * cfg.v_nom;
    T_i   = cfg.S / (cfg.beta_stance * v_cmd);
    stop  = opt.nCicli * T_i;

    fprintf('\n--- %.2fx :  T = %.3f s ---\n', fatt, T_i);
    try
        assignin('base','OVERRIDE_C2', false);
        assignin('base','OVERRIDE_GAIT', struct('T',T_i, 'H',cfg.H));
        evalin('base','init_gait');
        out = sim(opt.mdl, 'StopTime', num2str(stop));
        r = adatta_simscape(out, struct('task','T2', 'run',1, ...
                'condizione', sprintf('v%.2fx', fatt), ...
                'vel_d', [v_cmd 0]), struct('verbose',false));
    catch ME
        fprintf(2,'  la run e'' fallita: %s\n', ME.message);
        continue
    end

    if ~isfield(r,'pf') || isempty(r.pf) || ~isfield(r,'q') || isempty(r.q)
        fprintf(2,'  servono pf e q: lancia init_gait\n');
        continue
    end

    t   = r.t;
    reg = t >= 2*T_i;
    fb  = piedeNelCorpo(r.pf, r.p, r.rpy);

    % ---- 1. escursione picco-picco, per zampa, CICLO PER CICLO ----
    % Non su tutta la run: su tutta la run il picco-picco include la deriva e
    % sovrastima. Per ciclo e' la grandezza giusta, e la sua mediana e'
    % robusta ai cicli anomali.
    escX = nan(6,1);  escZ = nan(6,1);  escQ = nan(6,3);
    for iz = 1:6
        c = 3*(iz-1) + (1:3);
        escX(iz) = ppPerCiclo(fb(reg, c(1)), t(reg), T_i);
        escZ(iz) = ppPerCiclo(fb(reg, c(3)), t(reg), T_i);
        for j = 1:3
            escQ(iz,j) = ppPerCiclo(r.q(reg, 3*(iz-1)+j), t(reg), T_i);
        end
    end

    % ---- 2. il corpo saltella? ----
    zc   = r.p(reg,3);
    ppZ  = ppPerCiclo(zc, t(reg), T_i);
    rmsZ = std(zc);

    % ---- 3. quanto valeva la fase che causa_2x ha usato ----
    accordo = NaN;
    if isfield(r,'contact') && isfield(r,'contact_sched') ...
            && ~isempty(r.contact) && ~isempty(r.contact_sched)
        a = logical(r.contact(reg,:));  b = logical(r.contact_sched(reg,:));
        accordo = mean(a(:) == b(:));
    end

    fprintf('  piede nel frame corpo: x %.1f mm (atteso %.1f), z %.1f mm (atteso %.1f)\n', ...
            1e3*median(escX), 1e3*cfg.S, 1e3*median(escZ), 1e3*cfg.H);
    fprintf('  giunti: coxa %.1f deg, femore %.1f deg, tibia %.1f deg (picco-picco)\n', ...
            rad2deg(median(escQ(:,1))), rad2deg(median(escQ(:,2))), rad2deg(median(escQ(:,3))));
    fprintf('  corpo in z: %.1f mm picco-picco, %.1f mm rms\n', 1e3*ppZ, 1e3*rmsZ);
    if ~isnan(accordo)
        fprintf('  accordo contatto comandato/osservato: %.0f%%\n', 100*accordo);
    end

    for iz = 1:6
        I = [I; table(fatt, string(zampe{iz}), T_i, ...
                1e3*escX(iz), 1e3*escZ(iz), ...
                rad2deg(escQ(iz,1)), rad2deg(escQ(iz,2)), rad2deg(escQ(iz,3)), ...
                1e3*ppZ, 1e3*rmsZ, accordo, string(modo), ...
            'VariableNames', {'fattore','zampa','T','escX_mm','escZ_mm', ...
                              'coxa_deg','femore_deg','tibia_deg', ...
                              'corpoZ_pp_mm','corpoZ_rms_mm','accordo','attuazione'})];  %#ok<AGROW>
    end
end

evalin('base','clear OVERRIDE_GAIT OVERRIDE_C2');
evalin('base','init_gait');

if height(I) == 0
    fprintf(2,'\nnessuna run completata\n');
    return
end

%% ---- il quadro ----
fprintf('\n=============== QUADRO ===============\n');
R = table();
for iF = 1:numel(opt.fattori)
    s = I(I.fattore == opt.fattori(iF), :);
    if isempty(s), continue; end
    R = [R; table(opt.fattori(iF), median(s.escX_mm), median(s.escZ_mm), ...
            100*median(s.escX_mm)/(1e3*cfg.S), ...
            median(s.femore_deg), s.corpoZ_pp_mm(1), s.accordo(1)*100, ...
        'VariableNames', {'fattore','escX_mm','escZ_mm','escX_pct', ...
                          'femore_deg','corpoZ_pp_mm','accordo_pct'})];   %#ok<AGROW>
end
disp(R)

%% ---- il verdetto ----
fprintf('\n=============== VERDETTO ===============\n');
fine = R(end,:);
prima = R(1,:);
pctFine = fine.escX_pct;

if strcmpi(modo,'moto')
    fprintf(['  I giunti sono attuati in MOTO: la posizione e'' imposta e un\n' ...
             '  ritardo di inseguimento e'' impossibile per costruzione.\n' ...
             '  L''ipotesi A era esclusa a priori, e il mio verdetto\n' ...
             '  precedente non poteva essere giusto.\n\n']);
end

if pctFine > 85
    fprintf(2,'  I GIUNTI INSEGUONO. IL VERDETTO DI causa_2x ERA UN ARTEFATTO.\n');
    fprintf([ ...
      '  A %.2fx il piede spazza %.1f mm in x, il %.0f%% del passo nominale,\n' ...
      '  quindi la corsa comandata VIENE realizzata. Il valore sweep = -1.98\n' ...
      '  mm veniva da una finestra sfasata, non dai giunti: l''accordo fra\n' ...
      '  contatto comandato e osservato e'' %.0f%%, troppo basso per agganciare\n' ...
      '  la fase.\n\n' ...
      '  Resta quindi l''ipotesi C: il robot PERDE L''APPOGGIO. Il corpo\n' ...
      '  oscilla di %.1f mm picco-picco in z (a %.2fx erano %.1f), coerente\n' ...
      '  con i saltelli visti in animazione, e con meno di tre piedi a terra\n' ...
      '  il 61%% del tempo il modello quasi-statico non e'' piu'' applicabile.\n\n' ...
      '  In relazione: a %.2fx il tripode NON REGGE L''ANDATURA. Limite del\n' ...
      '  controllore, ma va scritto cosi'' e non come "scivola": lo\n' ...
      '  scivolamento e'' una conseguenza della perdita di appoggio, non la\n' ...
      '  causa.\n'], ...
      fine.fattore, fine.escX_mm, pctFine, fine.accordo_pct, ...
      fine.corpoZ_pp_mm, prima.fattore, prima.corpoZ_pp_mm, fine.fattore);
elseif pctFine < 50
    fprintf(2,'  I GIUNTI NON INSEGUONO DAVVERO.\n');
    fprintf([ ...
      '  A %.2fx il piede spazza %.1f mm invece di %.1f: il %.0f%%. La misura\n' ...
      '  e'' indipendente dalla finestra, quindi questa volta la conclusione\n' ...
      '  regge: il comando si perde prima dei giunti.\n\n' ...
      '  Il femore spazza %.1f deg contro %.1f a %.2fx. Prossimo passo:\n' ...
      '  saturazione di coppia (guarda run.tau contro il limite del giunto) o\n' ...
      '  tempo di campionamento del generatore di traiettoria.\n\n' ...
      '  In relazione: LIMITE DELLA CATENA DI COMANDO. La cella 2x va\n' ...
      '  esclusa, non attribuita a C1.\n'], ...
      fine.fattore, fine.escX_mm, 1e3*cfg.S, pctFine, ...
      fine.femore_deg, prima.femore_deg, prima.fattore);
else
    fprintf(2,'  INSEGUIMENTO PARZIALE.\n');
    fprintf([ ...
      '  Il piede spazza il %.0f%% del passo: i giunti inseguono in parte.\n' ...
      '  Va riportato come concausa, insieme alla perdita di appoggio\n' ...
      '  (corpo %.1f mm picco-picco in z, accordo di fase %.0f%%), e il\n' ...
      '  confronto a %.2fx va dichiarato debole invece di attribuito a una\n' ...
      '  causa sola.\n'], ...
      pctFine, fine.corpoZ_pp_mm, fine.accordo_pct, fine.fattore);
end
fprintf('\n');

if opt.verbose
    figure;
    subplot(2,1,1); hold on; grid on
    plot(R.fattore, R.escX_mm, 'o-', 'DisplayName','escursione x del piede');
    plot(R.fattore, R.escZ_mm, 's-', 'DisplayName','escursione z del piede');
    yline(1e3*cfg.S,'r:','DisplayName','S atteso');
    yline(1e3*cfg.H,'m:','DisplayName','H atteso');
    ylabel('[mm]'); legend('Location','best');
    title('Inseguimento: indipendente dalla fase');
    subplot(2,1,2); hold on; grid on
    plot(R.fattore, R.corpoZ_pp_mm, 'o-', 'DisplayName','corpo in z picco-picco');
    plot(R.fattore, R.accordo_pct,  's-', 'DisplayName','accordo di fase [%]');
    xlabel('velocita'' [x]'); legend('Location','best');
    title('Saltelli del corpo e affidabilita'' della fase');
end

end

%% ================================================================
function modo = modoAttuazione(mdl)
%MODOATTUAZIONE  I giunti sono attuati in moto (posizione imposta) o in coppia?
modo = 'ignoto';
b = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                'IncludeCommented','on', 'Type','block');
nMoto = 0;  nCoppia = 0;  mostrati = 0;
for k = 1:numel(b)
    nm = get_param(b{k}, 'Name');
    if isempty(regexpi(nm, 'revolute|prismatic|joint', 'once')), continue; end
    try, pars = fieldnames(get_param(b{k},'DialogParameters')); catch, continue; end
    for j = 1:numel(pars)
        if isempty(regexpi(pars{j}, 'actuat', 'once')), continue; end
        try, v = get_param(b{k}, pars{j}); catch, continue; end
        if ~ischar(v) || isempty(v), continue; end
        if mostrati < 8
            fprintf('  %-38s %-28s = %s\n', ...
                    abbrevia(strrep(nm,newline,' '),38), pars{j}, v);
            mostrati = mostrati + 1;
        end
        if ~isempty(regexpi(pars{j},'motion','once')) && ...
                ~isempty(regexpi(v,'input|provided','once'))
            nMoto = nMoto + 1;
        end
        if ~isempty(regexpi(pars{j},'torque','once')) && ...
                ~isempty(regexpi(v,'input|provided','once'))
            nCoppia = nCoppia + 1;
        end
    end
end
if nMoto > 0 && nCoppia == 0
    modo = 'moto';
elseif nCoppia > 0 && nMoto == 0
    modo = 'coppia';
elseif nMoto > 0 && nCoppia > 0
    modo = 'misto';
end
fprintf('  -> attuazione: %s   (moto %d, coppia %d)\n', modo, nMoto, nCoppia);
if strcmp(modo,'ignoto')
    fprintf(['     Non letto dai parametri: non e'' un problema, la misura\n' ...
             '     picco-picco qui sotto non ne ha bisogno.\n']);
end
end

function fb = piedeNelCorpo(pf, pb, rpy)
%PIEDENELCORPO  p_corpo = R' * (p_piede - p_corpo), con la R di adatta_simscape.
fb = nan(size(pf));
for n = 1:size(pf,1)
    R = rotZYX(rpy(n,:));
    for i = 1:6
        c = 3*(i-1) + (1:3);
        fb(n,c) = (R.' * (pf(n,c).' - pb(n,:).')).';
    end
end
end

function pp = ppPerCiclo(x, t, T)
%PPPERCICLO  Mediana del picco-picco calcolato ciclo per ciclo.
%
%   Su tutta la run il picco-picco include la deriva del corpo e sovrastima.
%   La mediana fra i cicli e' robusta ai cicli anomali, che a 2x ci sono.
x = x(:);  t = t(:);
if isempty(x), pp = NaN; return; end
bordi = t(1):T:t(end);
if numel(bordi) < 2, pp = max(x) - min(x); return; end
v = nan(numel(bordi)-1, 1);
for k = 1:numel(bordi)-1
    m = t >= bordi(k) & t < bordi(k+1);
    if sum(m) < 3, continue; end
    v(k) = max(x(m)) - min(x(m));
end
pp = median(v, 'omitnan');
if isnan(pp), pp = max(x) - min(x); end
end

function R = rotZYX(a)
cr = cos(a(1)); sr = sin(a(1));
cp = cos(a(2)); sp = sin(a(2));
cy = cos(a(3)); sy = sin(a(3));
R = [ cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr ; ...
      sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr ; ...
        -sp,            cp*sr,            cp*cr ];
end

function s = abbrevia(s, n)
if numel(s) > n, s = ['...' s(end-n+4:end)]; end
end
