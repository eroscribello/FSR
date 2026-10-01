function F = fattibilita(opt)
%FATTIBILITA  I controllori girerebbero sui motori veri? Dai CSV, senza simulare.
%
%   fattibilita
%   F = fattibilita(struct('tau_lim', 1.5))
%
% PERCHE' NON SI PUO' "SIMULARE LA SATURAZIONE" E BASTA
%   [28/9] In C1 e C2 i giunti sono attuati in POSIZIONE
%   (MotionActuationMode = InputMotion): il solutore impone la traiettoria e
%   la coppia e' una REAZIONE, calcolata a posteriori per far tornare i conti.
%   Non c'e' nessun ingresso di coppia su cui mettere un blocco Saturation, e
%   il Revolute Joint di Simscape non ha un parametro di coppia massima.
%
%   Per saturare davvero bisognerebbe passare i giunti ad attuazione in
%   coppia e chiudere attorno un anello di posizione con il limite dentro.
%   Ma quello e' un IMPIANTO DIVERSO - 24 gradi di liberta' invece di 6, 48
%   stati invece di 12 - ed e' la copia che abbiamo gia' costruito e poi
%   ritirato a settembre (docs/piano_mpc_simscape.md §3b). Soprattutto:
%   C1 e C2 con un anello di posizione attorno alla coppia NON sono piu' C1
%   e C2. Si confronterebbero quattro controllori, non due saturi.
%
% COSA MISURA QUESTO FILE, ALLORA
%   La coppia RICHIESTA dal controllore, contro quella che il motore puo'
%   dare. E' un indicatore di FATTIBILITA', non una simulazione:
%
%     dice      "questo controllore chiede 2.8 volte il limite del motore,
%                per il 12% dei campioni"
%     NON dice  "con i motori veri il robot cadrebbe cosi'"
%
%   La differenza va detta in relazione. Quello che si puo' concludere e'
%   che una richiesta molto sopra il limite rende la run NON REALIZZABILE
%   cosi' com'e'; come degraderebbe esattamente, non lo sappiamo.
%
% DA DOVE VENGONO I NUMERI
%   Da results/<task>_<controllore>.csv, cioe' dalle run di campagna gia'
%   fatte. metriche calcola gia' tutto:
%     tau_max                  picco su tutti i giunti e tutti gli istanti
%     tau_rms                  valore efficace
%     frazione_saturo          frazione di campioni-giunto sopra cfg.tau_max
%     tau_rms_giunto_peggiore  il giunto piu' caricato
%     potenza_max              picco di potenza meccanica
%   Nessuna simulazione: zero rischio di cambiare qualcosa per sbaglio.
%
% I DUE LIMITI, PERCHE' NON SONO D'ACCORDO
%   cfg.tau_max = 1.5 N*m e' il valore da datasheet del servo; l'URDF ne
%   dichiara 2.8. La tabella riporta il rapporto su entrambi, cosi' la
%   conclusione non dipende da quale dei due si sceglie - e se cambia, si
%   vede subito.
%
% [1/10] C3
%   LA DOMANDA: l'anello d'assetto somma un termine alla quota di ogni
%   piede, quindi chiede ai giunti un lavoro che C2 non chiede. Sta ancora
%   dentro il limite del servo?
%   PREVISIONE, SCRITTA PRIMA DI LANCIARE: C3 spende il 37-54% di energia in
%   piu' di C2 (stato_progetto §4), ma tau_rms di C3 e' leggermente PIU'
%   BASSO (-2/-7%, dichiarabile). Quindi il carico sostenuto dovrebbe restare
%   dove sta per C1 e C2 (41-46% del limite sul giunto peggiore); i picchi
%   possono salire (T3: 3.5 N*m contro 2.6 di C2).
%   STESSE SOGLIE di sempre, gia' nel codice: giunto peggiore < 70% del limite
%   = la marcia sta dentro; 70-100% = dentro senza margine; > 100% = fuori.
%   C3 e' attuato come C1 e C2 (giunti in posizione, stesso impianto): la
%   coppia e' una reazione anche qui, e vale la stessa avvertenza.
%   COSA E' CAMBIATO NEL CODICE: i controllori non sono piu' due fissi.
%   Il riassunto, la figura e il confronto dei picchi girano su opt.ctrl, e
%   il confronto si fa fra coppie ADIACENTI (C1-C2, C2-C3).
%
% [1/10, sera] LA SECONDA SOGLIA: QUELLA DEL COSTRUTTORE
%   Il 70% qui sotto e' una soglia NOSTRA, mai giustificata. Il manuale
%   ROBOTIS dell'AX-12A (docs/ROBOTIS_AX-12A_emanual.pdf, pag. 3) dice:
%     "Stall torque is the maximum instantaneous and static torque. Stable
%      motions are possible with robots designed for loads with 1/5 or less
%      of the stall torque."
%   cioe' cfg.tau_lavoro = 0.3 N*m. Il carico SOSTENUTO va confrontato con
%   quella, non con lo stallo: lo stallo resta il riferimento per i picchi.
%   Le due soglie vengono riportate insieme; la conclusione sulla marcia la
%   decide quella del costruttore.
%   PREVISIONE, SCRITTA PRIMA DI LANCIARE: sui numeri di prima dello slew
%   rate il giunto peggiore stava al 38-46% dello stallo, cioe' 1.9-2.3
%   volte tau_lavoro. Lo slew rate non cambia la coppia sostenuta (abbassa i
%   picchi): ci si aspetta lo stesso, cioe' "LA MARCIA SUPERA IL CARICO
%   RACCOMANDATO" per tutti e tre. Se fosse cosi', la frase "la marcia sta
%   dentro con margine" di stato_progetto §6 va ritirata.
%
% USO
%   fattibilita                       % tutti i task, C1, C2 e C3
%   F = fattibilita;  disp(F)
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1, opt = struct(); end
cfg = phantomx_config();
def = struct('tau_lim', cfg.tau_max, 'tau_lim_urdf', 2.8, 'tau_lavoro', cfg.tau_lavoro, ...
             'task', {{'T2','T3','T4','T4D','T5','T6','T7'}}, ...
             'ctrl', {{'C1','C2','C3'}}, 'grafico', true);
% [CORRETTO 29/9] LA CELLA NOMINALE, E PERCHE' SERVE.
%   Prendere il massimo su tutte le celle di un task fa dominare la tabella
%   dalla cella piu' stressata: su T2 e' v1.50x, dove il tripode e' gia'
%   oltre il suo limite (archivio/README.md: l'ultima cella con moto del
%   corpo stabile e' 1.20x) e la coppia di picco vale 14.6 N*m, dieci volte
%   il limite. Quel numero descrive un'andatura che non funziona, non un
%   controllore che chiede troppo. Il confronto si fa al PUNTO DI LAVORO
%   NOMINALE, e le celle estreme si riportano a parte.
nom = struct('T2','v1.00x', 'T3','yaw+0.100', 'T7','dv0.00');
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt,f{k}) || isempty(opt.(f{k})), opt.(f{k}) = def.(f{k}); end
end

F = table();
for i = 1:numel(opt.task)
    for j = 1:numel(opt.ctrl)
        p = fullfile('results', sprintf('%s_%s.csv', opt.task{i}, opt.ctrl{j}));
        if ~isfile(p), continue; end
        T = readtable(p, 'TextType','string');
        for r = 1:height(T)
            g = table();
            g.task    = string(opt.task{i});
            g.ctrl    = string(opt.ctrl{j});
            g.cella   = fa_campo(T, r, 'condizione', "-");
            g.tau_max = fa_num(T, r, 'tau_max');
            g.tau_rms = fa_num(T, r, 'tau_rms');
            g.frazione_saturo = fa_num(T, r, 'frazione_saturo');
            g.potenza_max     = fa_num(T, r, 'potenza_max');
            g.volte_datasheet = g.tau_max / opt.tau_lim;
            g.volte_urdf      = g.tau_max / opt.tau_lim_urdf;
            g.rms_su_limite   = g.tau_rms / opt.tau_lim;
            % [AGGIUNTO 29/9] IL GIUNTO PEGGIORE, NON LA MEDIA SUI 18.
            %   tau_rms media su tutti i giunti, e la coxa porta quasi niente
            %   (misurato validando la soglia di contatto: il contatto e'
            %   verticale, la coxa ruota attorno all'asse verticale). Mediare
            %   su diciotto giunti di cui sei scarichi sottostima quello che
            %   lavora. Il servo che si brucia e' il piu' caricato, non quello
            %   medio: la fattibilita' si giudica su questa colonna.
            g.tau_rms_peggiore = fa_num(T, r, 'tau_rms_giunto_peggiore');
            g.peggiore_su_limite = g.tau_rms_peggiore / opt.tau_lim;
            % [1/10] contro il carico raccomandato dal costruttore (1/5 stallo)
            g.peggiore_su_lavoro = g.tau_rms_peggiore / opt.tau_lavoro;
            g.rms_su_lavoro      = g.tau_rms / opt.tau_lavoro;
            % una run fallita non dice niente sulla coppia richiesta: se il
            % robot sta fermo la coppia e' bassa, e non e' una buona notizia
            ft = fa_num(T, r, 'frazione_task');
            cf = fa_campo(T, r, 'causa_fallimento', "");
            g.valida = ~(isnan(ft) || ft < 0.5 || strlength(strtrim(cf)) > 0);
            if isfield(nom, opt.task{i})
                g.nominale = (g.cella == string(nom.(opt.task{i})));
            else
                g.nominale = true;
            end
            F = [F; g];                                            %#ok<AGROW>
        end
    end
end

if isempty(F)
    fprintf(2,'\n  Nessun CSV di campagna trovato in results/.\n\n');
    return
end

%% ---- lettura ----
fprintf('\n=============== FATTIBILITA'' SUI MOTORI VERI ===============\n');
fprintf('  limite datasheet %.2f N*m   (l''URDF dichiara %.2f)\n\n', ...
        opt.tau_lim, opt.tau_lim_urdf);
fprintf('  %-6s %-5s %-10s %8s %8s %9s %9s %9s %8s\n', 'task', 'ctrl', 'cella', ...
        'rms_18', 'rms/lim', 'rms_peggio', 'peggio/lim', 'max/lim', 'sopra %');
for k = 1:height(F)
    mk = ' ';
    if F.nominale(k), mk = '*'; end
    if ~F.valida(k),  mk = '!'; end
    fprintf('%s %-6s %-5s %-10s %8.3f %7.0f%% %9.3f %9.0f%% %8.1f %7.2f%%\n', mk, ...
            F.task(k), F.ctrl(k), extractBefore(F.cella(k)+"          ", 11), ...
            F.tau_rms(k), 100*F.rms_su_limite(k), F.tau_rms_peggiore(k), ...
            100*F.peggiore_su_limite(k), F.volte_datasheet(k), 100*F.frazione_saturo(k));
end
fprintf('  * = punto di lavoro nominale    ! = run fallita, i numeri non si leggono\n');

%% ---- il riassunto per task: il confronto qualitativo che serve ----
fprintf('\n---- al punto di lavoro nominale ----\n');
nc = numel(opt.ctrl);
fprintf('  %-6s', 'task');
for j = 1:nc, fprintf(' %32s', sprintf('%s  peggio/lim, picco, sopra%%', opt.ctrl{j})); end
fprintf('\n');
N = F(F.nominale & F.valida, :);
for i = 1:numel(opt.task)
    r = N(N.task == opt.task{i}, :);
    if isempty(r), continue; end
    s = strings(1,nc);
    for j = 1:nc
        q = r(r.ctrl == opt.ctrl{j}, :);
        if isempty(q), s(j) = "-";
        else
            [~, kk] = max(q.tau_max);
            % [1/10] il giunto peggiore, non la media sui 18: e' la colonna su
            % cui si giudica (vedi sopra), e il riassunto deve mostrare quella
            s(j) = sprintf('%.0f%%   %.1fx   %.2f%%', 100*q.peggiore_su_limite(kk), ...
                           q.volte_datasheet(kk), 100*q.frazione_saturo(kk));
        end
    end
    fprintf('  %-6s', opt.task{i});  fprintf(' %32s', s);  fprintf('\n');
end

%% ---- le conclusioni che i numeri sostengono, e quelle che no ----
% [CORRETTO 29/9] LA PRIMA VERSIONE GUARDAVA SOLO IL PICCO, E CONCLUDEVA MALE.
%   Diceva "nessuno dei due e' realizzabile" appena tau_max superava il
%   limite. Ma tau_max e' il massimo su tutti i giunti e tutti gli istanti:
%   basta un urto. Il numero che dice se il motore regge la marcia e' il
%   VALORE EFFICACE, e accanto la frazione di tempo sopra il limite.
%   Sono due domande diverse e vanno separate.
rms_med = mean(N.rms_su_limite, 'omitnan');
peg_med = mean(N.peggiore_su_limite, 'omitnan');
peg_max = max(N.peggiore_su_limite);
sat_max = max(N.frazione_saturo);
pic     = N.volte_datasheet;

fprintf('\n---- cosa si puo'' dire ----\n');
fprintf('  coppia efficace media sui 18 giunti:       %.0f%% del limite\n', 100*rms_med);
fprintf('  coppia efficace del GIUNTO PEGGIORE:       %.0f%% del limite (max %.0f%%)\n', ...
        100*peg_med, 100*peg_max);
fprintf('  frazione di campioni sopra il limite:      da %.2f%% a %.2f%%\n', ...
        100*min(N.frazione_saturo), 100*sat_max);
fprintf('  picco richiesto:                           da %.1fx a %.1fx il limite\n', ...
        min(pic), max(pic));

% [1/10] il carico sostenuto contro la raccomandazione del costruttore
peg_lav_min = min(N.peggiore_su_lavoro);  peg_lav_max = max(N.peggiore_su_lavoro);
rms_lav     = mean(N.rms_su_lavoro, 'omitnan');
fprintf(['\n  CONTRO IL COSTRUTTORE (carico per moto stabile = 1/5 dello stallo,\n' ...
         '  %.2f N*m, manuale ROBOTIS AX-12A):\n'], opt.tau_lavoro);
fprintf('  giunto peggiore:                           da %.1fx a %.1fx quel carico\n', ...
        peg_lav_min, peg_lav_max);
fprintf('  media sui 18 giunti:                       %.1fx quel carico\n', rms_lav);
if peg_lav_max <= 1
    fprintf(['  -> LA MARCIA STA NEL CARICO RACCOMANDATO, anche sul giunto piu''\n' ...
             '     caricato.\n']);
else
    fprintf(['  -> LA MARCIA SUPERA IL CARICO RACCOMANDATO dal costruttore, fino a\n' ...
             '     %.1f volte sul giunto piu'' caricato. Resta sotto lo stallo, quindi\n' ...
             '     non e'' impossibile: ma un servo vero lavorerebbe fuori dalla zona\n' ...
             '     indicata per un moto stabile, e scalderebbe. Va detto cosi''.\n'], peg_lav_max);
end

fprintf('\n  CONTRO LO STALLO, DUE REGIMI, E VANNO DETTI SEPARATI:\n');
if peg_max < 0.7
    % [1/10] "con margine" era il giudizio sul 70%, soglia nostra: il margine
    % vero lo dice la riga del costruttore qui sopra
    fprintf(['  - SOTTO LO STALLO. Anche il giunto piu'' caricato resta al\n' ...
             '    %.0f%% dello stallo (soglia nostra 70%%), per tutti i controllori.\n'], 100*peg_max);
elseif peg_max < 1
    fprintf(['  - LA MARCIA STA DENTRO MA SENZA MARGINE. Il giunto piu'' caricato\n' ...
             '    arriva al %.0f%% del limite in continuo: sopra il 70%% un servo\n' ...
             '    scalda e degrada, quindi e'' un margine da dichiarare, non da\n' ...
             '    dare per buono.\n'], 100*peg_max);
else
    fprintf(['  - LA MARCIA NON STA DENTRO. Il giunto piu'' caricato chiede il\n' ...
             '    %.0f%% del limite in CONTINUO, non solo negli urti. Questo\n' ...
             '    cambia la conclusione: non e'' un problema di picchi.\n'], 100*peg_max);
end
fprintf(['  - GLI URTI NO. Il picco arriva a %.1f volte il limite, ma solo sul\n' ...
         '    %.2f%% dei campioni al massimo: sono transitori di impatto, non\n' ...
         '    una richiesta continua. Un servo vero li taglierebbe.\n'], max(pic), 100*sat_max);

% chi chiede di piu', a parita' di task nominale - [1/10] per coppie adiacenti
for j2 = 1:nc-1
    pa = opt.ctrl{j2};  pb = opt.ctrl{j2+1};
    pc = 0; ps = 0; pt = 0;
    for i2 = 1:numel(opt.task)
        a = N(N.task==opt.task{i2} & N.ctrl==pa, :);
        b = N(N.task==opt.task{i2} & N.ctrl==pb, :);
        if isempty(a) || isempty(b), continue; end
        pt = pt + 1;
        if max(b.tau_max) > max(a.tau_max), pc = pc + 1; end
        if max(b.frazione_saturo) < max(a.frazione_saturo), ps = ps + 1; end
    end
    fprintf(['\n  %s chiede un picco piu'' alto di %s in %d task su %d;\n' ...
             '  ha una frazione sopra il limite piu'' bassa in %d task su %d.\n'], ...
            pb, pa, pc, pt, ps, pt);
end
fprintf(['  [29/9, C1-C2] la coppia EFFICACE e'' praticamente identica fra i due, e\n' ...
         '  la frazione sopra il limite di C2 e'' spesso PIU'' BASSA: la ricerca del\n' ...
         '  terreno concentra la richiesta in urti piu'' netti e piu'' rari,\n' ...
         '  invece di distribuirla. E'' un''osservazione, non ancora una spiegazione.\n']);

fprintf(['\n  QUELLO CHE NON SI PUO'' DIRE: come si comporterebbero con i motori\n' ...
         '  veri. Con i giunti attuati in posizione la coppia e'' una reazione,\n' ...
         '  non un ingresso, quindi non c''e'' niente da saturare. Tagliare quei\n' ...
         '  picchi cambierebbe la traiettoria, e di quanto non lo sappiamo.\n\n']);

if ~isfolder(fullfile('results','diagnostica')), mkdir(fullfile('results','diagnostica')); end
writetable(F, fullfile('results','diagnostica','fattibilita.csv'));
fprintf('  scritto  %s\n', fullfile('results','diagnostica','fattibilita.csv'));

%% ---- figura ----
if opt.grafico
    fig = figure('Name','fattibilita','Position',[80 80 1050 400]);
    tiledlayout(fig, 1, 2, 'TileSpacing','compact', 'Padding','compact');
    et = string(opt.task);
    vr = nan(numel(et), nc);  vp = nan(numel(et), nc);
    for i3 = 1:numel(et)
        for j3 = 1:nc
            q = N(N.task==et(i3) & N.ctrl==opt.ctrl{j3}, :);
            if ~isempty(q)
                vr(i3,j3) = max(q.peggiore_su_limite);
                vp(i3,j3) = max(q.volte_datasheet);
            end
        end
    end

    nexttile;
    b = bar(100*vr); grid on; set(gca,'XTickLabel',et);
    yline(100, 'k-', 'stallo', 'LineWidth',1.8);
    % [1/10] la soglia del costruttore, e i titoli calcolati: prima dicevano
    % "la marcia sta dentro" a prescindere dai numeri
    yline(100*opt.tau_lavoro/opt.tau_lim, 'r--', 'carico raccomandato (1/5 stallo)', 'LineWidth',1.5);
    ylabel('coppia efficace del giunto peggiore / stallo  [%]'); ylim([0 110]);
    legend(b, opt.ctrl, 'Location','northwest');
    title(sprintf('Carico sostenuto: %.0f-%.0f%% dello stallo, %.1f-%.1fx il raccomandato', ...
          100*min(vr(:)), 100*max(vr(:)), min(vr(:))*opt.tau_lim/opt.tau_lavoro, ...
          max(vr(:))*opt.tau_lim/opt.tau_lavoro));

    nexttile;
    b = bar(vp); grid on; set(gca,'XTickLabel',et);
    yline(1, 'k-', 'stallo (datasheet)', 'LineWidth',1.8);
    yline(opt.tau_lim_urdf/opt.tau_lim, 'k--', 'limite URDF');
    ylabel('coppia di picco / stallo');
    title(sprintf('Picchi d''urto: fino a %.1fx lo stallo, su al massimo il %.2f%% dei campioni', ...
          max(vp(:)), 100*sat_max));
    salva_grafico('fattibilita', fig);
end
end

%% ================= helper =================
function v = fa_num(T, r, c)
v = NaN;
if ismember(c, T.Properties.VariableNames)
    x = T.(c)(r);
    if isnumeric(x), v = double(x); else, v = str2double(x); end
end
end

function s = fa_campo(T, r, c, dflt)
s = dflt;
if ismember(c, T.Properties.VariableNames), s = string(T.(c)(r)); end
end
