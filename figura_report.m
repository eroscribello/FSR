function figura_report(quale, arg2)
%FIGURA_REPORT  Dalle run della campagna alle figure del report.
%
%   figura_report('elenco', 'T5_C1')   stampa assi, curve e bande di un .fig
%   figura_report('rampa')             costruisce una figura, in italiano
%   figura_report('tutte')             costruisce tutte quelle configurate
%   figura_report('tutte', 'en')       le stesse, in inglese, con suffisso _en
%
% COSA PRODUCE
%   Per ogni figura, in grafici/report/:  <id>.fig  <id>.pdf  <id>.png
%   Il .fig si riapre e si modifica come qualunque figura MATLAB; il .pdf
%   e' vettoriale, per \includegraphics.
%
% QUATTRO SORGENTI
%   fonte 'fig'      apre i .fig gia' salvati dei tre controllori, estrae UNA
%                    curva per controllore e la ridisegna su un asse solo.
%   fonte 'csv'      legge results/*.csv e costruisce la figura da zero, dove
%                    il .fig non contiene la grandezza che serve.
%   fonte 'pacco'    legge le serie di results/diagnostica/ e confronta i TASK
%                    fra loro, non i controllori: il carico esiste solo in C3P.
%   fonte 'pacco2'   confronta DUE controllori sullo stesso task leggendo le
%                    serie del carico: serve al confronto C2P / C3P.
%   fonte 'zmp'      margine di stabilita' letto da stato_zmp, con i campioni
%                    non calcolabili lasciati come interruzioni della curva.
%   fonte 'tradotta' riapre un .fig gia' fatto e ne riscrive solo le scritte
%                    con il vocabolario qui sotto. Serve per fattibilita.fig,
%                    che e' prodotta da fattibilita.m e non da questo script:
%                    cosi' la versione inglese non richiede di rilanciare
%                    l'analisi ne' di toccare quello script.
%
% NON rilancia simulazioni, non apre modelli, non tocca i file originali.
%
% DUE FAMIGLIE DI CONFRONTO
%   Le figure normali confrontano C1, C2 e C3 sullo stesso task.
%   Le figure 'pacco_*' confrontano C3 con C3P, cioe' lo stesso controllore
%   scarico e carico: il campo 'ctrl' della riga dice quali controllori
%   leggere, e se e' vuoto si usano i tre di default.
%
% LE CURVE SI CERCANO PER NOME, MAI PER INDICE
%   L'ordine non e' lo stesso fra i task: nel pannello 1 di T4 la prima
%   curva e' il beccheggio, in T5 e' il rollio. Cercare per indice darebbe
%   la curva sbagliata senza accorgersene.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(quale), quale = 'tutte'; end

% La lingua arriva come secondo argomento, tranne in modalita' 'elenco'
% dove il secondo argomento e' il nome del .fig da ispezionare.
lingua = 'it';
if nargin >= 2 && ~strcmpi(quale, 'elenco')
    lingua = lower(char(arg2));
    if ~ismember(lingua, {'it','en'})
        error('figura_report:lingua', 'Lingua ''%s'' non prevista: usa ''it'' o ''en''.', lingua);
    end
end
en = strcmp(lingua, 'en');

%% ===== colori e tratti dei controllori =====
% Terna blu / arancione / verde acqua: resta leggibile anche con le forme
% piu' comuni di daltonismo, dove arancione e verde "normale" si
% confondono. Tutte e tre continue: il tratteggio spezzava le curve fitte.
% Spessore leggermente crescente: e' l'unica differenza che sopravvive a
% una stampa in bianco e nero, dove il colore non aiuta piu'.
STILE = struct( ...
    'C1',  struct('col', [0.000 0.447 0.698], 'tratto', '-', 'spess', 0.8), ...
    'C2',  struct('col', [0.835 0.369 0.000], 'tratto', '-', 'spess', 1.0), ...
    'C3',  struct('col', [0.000 0.620 0.451], 'tratto', '-', 'spess', 1.2), ...
    'C3P', struct('col', [0.400 0.200 0.600], 'tratto', ':', 'spess', 1.2));

CTRL = {'C1','C2','C3'};

% La figura del pacco confronta tre TASK di uno stesso controllore, non tre
% controllori. Riusare blu/arancione/verde la farebbe leggere come le altre,
% cioe' come un confronto fra C1, C2 e C3: qui la terna e' deliberatamente
% un'altra, nella famiglia del viola di C3P, e i tratti sono diversi fra loro
% perche' le tre curve si incrociano.
STILE_TASK = struct( ...
    'T4',  struct('col', [0.400 0.200 0.600], 'tratto', '-',  'spess', 1.0), ...
    'T4D', struct('col', [0.800 0.475 0.655], 'tratto', '--', 'spess', 1.0), ...
    'T6',  struct('col', [0.300 0.300 0.300], 'tratto', '-.', 'spess', 1.0));

%% ===== le figure =====
%   fonte  'fig' | 'csv'
%   asse   pannello del .fig sorgente (1 = in alto)
%   curva  DisplayName, verificato con 'elenco'
%   banda  [] niente | 'auto' = presa dal .fig del primo controllore
F = struct( ...
 'id',    {'rampa',                      'ostacolo',   'curva',        'impulso',                 'inviluppo'}, ...
 'fonte', {'fig',                        'fig',        'fig',          'fig',                     'csv'}, ...
 'patt',  {'T4_%s_8deg',                 'T5_%s',      'T3_%s',        'T7_%s',                   'T2_%s'}, ...
 'asse',  {1,                            1,            1,              2,                         0}, ...
 'curva', {'inclinazione (-beccheggio)', 'beccheggio', '+0.100 rad/s', '\Deltay massima',         ''}, ...
 'xlim',  {[],                           [],           [],             [],                        []}, ...
 'banda', {[],                           'auto',       [],             [],                        []}, ...
 'xl',    {'t [s]',                      't [s]',      'x [m]',        'impulso J [N s]',         'fattore di velocita'''}, ...
 'yl',    {'inclinazione del corpo [deg]','beccheggio [deg]','y [m]',  'deviazione massima [mm]', ''}, ...
 'rif',   {8,                            [],           [],             [],                        []}, ...
 'logx',  {false,                        false,        false,          true,                      false}, ...
 'segno', {'none',                       'none',       'none',         'o',                       'o'}, ...
 'xl_en', {'t [s]',                      't [s]',      'x [m]',        'impulse J [N s]',         'speed factor'}, ...
 'yl_en', {'body tilt [deg]',     'pitch [deg]','y [m]',        'maximum deviation [mm]',  ''});

% Quali controllori legge ciascuna figura. Vuoto = i tre di default.
[F.ctrl] = deal({});

% ----- la figura del pacco: aggiunta a mano, ha una struttura sua -----
% Non entra nel blocco qui sopra perche' non ha ne' controllori ne' pannello
% sorgente: le tre curve sono tre task, e i campi 'asse', 'curva', 'banda'
% non le riguardano. Aggiungerla in coda costa una riga invece di una colonna
% vuota in ciascuno dei quindici campi.
F(end+1) = F(end);
F(end).id    = 'pacco';
F(end).fonte = 'pacco';
F(end).patt  = 'pacco_serie_C3P_%s'; % results/diagnostica/pacco_serie_C3P_T4.csv
F(end).xl    = 't [s]';
F(end).yl    = 'spostamento del pacco [mm]';
F(end).xl_en = 't [s]';
F(end).yl_en = 'payload displacement [mm]';

% ----- le tre figure del carico: C3 scarico contro C3P carico -----
% Stessa macchina delle altre figure 'fig', ma con due controllori invece di
% tre: il carico esiste solo in C3P, quindi C1 e C2 non hanno un termine di
% paragone. Le tre curve dei task vengono dai .fig gia' salvati dalla
% campagna, non si rilancia niente.
PACCO = struct( ...
 'id',    {'pacco_rampa',               'pacco_dosso',                'pacco_ostacoli'}, ...
 'patt',  {'T4_%s_8deg',                'T4D_%s',                     'T6_%s'}, ...
 'curva', {'inclinazione (-beccheggio)','inclinazione (-beccheggio)', 'beccheggio'}, ...
 'yl',    {'inclinazione del corpo [deg]','inclinazione del corpo [deg]','beccheggio [deg]'}, ...
 'yl_en', {'body inclination [deg]',    'body inclination [deg]',     'pitch [deg]'}, ...
 'banda', {[],                          'auto',                       'auto'}, ...
 'rif',   {8,                           [],                           []});
for ip = 1:numel(PACCO)
    F(end+1) = F(1);                 %#ok<AGROW>  una riga con tutti i campi
    F(end).id    = PACCO(ip).id;
    F(end).fonte = 'fig';
    F(end).ctrl  = {'C3','C3P'};
    F(end).patt  = PACCO(ip).patt;
    F(end).asse  = 1;
    F(end).curva = PACCO(ip).curva;
    F(end).xlim  = [];
    F(end).banda = PACCO(ip).banda;
    F(end).xl    = 't [s]';
    F(end).yl    = PACCO(ip).yl;
    F(end).rif   = PACCO(ip).rif;
    F(end).logx  = false;
    F(end).segno = 'none';
    F(end).xl_en = 't [s]';
    F(end).yl_en = PACCO(ip).yl_en;
end

% ----- il confronto del carico: C2P contro C3P, un task per figura -----
% Le serie le scrive stato_pacco. La finestra di validita' non e' scritta qui:
% viene letta dal campo t_bordo della riga di results/, cosi' la figura e la
% tabella si fermano sempre nello stesso istante anche se le run cambiano.
for ip = 1:2
    tk = {'T4','T6'};  tk = tk{ip};
    F(end+1) = F(1);                 %#ok<AGROW>
    F(end).id    = ['pacco_' tk];
    F(end).fonte = 'pacco2';
    F(end).ctrl  = {'C2P','C3P'};
    F(end).patt  = ['pacco_serie_%s_' tk];
    F(end).asse  = 0;
    F(end).curva = tk;               % qui 'curva' porta il nome del task
    F(end).xlim  = [];
    F(end).banda = [];
    F(end).xl    = 't [s]';
    F(end).yl    = 'spostamento del pacco [mm]';
    F(end).rif   = [];
    F(end).logx  = false;
    F(end).segno = 'none';
    F(end).xl_en = 't [s]';
    F(end).yl_en = 'payload displacement [mm]';
end

% ----- il margine ZMP -----
% Esiste solo su phantomx_sim_zero, quindi solo per C1 e C2: in
% phantomx_sim_attitude il blocco di calcolo non c'e'.
F(end+1) = F(1);
F(end).id    = 'zmp';
F(end).fonte = 'zmp';
F(end).ctrl  = {'C2'};               % cambia qui per usare C1
F(end).patt  = 'zmp_%s_T2';          % results/diagnostica/zmp_C2_T2.csv
F(end).asse  = 0;
F(end).curva = '';
F(end).xlim  = [];
F(end).banda = [];
F(end).xl    = 't [s]';
F(end).yl    = 'margine di stabilita'' [m]';
F(end).rif   = [];
F(end).logx  = false;
F(end).segno = 'none';
F(end).xl_en = 't [s]';
F(end).yl_en = 'stability margin [m]';

% ----- la figura della fattibilita': non la costruiamo, la traduciamo -----
% Le scritte di grafici/fattibilita.fig arrivano da fattibilita.m. Qui si
% sostituiscono per frasi, non per stringhe intere, perche' i titoli sono
% costruiti con sprintf e contengono numeri che vanno lasciati dove sono.
VOCAB = { ...
 'coppia efficace del giunto peggiore / stallo',  'worst joint RMS torque/ stall'
 'coppia di picco / stallo',                      'peak torque / stall'
 'Carico sostenuto',                              'Continuous load'
 'Picchi d''urto',                                'Impact peaks'
 'dello stallo',                                  'of stall'
 'lo stallo',                                     'stall'
 'il raccomandato',                               'the recommended value'
 'su al massimo il',                              'on at most'
 'dei campioni',                                  'of samples'
 'fino a',                                        'up to' };

if en
    F(end+1) = F(end);                   % una riga in piu', riempita a mano
    F(end).id    = 'fattibilita';
    F(end).fonte = 'tradotta';
    F(end).patt  = 'fattibilita';        % grafici/fattibilita.fig
end

if strcmpi(quale, 'elenco')
    if nargin < 2, error('figura_report:elenco', 'Serve il nome del .fig, es. ''T5_C1''.'); end
    elenca(arg2);  return
end

if strcmpi(quale, 'tutte')
    da_fare = 1:numel(F);
else
    da_fare = find(strcmpi({F.id}, quale));
    if isempty(da_fare)
        error('figura_report:id', 'Figura ''%s'' non configurata. Disponibili: %s.', ...
            quale, strjoin({F.id}, ', '));
    end
end

dest = fullfile('grafici', 'report');
if ~isfolder(dest), mkdir(dest); end

for k = da_fare
    f = F(k);
    if en                                 % etichette inglesi al posto delle italiane
        f.xl = f.xl_en;
        f.yl = f.yl_en;
    end
    nome = f.id;  if en, nome = [nome '_en']; end
    fprintf('\n=== figura %s ===\n', nome);
    switch f.fonte
        case 'fig'
            ctrl_qui = CTRL;
            if ~isempty(f.ctrl), ctrl_qui = f.ctrl; end
            fig = da_fig(f, ctrl_qui, STILE);
        case 'csv',      fig = da_csv(f, CTRL, STILE, en);
        case 'pacco',    fig = da_pacco(f, STILE_TASK, en);
        case 'pacco2',   fig = da_pacco2(f, en);
        case 'zmp',      fig = da_zmp(f, en);
        case 'tradotta', fig = tradotta(f, VOCAB);
        otherwise,       error('figura_report:fonte', 'Fonte ''%s'' sconosciuta.', f.fonte);
    end
    salva(fig, nome, dest);
end
end

% =====================================================================
function fig = da_fig(f, CTRL, STILE)
%DA_FIG  Una curva per controllore, presa dai .fig della campagna.

D = struct('ctrl', {}, 'x', {}, 'y', {});
banda = [];

for i = 1:numel(CTRL)
    c   = CTRL{i};
    tag = sprintf(f.patt, c);
    src = fullfile('grafici', [tag '.fig']);
    if ~isfile(src)
        error('figura_report:file', 'Manca %s: la figura %s non si puo'' costruire.', src, f.id);
    end

    h = openfig(src, 'invisible');
    try
        assi = findobj(h, 'Type', 'axes');  assi = assi(end:-1:1);
        if numel(assi) < f.asse
            error('figura_report:asse', '%s ha %d pannelli, serviva il %d.', src, numel(assi), f.asse);
        end
        ax = assi(f.asse);

        L = findobj(ax, 'Type', 'line');  L = L(end:-1:1);
        nomi = arrayfun(@(o) string(o.DisplayName), L);
        j = find(strcmpi(nomi, string(f.curva)), 1);
        if isempty(j)
            error('figura_report:nome', ['In %s, pannello %d, non c''e'' la curva "%s".\n' ...
                'Presenti: %s.'], src, f.asse, f.curva, strjoin(cellstr(nomi), ' | '));
        end

        D(end+1) = struct('ctrl', c, 'x', L(j).XData(:), 'y', L(j).YData(:)); %#ok<AGROW>
        % La data serve: se due .fig vengono da campagne diverse la figura
        % confronta run non confrontabili, e nel grafico non si vede.
        info = dir(src);
        fprintf('  %-4s  %-20s  "%s"  %d punti   (%s)\n', c, [tag '.fig'], ...
                L(j).DisplayName, numel(L(j).XData), info.date);

        if isequal(f.banda, 'auto') && isempty(banda)
            % findall e non findobj: le fasce sono disegnate con
            % 'HandleVisibility','off' e findobj non le vede.
            P = findall(ax, 'Type', 'patch');
            if ~isempty(P)
                banda = zeros(numel(P), 2);
                for q = 1:numel(P)
                    xs = P(q).XData;
                    banda(q,:) = [min(xs(:)) max(xs(:))];
                end
                banda = sortrows(banda);
                for q = 1:size(banda,1)
                    fprintf('       banda %d da %s: %.2f - %.2f\n', q, c, banda(q,1), banda(q,2));
                end
            else
                fprintf(2, '       nessuna fascia in %s: la figura esce senza sfondo\n', src);
            end
        end
    catch err
        close(h);  rethrow(err);
    end
    close(h);
end

fig = asse_nuovo();
ax  = gca(fig);

if ~isempty(f.rif)
    yline(ax, f.rif, ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
end

for i = 1:numel(D)
    s = STILE.(D(i).ctrl);
    plot(ax, D(i).x, D(i).y, s.tratto, 'Color', s.col, 'LineWidth', s.spess, ...
         'Marker', f.segno, 'MarkerSize', 4, 'MarkerFaceColor', s.col, ...
         'DisplayName', D(i).ctrl);
end

rifinisci(ax, f);

% la banda va DOPO i limiti: 'axis tight' altrimenti li calcola su di lei
if ~isempty(banda)
    yl = ylim(ax);
    for q = 1:size(banda,1)
        pb = patch(ax, banda(q,[1 2 2 1]), yl([1 1 2 2]), [0.90 0.90 0.90], ...
                   'EdgeColor', 'none', 'HandleVisibility', 'off');
        uistack(pb, 'bottom');     % sotto le curve, che restano leggibili
    end
    ylim(ax, yl);
    % La griglia la disegna l'asse, e con Layer 'bottom' (predefinito) finisce
    % SOTTO la fascia, che quindi se la mangia. 'top' la riporta sopra, insieme
    % a riquadro e tacche. Lo faccio solo dove c'e' una fascia, per non
    % cambiare l'aspetto delle figure che non ne hanno.
    % Nota: niente FaceAlpha al posto di questo. La trasparenza costringe
    % exportgraphics a rasterizzare quella zona, e il PDF smette di essere
    % vettoriale proprio dove serve.
    set(ax, 'Layer', 'top');
end
end

% =====================================================================
function fig = da_csv(f, CTRL, STILE, en)
%DA_CSV  Inviluppo di velocita': rimbalzo del corpo e frazione del task.
% Il .fig di T2 contiene la velocita' istantanea, non queste due grandezze:
% vanno ricostruite dai CSV della campagna.

fig = figure('Visible', 'off', 'Units', 'centimeters', ...
             'Position', [2 2 16 10], 'Color', 'w');
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile(tl);  hold(a1, 'on');  grid(a1, 'on');  box(a1, 'on');
a2 = nexttile(tl);  hold(a2, 'on');  grid(a2, 'on');  box(a2, 'on');

for i = 1:numel(CTRL)
    c   = CTRL{i};
    src = fullfile('results', [sprintf(f.patt, c) '.csv']);
    if ~isfile(src), error('figura_report:csv', 'Manca %s.', src); end
    T = readtable(src);
    for col = {'fattore','z_max','frazione_task'}
        if ~ismember(col{1}, T.Properties.VariableNames)
            error('figura_report:colonna', '%s non ha la colonna %s.', src, col{1});
        end
    end
    [x, o] = sort(T.fattore);
    s = STILE.(c);
    plot(a1, x, 1e3*T.z_max(o), s.tratto, 'Color', s.col, 'LineWidth', s.spess, ...
         'Marker','o','MarkerSize',4,'MarkerFaceColor',s.col, 'DisplayName', c);
    plot(a2, x, 100*T.frazione_task(o), s.tratto, 'Color', s.col, 'LineWidth', s.spess, ...
         'Marker','o','MarkerSize',4,'MarkerFaceColor',s.col, 'DisplayName', c);
    fprintf('  %-4s  %-16s  %d celle di velocita''\n', c, [sprintf(f.patt,c) '.csv'], height(T));
end

if en
    ylabel(a1, 'body bounce [mm]', 'FontSize', 9);
    ylabel(a2, 'task fraction [%]', 'FontSize', 9);
else
    ylabel(a1, 'rimbalzo del corpo [mm]', 'FontSize', 9);
    ylabel(a2, 'frazione del task [%]',   'FontSize', 9);
end
xlabel(a2, f.xl, 'FontSize', 9);
set([a1 a2], 'FontSize', 9);
yline(a2, 100, ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
legend(a1, 'Location', 'northwest', 'FontSize', 9);
end

% =====================================================================
function fig = da_pacco(f, STILE_TASK, en)
%DA_PACCO  Spostamento del carico sul vassoio, un task per curva.
%
% Serve stato_pacco.m nella versione che scrive la serie: le run di ottobre
% calcolavano lo spostamento campione per campione e salvavano solo il
% massimo, quindi i tre CSV vanno rigenerati rilanciando T4, T4D e T6 in C3P.
% Se mancano, qui si ferma e dice quale manca.

TASK  = {'T4','T4D','T6'};
GIOCO = 50;                            % mm: gioco fra cubo e vassoio

fig = asse_nuovo();
ax  = gca(fig);

tmax = 0;
for i = 1:numel(TASK)
    tk  = TASK{i};
    src = fullfile('results', 'diagnostica', [sprintf(f.patt, tk) '.csv']);
    if ~isfile(src)
        error('figura_report:serie', ['Manca %s.\n' ...
            'La serie nel tempo non e'' sul disco: le run vanno rifatte con la\n' ...
            'versione di stato_pacco.m che scrive pacco_serie_<task>.csv.'], src);
    end
    T = readtable(src);
    for col = {'t','d'}
        if ~ismember(col{1}, T.Properties.VariableNames)
            error('figura_report:colonna', '%s non ha la colonna %s.', src, col{1});
        end
    end
    s = STILE_TASK.(tk);
    plot(ax, T.t, 1e3*T.d, s.tratto, 'Color', s.col, 'LineWidth', s.spess, ...
         'DisplayName', tk);
    fprintf('  %-4s  %-26s  %d campioni, max %.1f mm a t = %.2f s\n', ...
            tk, [sprintf(f.patt,tk) '.csv'], height(T), 1e3*max(T.d), ...
            T.t(find(T.d == max(T.d), 1)));
    tmax = max(tmax, max(T.t));
end

% La soglia e' il messaggio della figura: va etichettata, non lasciata muta.
if en, et = sprintf('clearance %d mm', GIOCO);
else,  et = sprintf('gioco %d mm', GIOCO);
end
yline(ax, GIOCO, '--', et, 'Color', [0.35 0.35 0.35], 'LineWidth', 1.0, ...
      'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');

xlabel(ax, f.xl, 'FontSize', 9);
ylabel(ax, f.yl, 'FontSize', 9);
% Niente 'axis tight' qui: l'asse y va da zero alla soglia. Con i limiti
% stretti sui dati le tre curve riempirebbero il riquadro e il margine
% rispetto al gioco - che e' il risultato della sezione - non si vedrebbe.
xlim(ax, [0 tmax]);
ylim(ax, [0 1.1*GIOCO]);
legend(ax, 'Location', 'northwest', 'FontSize', 9);
end

% =====================================================================
function fig = da_pacco2(f, en)
%DA_PACCO2  Spostamento del carico: anello d'assetto spento contro acceso.
%
% PERCHE' UNA TAVOLOZZA LOCALE
%   Qui le due curve non sono due controllori del confronto principale ma due
%   configurazioni dello stesso impianto carico. C2P prende l'arancione di C2,
%   di cui e' la versione con il pacco, ed e' tratteggiato; C3P prende il viola
%   che ha gia' nelle altre figure, ma continuo, perche' e' il caso che
%   funziona e deve leggersi come la curva di riferimento.
%
% LA FINESTRA DI VALIDITA' NON E' SCRITTA QUI
%   Si legge da results/<task>_<ctrl>.csv, colonna t_bordo: e' l'istante in cui
%   la run e' stata troncata perche' un piede ha raggiunto il bordo del
%   pavimento. Oltre quell'istante il cubo esce dalla scena insieme al robot e
%   lo spostamento misura una caduta, non uno scivolamento.

GIOCO = 50;                               % [mm] gioco fra cubo e vassoio
STL = struct('C2P', struct('col', [0.835 0.369 0.000], 'tratto', '--', 'spess', 1.1), ...
             'C3P', struct('col', [0.400 0.200 0.600], 'tratto', '-',  'spess', 1.3));
cfg  = phantomx_config();
t_in = 2*cfg.T;                           % stesso avvio di regime di stato_pacco
task = f.curva;

fig = asse_nuovo();
ax  = gca(fig);
tmax = 0;

for i = 1:numel(f.ctrl)
    c   = f.ctrl{i};
    src = fullfile('results', 'diagnostica', [sprintf(f.patt, c) '.csv']);
    if ~isfile(src)
        error('figura_report:serie', ['Manca %s.\n' ...
            'La serie si scrive con stato_pacco: dopo la run del task,\n' ...
            '  stato_pacco(<out>, ''%s'', ''%s'')'], src, task, c);
    end
    T = readtable(src);

    % finestra di validita' dalla riga di campagna
    t_fin = Inf;
    rig = fullfile('results', sprintf('%s_%s.csv', task, c));
    if isfile(rig)
        R = readtable(rig);
        if ismember('t_bordo', R.Properties.VariableNames) && ~isnan(R.t_bordo(1))
            t_fin = R.t_bordo(1);
        end
    else
        fprintf(2, '  manca %s: nessun troncamento al bordo per %s\n', rig, c);
    end

    sel = T.t >= t_in & T.t <= t_fin;
    s = STL.(c);
    plot(ax, T.t(sel), 1e3*T.d(sel), s.tratto, 'Color', s.col, ...
         'LineWidth', s.spess, 'DisplayName', c);
    tmax = max(tmax, max(T.t(sel)));
    fprintf('  %-4s  %-26s  fino a %.2f s   max %.1f mm\n', ...
            c, [sprintf(f.patt,c) '.csv'], max(T.t(sel)), 1e3*max(T.d(sel)));
end

xlabel(ax, f.xl, 'FontSize', 9);
ylabel(ax, f.yl, 'FontSize', 9);
xlim(ax, [t_in tmax]);
ylim(ax, [0 2*GIOCO]);
legend(ax, 'Location', 'northwest', 'FontSize', 9);

% La fascia va dopo i limiti, e opaca: con FaceAlpha exportgraphics
% rasterizzerebbe la zona e il PDF smetterebbe di essere vettoriale.
yl = ylim(ax);
pb = patch(ax, [t_in tmax tmax t_in], [GIOCO GIOCO yl(2) yl(2)], ...
           [0.90 0.90 0.90], 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(pb, 'bottom');
set(ax, 'Layer', 'top');
if en, et = sprintf('threshold, %d mm', GIOCO);
else,  et = sprintf('gioco cubo-vassoio, %d mm', GIOCO);
end
yline(ax, GIOCO, '-', et, 'Color', [0.25 0.25 0.25], 'LineWidth', 1.0, ...
      'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom', ...
      'HandleVisibility', 'off');
end

% =====================================================================
function fig = da_zmp(f, en)
%DA_ZMP  Margine di stabilita' nel tempo, con i buchi lasciati visibili.
%
% I CAMPIONI NON CALCOLABILI NON SI DISEGNANO
%   stato_zmp li marca nella colonna non_calcolabile e mette NaN nel margine:
%   sono gli istanti in cui meno di tre zampe superano Fmin e il poligono non
%   esiste. La curva si interrompe, e una riga di tacche in basso dice dove.
%   Disegnarli a zero, come fa lo Scope, li farebbe sembrare un margine nullo.

c   = f.ctrl{1};
src = fullfile('results', 'diagnostica', [sprintf(f.patt, c) '.csv']);
if ~isfile(src)
    error('figura_report:zmp', ['Manca %s.\n' ...
        'Si produce cosi'':  log_zmp(''on'');  script_T2;  stato_zmp(t2_out, ''T2'', ''%s'')'], ...
        src, c);
end
T = readtable(src);
cfg = phantomx_config();

fig = asse_nuovo();
ax  = gca(fig);

plot(ax, T.t, T.margine, '-', 'Color', [0 0.447 0.698], 'LineWidth', 0.8, ...
     'DisplayName', c);

% previsione geometrica del tripode: disegnata DOPO la curva, cosi' resta
% sopra e si legge anche dove la curva la attraversa
g_tri = zmp_geom_tripode(cfg);
yline(ax, g_tri, '--', 'Color', [0.15 0.15 0.15], 'LineWidth', 1.4, ...
      'DisplayName', 'tripod prediction');

rifinisci_zmp(ax, f, T, g_tri, en);
fprintf('  %-4s  %-22s  %d campioni, %.1f%% non calcolabili\n', ...
        c, [sprintf(f.patt,c) '.csv'], height(T), 100*mean(T.non_calcolabile));
end

function rifinisci_zmp(ax, f, T, g_tri, en)
xlabel(ax, f.xl, 'FontSize', 9);
ylabel(ax, f.yl, 'FontSize', 9);
xlim(ax, [0 max(T.t)]);
yl = [min(0, min(T.margine)) 1.15*max(T.margine)];
if ~all(isfinite(yl)), yl = [0 0.3]; end
ylim(ax, yl);

% tacche in basso dove la misura non c'e'
k = find(T.non_calcolabile > 0.5);
if ~isempty(k)
    h = yl(1) + 0.035*diff(yl);
    plot(ax, T.t(k), repmat(yl(1), numel(k), 1) + 0.5*(h-yl(1)), '.', ...
         'Color', [0.835 0.369 0], 'MarkerSize', 3, ...
         'DisplayName', 'not computable');
end
legend(ax, 'Location', 'northwest', 'FontSize', 9);
end

function d = zmp_geom_tripode(cfg)
p = zeros(2,6);
for i = 1:6
    p(:,i) = [cfg.p_hip(1,i) + cfg.r_offset*cos(cfg.alpha(i));
              cfg.p_hip(2,i) + cfg.r_offset*sin(cfg.alpha(i))];
end
d = Inf;
for fase = [0 0.5]
    tri = find(abs(cfg.phase(:)' - fase) < 1e-9);
    if numel(tri) ~= 3, continue; end
    for k = 1:3
        a = p(:, tri(k));  b = p(:, tri(mod(k,3)+1));
        e = b - a;
        d = min(d, abs(e(1)*(0-a(2)) - e(2)*(0-a(1))) / norm(e));
    end
end
end

% =====================================================================
function fig = tradotta(f, VOCAB)
%TRADOTTA  Riapre un .fig e ne riscrive le scritte, lasciando i dati com'e'.
% Tocca ogni oggetto testuale: etichette, titoli, legende, annotazioni.
src = fullfile('grafici', [f.patt '.fig']);
if ~isfile(src)
    error('figura_report:file', 'Manca %s: non c''e'' niente da tradurre.', src);
end
fig = openfig(src, 'invisible');
n = 0;
% findall e non findobj: legende e annotazioni sono spesso nascoste.
for o = findall(fig, '-property', 'String')'
    t = get(o, 'String');
    if isempty(t), continue, end
    [t, k] = sostituisci(t, VOCAB);
    if k > 0, set(o, 'String', t);  n = n + k; end
end
if n == 0
    fprintf(2, '  nessuna scritta tradotta in %s: controlla il vocabolario\n', src);
else
    fprintf('  %s: %d sostituzioni\n', src, n);
end
end

function [t, k] = sostituisci(t, VOCAB)
k = 0;
if iscell(t)                       % XTickLabel e legende sono cell array
    for i = 1:numel(t)
        [t{i}, ki] = sostituisci(t{i}, VOCAB);
        k = k + ki;
    end
    return
end
if ~(ischar(t) || isstring(t)), return, end
t = char(t);
for i = 1:size(VOCAB, 1)
    if contains(t, VOCAB{i,1})
        t = strrep(t, VOCAB{i,1}, VOCAB{i,2});
        k = k + 1;
    end
end
end

% =====================================================================
function fig = asse_nuovo()
% 16 cm: a pagina piena con margini da 25 mm la figura non viene
% rimpicciolita, quindi i caratteri a 9 pt restano 9 pt sulla pagina.
fig = figure('Visible', 'off', 'Units', 'centimeters', ...
             'Position', [2 2 16 6.2], 'Color', 'w');
ax = axes(fig);  hold(ax, 'on');  grid(ax, 'on');  box(ax, 'on');
set(ax, 'FontSize', 9);
end

function rifinisci(ax, f)
if f.logx, set(ax, 'XScale', 'log'); end
xlabel(ax, f.xl, 'FontSize', 9);
ylabel(ax, f.yl, 'FontSize', 9);
axis(ax, 'tight');
if ~isempty(f.xlim), xlim(ax, f.xlim); end
yl = ylim(ax);  ylim(ax, yl + 0.06*diff(yl)*[-1 1]);
legend(ax, 'Location', 'best', 'FontSize', 9);
end

% =====================================================================
function salva(fig, id, dest)
%SALVA  .fig (modificabile) + .pdf (vettoriale) + .png
% Si costruisce invisibile per non far lampeggiare finestre, ma si salva
% visibile: un .fig salvato invisibile si riapre invisibile.
set(fig, 'Visible', 'on');
f_fig = fullfile(dest, [id '.fig']);
savefig(fig, f_fig);
exportgraphics(fig, fullfile(dest, [id '.pdf']), 'ContentType', 'vector', 'BackgroundColor', 'white');
exportgraphics(fig, fullfile(dest, [id '.png']), 'Resolution', 300, 'BackgroundColor', 'white');
close(fig);
fprintf('  -> %s  (+ .pdf, .png)\n', f_fig);
fprintf('     \\includegraphics[width=\\linewidth]{figure/%s.pdf}\n', id);
end

% =====================================================================
function elenca(tag)
src = fullfile('grafici', [char(tag) '.fig']);
if ~isfile(src), error('figura_report:file', 'Manca %s.', src); end
h = openfig(src, 'invisible');
try
    assi = findobj(h, 'Type', 'axes');  assi = assi(end:-1:1);
    fprintf('\n=== %s : %d pannelli ===\n', src, numel(assi));
    for a = 1:numel(assi)
        ax = assi(a);
        yl = '';  try, yl = ax.YLabel.String; end %#ok<TRYNC>
        if iscell(yl), yl = strjoin(yl, ' '); end
        fprintf('\n  pannello %d   ylabel: %s\n', a, yl);
        L = findobj(ax, 'Type', 'line');  L = L(end:-1:1);
        for j = 1:numel(L)
            fprintf('     curva %d: "%s"  (%d punti, x da %.3g a %.3g)\n', ...
                j, L(j).DisplayName, numel(L(j).XData), min(L(j).XData), max(L(j).XData));
        end
        P = findall(ax, 'Type', 'patch');     % findall: le fasce sono nascoste
        for j = 1:numel(P)
            xs = P(j).XData;
            fprintf('     fascia %d: da %.3g a %.3g\n', j, min(xs(:)), max(xs(:)));
        end
    end
catch err
    close(h);  rethrow(err);
end
close(h);
fprintf('\n');
end
