function out = tabella_confronti(file_md)
%TABELLA_CONFRONTI  C1, C2 e C3 affiancati per task, con il verdetto sul rumore.
%
%   tabella_confronti                      % scrive results/tabella_confronti.md
%   out = tabella_confronti('altro.md')
%
% PERCHE' ESISTE
%   [1/10] Le tabelle dei risultati stavano scritte a mano in
%   docs/stato_progetto.md, una per task, e a ogni campagna andavano
%   ricopiate. Con il terzo controllore il lavoro raddoppia e un numero
%   ricopiato male non si vede. Qui la tabella si GENERA da results/: chi la
%   legge sa che ogni cella e' un CSV, non una trascrizione.
%
% COSA LEGGE
%   results/T*_C1.csv, _C2, _C3              le righe di campagna
%   results/diagnostica/rumore_slew.csv      rumore di C1, C2, C3 su T2, T3,
%                                            T4D, T5 (dall'1/10 sera, slew rate)
%   soglia_deriva_T6                         la soglia di validita' di T6
%   Non scrive nient'altro che il file .md. Non simula.
%
% LE REGOLE, LE STESSE DEL RESTO DEL PROGETTO
%   - si confrontano solo controllori ADIACENTI, C1-C2 e C2-C3: ognuno
%     aggiunge un meccanismo al precedente, quindi ogni confronto ne misura
%     uno solo;
%   - rumore di una colonna = escursione fra le tre perturbazioni di z0, per
%     quel controllore su quel task. Per una coppia vale il PEGGIORE dei due;
%   - rapporto = |differenza| / rumore. Si DICHIARA solo con rapporto >= 3
%     (in grassetto). Sotto e' "n.c.", non concludente;
%   - se il rumore di uno dei due manca (T4, T6, T7: mai misurato) il
%     verdetto e' "n.d.": il valore si mostra, la differenza non si dichiara;
%   - colonne di contatto (appoggio, slip, distacchi...) escluse: su T4, T4D,
%     T5 e T6 sono NaN per costruzione, i sensori vedono solo il pavimento;
%   - T6: solo l'esito binario, e "superato" vale solo con dev_lat_max sotto
%     la soglia di soglia_deriva_T6 (un robot che deriva di lato puo' passare
%     di fianco a un ostacolo invece che sopra).
%   Angoli in gradi nella tabella (nei CSV sono radianti).
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(file_md), file_md = fullfile('results', 'tabella_confronti.md'); end

ctrl  = {'C1','C2','C3'};
coppie = {'C1','C2'; 'C2','C3'};
SOGLIA_LETTURA = 3;

% {task, cella, titolo, specifica delle righe}
% riga: {colonna, etichetta, fattore, verso}   verso: -1 meno e' meglio,
% +1 piu' e' meglio, 0 nessun verso (si riporta la differenza e basta)
d = 180/pi;
pass = {'pitch_max','beccheggio max [deg]', d, -1
        'roll_max', 'rollio max [deg]',     d, -1
        'cot',      'costo di trasporto',   1, -1
        'energia',  'energia [J]',          1, -1
        'tau_rms',  'coppia RMS [N m]',     1, -1
        'tau_max',  'coppia di picco [N m]',1, -1
        'vel_media','velocita'' media [m/s]',1, +1
        'frazione_task','frazione del task',1, +1};
task = {
 'T2',  'v1.00x',    'T2 - piano, velocita'' nominale', pass
 'T3',  'yaw+0.100', 'T3 - curva, +0.1 rad/s',          [pass; {'yaw_rapporto','imbardata mis./comandata',1,0}]
 'T4',  'rampa8',    'T4 - rampa di 8 gradi',           {'incl_err','corpo meno rampa [deg]',d,0
                                                         'incl_pp','beccheggio p-p sulla rampa [deg]',d,-1
                                                         'roll_esc_rampa','rollio sulla rampa [deg]',d,-1
                                                         'cot','costo di trasporto',1,-1
                                                         'energia','energia [J]',1,-1
                                                         'v_rapporto','velocita'' in salita / in piano',1,+1
                                                         'salito','salita riuscita',1,0}
 'T4D', 'dosso8-8',  'T4D - dosso 8/8 gradi',           pass
 'T5',  'ost1',      'T5 - ostacolo singolo',           [pass; {'superato','ostacolo superato',1,0}]
 'T6',  'ost1-7',    'T6 - percorso a sette ostacoli',  {'distanza','distanza [m]',1,0
                                                         'superato','percorso superato',1,0
                                                         'dev_lat_max','deriva laterale max [m]',1,-1}
 'T7',  '',          'T7 - impulso laterale',           {'dev_fin_mm@dv1.00','deviazione residua, impulso 1x [mm]',1,-1
                                                         'dev_fin_mm@dv16.00','deviazione residua, impulso 16x [mm]',1,-1
                                                         'recuperato@dv16.00','recupera a 16x',1,0}};

% ---- rumore: escursione per (task, controllore, colonna) ----
R = table();
% [1/10, sera] Dopo lo slew rate un solo pavimento, tutti e tre i controllori
% (rumore_metriche, quarto giro). I tre file di prima sono in
% results/storico/pre_slew_20261001/diagnostica.
for f = {'rumore_slew'}
    p = fullfile('results','diagnostica',[f{1} '.csv']);
    if isfile(p)
        t = readtable(p, 'TextType','string');
        t = t(:, intersect({'controller','task','roll_max','pitch_max','cot','energia', ...
              'tau_rms','tau_max','vel_media','frazione_task','dev_lat_max'}, t.Properties.VariableNames, 'stable'));
        R = tc_unisci(R, t);
    end
end

S6 = soglia_deriva_T6(false);

L = {};
L{end+1} = '# Confronto C1 - C2 - C3';
L{end+1} = '';
L{end+1} = sprintf(['*Generata da `tabella_confronti.m` il %s dai CSV in `results/`. ' ...
                    'Non modificare a mano: si rigenera.*'], char(datetime('now','Format','d/M/yyyy HH:mm')));
L{end+1} = '';
L{end+1} = '| simbolo | significato |';
L{end+1} = '|---|---|';
L{end+1} = sprintf('| **meglio −40%%** (7.4×) | differenza dichiarabile: rapporto sul rumore ≥ %d |', SOGLIA_LETTURA);
L{end+1} = '| peggio +5% (n.c. 1.2×) | non concludente: la differenza sta nel rumore |';
L{end+1} = '| +12% (n.d.) | rumore non misurato su questo task: non si dichiara |';
L{end+1} = '';
L{end+1} = ['Confronti solo fra controllori adiacenti: **C1→C2** misura la ricerca del terreno, ' ...
            '**C2→C3** l''anello d''assetto. Rumore = escursione su tre run con z0 ±0.2 mm, ' ...
            'il peggiore dei due controllori.'];

out = struct();
for i = 1:size(task,1)
    [tk, cella, titolo, spec] = task{i,:};
    V = cell(1,3);
    for c = 1:3, V{c} = tc_riga(tk, ctrl{c}, cella); end

    L{end+1} = '';                                                      %#ok<AGROW>
    L{end+1} = ['## ' titolo];                                          %#ok<AGROW>
    if ~isempty(cella), L{end+1} = sprintf('*cella `%s`*', cella); L{end+1} = ''; end %#ok<AGROW>
    L{end+1} = '| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |';                %#ok<AGROW>
    L{end+1} = '|---|---:|---:|---:|---|---|';                          %#ok<AGROW>

    for j = 1:size(spec,1)
        [col, etich, fat, verso] = spec{j,:};
        val = NaN(1,3);
        for c = 1:3, val(c) = tc_valore(V{c}, col); end
        cel = cell(1,3);
        for c = 1:3, cel{c} = tc_fmt(val(c)*fat, col); end
        ver = cell(1,2);
        for q = 1:2
            a = find(strcmp(ctrl, coppie{q,1}));  b = find(strcmp(ctrl, coppie{q,2}));
            ver{q} = tc_verdetto(val(a), val(b), tc_rumore(R, tk, coppie(q,:), col), ...
                                 verso, col, fat, SOGLIA_LETTURA);
        end
        L{end+1} = sprintf('| %s | %s | %s | %s | %s | %s |', etich, cel{:}, ver{:}); %#ok<AGROW>
    end

    if strcmp(tk, 'T6')
        dl = NaN(1,3);  su = NaN(1,3);
        for c = 1:3, dl(c) = tc_valore(V{c}, 'dev_lat_max'); su(c) = tc_valore(V{c}, 'superato'); end
        cel = cell(1,3);
        for c = 1:3
            if isnan(su(c)), cel{c} = '–';
            elseif ~su(c),   cel{c} = 'no';
            elseif dl(c) < S6.soglia, cel{c} = '**sì**';
            else,            cel{c} = 'non vale (deriva)';
            end
        end
        L{end+1} = sprintf('| **superato, valido** | %s | %s | %s | | |', cel{:});            %#ok<AGROW>
        L{end+1} = '';                                                                       %#ok<AGROW>
        L{end+1} = sprintf(['*"Superato" vale solo con deriva laterale massima < %.3f m: oltre, il ' ...
            'robot puo'' aver mancato per intero l''ostacolo %d invece di passarci sopra ' ...
            '(soglia ricavata dalla geometria in `soglia_deriva_T6.m`, non dai controllori). ' ...
            'Le altre metriche di T6 non vanno citate: i tre controllori non incontrano gli stessi ostacoli.*'], ...
            S6.soglia, S6.k_min);                                                            %#ok<AGROW>
    end
    out.(tk) = V;
end

% ---- [1/10] fattibilita' sui motori veri, da results/diagnostica/fattibilita.csv ----
% Solo valori, nessun verdetto: il rumore della coppia per giunto non e'
% misurato. Punto di lavoro nominale, run valide (stesso filtro di fattibilita.m).
pf = fullfile('results','diagnostica','fattibilita.csv');
if isfile(pf)
    FA = readtable(pf, 'TextType','string');
    FA = FA(tc_vero(FA.nominale) & tc_vero(FA.valida), :);
    cfg = phantomx_config();
    L{end+1} = '';
    L{end+1} = '## Fattibilità sui motori veri';
    L{end+1} = sprintf(['*Da `fattibilita.m`, al punto di lavoro nominale. Coppia di stallo dell''AX-12A %.1f N·m ' ...
        '(manuale ROBOTIS, `docs/ROBOTIS_AX-12A_emanual.pdf`); carico raccomandato per un moto stabile 1/5 dello stallo. La campagna gira ad attuatore ideale e i giunti sono attuati in posizione: ' ...
        'la coppia è una reazione, non un ingresso. Qui si misura quanto il controllore **chiede**, ' ...
        'non come andrebbe con i motori veri. Rumore non misurato: nessun verdetto.*'], cfg.tau_max);
    L{end+1} = '';
    L{end+1} = 'RMS = coppia efficace del giunto più caricato / limite · picco = coppia massima / limite · sopra = campioni oltre il limite';
    L{end+1} = '';
    L{end+1} = '| task | RMS C1 | RMS C2 | RMS C3 | picco C1 | picco C2 | picco C3 | sopra C1 | sopra C2 | sopra C3 |';
    L{end+1} = '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|';
    for i = 1:size(task,1)
        tk = task{i,1};
        a = strings(1,3); b = a; c = a;
        for j = 1:3
            q = FA(FA.task == tk & FA.ctrl == ctrl{j}, :);
            if isempty(q), a(j) = "–"; b(j) = "–"; c(j) = "–"; continue; end
            [~, kk] = max(q.tau_max);
            a(j) = sprintf('%.0f%%', 100*q.peggiore_su_limite(kk));
            b(j) = sprintf('%.1f×', q.volte_datasheet(kk));
            c(j) = sprintf('%.2f%%', 100*q.frazione_saturo(kk));
        end
        L{end+1} = sprintf('| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |', tk, a, b, c); %#ok<AGROW>
    end
    L{end+1} = '';
    pmax = max(FA.peggiore_su_limite(ismember(FA.ctrl, string(ctrl))));
    smax = max(FA.frazione_saturo(ismember(FA.ctrl, string(ctrl))));
    % [1/10, sera] il giudizio sulla marcia si da' col carico raccomandato dal
    % costruttore (1/5 dello stallo, manuale ROBOTIS), non col 70% nostro
    rlav = cfg.tau_lavoro / cfg.tau_max;
    if pmax <= rlav
        mar = sprintf('la **marcia sta nel carico raccomandato** dal costruttore (giunto peggiore al massimo al %.0f%% dello stallo, raccomandato ≤ %.0f%%)', 100*pmax, 100*rlav);
    elseif pmax < 1
        mar = sprintf(['la **marcia supera il carico raccomandato** dal costruttore: giunto peggiore fino al ' ...
            '%.0f%% dello stallo, cioè %.1f volte il %.0f%% indicato da ROBOTIS per un moto stabile. ' ...
            'Resta sotto lo stallo'], 100*pmax, pmax/rlav, 100*rlav);
    else
        mar = sprintf('la **marcia supera lo stallo** (giunto peggiore fino al %.0f%%)', 100*pmax);
    end
    L{end+1} = sprintf('*Lettura: %s; i **picchi d''urto** escono dallo stallo, su al massimo il %.2f%% dei campioni.*', mar, 100*smax);
end

fid = fopen(file_md, 'w', 'n', 'UTF-8');
fprintf(fid, '%s\n', L{:});
fclose(fid);
fprintf('  scritto  %s\n', file_md);
end

%% ================= helper =================
function R = tc_unisci(R, t)
if isempty(R), R = t; return; end
c = intersect(R.Properties.VariableNames, t.Properties.VariableNames, 'stable');
R = [R(:, c); t(:, c)];
end

function b = tc_vero(x)
%TC_VERO  Colonna logica letta da CSV: puo' arrivare come numero o come testo.
if islogical(x), b = x;
elseif isnumeric(x), b = x ~= 0;
else, b = any(lower(string(x)) == ["true","1"], 2);
end
end

function T = tc_riga(task, ctrl, cella)
%TC_RIGA  La riga di campagna (tutte le righe per T7, che ha una cella per impulso).
T = [];
f = fullfile('results', sprintf('%s_%s.csv', task, ctrl));
if ~isfile(f), return; end
T = readtable(f, 'TextType', 'string');
if ~isempty(cella) && ismember('condizione', T.Properties.VariableNames)
    T = T(strcmp(T.condizione, cella), :);
end
end

function v = tc_valore(T, col)
%TC_VALORE  col oppure 'col@condizione' per i task con piu' celle (T7).
v = NaN;
if isempty(T), return; end
p = split(string(col), '@');
if numel(p) == 2
    T = T(strcmp(T.condizione, p(2)), :);
    col = char(p(1));
end
if isempty(T) || ~ismember(col, T.Properties.VariableNames), return; end
x = T.(col)(1);
if islogical(x) || isnumeric(x), v = double(x);
elseif any(strcmpi(string(x), ["true","1"])), v = 1;
elseif any(strcmpi(string(x), ["false","0"])), v = 0;
end
end

function r = tc_rumore(R, task, cop, col)
%TC_RUMORE  Il peggiore dei due rumori; NaN se uno dei due non e' misurato.
r = NaN;
if isempty(R) || ~ismember(col, R.Properties.VariableNames), return; end
e = NaN(1,2);
for q = 1:2
    s = strcmp(R.task, task) & strcmp(R.controller, cop{q});
    if nnz(s) >= 3, x = R.(col)(s); e(q) = max(x) - min(x); end
end
if all(~isnan(e)), r = max(e); end
end

function s = tc_fmt(v, col)
if isnan(v), s = '–'; return; end
if any(strcmp(col, {'salito','superato'})) || startsWith(col, 'recuperato')
    if v, s = 'sì'; else, s = 'no'; end
    return
end
if abs(v) >= 100, s = sprintf('%.0f', v);
elseif abs(v) >= 10, s = sprintf('%.1f', v);
elseif abs(v) >= 1, s = sprintf('%.2f', v);
else, s = sprintf('%.3g', v);
end
end

function s = tc_verdetto(a, b, rum, verso, col, fat, soglia)
if isnan(a) || isnan(b), s = ''; return; end
if any(strcmp(col, {'salito','superato'})) || startsWith(col, 'recuperato')
    if a == b, s = 'uguale'; else, s = 'cambia'; end
    return
end
dd = b - a;
% Le righe senza verso (es. corpo meno rampa) possono valere ~0 su un
% controllore: la percentuale esploderebbe. Li' si da' la differenza assoluta.
if verso == 0 || a == 0
    pc = sprintf('%+.3g', dd*fat);
else
    pc = sprintf('%+.0f%%', 100*dd/abs(a));
end
pc = strrep(pc, '-', '−');
if verso == 0 || dd == 0, giud = '';
elseif sign(dd) == verso, giud = 'meglio ';
else, giud = 'peggio ';
end
if isnan(rum)
    s = sprintf('%s%s (n.d.)', giud, pc);
elseif rum == 0
    s = sprintf('%s%s (rumore nullo)', giud, pc);
else
    k = abs(dd) / rum;
    kt = floor(10*k)/10;          % troncato: 2.97 non deve leggersi "3.0, n.c."
    if k >= soglia, s = sprintf('**%s%s** (%.1f×)', giud, pc, kt);
    else,           s = sprintf('%s%s (n.c. %.1f×)', giud, pc, kt);
    end
end
end
