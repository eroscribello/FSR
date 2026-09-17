function info = applica_terreno(task, verbose, mdl)
%APPLICA_TERRENO  Configura il terreno per un task, senza salvare il modello.
%
%   applica_terreno('T1')        piano liscio
%   applica_terreno('T4')        piano liscio + rampa inclinata
%   applica_terreno('T5')        piano liscio + un ostacolo
%   applica_terreno('T6')        terreno imperfetto + tutti gli ostacoli
%   applica_terreno('TUTTO')     tutto acceso
%
% USO
%   init_gait
%   applica_terreno('T5')
%   out = sim('phantomx_sim_zero','StopTime','15');
%
% COME FUNZIONA
%   Ogni elemento del terreno e' una terna gia' presente nel modello:
%     solido + trasformazione + un blocco di contatto in ciascuno dei sei
%     piedi.
%   La funzione commenta le terne non richieste e scommenta quelle del task
%   con set_param. Il modello NON viene salvato: il .slx sul disco resta
%   identico, quindi niente conflitti git.
%
%   Commentare solo il solido non basta: i sei Spatial Contact Force che lo
%   cercano restano senza geometria e il modello non compila.
%
% NIENTE E' PIU' DEDOTTO DAI NUMERI
%   La versione precedente dava per scontato che Spatial Contact Force(k+1)
%   fosse il contatto di File Solid k. Falso su cinque slot su sette: la
%   numerazione dei blocchi non segue quella delle etichette.
%       File Solid1 ('ostacolo')  -> Force6, non Force2
%       File Solid2 ('ostacolo2') -> Force2, non Force3
%       ...
%   T6 non se ne accorgeva, perche' accende tutti gli ostacoli insieme e
%   qualunque permutazione va bene. T5 ne accende uno solo: spegneva la
%   geometria di Force2 e il modello non compilava.
%
%   Adesso la corrispondenza etichetta -> blocco di contatto viene LETTA dal
%   modello a ogni chiamata, piede per piede (vedi mappaPiede in fondo), e
%   tutto il lavoro si fa per NOME dell'etichetta. Se il collega rinumera o
%   riordina i blocchi, questa funzione continua a funzionare.
%
% LA RAMPA (T4)
%   Nei sei piedi esistono solo i contatti 'pavimento' e 'ostacolo*':
%   nessuno cerca l'etichetta 'rampa'. E non si puo' dare alla rampa
%   l'etichetta 'pavimento': Simscape ammette UNA SOLA geometria per linea,
%   e il piano liscio la occupa gia'
%     "there must be only one source of geometry connected to any geometry line"
%
%   Percio' la rampa prende in prestito lo SLOT 'ostacolo7': il suo
%   Connection Label diventa 'ostacolo7', File Solid7 viene spento e il suo
%   Connection Label spostato su un'etichetta che nessuno usa, mentre il
%   contatto di quello slot resta acceso.
%
%   La rampa nel modello sta a x = -2.5, cioe' DIETRO al robot, che avanza
%   verso +x: non la incontrerebbe mai. Qui viene riposizionata secondo
%   cfg.terreno.rampa_pos, e l'inclinazione secondo cfg.terreno.rampa_gradi.
%
% ATTENZIONE: LE TRASFORMAZIONI NON SI COMMENTANO MAI
%   I Rigid Transform del terreno fanno parte della catena che ancora il ramo
%   al World, non del singolo elemento: commentarne uno stacca tutto quello
%   che pende da li' e il pavimento cade come corpo libero, trascinando giu'
%   il robot. Misurato: T1 (Rigid Transform9 attivo) cammina, lo stesso
%   blocco commentato fa cadere tutto.
%   Un Rigid Transform attivo il cui solido e' commentato e' invece innocuo.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(task),    task = 'T1'; end
if nargin < 2 || isempty(verbose), verbose = true; end
if nargin < 3 || isempty(mdl),     mdl = 'phantomx_sim_zero'; end
task = upper(strtrim(char(task)));

PIEDI = {'Subsystem','Subsystem1','Subsystem2','Subsystem3','Subsystem4','Subsystem6'};
cfg = phantomx_config();

%% ---- catalogo: nome dell'elemento -> solido che lo disegna ----
cat = {
%   nome           solido
    'liscio'      'Brick Solid3'
    'rampa'       'Brick Solid1'
    'piano2'      'Brick Solid2'
    'imperfetto'  'File Solid'
    };
for k = 1:7
    cat(end+1,:) = {sprintf('ost%d',k), sprintf('File Solid%d',k)}; %#ok<AGROW>
end
nomi = cat(:,1);

%% ---- cosa serve a ogni task ----
switch task
    case {'T1','T2','T3','T7'}, vuoi = {'liscio'};
    case 'T4',                  vuoi = {'liscio','rampa'};
    % T5: su pavimento liscio sono a filo solo ost1..ost3 (vedi quota_terreno)
    case 'T5',                  vuoi = {'liscio','ost1'};
    case 'T6',                  vuoi = [{'imperfetto'}, arrayfun(@(k) sprintf('ost%d',k), 1:7, 'UniformOutput',false)];
    case 'TUTTO',               vuoi = nomi.';
    otherwise
        error('applica_terreno:task', ...
          'Task ''%s'' non previsto.\nPrevisti: T1 T2 T3 T4 T5 T6 T7 TUTTO', task);
end

if ~bdIsLoaded(mdl), load_system(mdl); end
mancanti = {};

%% ---- normalizzazione 1: tutte le trasformazioni del terreno attive ----
% Le prove precedenti possono averne lasciata qualcuna commentata: finche'
% lo e', il ramo resta staccato dal World e il pavimento cade. Vanno
% riattivate PRIMA di qualunque altra cosa.
for k = [7 8 9 10 11 12 13 14 15 16]
    commenta([mdl '/' rt(k)], 'off');
end
commenta([mdl '/' rt([])], 'off');       % Rigid Transform, quello del Cube

%% ---- normalizzazione 2: etichette della rampa al loro posto ----
% Una chiamata precedente con T4 puo' averle lasciate scambiate. Vanno
% rimesse a posto prima di leggere le etichette dei solidi, altrimenti la
% mappa che ne ricaviamo e' quella di ieri.
etichetta([mdl '/Connection Label7'],  'rampa');
etichetta([mdl '/Connection Label12'], 'ostacolo7');

%% ---- quota degli ostacoli ----
% Le mesh degli ostacoli sono state posate contro il CUBE ('imperfetto'), la
% cui superficie sta 25 mm piu' in alto di quella del Brick liscio (Brick:
% spessore 0.05 centrato a 0.025 -> 0.050; Cube: mesh +-0.05 -> 0.075).
% Su 'liscio' galleggiano percio' di quei 25 mm, misurati da quota_terreno.
% Si trasla il loro Rigid Transform dello scarto: cosi' conservano la quota
% relativa che avevano sul Cube, incluso il gioco di 0.15 mm.
%
% IL SEGNO E' POSITIVO, E NON E' UN REFUSO. Quel Rigid Transform e' percorso
% nel verso opposto a quello che verrebbe da assumere, quindi la sua z e'
% invertita rispetto alla z del mondo: con -0.025 l'ostacolo SALE. Misurato,
% non dedotto - il primo tentativo con il segno "ovvio" lo ha alzato di
% altri 25 mm.
%
% NOTA: ost4..ost7 poggiano sui RILIEVI del terreno imperfetto (da +35 a
% +71 mm). Su pavimento liscio quei rilievi non esistono e restano in aria
% anche dopo la correzione: su 'liscio' sono utilizzabili solo ost1..ost3.
lisciOn = ismember('liscio', vuoi);
if lisciOn
    if isfield(cfg,'terreno') && isfield(cfg.terreno,'ost_dz')
        ost_dz = cfg.terreno.ost_dz;
    else
        ost_dz = 0.025;
    end
else
    ost_dz = 0;
end
assignin('base', 'ost_dz', ost_dz);
for k = 10:16
    try
        if ost_dz == 0
            set_param([mdl '/' rt(k)], 'TranslationCartesianOffset', 'floor_off');
        else
            set_param([mdl '/' rt(k)], 'TranslationCartesianOffset', ...
                      'floor_off(:).'' + [0 0 ost_dz]');
        end
    catch
        mancanti{end+1} = rt(k); %#ok<AGROW>
    end
end

%% ---- lettura dal modello: etichetta di ogni solido ----
lblSolido = cell(size(nomi));
for k = 1:numel(nomi)
    lblSolido{k} = labelDi([mdl '/' cat{k,2}]);
end

%% ---- lettura dal modello: etichetta -> contatto, piede per piede ----
mappe = cell(size(PIEDI));
for p = 1:numel(PIEDI)
    mappe{p} = mappaPiede(mdl, PIEDI{p});
end

%% ---- solidi ----
acceso = ismember(nomi, vuoi);
for k = 1:numel(nomi)
    mancanti = [mancanti, commenta([mdl '/' cat{k,2}], ...
                                   ternario(acceso(k),'off','on'))]; %#ok<AGROW>
end

%% ---- contatti, per NOME dell'etichetta ----
% 'liscio' e 'imperfetto' condividono l'etichetta 'pavimento': il contatto
% del pavimento resta acceso se almeno uno dei due e' attivo, altrimenti il
% terreno diventa attraversabile e il robot cade nel vuoto.
attive = unique(lblSolido(acceso));
for p = 1:numel(PIEDI)
    m = mappe{p};
    for lab = keys(m)
        on = ismember(lab{1}, attive);
        mancanti = [mancanti, commenta( ...
            sprintf('%s/%s/%s', mdl, PIEDI{p}, m(lab{1})), ...
            ternario(on,'off','on'))]; %#ok<AGROW>
    end
end

%% ---- rampa: prende in prestito lo slot 'ostacolo7' ----
SLOT_RAMPA = 'ostacolo7';
rampaOn = ismember('rampa', vuoi);
if rampaOn
    % la rampa si presenta come 'ostacolo7'; File Solid7 e' gia' spento
    % (non e' fra i 'vuoi'), ma il suo Connection Label va tolto di mezzo:
    % due sorgenti di geometria sulla stessa linea non sono ammesse. I
    % Connection Label non si possono commentare
    %   "Block ... cannot be commented as it is not supported"
    % quindi quello di troppo va su un'etichetta che nessuno usa.
    etichetta([mdl '/Connection Label7'],  SLOT_RAMPA);
    etichetta([mdl '/Connection Label12'], 'slot_libero');
    for p = 1:numel(PIEDI)
        m = mappe{p};
        if isKey(m, SLOT_RAMPA)
            mancanti = [mancanti, commenta( ...
                sprintf('%s/%s/%s', mdl, PIEDI{p}, m(SLOT_RAMPA)), 'off')]; %#ok<AGROW>
        end
    end
    try
        set_param([mdl '/' rt(7)], 'Commented', 'off');
        set_param([mdl '/' rt(7)], 'TranslationCartesianOffset', mat2str(cfg.terreno.rampa_pos));
        set_param([mdl '/' rt(7)], 'RotationMethod',       'StandardAxis');
        set_param([mdl '/' rt(7)], 'RotationStandardAxis', '+Y');
        set_param([mdl '/' rt(7)], 'RotationAngle',        num2str(cfg.terreno.rampa_gradi));
    catch ME
        fprintf(2,'  rampa: posa non impostata (%s)\n', ME.message);
    end
end

% Il terreno attivo va DICHIARATO nel base workspace, non ricordato a mente.
% adatta_simscape lo legge e lo mette in run.meta.terreno, quindi finisce in
% una colonna di ogni riga di metriche.
%
% PERCHE': i runner delle campagne non impostavano il terreno, usavano quello
% rimasto dalla chiamata precedente. Una campagna T2 e' partita su T5, con il
% gradino, e nulla nei risultati lo avrebbe detto - i numeri sarebbero stati
% plausibili e sbagliati. Con la colonna in tabella l'errore si vede subito.
assignin('base', 'TERRENO_ATTIVO', task);

info = struct('task',task, 'attivi',{vuoi}, 'etichette',{attive}, ...
              'mappa',mappe{1}, 'modello',mdl);

%% ---- riepilogo ----
if verbose
    fprintf('\n--- TERRENO: %s ---\n', task);
    for k = 1:numel(nomi)
        fprintf('  %-12s %-10s  etichetta %-12s\n', ...
                nomi{k}, ternario(acceso(k),'ATTIVO','-'), lblSolido{k});
    end
    if rampaOn
        fprintf('  rampa: %s, %g deg attorno a +Y, su slot ''%s''\n', ...
                mat2str(cfg.terreno.rampa_pos), cfg.terreno.rampa_gradi, SLOT_RAMPA);
    end
    if ~isempty(mancanti)
        fprintf(2,'  blocchi non trovati: %s\n', strjoin(unique(mancanti), ', '));
    end
    fprintf('  (il modello NON e'' stato salvato)\n\n');
end

end

%% ================================================================
function m = mappaPiede(mdl, piede)
%MAPPAPIEDE  etichetta -> nome del blocco di contatto, letta dal modello.
%
%   Il percorso della geometria del terreno e'
%       Spatial Contact Force, porta B -> Connection Port n (dentro il piede)
%       -> porta n del Subsystem (al livello di sopra) -> Connection Label
%       -> etichetta -> solido
%
%   Le Connection Port sono numerate 1..N su un'unica sequenza che comprende
%   entrambi i lati del Subsystem, mentre le porte del livello di sopra sono
%   LConn1..LConn8 (le etichette del terreno) e RConn1 (il nodo condiviso
%   della sfera del piede). L'offasamento fra le due numerazioni si ricava
%   dai dati - non si assume - e viene verificato.

m = containers.Map('KeyType','char','ValueType','char');

pc = get_param([mdl '/' piede], 'PortConnectivity');
lab = {};
for p = 1:numel(pc)
    t = pc(p).Type;
    if ~ischar(t) || ~startsWith(t,'LConn'), continue; end
    n = str2double(t(6:end));
    if isnan(n), continue; end
    h = [pc(p).SrcBlock, pc(p).DstBlock];  h = h(h > 0);
    for j = 1:numel(h)
        nome = get_param(h(j),'Name');
        if startsWith(strrep(nome,newline,' '), 'Connection Label')
            lab{n} = get_param([mdl '/' nome], 'Label'); %#ok<AGROW>
        end
    end
end

% Due trappole di find_system, tutte e due gia' costate un giro a vuoto:
%
%   1. ESCLUDE I BLOCCHI COMMENTATI. La mappa va letta proprio quando
%      quasi tutti i contatti sono spenti da una chiamata precedente:
%      senza IncludeCommented ne tornavano due su otto, e le porte
%      risultavano [2 4] invece di [2..9].
%   2. Il filtro per nome non vede i nomi con l'a capo dentro
%      ('Spatial Contact\nForce3'). get_param invece l'a capo lo normalizza
%      da solo quando risolve un percorso - ed e' il motivo per cui
%      mappa_contatti, che costruiva i nomi con sprintf, li trovava tutti.
%      Percio' qui si prendono tutti i blocchi e si filtra a mano.
try
    tutti = find_system([mdl '/' piede], 'SearchDepth',1, ...
                        'LookUnderMasks','all', 'FollowLinks','on', ...
                        'IncludeCommented','on', 'Type','block');
catch
    % release che non conosce IncludeCommented
    tutti = find_system([mdl '/' piede], 'SearchDepth',1, ...
                        'LookUnderMasks','all', 'FollowLinks','on', 'Type','block');
end
blocchi = {};
for i = 1:numel(tutti)
    n = strrep(get_param(tutti{i},'Name'), newline, ' ');
    if startsWith(n, 'Spatial Contact Force')
        blocchi{end+1} = tutti{i}; %#ok<AGROW>
    end
end
% Rete di sicurezza: se find_system ne ha resi meno delle etichette, si
% ripiega sui nomi costruiti. get_param risolve il percorso anche per un
% blocco commentato e anche se il nome vero contiene un a capo.
if numel(blocchi) < numel(lab)
    % il confronto e' sull'handle, non sulla stringa: lo stesso blocco puo'
    % comparire come 'Spatial Contact Force3' e 'Spatial Contact\nForce3'
    noti = zeros(size(blocchi));
    for i = 1:numel(blocchi), noti(i) = getSimulinkBlockHandle(blocchi{i}); end
    for k = 1:numel(lab)
        b = sprintf('%s/%s/Spatial Contact Force%d', mdl, piede, k);
        h = getSimulinkBlockHandle(b);
        if h > 0 && ~any(noti == h)
            blocchi{end+1} = b;  noti(end+1) = h; %#ok<AGROW>
        end
    end
end

porta = nan(size(blocchi));
for i = 1:numel(blocchi)
    porta(i) = portaB(blocchi{i});
end
ok = ~isnan(porta);
blocchi = blocchi(ok);  porta = porta(ok);

if isempty(porta) || isempty(lab)
    warning('applica_terreno:mappa', ...
        'Piede %s: non sono riuscito a leggere la mappa dei contatti.', piede);
    return
end

off = min(porta) - 1;
atteso = off + (1:numel(lab));
if ~isequal(sort(porta(:)).', atteso)
    error('applica_terreno:mappa', ...
        ['Piede %s: %d contatti trovati, %d etichette di terreno, porte %s.\n' ...
         'Le porte dovrebbero essere %d consecutive: non lo sono, quindi la\n' ...
         'corrispondenza etichetta -> contatto non e'' ricostruibile.\n' ...
         'Lancia mappa_contatti(''%s'',''%s'') e guarda il cablaggio.'], ...
        piede, numel(porta), numel(lab), mat2str(sort(porta(:)).'), ...
        numel(lab), mdl, piede);
end

for i = 1:numel(blocchi)
    e = lab{porta(i) - off};
    if isempty(e), continue; end
    m(e) = strrep(blocchi{i}, [mdl '/' piede '/'], '');
end
end

function n = portaB(blocco)
%PORTAB  Numero della Connection Port attaccata DIRETTAMENTE al blocco.
%   Un salto solo: gli otto contatti di un piede condividono la porta A
%   sulla stessa sfera, quindi qualunque attraversamento della rete li
%   raggiunge tutti e la lettura diventa inutilizzabile.
n = NaN;
padre = get_param(blocco,'Parent');
try pc = get_param(blocco,'PortConnectivity'); catch, return; end
for p = 1:numel(pc)
    h = [pc(p).SrcBlock, pc(p).DstBlock];  h = h(h > 0);
    for j = 1:numel(h)
        nome = get_param(h(j),'Name');
        blk  = [padre '/' nome];
        bt = '';  try bt = get_param(blk,'BlockType'); catch, end
        if strcmp(bt,'PMIOPort')
            v = str2double(get_param(blk,'Port'));
            if ~isnan(v) && (isnan(n) || v > n), n = v; end
        end
    end
end
end

function lab = labelDi(blocco)
%LABELDI  Etichetta del Connection Label attaccato direttamente al solido.
lab = '';
padre = get_param(blocco,'Parent');
try pc = get_param(blocco,'PortConnectivity'); catch, return; end
for p = 1:numel(pc)
    h = [pc(p).SrcBlock, pc(p).DstBlock];  h = h(h > 0);
    for j = 1:numel(h)
        n = get_param(h(j),'Name');
        if startsWith(strrep(n,newline,' '), 'Connection Label')
            try lab = get_param([padre '/' n],'Label'); return; catch, end
        end
    end
end
end

function manca = commenta(blocco, stato)
manca = {};
try set_param(blocco, 'Commented', stato); catch, manca = {blocco}; end
end

function etichetta(blocco, valore)
try set_param(blocco, 'Label', valore); catch, end
end

function s = rt(k)
if isempty(k), s = sprintf('Rigid\nTransform');
else,          s = sprintf('Rigid\nTransform%d', k);
end
end

function v = ternario(c,a,b)
if c, v = a; else, v = b; end
end