function T = trova_filtri(mdl)
%TROVA_FILTRI  Chi usa sys_filter (o tau) nel modello, e serve a qualcosa?
%
%   T = trova_filtri
%   T = trova_filtri('phantomx_sim_zero')
%
% LA DOMANDA
%   Lo sweep di tau ha dato cinque volte gli STESSI numeri, alla quinta cifra,
%   e il residuo in workspace dice che il polo del filtro era -500 (cioe'
%   -1/0.002, l'ultima cella): quindi tau ARRIVA al filtro, il polo cambia di
%   un fattore 25, e il robot cammina identico.
%
%   Se la dinamica del filtro cambia e il risultato no, il filtro non e' nel
%   percorso del comando. Le possibilita' sono tre, e vanno distinte prima di
%   scrivere qualsiasi cosa in relazione:
%     1. il blocco che usa sys_filter e' COMMENTATO -> il filtro non esiste;
%     2. il blocco esiste ma la sua uscita non e' collegata ai giunti;
%     3. sys_filter e' un residuo di una versione precedente e nessun blocco
%        lo nomina piu'.
%
%   In tutti e tre i casi la conclusione sulla cella 2x cambia: i filtri
%   escono dalla lista dei sospetti e il fallimento resta senza spiegazione,
%   che e' un esito diverso da "il limite e' del controllore".
%
% PERCHE' init_gait NON BASTA A VERIFICARE
%   L'InitFcn e' una callback del MODELLO: gira all'avvio della simulazione,
%   non quando chiami init_gait. Leggere tau e sys_filter dal workspace dopo
%   init_gait restituisce i valori dell'ULTIMA simulazione. Questa funzione
%   lancia una sim breve prima di leggere.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(mdl), mdl = 'phantomx_sim_zero'; end
load_system(mdl);

fprintf('\n=========== CHI USA I FILTRI ===========\n');

%% ---- 1. il polo segue tau_filtro? (con la sim, non senza) ----
fprintf('\n--- il collegamento tau_filtro -> filtro ---\n');
tau_prova = 0.007;
vecchio = leggiBase('tau_filtro');
assignin('base','tau_filtro', tau_prova);
try
    evalin('base','init_gait');
    evalin('base', sprintf('evalc(''sim(''''%s'''',''''StopTime'''',''''0.2'''')'')', mdl));
    ok_sim = true;
catch ME
    fprintf(2,'  la sim di prova e'' fallita: %s\n', ME.message);
    ok_sim = false;
end

if ok_sim
    tau_letto = leggiBase('tau');
    polo      = poloFiltro();
    fprintf('  tau_filtro imposto  : %.4f s\n', tau_prova);
    fprintf('  tau nel workspace   : %.4f s\n', tau_letto);
    if isnan(polo)
        fprintf(2,'  polo del filtro     : non leggibile\n');
    else
        fprintf('  polo del filtro     : %.2f   (atteso %.2f)\n', polo, -1/tau_prova);
        if abs(polo - (-1/tau_prova)) < 0.01*abs(1/tau_prova)
            fprintf('  -> il filtro SEGUE tau_filtro. L''InitFcn e'' a posto.\n');
        else
            fprintf(2,'  -> il filtro NON segue tau_filtro: override dopo ss().\n');
        end
    end
end

%% ---- 2. quali blocchi nominano sys_filter o tau ----
% Si guardano TUTTI i parametri di dialogo di ogni blocco, compresi i
% commentati: un filtro commentato e' esattamente l'ipotesi da falsificare, e
% find_system senza IncludeCommented lo nasconderebbe.
fprintf('\n--- blocchi che nominano i filtri ---\n');
blocchi = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                      'IncludeCommented','on', 'Type','block');
fprintf('  %d blocchi esaminati\n\n', numel(blocchi));

nome = {}; param = {}; valore = {}; comm = {}; tipo = {};
chiavi = {'sys_filter','tau'};
for k = 1:numel(blocchi)
    b = blocchi{k};
    try, pars = get_param(b,'DialogParameters'); catch, continue; end
    if isempty(pars), continue; end
    campi = fieldnames(pars);
    for j = 1:numel(campi)
        try, v = get_param(b, campi{j}); catch, continue; end
        if ~(ischar(v) || isstring(v)), continue; end
        v = char(v);
        if isempty(v), continue; end
        if ~any(cellfun(@(c) contieneIdent(v,c), chiavi)), continue; end
        nome{end+1,1}   = strrep(b, newline, ' ');                %#ok<AGROW>
        param{end+1,1}  = campi{j};                               %#ok<AGROW>
        valore{end+1,1} = v;                                      %#ok<AGROW>
        comm{end+1,1}   = get_param(b,'Commented');               %#ok<AGROW>
        tipo{end+1,1}   = get_param(b,'BlockType');               %#ok<AGROW>
    end
end

if isempty(nome)
    fprintf(2,['  NESSUN blocco nomina sys_filter o tau.\n' ...
               '  Allora i filtri sono un residuo: l''InitFcn li costruisce e il\n' ...
               '  modello non li usa. La cella 2x non c''entra con i filtri.\n']);
    T = table();
else
    T = table(string(nome), string(tipo), string(param), string(valore), string(comm), ...
              'VariableNames', {'blocco','tipo','parametro','valore','commentato'});
    for k = 1:height(T)
        marca = '';
        if strcmp(T.commentato(k),'on'), marca = '   <-- COMMENTATO'; end
        fprintf('  %-45s  %-18s = %s%s\n', ...
                abbrevia(char(T.blocco(k)), 45), char(T.parametro(k)), ...
                char(T.valore(k)), marca);
    end

    attivi = ~strcmp(T.commentato, 'on');
    fprintf('\n  %d riferimenti, di cui %d attivi\n', height(T), sum(attivi));
    if ~any(attivi)
        fprintf(2,['  Tutti COMMENTATI: il filtro non esiste in simulazione.\n' ...
                   '  Spiega lo sweep piatto, e i filtri escono dai sospetti.\n']);
    end
end

%% ---- 3. se i blocchi ci sono, la loro uscita e' collegata? ----
nCollegati = 0;  nStaccatiDelTutto = 0;  visti = {};
if ~isempty(nome)
    fprintf('\n--- le uscite dei blocchi attivi ---\n');
    visti = unique(nome(~strcmp(comm,'on')));
    if isempty(visti)
        fprintf('  nessun blocco attivo da controllare\n');
    end
    for k = 1:numel(visti)
        b = visti{k};
        try
            pc = get_param(b, 'PortConnectivity');
        catch
            fprintf('  %s: connettivita'' non leggibile\n', abbrevia(b,45));
            continue;
        end
        nUscite = 0;  nStaccate = 0;
        for p = 1:numel(pc)
            if isempty(pc(p).SrcBlock) && ~isempty(pc(p).Type) ...
                    && ~isnan(str2double(pc(p).Type))
                % porta di uscita: Type e' il numero della porta
                nUscite = nUscite + 1;
                if isempty(pc(p).DstBlock), nStaccate = nStaccate + 1; end
            end
        end
        stato = 'collegata';
        if nStaccate > 0, stato = sprintf('%d uscite STACCATE', nStaccate); end
        if nUscite > 0 && nStaccate == nUscite
            nStaccatiDelTutto = nStaccatiDelTutto + 1;
        else
            nCollegati = nCollegati + 1;
        end
        fprintf('  %-45s  %d uscite, %s\n', abbrevia(b,45), nUscite, stato);
    end
end

%% ---- 4. esiste un filtro con i coefficienti scritti a numero? ----
% La ricerca per identificatore non lo vedrebbe: un Transfer Fcn con
% denominatore [0.05 1] filtra esattamente come sys_filter e non nomina ne'
% tau ne' sys_filter. Senza questo censimento la conclusione "nessun filtro nel
% percorso del comando" sarebbe un salto: e' dimostrato solo che sys_filter non
% e' usato, che e' un'altra affermazione.
fprintf('\n--- blocchi dinamici nel modello (filtri possibili) ---\n');
tipiFiltro = {'TransferFcn','StateSpace','ZeroPole','DiscreteFilter', ...
              'DiscreteTransferFcn','DiscreteStateSpace','DiscreteZeroPole', ...
              'RateLimiter','FirstOrderHold','TransportDelay','VariableTransportDelay', ...
              'UnitDelay','Memory','Derivative','Integrator','DiscreteIntegrator'};
nFiltro = 0;  nFiltroAttivi = 0;
for k = 1:numel(blocchi)
    b = blocchi{k};
    try, bt = get_param(b,'BlockType'); catch, continue; end
    if ~any(strcmp(bt, tipiFiltro)), continue; end
    nFiltro = nFiltro + 1;
    cm = get_param(b,'Commented');
    if ~strcmp(cm,'on'), nFiltroAttivi = nFiltroAttivi + 1; end
    dett = '';
    for pn = {'Denominator','Numerator','A','RisingSlewLimit','SampleTime','Gain'}
        try
            v = get_param(b, pn{1});
            if ischar(v) && ~isempty(v)
                dett = [dett sprintf(' %s=%s', pn{1}, v)];     %#ok<AGROW>
            end
        catch
        end
    end
    marca = '';
    if strcmp(cm,'on'), marca = '  <-- COMMENTATO'; end
    fprintf('  %-40s %-22s%s%s\n', abbrevia(strrep(b,newline,' '),40), bt, dett, marca);
end
fprintf('  %d blocchi dinamici, %d attivi\n', nFiltro, nFiltroAttivi);

%% ---- ripristino ----
if isnan(vecchio)
    evalin('base','clear tau_filtro');
else
    assignin('base','tau_filtro', vecchio);
end
try, evalin('base','init_gait'); catch, end

%% ---- il verdetto, deciso dai dati ----
% [CORRETTO] La prima versione stampava TUTTE le letture possibili lasciando a
% chi legge il compito di scegliere quella giusta. Non e' una diagnosi, e' un
% menu: il verdetto lo devono decidere i conteggi appena raccolti.
fprintf('\n=============== VERDETTO ===============\n');
if isempty(nome)
    fprintf(2,'  I FILTRI SONO UN RESIDUO.\n');
    fprintf([ ...
      '  Nessun blocco del modello nomina sys_filter o tau: l''InitFcn li\n' ...
      '  costruisce e nessuno li usa. Lo sweep di ieri era corretto nel\n' ...
      '  meccanismo (il polo cambiava da -20 a -500) e vuoto nell''effetto.\n\n' ...
      '  Conseguenza in relazione: sys_filter esce dai sospetti, e la cella\n' ...
      '  2x resta SENZA CAUSA ATTRIBUITA. Non e'' la stessa cosa di "limite\n' ...
      '  del controllore cinematico" e non va scritta cosi''.\n\n' ...
      '  ATTENZIONE AL SALTO: questo dimostra che sys_filter non e'' usato,\n' ...
      '  non che nel percorso del comando non ci sia un filtro. Il censimento\n' ...
      '  qui sopra ha trovato %d blocchi dinamici attivi: se fra quelli c''e''\n' ...
      '  un Transfer Fcn o un Rate Limiter sui comandi di giunto, e'' LUI il\n' ...
      '  filtro vero, con i coefficienti scritti a numero, e va sweepato\n' ...
      '  quello.\n'], nFiltroAttivi);
elseif ~any(attivi)
    fprintf(2,'  I FILTRI SONO COMMENTATI.\n');
    fprintf([ ...
      '  Tutti i %d riferimenti stanno in blocchi commentati: in simulazione\n' ...
      '  il filtro non esiste. Spiega lo sweep piatto, con una ragione in\n' ...
      '  piu'' rispetto al residuo.\n\n' ...
      '  Conseguenza identica: i filtri escono dai sospetti e la cella 2x\n' ...
      '  resta senza causa. Prossimo sospetto la traiettoria di swing.\n'], ...
      height(T));
elseif nCollegati == 0
    fprintf(2,'  I FILTRI HANNO LE USCITE STACCATE.\n');
    fprintf([ ...
      '  %d blocchi attivi nominano i filtri, e nessuno ha un''uscita\n' ...
      '  collegata: il filtro gira a vuoto. Come sopra per la relazione.\n'], ...
      nStaccatiDelTutto);
else
    fprintf('  I FILTRI SONO COLLEGATI, E IRRILEVANTI.\n');
    fprintf([ ...
      '  %d blocchi attivi con uscite collegate, elencati sopra. Allora\n' ...
      '  l''effetto nullo dello sweep e'' un DATO FISICO: venticinque volte\n' ...
      '  il polo non cambia nulla nell''andatura.\n\n' ...
      '  Va dichiarato, non corretto - ma va anche verificato a mano che i\n' ...
      '  blocchi elencati stiano davvero fra il generatore di traiettoria e i\n' ...
      '  giunti, e non su un ramo di logging.\n'], nCollegati);
end
fprintf('\n');

end

%% ================================================================
function tf = contieneIdent(testo, ident)
%CONTIENEIDENT  L'identificatore compare come PAROLA, non come sottostringa.
%   Serve per 'tau': senza questo, ogni blocco con 'taurus' o 'default'
%   finirebbe in elenco. \< e \> sono i confini di parola di regexp.
tf = ~isempty(regexp(testo, ['\<' regexptranslate('escape',ident) '\>'], 'once'));
end

function s = abbrevia(s, n)
if numel(s) > n, s = ['...' s(end-n+4:end)]; end
end

function v = leggiBase(nome)
v = NaN;
try
    if evalin('base', sprintf('exist(''%s'',''var'')', nome))
        x = evalin('base', nome);
        if ~isempty(x) && isnumeric(x), v = double(x(1)); end
    end
catch
end
end

function p = poloFiltro()
p = NaN;
try
    if evalin('base','exist(''sys_filter'',''var'')')
        A = evalin('base','sys_filter.A');
        if ~isempty(A), p = double(A(1,1)); return; end
    end
catch
end
for nome = {'A','a'}
    try
        if evalin('base', sprintf('exist(''%s'',''var'')', nome{1}))
            M = evalin('base', nome{1});
            if ~isempty(M) && isnumeric(M), p = double(M(1,1)); return; end
        end
    catch
    end
end
end
