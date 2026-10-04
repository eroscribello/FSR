%% estrai_funzioni.m 
% Estrae dal .slx il codice dei blocchi MATLAB Function
%

function info = estrai_funzioni(mdl, etichetta)

if nargin < 1 || isempty(mdl),       mdl = 'phantomx_sim_zero'; end
if nargin < 2 || isempty(etichetta), etichetta = mdl;           end

if ~bdIsLoaded(mdl), load_system(mdl); end

%% ---- i blocchi MATLAB Function ----
bb = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                 'BlockType','SubSystem', 'SFBlockType','MATLAB Function');

if isempty(bb)
    % Non ci si arrende in silenzio: si dice cosa c'e', come per i giunti.
    fprintf(2, '\n  Nessun blocco MATLAB Function trovato in %s.\n', mdl);
    fprintf(2, '  Sottosistemi presenti, con il loro SFBlockType:\n');
    ss = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                     'BlockType','SubSystem');
    tipi = containers.Map('KeyType','char','ValueType','double');
    for i = 1:numel(ss)
        t = '(nessuno)';
        try, t = get_param(ss{i}, 'SFBlockType'); end %#ok<TRYNC>
        if isempty(t), t = '(vuoto)'; end
        if isKey(tipi,t), tipi(t) = tipi(t)+1; else, tipi(t) = 1; end
    end
    k = keys(tipi);
    for i = 1:numel(k), fprintf(2, '    %-28s %d\n', k{i}, tipi(k{i})); end
    info = table();
    return
end

%% ---- la cartella ----
dest = fullfile('estratti', matlab.lang.makeValidName(char(etichetta)));
if ~isfolder(dest), mkdir(dest); end

fprintf('\n=========== FUNZIONI DI %s ===========\n', mdl);
fprintf('  %d blocchi MATLAB Function -> %s\n\n', numel(bb), dest);

nome = strings(0,1);  percorso = strings(0,1);
righe = [];  file = strings(0,1);  sid = strings(0,1);

for i = 1:numel(bb)
    blk = bb{i};
    nm  = regexprep(get_param(blk,'Name'), '\s+', ' ');

    codice = ef_codice(blk);
    if isempty(codice)
        fprintf(2, '  %-26s codice non leggibile\n', nm);
        continue
    end

    s = '';
    try, s = num2str(get_param(blk,'SID')); end %#ok<TRYNC>

    % intestazione: da dove viene, cosi' il file estratto si spiega da solo
    testa = sprintf(['%% ==== estratto da %s ====\n' ...
                     '%% blocco : %s\n' ...
                     '%% SID    : %s\n' ...
                     '%% quando : %s\n' ...
                     '%% Generato da estrai_funzioni.m - NON e'' un file del progetto:\n' ...
                     '%% serve solo a poter fare un diff su codice che vive nel .slx.\n\n'], ...
                     mdl, regexprep(blk,'\s+',' '), s, datestr(now,'yyyy-mm-dd HH:MM'));

    f = fullfile(dest, [matlab.lang.makeValidName(nm) '.m']);
    fid = fopen(f, 'w');
    if fid < 0
        fprintf(2, '  %-26s non posso scrivere %s\n', nm, f);
        continue
    end
    fwrite(fid, [testa codice]);
    fclose(fid);

    n = numel(strfind(codice, newline)) + 1;
    fprintf('  %-26s %5d righe  ->  %s\n', nm, n, f);

    nome(end+1,1)     = string(nm);                       %#ok<AGROW>
    percorso(end+1,1) = string(regexprep(blk,'\s+',' ')); %#ok<AGROW>
    righe(end+1,1)    = n;                                %#ok<AGROW>
    file(end+1,1)     = string(f);                        %#ok<AGROW>
    sid(end+1,1)      = string(s);                        %#ok<AGROW>
end

info = table(nome, sid, righe, percorso, file, ...
             'VariableNames', {'blocco','SID','righe','percorso','file'});

fprintf('\n  Solo lettura sul modello: nessun set_param, nessun save_system.\n');
fprintf('  Per confrontare con un''altra versione: bdclose all, poi la stessa\n');
fprintf('  chiamata con un''etichetta diversa, e infine visdiff sui due file.\n\n');
end

%% ================= helper =================
function c = ef_codice(blk)
%EF_CODICE  Il sorgente di un blocco MATLAB Function.
%   Due strade perche' l'API e' cambiata: dalla R2019b c'e' la proprieta'
%   MATLABFunctionConfiguration, prima serviva l'API Stateflow. Si prova la
%   nuova e si ripiega sulla vecchia, invece di fissare una versione.
c = '';
try
    cfgObj = get_param(blk, 'MATLABFunctionConfiguration');
    c = char(cfgObj.FunctionScript);
    if ~isempty(c), return; end
catch
end
try
    rt = sfroot;
    ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
    if ~isempty(ch), c = char(ch(1).Script); end
catch
end
end
