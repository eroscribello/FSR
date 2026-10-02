function verifica_log(mdl)
%VERIFICA_LOG  Perche' Fleg non arriva nell'uscita della simulazione.
%
%   verifica_log
%
% IL FATTO
%   abilita_log('on') non ha prodotto effetto: appoggio_medio, disp_carico,
%   slip_tot e Fz_max_norm restano NaN, perche' adatta_simscape costruisce
%   run.Fc e run.contact solo da Fleg (riga 262).
%
% TRE CAUSE POSSIBILI, IN ORDINE DI PROBABILITA'
%   1. il To Workspace e' dentro un SOTTOSISTEMA a sua volta commentato.
%      Scommentare il blocco non serve: un antenato commentato lo esclude
%      comunque, e get_param sul blocco continua a dire Commented = off.
%   2. il To Workspace e' attivo ma il segnale che lo alimenta arriva da un
%      blocco commentato, quindi la linea e' interrotta.
%   3. il nome della variabile non e' 'Fleg' (maiuscole, spazi, un suffisso).
%
%   Questa funzione le distingue: elenca ogni To Workspace con il suo nome di
%   variabile, il proprio stato di commento, lo stato di TUTTI i suoi
%   antenati, e la sorgente del suo segnale con lo stato di quella.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(mdl), mdl = 'phantomx_sim_zero'; end
if ~bdIsLoaded(mdl), load_system(mdl); end

b = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                'IncludeCommented','on', 'BlockType','ToWorkspace');

fprintf('\n=========== TO WORKSPACE DEL MODELLO ===========\n');
fprintf('  trovati %d blocchi\n\n', numel(b));
fprintf('  %-14s %-9s %-9s %s\n', 'variabile', 'blocco', 'antenati', 'sorgente del segnale');
fprintf('  %s\n', repmat('-',1,78));

problemi = {};

for k = 1:numel(b)
    v   = get_param(b{k}, 'VariableName');
    cb  = get_param(b{k}, 'Commented');

    % stato degli antenati
    catena = {};
    p = get_param(b{k}, 'Parent');
    while ~isempty(p) && ~strcmp(p, mdl)
        try
            cp = get_param(p, 'Commented');
        catch
            cp = '?';
        end
        if ~strcmp(cp, 'off')
            catena{end+1} = sprintf('%s=%s', ...
                strrep(get_param(p,'Name'), newline, ' '), cp);        %#ok<AGROW>
        end
        p = get_param(p, 'Parent');
    end
    if isempty(catena), ant = 'ok'; else, ant = strjoin(catena, ','); end

    % sorgente del segnale
    src = '(nessuna)';
    try
        pc = get_param(b{k}, 'PortConnectivity');
        for q = 1:numel(pc)
            h = pc(q).SrcBlock;
            if h > 0
                nm = strrep(get_param(h,'Name'), newline, ' ');
                cs = get_param(h, 'Commented');
                src = sprintf('%s [%s]', nm, cs);
            end
        end
    catch
    end

    fprintf('  %-14s %-9s %-9s %s\n', v, cb, ant, src);

    if strcmpi(v,'Fleg') || strcmpi(v,'Fsum')
        if ~strcmp(cb,'off')
            problemi{end+1} = sprintf('%s: il blocco e'' commentato', v); %#ok<AGROW>
        end
        if ~strcmp(ant,'ok')
            problemi{end+1} = sprintf('%s: un antenato e'' commentato (%s)', v, ant); %#ok<AGROW>
        end
        if contains(src,'[on]')
            problemi{end+1} = sprintf('%s: la sorgente del segnale e'' commentata (%s)', v, src); %#ok<AGROW>
        end
    end
end

%% ---- Fleg e Fsum esistono? ----
nomi = cellfun(@(x) get_param(x,'VariableName'), b, 'UniformOutput', false);
fprintf('\n');
for v = {'Fleg','Fsum'}
    if any(strcmpi(nomi, v{1}))
        fprintf('  %-6s presente nel modello\n', v{1});
    else
        fprintf(2,'  %-6s NON ESISTE come To Workspace. Nomi presenti:\n', v{1});
        fprintf(2,'         %s\n', strjoin(unique(nomi), ', '));
        problemi{end+1} = sprintf('%s non esiste: il nome e'' un altro', v{1}); %#ok<AGROW>
    end
end

%% ---- verdetto ----
fprintf('\n--- verdetto ---\n');
if isempty(problemi)
    fprintf(['  Nessun impedimento nello schema: Fleg e Fsum sono attivi, senza\n' ...
             '  antenati commentati e con sorgente attiva.\n' ...
             '  Allora il problema non e'' qui. Prova:\n' ...
             '      init_gait ; applica_terreno(''T1'')\n' ...
             '      out = sim(''%s'',''StopTime'',''3'');\n' ...
             '      disp(out)\n' ...
             '  Se Fleg non compare fra le variabili di out, guarda il campo\n' ...
             '  SaveFormat del blocco e il "Limit data points to last" - un\n' ...
             '  To Workspace con Decimation o limite stretto puo'' restituire\n' ...
             '  un oggetto vuoto senza segnalare nulla.\n'], mdl);
else
    for k = 1:numel(problemi)
        fprintf(2,'  %d. %s\n', k, problemi{k});
    end
    fprintf(['\n  Un antenato commentato e'' il caso piu'' frequente e il piu''\n' ...
             '  insidioso: get_param sul To Workspace continua a dire\n' ...
             '  Commented = off, quindi abilita_log sembra riuscito.\n' ...
             '  Va scommentato il SOTTOSISTEMA, non il blocco.\n']);
end
fprintf('\n');

end