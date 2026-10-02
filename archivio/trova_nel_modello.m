function T = trova_nel_modello(testo, modello)
%TROVA_NEL_MODELLO  Cerca una stringa nei parametri di tutti i blocchi.
%
%   trova_nel_modello('body_z0')
%   trova_nel_modello('0.19')
%   T = trova_nel_modello('I_tibia', 'phantomx_sim_zero')
%
% Serve a rispondere a "dove diavolo viene usato questo valore?" senza
% aprire trenta maschere a mano. Utile per:
%   - capire dove entra una variabile prima di cambiarla
%   - scoprire i numeri scritti a mano che dovrebbero venire da phantomx_config
%   - controllare che un set_param abbia davvero attecchito
%
% Ignora MaskValueString (e' la maschera serializzata: duplica tutto).
%
% Progetto FSR PhantomX - A. Russo

if nargin < 2 || isempty(modello), modello = 'phantomx_sim_zero'; end
[~, modello] = fileparts(modello);

giaAperto = bdIsLoaded(modello);
if ~giaAperto
    fprintf('Carico %s ...\n', modello);
    load_system(modello);
end

IGNORA = {'MaskValueString','MaskVariables','MaskPromptString', ...
          'MaskDisplay','MaskInitialization','MaskHelp','MaskDescription', ...
          'Description','AttributesFormatString'};

blocchi = find_system(modello, 'LookUnderMasks','all', 'FollowLinks','on', ...
                      'MatchFilter', @Simulink.match.allVariants, 'Type','Block');

blocco = {}; parametro = {}; valore = {};

for k = 1:numel(blocchi)
    b = blocchi{k};
    try
        pars = fieldnames(get_param(b,'ObjectParameters'));
    catch
        continue
    end
    for j = 1:numel(pars)
        if any(strcmp(pars{j}, IGNORA)), continue; end
        try
            v = get_param(b, pars{j});
        catch
            continue
        end
        if ~(ischar(v) || (isstring(v) && isscalar(v))), continue; end
        v = char(v);
        if isempty(v) || ~contains(v, testo), continue; end
        blocco{end+1}    = strrep(b, [modello '/'], '');   %#ok<AGROW>
        parametro{end+1} = pars{j};                        %#ok<AGROW>
        valore{end+1}    = strtrim(v);                     %#ok<AGROW>
    end
end

if ~giaAperto, close_system(modello, 0); end

if isempty(blocco)
    fprintf(2,'\n"%s" non compare in nessun parametro di %s.\n\n', testo, modello);
    T = table();
    return
end

T = table(blocco(:), parametro(:), valore(:), ...
    'VariableNames', {'Blocco','Parametro','Valore'});

fprintf('\n=== "%s" in %s: %d occorrenze ===\n\n', testo, modello, height(T));
for k = 1:height(T)
    fprintf('  %-34s  %-22s  %s\n', T.Blocco{k}, T.Parametro{k}, T.Valore{k});
end
fprintf('\n');

end