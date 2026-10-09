function info = commenta_carico(mdl, stato)
% Toglie (o rimette) il carico dal modello di C3, in memoria.
%
%   commenta_carico                         % phantomx_sim_attitude, carico tolto
%   commenta_carico(mdl, 'off')             % lo rimette
%   info = commenta_carico(...)

if nargin < 1 || isempty(mdl),   mdl   = 'phantomx_sim_attitude'; end
if nargin < 2 || isempty(stato), stato = 'on'; end
if ~bdIsLoaded(mdl), load_system(mdl); end

blocchi = {'Brick Solid', 'Brick Solid1', '6-DOF Joint1', ['Spatial' newline 'Contact Force']};
% Catena del Transform Sensor sul pacco (aggiunta dopo il merge): se resta
% attiva col cubo commentato, Rigid Transform7 rimane un telaio senza massa
% agganciato al corpo dal solo sensore e Simscape lo chiude con un 6-DOF
% implicito a massa nulla ("degenerate mass distribution"). Va spenta insieme
% al carico. Facoltativa: nei modelli senza sensore questi blocchi non ci sono.
extra = {['Rigid' newline 'Transform7'], ['Transform' newline 'Sensor1'], ['PS-Simulink' newline 'Converter81']};

info = struct('mdl', mdl, 'stato', stato, 'blocchi', {strrep(blocchi, newline, ' ')});
for k = 1:numel(blocchi)
    b = [mdl '/' blocchi{k}];
    if getSimulinkBlockHandle(b) == -1
        error('commenta_carico:blocco', ...
            ['Il blocco "%s" non esiste in %s.\nIl carico potrebbe essere stato ' ...
             'rinominato: va ritrovato prima di misurare, non ignorato.'], ...
            strrep(blocchi{k}, newline, ' '), mdl);
    end
    set_param(b, 'Commented', stato);
end
for k = 1:numel(extra)
    b = [mdl '/' extra{k}];
    if getSimulinkBlockHandle(b) ~= -1
        set_param(b, 'Commented', stato);
        info.blocchi{end+1} = strrep(extra{k}, newline, ' ');
    end
end

if strcmp(stato, 'on')
    fprintf('  [commenta_carico] carico tolto da %s (%d blocchi, in memoria)\n', mdl, numel(info.blocchi));
else
    fprintf(2, '  [commenta_carico] carico RIMESSO in %s\n', mdl);
end
end
