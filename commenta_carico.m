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

if strcmp(stato, 'on')
    fprintf('  [commenta_carico] carico tolto da %s (4 blocchi, in memoria)\n', mdl);
else
    fprintf(2, '  [commenta_carico] carico RIMESSO in %s\n', mdl);
end
end
