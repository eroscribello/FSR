function percorso = salva_grafico(tag, fig)
% Salva una figura in grafici/<tag>.fig e grafici/<tag>.png.
%
%   salva_grafico('T4_C1_8deg')        salva la figura corrente
%   salva_grafico(tag, fig)            salva quella figura
%   p = salva_grafico(tag)             restituisce il percorso del .fig

if nargin < 2 || isempty(fig)
    if isempty(findobj('Type', 'figure'))
        warning('salva_grafico:nessuna', 'Nessuna figura aperta: non salvo niente.');
        percorso = '';  return
    end
    fig = gcf;
end

tag = regexprep(char(tag), '[^\w\-\.]', '_');    
if ~isfolder('grafici'), mkdir('grafici'); end

percorso = fullfile('grafici', [tag '.fig']);
set(fig, 'Name', tag, 'NumberTitle', 'off');    
savefig(fig, percorso);

png = fullfile('grafici', [tag '.png']);
try
    exportgraphics(fig, png, 'Resolution', 200);  
catch
    print(fig, png, '-dpng', '-r200');            
end

fprintf('  grafico  %s  (+ .png)\n', percorso);
end
