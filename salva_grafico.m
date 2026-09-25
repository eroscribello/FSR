function percorso = salva_grafico(tag, fig)
%SALVA_GRAFICO  Salva una figura in grafici/<tag>.fig e grafici/<tag>.png.
%
%   salva_grafico('T4_C1_8deg')        salva la figura corrente
%   salva_grafico(tag, fig)            salva quella figura
%   p = salva_grafico(tag)             restituisce il percorso del .fig
%
% PERCHE' IL .FIG E NON SOLO IL .PNG
%   Il .fig conserva dati, assi, legende e titoli come oggetti: quando scrivi
%   la relazione lo riapri e cambi testi e formato senza rilanciare la
%   simulazione. Il .png serve solo a ritrovare al volo la figura giusta.
%   Le serie temporali non si rigenerano: la struttura della run vive nel
%   workspace e sparisce chiudendo MATLAB. Le curve aggregate invece si
%   rifanno quando vuoi dai CSV in results/.
%
% RIMETTERE I TESTI IN INGLESE, DOPO
%   f  = openfig('grafici\T4_C1_8deg.fig');
%   ax = flipud(findobj(f, 'Type', 'axes'));    % dall'alto in basso
%   ax(1).Title.String  = 'Ramp climb - open-loop kinematic controller';
%   ax(1).YLabel.String = 'attitude [deg]';
%   ax(3).XLabel.String = 'time [s]';
%   savefig(f, 'grafici\relazione\T4_C1_8deg.fig');
%   exportgraphics(f, 'grafici\relazione\T4_C1_8deg.pdf');   % vettoriale
%   In relazione conviene il PDF: il testo resta testo e non sgrana.
%
% NOTE
%   Sovrascrive senza chiedere: lo stesso tag della stessa run va sostituito,
%   come per i CSV. grafici/ e' tracciata da git (results/**/*.fig no).
%
% Progetto FSR PhantomX - A. Russo

if nargin < 2 || isempty(fig)
    if isempty(findobj('Type', 'figure'))
        warning('salva_grafico:nessuna', 'Nessuna figura aperta: non salvo niente.');
        percorso = '';  return
    end
    fig = gcf;
end

tag = regexprep(char(tag), '[^\w\-\.]', '_');     % niente caratteri buoni per i nomi file
if ~isfolder('grafici'), mkdir('grafici'); end

percorso = fullfile('grafici', [tag '.fig']);
set(fig, 'Name', tag, 'NumberTitle', 'off');      % cosi' la finestra dice cos'e'
savefig(fig, percorso);

png = fullfile('grafici', [tag '.png']);
try
    exportgraphics(fig, png, 'Resolution', 200);  % R2020a in poi
catch
    print(fig, png, '-dpng', '-r200');            % ripiego
end

fprintf('  grafico  %s  (+ .png)\n', percorso);
end
