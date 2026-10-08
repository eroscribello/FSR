function R = stato_pacco(out, task, ctrl, cfg, t_lim)
% Verifica se il pacco e' rimasto sul robot
%   script_T4;  stato_pacco(t4_out, 'T4')          % C3P, come prima
%   stato_pacco(t4_out, 'T4', 'C2P')               % un altro controllore
%
% IL CONTROLLORE VA PASSATO, NON DEDOTTO
%   Prima era scritto 'C3P' dentro la funzione: lanciarla dopo una run di
%   C2P scriveva una riga etichettata C3P e sovrascriveva quella vera.
%   Ora controllore e nomi dei file vengono dall'argomento.
%
% SI PUO' CHIAMARE ANCHE SE LO SCRIPT DEL TASK E' ANDATO IN ERRORE dopo la
% sim: serve solo l'uscita della simulazione, non le metriche. Quando il
% pacco cade, le metriche di posizione del robot non sono utilizzabili
% (run.p e' il baricentro dell'intero sistema, pacco compreso) ma questa
% misura resta valida, perche' legge il giunto fra vassoio e cubo.
%

if nargin < 3 || isempty(ctrl), ctrl = 'C3P'; end
if isstruct(ctrl)                       % vecchia firma: stato_pacco(out, task, cfg)
    if nargin < 4 || isempty(cfg), cfg = ctrl; end
    ctrl = 'C3P';
end
if nargin < 4 || isempty(cfg), cfg = phantomx_config(); end
if nargin < 5, t_lim = Inf; end
task = upper(char(task));
ctrl = upper(char(ctrl));

%% ---- il nodo del giunto nel log ----
lg = [];
for nome = {'simlog','simlog_phantomx_sim_attitude'}
    try lg = out.(nome{1}); break; catch, end
end
if isempty(lg)
    error('stato_pacco:log', 'Nessun log di Simscape nell''uscita della simulazione.');
end
ids = lg.childIds;
k = find(~cellfun(@isempty, regexp(ids, '6_DOF_Joint1$', 'once')), 1);
if isempty(k)
    error('stato_pacco:giunto', ['Nel log non c''e'' 6-DOF Joint1. Nodi di primo ' ...
        'livello: %s.\nIl carico e'' stato commentato, o il blocco rinominato.'], ...
        strjoin(ids, ', '));
end
g = lg.(ids{k});

P = [];
for c = {'Px','Py','Pz'}
    s = g.(c{1}).p.series;
    t = s.time;   t = t(:);        % s.time(:) e' letto come chiamata di metodo
    v = s.values('m');
    P = [P, v(:)];                                                %#ok<AGROW>
end

%% ---- spostamento dal riferimento ----
ref = t >= 1.5 & t <= 2.0;
if ~any(ref)
    error('stato_pacco:corta', 'La run e'' piu'' corta di 2 s: niente riferimento.');
end
P0 = median(P(ref,:), 1);
d  = vecnorm(P - P0, 2, 2);
% FINESTRA DI VALIDITA'
%   Gli script dei task troncano la run quando un piede raggiunge il bordo
%   del pavimento, perche' da li' in poi si misura la caduta dalla lastra e
%   non il controllore. La sim pero' arriva fino in fondo, e questa funzione
%   legge l'uscita grezza: senza t_lim il massimo dello spostamento diventa
%   il cubo che esce dal mondo insieme al robot. E' successo su T6, dove
%   l'ultimo campione dava 1138 mm contro i 20 mm di tutta la corsa.
%   Passare t_lim = t_bordo della riga corrispondente di results/.
reg = t >= 2*cfg.T & t <= t_lim;
if ~any(reg)
    error('stato_pacco:finestra', ...
        'Nessun campione fra %.2f s e t_lim = %.2f s.', 2*cfg.T, t_lim);
end

R = struct();
R.task        = string(task);
R.controller  = string(ctrl);
R.spost_max   = max(d(reg));
R.spost_fin   = d(find(reg, 1, 'last'));   % ultimo campione VALIDO, non della sim
[~, im]       = max(d .* reg);
R.t_max       = t(im);
R.dz_max      = max(abs(P(reg,3) - P0(3)));
if R.spost_max < 0.02,     R.esito = "fermo";
elseif R.spost_max < 0.05, R.esito = "scivola, resta sul vassoio";
else,                      R.esito = "al bordo o caduto";
end
R.t_lim = t_lim;
R.data = string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm'));

fprintf('\n=============== PACCO - %s, %s ===============\n', task, ctrl);
fprintf('  spostamento dal riferimento   max %.1f mm (t = %.2f s)   finale %.1f mm\n', ...
        1e3*R.spost_max, R.t_max, 1e3*R.spost_fin);
fprintf('  di cui in Pz                  max %.1f mm\n', 1e3*R.dz_max);
% Un massimo nell'ultimo ciclo d'andatura e' sospetto: o il pacco sta ancora
% scivolando quando la run finisce, o la run e' andata oltre il bordo del
% pavimento e quello che si misura e' una caduta.
if isfinite(R.t_max) && R.t_max > t(end) - cfg.T
    fprintf(2, ['  [attenzione] il massimo cade nell''ultimo ciclo (%.2f s su %.2f s).\n' ...
                '  Se la run e'' stata troncata al bordo del pavimento, rilancia con\n' ...
                '  t_lim = t_bordo:  stato_pacco(out, ''%s'', ''%s'', [], t_bordo)\n'], ...
            R.t_max, t(end), task, ctrl);
end
if R.esito == "al bordo o caduto"
    fprintf(2, '  -> %s: guardare l''animazione prima di dichiarare il task.\n', R.esito);
else
    fprintf('  -> %s\n', R.esito);
end

%% ---- CSV della serie ----
% Il riassunto da solo non dice se il pacco si assesta o se continua a
% scivolare: per quello serve l'andamento nel tempo, che finora veniva
% calcolato e buttato via. Una riga per campione, un file per task.
dest = fullfile('results', 'diagnostica');
if ~isfolder(dest), mkdir(dest); end
fs = fullfile(dest, sprintf('pacco_serie_%s_%s.csv', ctrl, R.task));
% La serie si salva INTERA, non troncata: serve anche a vedere che cosa
% succede fuori dalla finestra valida.
writetable(table(t, d, P(:,3) - P0(3), 'VariableNames', {'t','d','dz'}), fs);
fprintf('  scritto  %s\n', fs);

%% ---- CSV del riassunto ----
f = fullfile(dest, sprintf('pacco_%s.csv', ctrl));
T = struct2table(R);
if isfile(f)
    o = detectImportOptions(f, 'TextType', 'string');
    o = setvartype(o, {'task','controller','esito','data'}, 'string');
    V = readtable(f, o);
    % Il CSV sul disco puo' essere stato scritto da una versione precedente,
    % con meno colonne: le mancanti si aggiungono vuote invece di far fallire
    % il merge e perdere le righe degli altri task.
    manca = setdiff(T.Properties.VariableNames, V.Properties.VariableNames, 'stable');
    for k_ = 1:numel(manca)
        if isnumeric(T.(manca{k_}))
            V.(manca{k_}) = nan(height(V), 1);
        else
            V.(manca{k_}) = strings(height(V), 1);
        end
    end
    V = V(V.task ~= R.task, T.Properties.VariableNames);
    T = [V; T];
end
writetable(sortrows(T, 'task'), f);
fprintf('  scritto  %s\n\n', f);
end
