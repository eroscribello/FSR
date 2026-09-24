%% corpi_degeneri.m - quale corpo ha una distribuzione di massa che non regge
%
% PERCHE'
%   Liberando i 18 giunti (copia per C3) compare "degenerate mass distribution
%   on the follower side" del 6-DOF Joint, e il robot diverge al primo passo.
%   Sul modello base lo stesso messaggio NON compare: con i giunti comandati
%   in posizione quei gradi di liberta' sono eliminati e l'inerzia dei singoli
%   link non viene mai usata. Adesso viene usata, e un corpo con inerzia
%   degenere da' una matrice di massa singolare: accelerazioni infinite, cioe'
%   il robot che sparisce in un fotogramma invece di cadere.
%
% COSA CONTROLLA, PER OGNI CORPO CON MASSA
%   1. tipo di inerzia: una "Point Mass" non ha inerzia rotazionale, e con un
%      giunto rotoidale libero e' degenere per definizione;
%   2. momenti nulli o negativi;
%   3. disuguaglianza triangolare: per un corpo reale Ixx + Iyy >= Izz e le
%      permutazioni. Se non vale, quel tensore non corrisponde a nessun corpo
%      fisico - e' il controllo che avrebbe preso le inerzie dell'URDF molto
%      prima di noi;
%   4. tensore completo (con i prodotti) definito positivo.
%
% SOLO LETTURA. Nessun modello viene modificato o salvato.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   corpi_degeneri                      % sul modello base
%   corpi_degeneri('phantomx_sim_mpc')  % sulla copia
%
% Progetto FSR PhantomX - A. Russo

function corpi_degeneri(mdl)

if nargin < 1 || isempty(mdl), mdl = 'phantomx_sim_zero'; end
if ~bdIsLoaded(mdl), load_system(mdl); end

bb = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Type','block');

fprintf('\n=========== CORPI DI %s ===========\n', mdl);
fprintf('  %-26s %-22s %9s  %s\n', 'blocco', 'tipo inerzia', 'massa', 'momenti / verdetto');

n_tot = 0;  n_bad = 0;
for i = 1:numel(bb)
    try
        m = str2double(get_param(bb{i}, 'Mass'));
    catch
        continue
    end
    if ~(m > 0), continue; end
    n_tot = n_tot + 1;

    try, tipo = get_param(bb{i}, 'InertiaType'); catch, tipo = '(nessuno)'; end
    nome = regexprep(get_param(bb{i}, 'Name'), '\s+', ' ');
    padre = regexprep(get_param(get_param(bb{i},'Parent'), 'Name'), '\s+', ' ');
    etich = sprintf('%.14s/%.11s', padre, nome);

    I = [];  P = [0 0 0];
    try, I = str2num(get_param(bb{i}, 'MomentsOfInertia'));  end %#ok<ST2NM,TRYNC>
    try, P = str2num(get_param(bb{i}, 'ProductsOfInertia')); end %#ok<ST2NM,TRYNC>

    motivi = {};
    if contains(tipo, 'Point', 'IgnoreCase', true)
        motivi{end+1} = 'massa puntiforme: nessuna inerzia rotazionale';  %#ok<AGROW>
    end
    if ~isempty(I) && numel(I) == 3
        if any(I <= 0)
            motivi{end+1} = 'momento nullo o negativo';                   %#ok<AGROW>
        else
            if I(1)+I(2) < I(3) || I(2)+I(3) < I(1) || I(3)+I(1) < I(2)
                motivi{end+1} = 'disuguaglianza triangolare violata';     %#ok<AGROW>
            end
            if numel(P) == 3
                T = [I(1) -P(3) -P(2); -P(3) I(2) -P(1); -P(2) -P(1) I(3)];
                if min(eig(T)) <= 0
                    motivi{end+1} = 'tensore non definito positivo';      %#ok<AGROW>
                end
            end
        end
    elseif ~contains(tipo, 'Point', 'IgnoreCase', true)
        motivi{end+1} = 'momenti non leggibili';                          %#ok<AGROW>
    end

    if isempty(motivi)
        continue        % i corpi sani non si stampano: interessano gli altri
    end
    n_bad = n_bad + 1;
    if isempty(I)
        fprintf(2, '  %-26s %-22s %9.5f  %s\n', etich, tipo, m, strjoin(motivi, '; '));
    else
        fprintf(2, '  %-26s %-22s %9.5f  [%.3g %.3g %.3g]  %s\n', ...
                etich, tipo, m, I(1), I(2), I(3), strjoin(motivi, '; '));
    end
end

fprintf('\n  corpi con massa: %d,  problematici: %d\n', n_tot, n_bad);
if n_bad == 0
    fprintf(['  Nessun corpo degenere. Allora il messaggio del 6-DOF Joint non\n' ...
             '  viene da un''inerzia: guarda i corpi SENZA massa fra due giunti\n' ...
             '  liberi (un link massless e'' altrettanto singolare).\n']);
end

%% ---- i corpi senza massa, che con i giunti liberi contano ----
fprintf('\n--- blocchi solido SENZA massa (con giunti liberi diventano singolari) ---\n');
n_zero = 0;
for i = 1:numel(bb)
    try, mt = get_param(bb{i}, 'MaskType'); catch, continue; end
    if ~contains(mt, 'Solid'), continue; end
    m = NaN;
    try, m = str2double(get_param(bb{i}, 'Mass')); end %#ok<TRYNC>
    if isnan(m) || m > 0, continue; end
    n_zero = n_zero + 1;
    fprintf('  %-26s %s\n', ...
        regexprep(get_param(bb{i},'Name'),'\s+',' '), mt);
end
fprintf('  totale: %d\n\n', n_zero);
end
