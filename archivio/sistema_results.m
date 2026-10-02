%% sistema_results.m - riordina results/ una volta sola   [DA LANCIARE UNA VOLTA]
%
% PERCHE'
%   In results/ convivono cose di natura diversa: la campagna corrente, la
%   stessa campagna con le inerzie vecchie, le tarature del 17/9, le misure di
%   C3 sul simulatore SRB del 7/9 e i confronti A/B sulle inerzie. Tutti .csv,
%   tutti nella stessa cartella: non si capisce piu' quale numero sta dietro
%   quale conclusione.
%
% COSA FA (solo spostamenti, niente riscritture di dati)
%   results/
%     T2_C1.csv ... T6_C2.csv        campagna CORRENTE, quella che si rifa' ora
%     T4_limite/                     run per angolo della campagna corrente
%     controllo_inerzie.csv          verifica pre-campagna
%     diagnostica/                   prova_inerzie.csv, prova_inerzie_T6.csv
%     storico/
%       inerzie_URDF/                campagna completa con le inerzie sbagliate
%       taratura_T2_20260917/        griglia e scelte della taratura del 17/9
%       C3_srb_20260907/             misure di C3 sul simulatore SRB (task T2a)
%     LEGGIMI.md                     cosa e' cosa
%
% CANCELLA QUALCOSA?
%   Solo doppioni, e solo dopo aver verificato byte per byte che il file
%   identico esiste nella sua cartella nuova. Se due file con lo stesso nome
%   differiscono, li lascia tutti e due e lo dice. Niente viene buttato senza
%   una copia altrove.
%
% E' RIPETIBILE
%   Lanciarlo due volte non fa danni: quello che ha gia' spostato non c'e' piu'
%   al posto vecchio e viene saltato.
%
% USO
%   sistema_results
%
% Progetto FSR PhantomX - A. Russo

sr_base = 'results';
if ~isfolder(sr_base)
    error('sistema_results:results', 'Cartella %s non trovata: lancialo dalla radice del progetto.', sr_base);
end

sr_stor = fullfile(sr_base, 'storico');
sr_dest = struct( ...
    'diag',    fullfile(sr_base, 'diagnostica'), ...
    'vecchie', fullfile(sr_stor, 'inerzie_URDF'), ...
    'tar',     fullfile(sr_stor, 'taratura_T2_20260917'), ...
    'srb',     fullfile(sr_stor, 'C3_srb_20260907'));
for c = fieldnames(sr_dest)'
    if ~isfolder(sr_dest.(c{1})), mkdir(sr_dest.(c{1})); end
end

sr_log = {};   % {azione, da, a}

%% ---- 1. la vecchia campagna: results/inerzie_URDF -> results/storico/inerzie_URDF ----
sr_old = fullfile(sr_base, 'inerzie_URDF');
if isfolder(sr_old)
    sr_ff = dir(fullfile(sr_old, '*.csv'));
    for k = 1:numel(sr_ff)
        sr_log = sr_agg(sr_log, sr_muovi(fullfile(sr_old, sr_ff(k).name), ...
                                         fullfile(sr_dest.vecchie, sr_ff(k).name)));
    end
    if numel(dir(sr_old)) <= 2      % restano solo . e ..
        rmdir(sr_old);
    end
end

% le run per angolo di T4 limite: anche quelle verranno sovrascritte dalla
% campagna nuova, quindi la copia vecchia va messa al sicuro adesso
sr_lim = fullfile(sr_base, 'T4_limite');
if isfolder(sr_lim)
    sr_limV = fullfile(sr_dest.vecchie, 'T4_limite');
    if ~isfolder(sr_limV), mkdir(sr_limV); end
    sr_ff = dir(fullfile(sr_lim, '*.csv'));
    for k = 1:numel(sr_ff)
        sr_log = sr_agg(sr_log, sr_muovi(fullfile(sr_lim, sr_ff(k).name), ...
                                         fullfile(sr_limV, sr_ff(k).name)));
    end
end

%% ---- 2. tarature del 17/9 e misure SRB del 7/9 ----
sr_gruppi = { 'taratura_T2_C1_*.csv', sr_dest.tar
              'misure_2026*.csv',     sr_dest.srb
              'misure_2026*.mat',     sr_dest.srb
              'prova_inerzie*.csv',   sr_dest.diag };
for g = 1:size(sr_gruppi,1)
    sr_ff = dir(fullfile(sr_base, sr_gruppi{g,1}));
    for k = 1:numel(sr_ff)
        sr_log = sr_agg(sr_log, sr_muovi(fullfile(sr_base, sr_ff(k).name), ...
                                         fullfile(sr_gruppi{g,2}, sr_ff(k).name)));
    end
end

%% ---- 3. doppioni finiti per sbaglio nella cartella della campagna vecchia ----
% inerzie_URDF e' nata da copyfile('results\*.csv'), quindi si e' portata
% dentro anche tarature, misure SRB e prove A/B: non sono risultati di
% campagna e stanno gia' nelle cartelle giuste.
sr_fuori = [dir(fullfile(sr_dest.vecchie, 'taratura_T2_C1_*.csv')); ...
            dir(fullfile(sr_dest.vecchie, 'misure_2026*.csv'));     ...
            dir(fullfile(sr_dest.vecchie, 'prova_inerzie*.csv'))];
for k = 1:numel(sr_fuori)
    sr_qui = fullfile(sr_dest.vecchie, sr_fuori(k).name);
    if startsWith(sr_fuori(k).name, 'taratura'),     sr_la = fullfile(sr_dest.tar,  sr_fuori(k).name);
    elseif startsWith(sr_fuori(k).name, 'misure'),   sr_la = fullfile(sr_dest.srb,  sr_fuori(k).name);
    else,                                            sr_la = fullfile(sr_dest.diag, sr_fuori(k).name);
    end
    if isfile(sr_la) && sr_uguali(sr_qui, sr_la)
        delete(sr_qui);
        sr_log = sr_agg(sr_log, {'doppione tolto', sr_qui, sr_la});
    else
        sr_log = sr_agg(sr_log, {'LASCIATO (diverso)', sr_qui, sr_la});
    end
end

%% ---- resoconto ----
fprintf('\n=============== RESULTS RIORDINATO ===============\n');
if isempty(sr_log)
    fprintf('  niente da fare: era gia'' a posto.\n');
else
    for k = 1:size(sr_log,1)
        fprintf('  %-18s %s\n', sr_log{k,1}, sr_log{k,2});
    end
end
fprintf('\n  campagna corrente : %s\\T*_C*.csv  +  %s\\T4_limite\\\n', sr_base, sr_base);
fprintf('  storico           : %s\n', sr_stor);
fprintf('  diagnostica       : %s\n', sr_dest.diag);
fprintf('  leggi             : %s\\LEGGIMI.md\n\n', sr_base);
fprintf('  Nota: i CSV T2..T6 nella radice sono ANCORA quelli con le inerzie\n');
fprintf('  vecchie e verranno sovrascritti dalla campagna nuova. La copia di\n');
fprintf('  sicurezza e'' in storico\\inerzie_URDF.\n\n');

%% ================= helper =================
function L = sr_agg(L, riga)
if isempty(riga{1}), return; end      % il file non c'era: niente da annotare
L(end+1,:) = riga;
end

function riga = sr_muovi(da, a)
if ~isfile(da), riga = {'', '', ''}; return; end
if isfile(a) && sr_uguali(da, a)
    delete(da);
    riga = {'doppione tolto', da, a};
else
    if isfile(a)
        [p, n, e] = fileparts(a);
        a = fullfile(p, [n '_bis' e]);       % non si sovrascrive mai un risultato
    end
    movefile(da, a);
    riga = {'spostato', da, a};
end
end

function tf = sr_uguali(a, b)
fa = dir(a);  fb = dir(b);
tf = false;
if fa.bytes ~= fb.bytes, return; end
x = fileread(a);  y = fileread(b);
tf = strcmp(x, y);
end
