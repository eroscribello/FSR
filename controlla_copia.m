%% controlla_copia.m - cosa e' cambiato davvero fra il modello base e la copia
%
% PERCHE'
%   Dopo crea_modello_mpc la copia ha dato "degenerate mass distribution" sul
%   6-DOF Joint e il robot e' sparito al primo istante. E' il sintomo di una
%   catena cinematica interrotta: se un solido o una zampa si scollegano, il
%   solutore si trova masse che non puo' risolvere.
%   Il sospetto e' il ricablaggio: cm_porta_coppia, non trovando una porta di
%   nome 't', ripiegava su "l'unica porta fisica libera" - che puo' benissimo
%   essere una porta di FRAME (B o F).
%
% COSA FA
%   Per ogni giunto rotoidale, in ENTRAMBI i modelli, elenca a cosa e'
%   attaccata ciascuna porta fisica. Poi confronta: se nella copia una porta
%   che prima andava a un solido adesso va a un convertitore, il colpevole e'
%   li'.
%
%   Solo lettura su tutti e due i modelli. Nessun salvataggio.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   controlla_copia
%
% Progetto FSR PhantomX - A. Russo

cc_base = 'phantomx_sim_zero';
cc_cop  = 'phantomx_sim_mpc';

for m = {cc_base, cc_cop}
    if ~bdIsLoaded(m{1}), load_system(m{1}); end
end

cc_B = cc_mappa(cc_base);
cc_C = cc_mappa(cc_cop);

fprintf('\n=========== PORTE FISICHE DEI GIUNTI ===========\n');
fprintf('  giunti nel base: %d, nella copia: %d\n\n', numel(cc_B), numel(cc_C));

cc_diff = 0;
for k = 1:numel(cc_B)
    nome = cc_B(k).nome;
    j = find(strcmp({cc_C.nome}, nome), 1);
    if isempty(j)
        fprintf(2, '  %-14s MANCA nella copia\n', nome);
        cc_diff = cc_diff + 1;
        continue
    end
    if ~isequal(cc_B(k).vicini, cc_C(j).vicini)
        cc_diff = cc_diff + 1;
        fprintf('  %-14s\n', nome);
        fprintf('     base  : %s\n', strjoin(cc_B(k).vicini, ' | '));
        fprintf('     copia : %s\n', strjoin(cc_C(j).vicini, ' | '));
    end
end

if cc_diff == 0
    fprintf('  Nessuna differenza nelle connessioni dei giunti.\n');
    fprintf('  Allora il problema non e'' il ricablaggio: guarda il 6-DOF Joint\n');
    fprintf('  e le inerzie (applica_inerzie e'' stato lanciato sulla COPIA?).\n');
else
    fprintf('\n  %d giunti con connessioni diverse.\n', cc_diff);
end

%% ---- il 6-DOF Joint, che e' quello che si lamenta ----
fprintf('\n--- 6-DOF Joint ---\n');
for m = {cc_base, cc_cop}
    b = find_system(m{1}, 'LookUnderMasks','all', 'FollowLinks','on', ...
                    'MaskType','6-DOF Joint');
    if isempty(b), fprintf('  %-22s: non trovato\n', m{1}); continue; end
    fprintf('  %-22s: %s\n', m{1}, strjoin(cc_attaccati(b{1}), ' | '));
end

%% ---- le inerzie: la copia le ha? ----
fprintf('\n--- solidi con massa, per modello ---\n');
for m = {cc_base, cc_cop}
    n = 0;  mtot = 0;
    bb = find_system(m{1}, 'LookUnderMasks','all', 'FollowLinks','on', 'Type','block');
    for i = 1:numel(bb)
        try
            mm = str2double(get_param(bb{i}, 'Mass'));
        catch
            continue
        end
        if mm > 0, n = n + 1;  mtot = mtot + mm; end
    end
    fprintf('  %-22s: %d blocchi con massa, totale %.4f kg\n', m{1}, n, mtot);
end

fprintf('\n  Solo lettura: nessun modello e'' stato modificato.\n\n');

%% ================= helper =================
function S = cc_mappa(mdl)
%CC_MAPPA  Per ogni giunto rotoidale, i blocchi attaccati alle porte fisiche.
S = struct('nome', {}, 'vicini', {});
g = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                'MaskType','Revolute Joint');
for k = 1:numel(g)
    S(k).nome   = regexprep(get_param(g{k},'Name'), '\s+', ' ');          %#ok<AGROW>
    S(k).vicini = cc_attaccati(g{k});                                     %#ok<AGROW>
end
end

function v = cc_attaccati(blocco)
%CC_ATTACCATI  I nomi dei blocchi collegati alle porte fisiche, in ordine.
v = {};
h = get_param(blocco, 'Handle');
ph = get_param(blocco, 'PortHandles');
porte = [ph.LConn(:); ph.RConn(:)];
for k = 1:numel(porte)
    l = get_param(porte(k), 'Line');
    if l < 0
        v{end+1} = sprintf('p%d:LIBERA', k);                              %#ok<AGROW>
        continue
    end
    capi = [get_param(l,'SrcBlockHandle'); get_param(l,'DstBlockHandle')];
    altro = capi(capi > 0 & capi ~= h);
    if isempty(altro)
        v{end+1} = sprintf('p%d:penzolante', k);                          %#ok<AGROW>
    else
        v{end+1} = sprintf('p%d:%s', k, ...
            regexprep(get_param(altro(1),'Name'), '\s+', ' '));           %#ok<AGROW>
    end
end
end
