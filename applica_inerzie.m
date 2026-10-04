function info = applica_inerzie(mdl, verbose, modo)
% APPLICA_INERZIE  Mette le inerzie corrette nei 25 solidi del robot, in memoria.
%
%   applica_inerzie                      corregge phantomx_sim_zero
%   applica_inerzie(mdl)                 corregge un altro modello
%   info = applica_inerzie(mdl, true)    con riepilogo a schermo
%   applica_inerzie(mdl, true, 'controlla')   NON scrive: dice solo come stanno
%
%   Nel .slx le inerzie vengono dall'URDF e sono ~1000 volte
%   troppo grandi: il corpo (1 kg, 25 x 20 cm) ha 3-6 kg*m^2, come se la sua
%   massa stesse a 1.7 m dal centro; ogni link delle zampe (24 g, 6-15 cm) ha
%   5e-3 kg*m^2, come un'asta da mezzo metro. Le masse invece sono giuste.
%
%   Scrive i parametri dei blocchi IN MEMORIA, come applica_terreno: il file
%   .slx NON viene salvato e i valori originali restano li'. Per tornare
%   all'originale basta non chiamare questa funzione e ricaricare il modello
%
% DA DOVE VENGONO I VALORI
%   phantomx_config: cfg.I_body, I_c1, I_c2, I_thigh, I_tibia; prodotti
%   d'inerzia a zero. Per il corpo tornano con il conto a mano di un
%   parallelepipedo da 1 kg e 25 x 20 x 5 cm (I_zz ~ 8.6e-3 kg*m^2).

if nargin < 1 || isempty(mdl),     mdl = 'phantomx_sim_zero'; end
if nargin < 2 || isempty(verbose), verbose = false; end
if nargin < 3 || isempty(modo),    modo = 'correggi'; end

cfg = phantomx_config();
if ~bdIsLoaded(mdl), load_system(mdl); end

%% ---- i solidi del robot: corpo + 24 link ----
S = struct('blocco',{},'nome',{},'I_ora',{},'I_giusta',{});
bb = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Type','block');
for i = 1:numel(bb)
    try
        tipo = get_param(bb{i}, 'InertiaType');
        m    = str2double(get_param(bb{i}, 'Mass'));
    catch
        continue                       % il blocco non ha inerzia: non ci riguarda
    end
    if ~strcmp(tipo,'Custom') || ~(m > 0), continue; end
    padre = regexprep(get_param(get_param(bb{i},'Parent'),'Name'), '\s+', ' ');
    if     startsWith(padre,'MP_BODY'), In = cfg.I_body;
    elseif startsWith(padre,'c1_'),     In = cfg.I_c1;
    elseif startsWith(padre,'c2_'),     In = cfg.I_c2;
    elseif startsWith(padre,'thigh_'),  In = cfg.I_thigh;
    elseif startsWith(padre,'tibia_'),  In = cfg.I_tibia;
    else,  continue                    % pavimento, ostacoli, sfere: non si toccano
    end
    S(end+1) = struct('blocco', bb{i}, 'nome', padre, ...
                      'I_ora', get_param(bb{i},'MomentsOfInertia'), ...
                      'I_giusta', mat2str(In, 6));                    
end

if numel(S) ~= 25
    error('applica_inerzie:solidi', ...
        ['Trovati %d solidi del robot, ne attendevo 25 (corpo + 24 link).\n' ...
         'Il modello e'' cambiato: controlla prima di correggere.'], numel(S));
end

%% ---- quanti sono gia' a posto ----
gia = false(1, numel(S));
for k = 1:numel(S)
    a = str2num(S(k).I_ora);       %#ok<ST2NM>
    b = str2num(S(k).I_giusta);    %#ok<ST2NM>
    gia(k) = numel(a) == numel(b) && all(abs(a(:) - b(:)) <= 1e-12 + 1e-9*abs(b(:)));
end

info = struct('modello', mdl, 'solidi', numel(S), 'gia_corretti', sum(gia), ...
              'corretti', 0, 'modo', modo);

if strcmp(modo, 'controlla')
    if verbose || true
        fprintf('\n--- INERZIE (solo controllo): %d solidi, %d gia'' corretti ---\n', ...
                numel(S), sum(gia));
        fprintf('  %-12s %-34s %s\n', 'solido', 'nel modello', 'valore corretto');
        for k = [1 2 numel(S)]
            fprintf('  %-12s %-34s %s\n', S(k).nome, S(k).I_ora, S(k).I_giusta);
        end
    end
    return
end

%% ---- correzione ----
for k = 1:numel(S)
    set_param(S(k).blocco, 'MomentsOfInertia', S(k).I_giusta, ...
                           'ProductsOfInertia', '[0 0 0]');
end
info.corretti = numel(S);

if verbose
    fprintf('\n--- INERZIE CORRETTE: %d solidi ---\n', numel(S));
    for k = [1 2 numel(S)]
        fprintf('  %-12s %-34s -> %s\n', S(k).nome, S(k).I_ora, S(k).I_giusta);
    end
    fprintf('  (in memoria: il .slx NON e'' stato salvato)\n\n');
end
end
