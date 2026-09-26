%% ispeziona_giunti.m - com'e' attuato ogni giunto, e cosa gli e' attaccato
%
% PERCHE'
%   Per C3 i 18 giunti devono passare da comando in POSIZIONE a comando in
%   COPPIA. Cambiare la modalita' cambia le porte del blocco, quindi rompe i
%   collegamenti esistenti: prima di scrivere crea_modello_mpc bisogna sapere
%   con precisione come sono messi adesso.
%
% COME LI TROVA (v2, 24/9)
%   La prima versione cercava BlockType 'Revolute Joint' e, in ripiego, i
%   blocchi con "Revolute" nel nome: zero risultati. In Simscape Multibody i
%   giunti sono sottosistemi mascherati, e in questo modello hanno i nomi dei
%   giunti del robot, non quelli di libreria.
%   Quindi non si cerca per nome ne' per tipo: si cerca per PARAMETRO. Un
%   blocco che possiede un parametro di attuazione del moto E' un giunto,
%   comunque si chiami. Il nome esatto del parametro lo dice il blocco.
%
% COSA NON FA
%   Nessun set_param, nessun save_system: solo lettura. Si puo' lanciare con
%   il modello aperto e anche mentre il collega ci lavora.
%
% USO
%   clear all; bdclose all; startup_phantomx
%   ispeziona_giunti
%
% Progetto FSR PhantomX - A. Russo

ig_mdl = 'phantomx_sim_zero';
if ~bdIsLoaded(ig_mdl), load_system(ig_mdl); end

ig_tutti = find_system(ig_mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                       'Type','block');
fprintf('\nBlocchi nel modello: %d\n', numel(ig_tutti));

%% ---- 1. chi ha un parametro che somiglia a un'attuazione ----
ig_b = {};  ig_par = {};
for ig_i = 1:numel(ig_tutti)
    try
        ig_nomi = fieldnames(get_param(ig_tutti{ig_i}, 'ObjectParameters'));
    catch
        continue
    end
    ig_att = ig_nomi(contains(ig_nomi, 'ActuationMotion'));
    if ~isempty(ig_att)
        ig_b{end+1}   = ig_tutti{ig_i};                        %#ok<SAGROW>
        ig_par{end+1} = ig_nomi;                               %#ok<SAGROW>
    end
end
fprintf('Blocchi con un parametro *ActuationMotion*: %d\n', numel(ig_b));

if isempty(ig_b)
    % Ripiego: mostra i tipi di maschera presenti, cosi' si vede cosa c'e'
    fprintf(2, '\nNessuno. Ecco i MaskType presenti nel modello:\n');
    ig_mt = {};
    for ig_i = 1:numel(ig_tutti)
        try, ig_m = get_param(ig_tutti{ig_i},'MaskType'); catch, ig_m = ''; end
        if ~isempty(ig_m), ig_mt{end+1} = ig_m; end            %#ok<SAGROW>
    end
    [ig_u, ~, ig_j] = unique(ig_mt);
    for ig_k = 1:numel(ig_u)
        fprintf('  %4d x  %s\n', sum(ig_j == ig_k), ig_u{ig_k});
    end
    return
end

%% ---- 2. i parametri interessanti del primo giunto, con i valori ----
fprintf('\n--- primo giunto: %s ---\n', regexprep(get_param(ig_b{1},'Name'),'\s+',' '));
ig_chiave = ig_par{1}(contains(ig_par{1}, {'Actuation','Sense','Torque','Motion'}));
for ig_k = 1:numel(ig_chiave)
    try
        fprintf('  %-34s = %s\n', ig_chiave{ig_k}, get_param(ig_b{1}, ig_chiave{ig_k}));
    catch
        fprintf('  %-34s = (non leggibile come stringa)\n', ig_chiave{ig_k});
    end
end

fprintf('\n--- valori ammessi (enum) per i parametri di attuazione ---\n');
ig_obj = get_param(ig_b{1}, 'ObjectParameters');
for ig_k = 1:numel(ig_chiave)
    ig_v = ig_obj.(ig_chiave{ig_k});
    if isfield(ig_v,'Enum') && ~isempty(ig_v.Enum)
        fprintf('  %-34s : %s\n', ig_chiave{ig_k}, strjoin(ig_v.Enum(:).', ' | '));
    end
end

%% ---- 3. tutti i giunti, uno per riga ----
ig_nMot = ig_par{1}(contains(ig_par{1}, 'ActuationMotion'));
ig_nTau = ig_par{1}(contains(ig_par{1}, 'ActuationTorque'));

fprintf('\n--- i giunti, uno per riga (%d) ---\n', numel(ig_b));
fprintf('  %-28s %-16s %-16s %s\n', 'blocco', 'motion', 'torque', 'porte');
for ig_i = 1:numel(ig_b)
    ig_m = '';  ig_t = '';
    if ~isempty(ig_nMot), ig_m = get_param(ig_b{ig_i}, ig_nMot{1}); end
    if ~isempty(ig_nTau), ig_t = get_param(ig_b{ig_i}, ig_nTau{1}); end
    ig_ph = get_param(ig_b{ig_i}, 'PortHandles');
    fprintf('  %-28s %-16s %-16s in %d out %d L %d R %d\n', ...
        regexprep(get_param(ig_b{ig_i},'Name'),'\s+',' '), ig_m, ig_t, ...
        numel(ig_ph.Inport), numel(ig_ph.Outport), ...
        numel(ig_ph.LConn), numel(ig_ph.RConn));
end

%% ---- 4. cosa alimenta l'ingresso del primo giunto ----
fprintf('\n--- a monte dell''ingresso del primo giunto ---\n');
ig_ph = get_param(ig_b{1}, 'PortHandles');
if isempty(ig_ph.Inport)
    fprintf('  nessuna porta di ingresso di segnale.\n');
else
    ig_h = get_param(get_param(ig_ph.Inport(1),'Line'), 'SrcBlockHandle');
    for ig_liv = 1:5
        if ig_h < 0, break; end
        fprintf('  %d. %-30s %s\n', ig_liv, ...
                regexprep(get_param(ig_h,'Name'),'\s+',' '), get_param(ig_h,'BlockType'));
        if strcmp(get_param(ig_h,'BlockType'),'From')
            ig_g = find_system(ig_mdl,'LookUnderMasks','all','FollowLinks','on', ...
                               'BlockType','Goto','GotoTag',get_param(ig_h,'GotoTag'));
            fprintf('     tag %s\n', get_param(ig_h,'GotoTag'));
            if isempty(ig_g), break; end
            ig_h = get_param(ig_g{1},'Handle');
            continue
        end
        ig_pp = get_param(ig_h,'PortHandles');
        if isempty(ig_pp.Inport), break; end
        ig_h = get_param(get_param(ig_pp.Inport(1),'Line'), 'SrcBlockHandle');
    end
end

fprintf('\n  Solo lettura: il modello non e'' stato modificato ne'' salvato.\n\n');
