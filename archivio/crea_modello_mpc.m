function info = crea_modello_mpc(modo, nome_nuovo)
%CREA_MODELLO_MPC  La copia del modello con i giunti comandati in COPPIA.
%
%   crea_modello_mpc                 CONTROLLA e basta: non scrive niente
%   crea_modello_mpc('crea')         genera phantomx_sim_mpc.slx
%   crea_modello_mpc('rigenera')     lo cancella e lo rifa' da zero
%
% PERCHE' UNA COPIA E NON UNA VARIANTE
%   C1 e C2 comandano i giunti in posizione (MotionActuationMode = InputMotion),
%   C3 li comanda in coppia (TorqueActuationMode = InputTorque). Sono due
%   impianti diversi e non possono convivere nello stesso .slx senza una
%   modifica pesante al modello del collega. La copia la genera questo script:
%   se il collega cambia il modello base, si rilancia con 'rigenera'.
%
% COSA FA, IN ORDINE (l'ordine e' il punto)
%   1. mappa QUALE convertitore Simulink-PS alimenta QUALE giunto.
%      Va fatto PRIMA di toccare l'attuazione: cambiando modalita' la porta
%      del giunto passa da q (posizione) a t (coppia), Simulink lascia la
%      linea penzolante e quella corrispondenza non e' piu' leggibile.
%   2. porta i 18 giunti a coppia in ingresso;
%   3. porta i 18 convertitori da Unit = 1 a N*m. In Simscape il RADIANTE E'
%      ADIMENSIONALE, per questo oggi Unit = 1 alimenta una porta di
%      posizione senza problemi; la coppia non lo e' e il modello non
%      compilerebbe, con un errore che sembra venire da tutt'altro;
%   4. ricollega ogni convertitore alla porta di coppia del suo giunto;
%   5. mette la posa d'appoggio negli State Target dei 18 giunti. Senza,
%      con i giunti in coppia nessuno impone piu' la posizione: il robot
%      parte a zampe dritte e si accascia comunque, e S0 fallisce per il
%      motivo sbagliato;
%   6. stacca il Mux dell'IK dal Goto ref_joints e ci mette un Constant con
%      il vettore tau_S0. Da li' in poi i 18 canali esistenti trasportano
%      coppie invece di angoli, senza altro cablaggio.
%
% NIENTE VALORI INDOVINATI
%   I nomi dei parametri li chiede al blocco e li sceglie per contenuto. Se
%   un nome non lo trova, o ne trova due, si FERMA e stampa cosa ha visto:
%   meglio un errore leggibile che un modello modificato a meta'.
%
% COSA NON FA MAI
%   Non tocca phantomx_sim_zero: lo apre, ne salva una COPIA, e lavora sulla
%   copia. Il modello del collega resta identico.
%
% DOPO
%   tau_S0 = tau_statico(false);   % nel base workspace, prima di simulare
%   applica_terreno('T1', false, 'phantomx_sim_mpc');
%   applica_inerzie('phantomx_sim_mpc');
%   sim('phantomx_sim_mpc','StopTime','2');
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(modo),       modo = 'controlla'; end
if nargin < 2 || isempty(nome_nuovo), nome_nuovo = 'phantomx_sim_mpc'; end
cm_base   = 'phantomx_sim_zero';
cm_scrivi = any(strcmpi(modo, {'crea','rigenera'}));
cm_forza  = strcmpi(modo, 'rigenera');

if bdIsLoaded(cm_base) && strcmp(get_param(cm_base,'Dirty'),'on')
    error('crea_modello_mpc:dirty', ['%s e'' aperto con modifiche non salvate. ' ...
        'Salvale o chiudilo (bdclose all) prima: la copia deve nascere da uno ' ...
        'stato noto.'], cm_base);
end

%% ---- la copia ----
cm_mdl = cm_base;
if cm_scrivi
    cm_dest = fullfile(fileparts(which([cm_base '.slx'])), [nome_nuovo '.slx']);
    if isfile(cm_dest)
        if ~cm_forza
            error('crea_modello_mpc:esiste', ['%s esiste gia''. Usa ' ...
                'crea_modello_mpc(''rigenera'') per rifarlo da zero.'], cm_dest);
        end
        if bdIsLoaded(nome_nuovo), close_system(nome_nuovo, 0); end
        delete(cm_dest);
        fprintf('Copia precedente cancellata.\n');
    end
    load_system(cm_base);
    save_system(cm_base, cm_dest);
    close_system(cm_base, 0);
    load_system(cm_dest);
    cm_mdl = nome_nuovo;
    fprintf('Copia creata: %s\n', cm_dest);
else
    load_system(cm_base);
    fprintf('MODO CONTROLLA: leggo %s e non scrivo niente.\n', cm_base);
end

%% ---- 1. la mappa giunto -> convertitore, finche' e' leggibile ----
cm_g = find_system(cm_mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                   'MaskType','Revolute Joint');
if numel(cm_g) ~= 18
    error('crea_modello_mpc:giunti', 'Trovati %d giunti rotoidali, ne attendevo 18.', numel(cm_g));
end

cm_conv = cell(18,1);
for cm_i = 1:18
    cm_v = cm_vicini(cm_g{cm_i}, 'Simulink-PS');
    if numel(cm_v) ~= 1
        error('crea_modello_mpc:mappa', ['Il giunto %s ha %d convertitori ' ...
            'collegati, ne attendevo 1. La mappa non e'' affidabile: fermo qui.'], ...
            regexprep(get_param(cm_g{cm_i},'Name'),'\s+',' '), numel(cm_v));
    end
    cm_conv{cm_i} = cm_v{1};
end
fprintf('\nMappa giunto -> convertitore: 18 coppie, tutte univoche.\n');

%% ---- i valori dei parametri, chiesti al blocco ----
cm_obj  = get_param(cm_g{1}, 'ObjectParameters');
cm_vMot = cm_scegli(cm_obj, 'MotionActuationMode', 'Computed');
cm_vTau = cm_scegli(cm_obj, 'TorqueActuationMode', 'Input');

% I nomi dei parametri di State Target. [MISURATO 24/9] In questa versione
% sono PositionTargetSpecify / PositionTargetValue / PositionTargetValueUnits;
% accanto a Value esistono anche PositionTargetValue_conf e ...Units, per
% questo la ricerca per sola sottostringa non bastava. Si prova prima il nome
% esatto e solo in mancanza si torna allo schema: cosi' funziona anche se una
% versione diversa li chiama in un altro modo.
cm_pSpec = cm_nome_par(cm_obj, {'PositionTargetSpecify'}, ...
                       {'Position','Specify'}, {'Velocity'});
cm_pVal  = cm_nome_par(cm_obj, {'PositionTargetValue'}, ...
                       {'Position','Value'},   {'Velocity','_conf','Units'});
cm_pUni  = cm_nome_par(cm_obj, {'PositionTargetValueUnits','PositionTargetUnits'}, ...
                       {'Position','Unit'},    {'Velocity','_conf'});

fprintf('\n--- parametri ---\n');
fprintf('  MotionActuationMode : %-16s ->  %s\n', get_param(cm_g{1},'MotionActuationMode'), cm_vMot);
fprintf('  TorqueActuationMode : %-16s ->  %s\n', get_param(cm_g{1},'TorqueActuationMode'), cm_vTau);
fprintf('  state target        : %s / %s / %s\n', cm_pSpec, cm_pVal, cm_pUni);
fprintf('  unita'' convertitore : %s\n', get_param(cm_conv{1}, 'Unit'));

%% ---- la posa d'appoggio, per tipo di giunto ----
% Da inv_kyn in posa nominale: coxa 0, femore -10.05 gradi, tibia -2.19.
% Uguali per tutte e sei le zampe: la simmetria e' gia' dentro inv_kyn.
cfg = phantomx_config();
[cm_q1, cm_q2, cm_q3] = inv_kyn(0, 0, cfg.z0, cfg.side(1), cfg.alpha(1));
cm_posa = struct('c1', cm_q1, 'thigh', cm_q2, 'tibia', cm_q3);
fprintf('\n  posa iniziale [deg]: coxa %.2f, femore %.2f, tibia %.2f\n', ...
        rad2deg(cm_q1), rad2deg(cm_q2), rad2deg(cm_q3));

info = struct('modello', cm_mdl, 'giunti', 18, 'motion', cm_vMot, ...
              'torque', cm_vTau, 'posa', cm_posa, 'scritto', cm_scrivi);

if ~cm_scrivi
    fprintf(['\n  Niente e'' stato modificato. Se i valori tornano:\n' ...
             '    crea_modello_mpc(''crea'')\n\n']);
    return
end

%% ---- 2-3. attuazione e unita' ----
for cm_i = 1:18
    set_param(cm_g{cm_i}, 'TorqueActuationMode', cm_vTau);
    set_param(cm_g{cm_i}, 'MotionActuationMode', cm_vMot);

    cm_u = get_param(cm_conv{cm_i}, 'Unit');
    if ~any(strcmpi(cm_u, {'1','rad','unitless'}))
        error('crea_modello_mpc:unita', ['Il convertitore %s ha Unit = %s, ' ...
            'inatteso. Guardalo a mano.'], cm_conv{cm_i}, cm_u);
    end
    set_param(cm_conv{cm_i}, 'Unit', 'N*m');
end
fprintf('\n  18 giunti in coppia, 18 convertitori a N*m.\n');

%% ---- 4. ricollegamento alla porta di coppia ----
cm_rif = 0;
for cm_i = 1:18
    cm_pt = cm_porta_coppia(cm_g{cm_i});
    cm_po = get_param(cm_conv{cm_i}, 'PortHandles');
    cm_so = [cm_po.Outport(:); cm_po.RConn(:); cm_po.LConn(:)];
    if isempty(cm_so)
        error('crea_modello_mpc:convSenzaUscita', 'Il convertitore %s non ha uscite.', cm_conv{cm_i});
    end
    % via la linea penzolante rimasta dal collegamento vecchio
    cm_l = get_param(cm_so(1), 'Line');
    if cm_l >= 0, delete_line(cm_l); end
    add_line(get_param(cm_g{cm_i},'Parent'), cm_so(1), cm_pt, 'autorouting','on');
    cm_rif = cm_rif + 1;
end
fprintf('  %d convertitori ricollegati alla porta di coppia.\n', cm_rif);

%% ---- 5. posa iniziale ----
for cm_i = 1:18
    cm_nome = regexprep(get_param(cm_g{cm_i},'Name'), '\s+', ' ');
    if     contains(cm_nome, 'c1'),    cm_val = cm_q1;
    elseif contains(cm_nome, 'thigh'), cm_val = cm_q2;
    elseif contains(cm_nome, 'tibia'), cm_val = cm_q3;
    else
        error('crea_modello_mpc:nomeGiunto', ['Il giunto %s non ha c1, thigh ' ...
            'o tibia nel nome: non so che posa dargli.'], cm_nome);
    end
    set_param(cm_g{cm_i}, cm_pSpec, 'on');
    set_param(cm_g{cm_i}, cm_pVal,  num2str(cm_val, 10));
    set_param(cm_g{cm_i}, cm_pUni,  'rad');
end
fprintf('  posa d''appoggio scritta negli State Target (in rad).\n');

%% ---- 6. il comando: Constant tau_S0 al posto del Mux dell'IK ----
cm_goto = find_system(cm_mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                      'BlockType','Goto', 'GotoTag','ref_joints');
if numel(cm_goto) ~= 1
    error('crea_modello_mpc:refJoints', 'Goto ref_joints trovati: %d (ne attendevo 1).', numel(cm_goto));
end
cm_ph = get_param(cm_goto{1}, 'PortHandles');
cm_li = get_param(cm_ph.Inport(1), 'Line');
if cm_li >= 0, delete_line(cm_li); end

cm_pos  = get_param(cm_goto{1}, 'Position');
cm_padre = get_param(cm_goto{1}, 'Parent');
cm_cost = [cm_padre '/tau_S0'];
if getSimulinkBlockHandle(cm_cost) >= 0, delete_block(cm_cost); end
add_block('simulink/Sources/Constant', cm_cost, ...
          'Value', 'tau_S0', ...
          'Position', [cm_pos(1)-140, cm_pos(2)-10, cm_pos(1)-60, cm_pos(2)+30]);
add_line(cm_padre, 'tau_S0/1', [get_param(cm_goto{1},'Name') '/1'], 'autorouting','on');
fprintf('  Constant tau_S0 collegato a ref_joints (il Mux dell''IK e'' scollegato).\n');

%% ---- verifica finale ----
for cm_i = 1:18
    if ~strcmp(get_param(cm_g{cm_i},'TorqueActuationMode'), cm_vTau) || ...
       ~strcmp(get_param(cm_g{cm_i},'MotionActuationMode'), cm_vMot) || ...
       ~strcmp(get_param(cm_conv{cm_i},'Unit'), 'N*m') || ...
       ~strcmpi(get_param(cm_g{cm_i}, cm_pSpec), 'on') || ...
       abs(str2double(get_param(cm_g{cm_i}, cm_pVal)) - cm_atteso(cm_g{cm_i}, cm_q1, cm_q2, cm_q3)) > 1e-9
        error('crea_modello_mpc:nonApplicato', ...
              'Il giunto %s non ha accettato tutte le modifiche.', cm_g{cm_i});
    end
end

save_system(cm_mdl);
fprintf('\n  salvato. phantomx_sim_zero non e'' stato toccato.\n');
fprintf(['\n  PRIMA DI SIMULARE:  tau_S0 = tau_statico(false);\n' ...
         '  senza quella variabile nel workspace il Constant non compila.\n\n']);
end

%% ================= helper =================
function v = cm_atteso(giunto, q1, q2, q3)
%CM_ATTESO  La posa che quel giunto deve avere, dal suo nome.
nome = regexprep(get_param(giunto,'Name'), '\s+', ' ');
if     contains(nome,'c1'),    v = q1;
elseif contains(nome,'thigh'), v = q2;
else,                          v = q3;
end
end

function v = cm_scegli(obj, campo, chiave)
%CM_SCEGLI  Il valore ammesso che contiene 'chiave'. Niente valori cablati.
if ~isfield(obj, campo)
    error('crea_modello_mpc:campo', 'Il giunto non ha il parametro %s.', campo);
end
e = obj.(campo).Enum;
sel = find(contains(e, chiave, 'IgnoreCase', true));
if numel(sel) ~= 1
    error('crea_modello_mpc:enum', ['Per %s i valori che contengono "%s" sono %d ' ...
        '(%s).'], campo, chiave, numel(sel), strjoin(e, ', '));
end
v = e{sel};
end

function nome = cm_nome_par(obj, esatti, deve, nonDeve)
%CM_NOME_PAR  Il nome del parametro: prima per nome esatto, poi per schema.
n = fieldnames(obj);
for k = 1:numel(esatti)
    if any(strcmp(n, esatti{k}))
        nome = esatti{k};  return
    end
end
nome = cm_trova(obj, deve, nonDeve);
end

function nome = cm_trova(obj, deve, nonDeve)
%CM_TROVA  Il parametro il cui nome contiene tutte le parole di 'deve' e
%   nessuna di 'nonDeve'. Se non e' esattamente uno, si ferma e li elenca:
%   i nomi dei parametri di Simscape cambiano fra versioni e non vanno
%   scritti a memoria.
n = fieldnames(obj);
ok = true(size(n));
for k = 1:numel(deve),    ok = ok & contains(n, deve{k});     end
for k = 1:numel(nonDeve), ok = ok & ~contains(n, nonDeve{k}); end
cand = n(ok);
if numel(cand) ~= 1
    error('crea_modello_mpc:param', ['Parametri che contengono {%s} e non {%s}: ' ...
        '%d -> %s'], strjoin(deve,','), strjoin(nonDeve,','), numel(cand), ...
        strjoin(cand.', ', '));
end
nome = cand{1};
end

function p = cm_porta_coppia(giunto)
%CM_PORTA_COPPIA  La porta fisica di ingresso coppia ('t') del giunto.
ph = get_param(giunto, 'PortHandles');
tutte = [ph.LConn(:); ph.RConn(:)];
nomi  = cell(size(tutte));
for k = 1:numel(tutte)
    try, nomi{k} = get_param(tutte(k), 'Name'); catch, nomi{k} = ''; end
end
sel = find(strcmpi(nomi, 't'));
if numel(sel) == 1
    p = tutte(sel);  return
end
% ripiego: l'unica porta fisica rimasta libera dopo il cambio di modalita'
libere = [];
for k = 1:numel(tutte)
    if get_param(tutte(k), 'Line') < 0, libere(end+1) = k; end            %#ok<AGROW>
end
if numel(libere) == 1
    p = tutte(libere);  return
end
error('crea_modello_mpc:portaT', ['Non riesco a identificare la porta di coppia ' ...
    'di %s. Porte fisiche: %s; libere: %d.'], giunto, strjoin(nomi.', ','), numel(libere));
end

function v = cm_vicini(blocco, tipo)
%CM_VICINI  I blocchi attaccati al giunto la cui maschera contiene 'tipo'.
%
% [CORRETTO 24/9] La prima versione guardava solo PortHandles.Inport e
% trovava zero convertitori: in Simscape l'ingresso di attuazione di un
% giunto e' una PORTA FISICA (LConn/RConn), non una Inport Simulink. E le
% connessioni fisiche non hanno un verso, quindi non basta SrcBlockHandle:
% si guardano entrambi i capi della linea e si scarta il giunto stesso.
v = {};
h_blocco = get_param(blocco, 'Handle');
ph = get_param(blocco, 'PortHandles');
porte = [ph.Inport(:); ph.LConn(:); ph.RConn(:)];
for k = 1:numel(porte)
    l = get_param(porte(k), 'Line');
    if l < 0, continue; end
    capi = [get_param(l,'SrcBlockHandle'); get_param(l,'DstBlockHandle')];
    for c = capi(:).'
        if c < 0 || c == h_blocco, continue; end
        try, mt = get_param(c, 'MaskType'); catch, mt = ''; end
        if contains(mt, tipo)
            v{end+1} = getfullname(c);                              %#ok<AGROW>
        end
    end
end
v = unique(v);
end
