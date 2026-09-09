function misura_quota(varargin)
%MISURA_QUOTA  Misura la quota di equilibrio del corpo invece di calcolarla.
%
%   misura_quota            parte da 0.25, andatura ferma, 3 s
%   misura_quota 0.30 5     parte da 0.30, 5 s
%
% PERCHE'
%   La quota corretta del corpo dipende da una catena di grandezze che
%   abbiamo ricostruito leggendo il modello pezzo per pezzo: z0, raggio della
%   sfera, quota dell'anca, faccia del pavimento, lunghezza della tibia. Su
%   quest'ultima siamo gia' passati da 0.12 a 0.16 a 0.152971, e resta un
%   dubbio: la traslazione del piede vale [0.03 -0.15 0], e i 3 cm potrebbero
%   essere fuori dal piano della gamba.
%
%   Invece di continuare a sommare numeri, misuriamo. Si parte da una quota
%   sicuramente alta, il robot cade, atterra e si assesta. La quota a cui si
%   ferma E' la quota giusta, per costruzione, qualunque sia la geometria che
%   la produce.
%
% COSA FA
%   - forza l'andatura ferma (S = 0, H = 0): niente moto, solo appoggio
%   - alza body_z0 e simula
%   - legge la posizione verticale del corpo dal log di Simscape
%   - riporta la quota di equilibrio e cosa mettere in cfg
%
%   Non modifica il modello ne' i file: tocca solo variabili del workspace,
%   e alla fine rilancia init_gait per rimettere tutto a posto.
%
% PRIMA
%   startup_phantomx
%   init_gait
%
% Progetto FSR PhantomX

z_start = 0.25;
t_stop  = 3;
if numel(varargin) >= 1, z_start = str2double(varargin{1}); end
if numel(varargin) >= 2, t_stop  = str2double(varargin{2}); end

mdl = 'phantomx_sim_zero';
if ~bdIsLoaded(mdl), load_system(mdl); end
cfg = phantomx_config();

fprintf('\n================ MISURA QUOTA ================\n');
fprintf('  partenza %.3f m   durata %.1f s   andatura ferma\n\n', z_start, t_stop);

%% ---- prepara il workspace ----
if ~evalin('base','exist(''gait'',''var'')')
    fprintf(2,'  gait non esiste: lancia init_gait prima di questo script.\n\n');
    return
end
assignin('base','gait_backup',  evalin('base','gait'));
assignin('base','bz0_backup',   evalin('base','body_z0'));

% L'azzeramento va chiesto a init_gait, non fatto qui: init_gait e' l'InitFcn
% del modello e riscriverebbe gait all'avvio della simulazione.
assignin('base','FORZA_STATICO', true);
assignin('base','body_z0', z_start);

%% ---- log di Simscape ----
logPrima = get_param(mdl,'SimscapeLogType');
logNome  = get_param(mdl,'SimscapeLogName');
set_param(mdl,'SimscapeLogType','all');

ripristina = onCleanup(@() chiudi(mdl, logPrima));

%% ---- simula ----
fprintf('  simulo...\n');
try
    out = sim(mdl, 'StopTime', num2str(t_stop));
catch ME
    fprintf(2,'\n  La simulazione si e'' fermata: %s\n', ME.message);
    fprintf(2,'  Se il messaggio parla di stato non finito, il robot e''\n');
    fprintf(2,'  caduto nel vuoto anche partendo da %.2f m: allora non e''\n', z_start);
    fprintf(2,'  una questione di quota. Dimmelo e cambiamo strada.\n\n');
    return
end

%% ---- estrai la quota del corpo ----
if isprop(out,logNome) || isfield(out,logNome)
    simlog = out.(logNome);
else
    fprintf(2,'  Il log di Simscape (%s) non e'' nell''uscita. Nomi presenti:\n', logNome);
    disp(out);
    return
end

nodi = cerca(simlog, 'Pz', {});
if isempty(nodi)
    fprintf(2,['  Non trovo la primitiva Pz nel log. Il giunto del corpo\n' ...
               '  potrebbe chiamare diversamente l''asse verticale.\n' ...
               '  Guarda l''albero con:  simlog.print\n\n']);
    return
end

% se ce n'e' piu' di uno, prendo quello con l'escursione maggiore: e' il corpo
z = []; t = [];
for k = 1:numel(nodi)
    try
        s  = nodi{k}.p.series;
        zk = s.values('m');  tk = s.time;
    catch
        continue
    end
    if isempty(z) || (max(zk) - min(zk)) > (max(z) - min(z))
        z = zk; t = tk;
    end
end
if isempty(z)
    fprintf(2,'  Nodo Pz trovato ma senza serie leggibile.\n\n');
    return
end

%% ---- risultato ----
% media sull'ultimo 20% della simulazione: e' l'equilibrio
i1 = find(t >= 0.8*t(end), 1);
z_eq  = mean(z(i1:end));
z_osc = max(z(i1:end)) - min(z(i1:end));

fprintf('\n--- RISULTATO ---\n');
fprintf('  quota iniziale      %.6f m\n', z(1));
fprintf('  quota di equilibrio %.6f m   (media ultimo 20%%)\n', z_eq);
fprintf('  oscillazione residua %.3f mm\n', 1000*z_osc);

if z_eq < 0.02
    fprintf(2,'\n  Il corpo e'' finito quasi a terra: non si e'' appoggiato,\n');
    fprintf(2,'  e'' collassato. Il problema non e'' la quota di partenza.\n\n');
    return
end
if z_osc > 2e-3
    fprintf(2,'\n  Oscillazione residua %.1f mm: la misura NON e'' valida.\n', 1000*z_osc);
    fprintf(2,'  Non do un verdetto, perche'' sarebbe falso.\n\n');
    fprintf(2,'  Causa quasi certa: le zampe si stanno muovendo. Questo script\n');
    fprintf(2,'  chiede la posa ferma impostando FORZA_STATICO nel workspace, ma\n');
    fprintf(2,'  init_gait deve saperlo gestire. Verifica che init_gait contenga,\n');
    fprintf(2,'  prima delle stampe di controllo:\n\n');
    fprintf(2,'      if exist(''FORZA_STATICO'',''var'') && FORZA_STATICO\n');
    fprintf(2,'          gait.S = 0;  gait.H = 0;\n');
    fprintf(2,'      end\n\n');
    fprintf(2,'  Se durante la simulazione non hai visto la riga\n');
    fprintf(2,'  "*** PROVA STATICA ***", la patch manca.\n');
    fprintf(2,'  Se invece le zampe erano gia'' ferme, allora oscilla davvero:\n');
    fprintf(2,'  rilancia piu'' lungo,  misura_quota %.2f %d\n\n', z_start, ceil(2*t_stop));
    return
end

fprintf('\n--- CONFRONTO CON IL CALCOLO ---\n');
z_calc  = cfg.body_z0_geom;                          % formula geometrica di cfg
pen     = cfg.mass*cfg.g / cfg.contact.k;            % compenetrazione elastica
z_atteso = z_calc + pen;
residuo  = z_eq - z_atteso;

fprintf('  geometrica (cfg.body_z0_geom)   %.6f m\n', z_calc);
fprintf('  + compenetrazione mg/k          %.6f m   (%.2f mm)\n', pen, 1000*pen);
fprintf('  --------------------------------------------\n');
fprintf('  equilibrio atteso               %.6f m\n', z_atteso);
fprintf('  equilibrio misurato             %.6f m\n', z_eq);
fprintf('  RESIDUO                         %+.2f mm\n', 1000*residuo);

fprintf('\n--- VERDETTO ---\n');
if abs(residuo) < 1e-3
    fprintf('  PASSA. La geometria di cfg corrisponde al modello entro 1 mm.\n');
    fprintf('  Lascia cfg.body_z0 = cfg.body_z0_geom + margine: la formula\n');
    fprintf('  segue da sola z0, foot_r, il pavimento e la lunghezza della zampa.\n');
else
    fprintf(2,'  NON PASSA: %.1f mm di scarto non spiegato.\n', 1000*residuo);
    fprintf(2,'  Qualcosa in cfg non descrive il modello. Sospetti, in ordine:\n');
    fprintf(2,'    - lo spessore/posizione del pavimento (%.0f mm)\n', 1000*cfg.floor_dim(3));
    fprintf(2,'    - il raggio della sfera di contatto (%.0f mm)\n', 1000*cfg.contact.foot_r);
    fprintf(2,'    - la lunghezza efficace della tibia\n');
    fprintf(2,'  NON aggiustare cfg.body_z0 a mano con il valore misurato:\n');
    fprintf(2,'  nasconderebbe il disallineamento invece di risolverlo.\n');
end

fprintf('\n==============================================\n\n');

end

%% ================================================================
function nodi = cerca(nodo, nome, nodi)
try
    ids = nodo.childIds;
catch
    return
end
for k = 1:numel(ids)
    try
        c = nodo.(ids{k});
    catch
        continue
    end
    if strcmp(ids{k}, nome), nodi{end+1} = c; end             %#ok<AGROW>
    nodi = cerca(c, nome, nodi);
end
end

function chiudi(mdl, logPrima)
try, set_param(mdl,'SimscapeLogType',logPrima); catch, end
try
    evalin('base','gait = gait_backup; body_z0 = bz0_backup;');
    evalin('base','clear gait_backup bz0_backup FORZA_STATICO');
    fprintf('  (workspace ripristinato: gait, body_z0, FORZA_STATICO)\n\n');
catch
end
end