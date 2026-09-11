function verifica_ik(foot_urdf)
%VERIFICA_IK  Dove finisce davvero il piede rispetto a dove l'IK crede.
%
%   verifica_ik                  usa cfg.foot_xyz
%   verifica_ik([0 0.15 0])      usa l'offset indicato
%
% ATTENZIONE AI DUE FRAME
%   L'offset del piede ha due espressioni diverse, e non sono intercambiabili:
%
%     frame Simscape (i sei Rigid Transform del modello)   [0, -0.15, 0]
%     frame URDF     (questo controllo, robotModel)        [0, +0.15, 0]
%
%   Differiscono perche' j_tibia_*_OriginTransform contiene una rotazione di
%   pi attorno a z, che ribalta il segno di y. Con l'offset sbagliato la
%   cinematica diretta mette il piede SOPRA il corpo e ogni misura e' priva
%   di senso.
%
%   Per l'IK non cambia nulla: conta solo la lunghezza, lt = 0.15.
%
% COSA MISURA
%   La quota del piede lungo il passo, per una zampa angolare (FL) e una
%   centrale (ML). Se l'IK fosse esatta sarebbe costante. Non conta il valore
%   assoluto - un errore costante lo assorbe body_z0 - conta l'ESCURSIONE:
%   e' quella che rende i piedi non complanari e stacca le zampe da terra.
%
% PRIMA
%   startup_phantomx ; init_gait
%
% Progetto FSR PhantomX

cfg = phantomx_config();
if nargin < 1 || isempty(foot_urdf)
    foot_urdf = cfg.foot_xyz;
    fonte = 'cfg.foot_xyz';
else
    fonte = 'argomento';
end

if ~evalin('base', 'exist(''robotModel'',''var'')')
    fprintf(2,'\n  robotModel non c''e''. Lancia init_gait.\n\n');
    return
end
rbt = evalin('base','robotModel');

fprintf('\n============ VERIFICA IK ============\n');
fprintf('  offset del piede (frame URDF): [%+.4f %+.4f %+.4f]   da %s\n', ...
        foot_urdf, fonte);

prove = { 'FL', 1, 'tibia_lf'
          'ML', 3, 'tibia_lm' };

xs = linspace(-cfg.S/2, +cfg.S/2, 7);
Z  = nan(numel(xs), size(prove,1));

fprintf('\n--- QUOTA DEL PIEDE SOTTO L''ANCA ---\n');
fprintf('  comandata: %.4f m per ogni x\n\n', cfg.z0);
fprintf('%-6s', 'x[mm]');
for p = 1:size(prove,1), fprintf(' %14s', prove{p,1}); end
fprintf('\n');

for k = 1:numel(xs)
    fprintf('%-6.0f', 1000*xs(k));
    for p = 1:size(prove,1)
        i = prove{p,2};
        [th, ph, ps] = inv_kyn(xs(k), 0, cfg.z0, cfg.side(i), cfg.alpha(i));
        pf = fk(rbt, prove{p,3}, prove{p,1}, [th ph ps], foot_urdf);
        if any(isnan(pf))
            Z(k,p) = NaN;  fprintf(' %14s','?');
        else
            % profondita' del piede sotto l'anca della sua zampa
            Z(k,p) = cfg.p_hip(3,i) - pf(3);
            fprintf(' %14.5f', Z(k,p));
        end
    end
    fprintf('\n');
end

%% ---- verdetto ----
fprintf('\n--- ESCURSIONE (quella che conta) ---\n');
pen = cfg.mass*cfg.g/(3*cfg.contact.k);
peggio = 0;
for p = 1:size(prove,1)
    e = max(Z(:,p)) - min(Z(:,p));
    peggio = max(peggio, e);
    fprintf('  %-4s  media %.5f m   escursione %6.2f mm\n', ...
            prove{p,1}, mean(Z(:,p)), 1000*e);
end
d = Z(:,1) - Z(:,2);
fprintf('  scarto fra le due zampe: escursione %.2f mm\n', 1000*(max(d)-min(d)));
fprintf('  penetrazione disponibile: %.2f mm\n', 1000*pen);

fprintf('\n--- VERDETTO ---\n');
if peggio < 0.3*pen && (max(d)-min(d)) < 0.3*pen
    fprintf('  PASSA. I sei piedi restano complanari lungo tutto il passo:\n');
    fprintf('  il tripode e'' possibile. Un eventuale errore costante sulla\n');
    fprintf('  quota media lo assorbe body_z0.\n\n');
else
    fprintf(2,'  NON PASSA: %.2f mm di escursione contro %.2f di penetrazione.\n', ...
            1000*peggio, 1000*pen);
    fprintf(2,'  I piedi non possono toccare insieme.\n\n');
end
end

%% ================================================================
function p = fk(rbt, corpo, zampa, q, foot)
p = nan(3,1);
u = lower(fliplr(zampa));
g = {['j_c1_' u], ['j_thigh_' u], ['j_tibia_' u]};
try
    conf = homeConfiguration(rbt);
    if isstruct(conf)
        for j = 1:3
            t = find(strcmp({conf.JointName}, g{j}),1);
            if isempty(t), return; end
            conf(t).JointPosition = q(j);
        end
    else
        nomi = {};
        for b = 1:numel(rbt.Bodies)
            if ~strcmp(rbt.Bodies{b}.Joint.Type,'fixed')
                nomi{end+1} = rbt.Bodies{b}.Joint.Name; %#ok<AGROW>
            end
        end
        for j = 1:3
            t = find(strcmp(nomi, g{j}),1);
            if isempty(t), return; end
            conf(t) = q(j);
        end
    end
    T = getTransform(rbt, conf, corpo, rbt.BaseName);
    v = T * [foot(:); 1];
    p = v(1:3);
catch
end
end