%% applica_disturbo.m
%
%   T7: spinta laterale impulsiva mentre il robot cammina.
%
%   applica_disturbo(mdl, impulso)              % [N*s] lungo +y
%   applica_disturbo(mdl, impulso, 'applica', struct('t0',4,'durata',0.05))
%
% Progetto FSR PhantomX - A. Russo

function info = applica_disturbo(mdl, impulso, modo, opt)

if nargin < 1 || isempty(mdl),     mdl = 'phantomx_sim_zero'; end
if nargin < 2 || isempty(impulso), impulso = 0;      end
if nargin < 3 || isempty(modo),    modo = 'applica'; end
if nargin < 4, opt = struct(); end
if ~isfield(opt,'t0'),     opt.t0     = 4.0;   end   % [s] quando arriva la spinta
if ~isfield(opt,'durata'), opt.durata = 0.05;  end   % [s] larghezza dell'impulso
if ~isfield(opt,'asse'),   opt.asse   = 'y';   end

if ~bdIsLoaded(mdl), load_system(mdl); end

ad_6dof = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                      'MaskType','6-DOF Joint');
if numel(ad_6dof) ~= 1
    error('applica_disturbo:sixdof', '6-DOF Joint trovati: %d.', numel(ad_6dof));
end
ad_6dof = ad_6dof{1};

%% ================= modalita' 'controlla' =================
if strcmpi(modo, 'controlla')
    fprintf('\n=========== COSA C''E'', PRIMA DI TOCCARE ===========\n');
    fprintf('  modello   : %s\n', mdl);
    fprintf('  6-DOF Joint: %s\n', get_param(ad_6dof,'Name'));
    fprintf('  padre      : %s\n\n', get_param(ad_6dof,'Parent'));

    ph = get_param(ad_6dof, 'PortHandles');
    fprintf('  porte fisiche (serve sapere QUALE e'' il follower, cioe'' il corpo):\n');
    ad_stampa_porte(ad_6dof, 'LConn', ph.LConn);
    ad_stampa_porte(ad_6dof, 'RConn', ph.RConn);

    % il blocco della forza, esaminato in un modello usa-e-getta
    fprintf('\n  parametri di External Force and Torque:\n');
    ad_tmp = ['ad_scratch_' char(java.util.UUID.randomUUID.toString.replace('-','_'))];
    ad_tmp = ad_tmp(1:20);
    try
        new_system(ad_tmp);
        add_block('sm_lib/Forces and Torques/External Force and Torque', ...
                  [ad_tmp '/F']);
        n = fieldnames(get_param([ad_tmp '/F'], 'DialogParameters'));
        for k = 1:numel(n)
            v = '';
            try v = get_param([ad_tmp '/F'], n{k}); end %#ok<TRYNC>
            if ~ischar(v), v = '(non testo)'; end
            fprintf('    %-28s = %s\n', n{k}, v);
        end
        ph2 = get_param([ad_tmp '/F'], 'PortHandles');
        fprintf('    porte: %d Inport, %d LConn, %d RConn\n', ...
                numel(ph2.Inport), numel(ph2.LConn), numel(ph2.RConn));
        bdclose(ad_tmp);
    catch ME
        try bdclose(ad_tmp); end %#ok<TRYNC>
        fprintf(2, '    non leggibile: %s\n', ME.message);
        fprintf(2, '    (la libreria Simscape Multibody si chiama sm_lib: se il\n');
        fprintf(2, '     percorso e'' cambiato, cercalo con  lb = libinfo(...))\n');
    end

    fprintf('\n  Niente e'' stato modificato. Mandami questo output.\n\n');
    info = struct('modo','controlla', 'sixdof', ad_6dof);
    return
end

%% ================= modalita' 'azzera' =================
ad_pad  = get_param(ad_6dof, 'Parent');
ad_forz = [ad_pad '/dist_forza'];
ad_conv = [ad_pad '/dist_conv'];
ad_sorg = [ad_pad '/dist_sorgente'];

if strcmpi(modo, 'azzera')
    for b = {ad_sorg, ad_conv, ad_forz}
        if getSimulinkBlockHandle(b{1}) >= 0, delete_block(b{1}); end
    end
    fprintf('  disturbo rimosso (nessun salvataggio).\n');
    info = struct('modo','azzera');
    return
end

%% ================= modalita' 'applica' =================
% La serie temporale: un impulso rettangolare di area = impulso.
F = impulso / opt.durata;                       % [N]
t = [0; opt.t0-1e-4; opt.t0; opt.t0+opt.durata; opt.t0+opt.durata+1e-4; 1e4];
u = [0;           0;     F;                  F;                      0;   0];
assignin('base', 'dist_ts', timeseries(u, t));

fprintf('\nDisturbo: %.4f N*s lungo %s a t = %.2f s (%.1f N per %.0f ms)\n', ...
        impulso, opt.asse, opt.t0, F, 1e3*opt.durata);

if getSimulinkBlockHandle(ad_forz) >= 0
    info = struct('modo','applica', 'impulso', impulso, 'F', F, 'opt', opt, ...
                  'nuovo', false);
    return
end

%% ---- il blocco di forza ----
ad_pos = get_param(ad_6dof, 'Position');
ad_pos = ad_pos + [0 160 0 160];

add_block('sm_lib/Forces and Torques/External Force and Torque', ad_forz, ...
          'Position', ad_pos);

switch lower(opt.asse)
    case 'x', ad_par = 'EnableForceX';
    case 'y', ad_par = 'EnableForceY';
    case 'z', ad_par = 'EnableForceZ';
    otherwise, error('applica_disturbo:asse', 'Asse ''%s'' non previsto.', opt.asse);
end
set_param(ad_forz, ad_par, 'on');
ad_ris = '(non impostato)';
for c = {'World', 'Base', 'AttachedFrame'}
    try
        set_param(ad_forz, 'ForceResolutionFrame', c{1});
        ad_ris = c{1};
        break
    catch
        continue
    end
end
fprintf('  forza risolta nel frame: %s\n', ad_ris);

%% ---- il convertitore ----
ad_conv_src = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                          'MaskType','Simulink-PS Converter');
if isempty(ad_conv_src)
    ad_conv_src = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', ...
                              'BlockType','SubSystem', 'Name','Simulink-PS Converter');
end
if isempty(ad_conv_src)
    error('applica_disturbo:conv', ...
          'Nessun Simulink-PS Converter da copiare in %s.', mdl);
end
add_block(ad_conv_src{1}, ad_conv, 'Position', ad_pos - [180 0 180 0]);
try, set_param(ad_conv, 'Unit', 'N'); catch
    fprintf(2, '  Unit del convertitore non impostabile a N: controllare.\n');
end

%% ---- la sorgente ----
add_block('simulink/Sources/From Workspace', ad_sorg, ...
          'VariableName', 'dist_ts', 'Interpolate', 'on', ...
          'OutputAfterFinalValue', 'Holding final value', ...
          'Position', ad_pos - [360 0 360 0]);

%% ---- i collegamenti ----
add_line(ad_pad, 'dist_sorgente/1', 'dist_conv/1', 'autorouting','on');

ad_pc = get_param(ad_conv, 'PortHandles');
ad_pf = get_param(ad_forz, 'PortHandles');
ad_ps_out = [ad_pc.RConn(:); ad_pc.LConn(:)];         % uscita PS del convertitore
ad_in_f   = ad_pf.LConn(:);                            % ingresso PS della forza
if isempty(ad_in_f)
    error('applica_disturbo:portaForza', ...
        ['Abilitando %s non e'' comparsa nessuna porta di segnale fisico sul\n' ...
         'blocco di forza. Rilancia ''controlla'' e guarda il conteggio porte.'], ad_par);
end
add_line(ad_pad, ad_ps_out(1), ad_in_f(1), 'autorouting','on');

ad_p6 = get_param(ad_6dof, 'PortHandles');
try
    add_line(ad_pad, ad_p6.RConn(1), ad_pf.RConn(1), 'autorouting','on');
catch ME
    error('applica_disturbo:frame', ...
        ['Non sono riuscito a diramare dal nodo del corpo: %s\n' ...
         'Alternativa: collegare alla porta di base_link invece che a quella\n' ...
         'del giunto. Mandami l''errore invece di provare a mano.'], ME.message);
end

fprintf('  aggiunti dist_sorgente -> dist_conv -> dist_forza -> corpo.\n');
fprintf('  NESSUN salvataggio: bdclose riporta %s com''era.\n', mdl);

info = struct('modo','applica', 'impulso', impulso, 'F', F, 'opt', opt, ...
              'nuovo', true, 'frame_forza', ad_ris);
end

%% ================= helper =================
function ad_stampa_porte(blocco, nome, porte)
h = get_param(blocco, 'Handle');
for k = 1:numel(porte)
    l = get_param(porte(k), 'Line');
    if l < 0
        fprintf('    %s(%d): LIBERA\n', nome, k);
        continue
    end
    capi  = [get_param(l,'SrcBlockHandle'); get_param(l,'DstBlockHandle')];
    altro = capi(capi > 0 & capi ~= h);
    if isempty(altro)
        fprintf('    %s(%d): penzolante\n', nome, k);
    else
        fprintf('    %s(%d): -> %s   [%s]\n', nome, k, ...
            regexprep(get_param(altro(1),'Name'),'\s+',' '), ...
            ad_tipo(altro(1)));
    end
end
end

function s = ad_tipo(h)
s = '';
try, s = get_param(h, 'MaskType'); end %#ok<TRYNC>
if isempty(s)
    try, s = get_param(h, 'BlockType'); end %#ok<TRYNC>
end
end
