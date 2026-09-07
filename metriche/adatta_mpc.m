function run = adatta_mpc(ws, p, meta)
%ADATTA_MPC  Converte l'output di MAIN.m nella struttura normalizzata run.
%
%   run = adatta_mpc                    % legge dal base workspace
%   run = adatta_mpc(ws)                % ws = struttura con i campi di MAIN
%   run = adatta_mpc(ws, p, meta)
%
% USO TIPICO
%   MAIN
%   run = adatta_mpc();
%   run.meta.controller = 'C3';
%   run.meta.task = 'T1';
%   riga = metriche(run)
%
% COSA NON C'E' E PERCHE'
%   q, qd, tau restano vuoti: il modello a corpo rigido singolo non ha
%   giunti, quindi la famiglia C (sforzo di attuazione) non e' misurabile
%   qui. Lo sara' quando l'MPC girera' in Simscape.
%
%   Lo scivolamento (famiglia D) risulta praticamente nullo, ma non perche'
%   il controllore sia perfetto: nell'SRB il piede in appoggio e' vincolato
%   per costruzione, non c'e' un modello di attrito che possa cedere. E' un
%   numero da NON riportare come risultato.
%
% Progetto FSR PhantomX - A. Russo

%% ---------- raccolta variabili ----------
if nargin < 1 || isempty(ws)
    ws = struct();
    for v = {'tout','Xout','Uout','Xdout','Udout','FSMout','Uext', ...
             'SimTimeDuration','dt_sim','gait'}
        if evalin('base', sprintf('exist(''%s'',''var'')', v{1}))
            ws.(v{1}) = evalin('base', v{1});
        end
    end
end
if nargin < 2 || isempty(p)
    if evalin('base','exist(''p'',''var'')'), p = evalin('base','p'); else, p = []; end
end
if nargin < 3, meta = struct(); end

for v = {'tout','Xout','Uout'}
    if ~isfield(ws,v{1}) || isempty(ws.(v{1}))
        error('adatta_mpc:manca', ...
            ['Manca %s nel workspace. Hai lanciato MAIN in questa sessione?\n' ...
             'Se MAIN gira dentro una funzione, passa le variabili a mano:\n' ...
             '   run = adatta_mpc(struct(''tout'',tout,''Xout'',Xout,''Uout'',Uout));'], v{1});
    end
end

t    = ws.tout(:);
Xout = ws.Xout;
N    = numel(t);
assert(size(Xout,1)==N, 'adatta_mpc:lunghezza', ...
    'tout ha %d campioni, Xout ne ha %d.', N, size(Xout,1));
assert(size(Xout,2)==36, 'adatta_mpc:statoX', ...
    ['Xout ha %d colonne, ne servono 36:\n' ...
     '  1:3 posizione, 4:6 velocita'', 7:15 vec(R), 16:18 omega, 19:36 piedi.\n' ...
     'Con 30 colonne stai guardando la versione quadrupede.'], size(Xout,2));

%% ---------- struttura ----------
run = run_vuoto(0);

run.t   = t;
run.p   = Xout(:,1:3);
run.v   = Xout(:,4:6);
run.w   = Xout(:,16:18);
run.pf  = Xout(:,19:36);          % frame mondo, ordine CAN
run.Fc  = ws.Uout;                % forze di contatto, frame mondo

%% ---------- assetto ----------
% R e' memorizzata come vec(R), cioe' per colonne:
%   col 7  8  9 10 11 12 13 14 15
%       R11 R21 R31 R12 R22 R32 R13 R23 R33
% Convenzione ZYX: yaw attorno a z, poi pitch attorno a y, poi roll attorno a x.
R11 = Xout(:, 7);  R21 = Xout(:, 8);  R31 = Xout(:, 9);
R32 = Xout(:,12);  R33 = Xout(:,15);

roll  = atan2(R32, R33);
pitch = atan2(-R31, hypot(R32, R33));
yaw   = atan2(R21, R11);
run.rpy = [roll, pitch, yaw];

% controllo di sanita': R deve essere ortonormale
n1 = vecnorm(Xout(:,7:9), 2, 2);
if max(abs(n1 - 1)) > 1e-3
    warning('adatta_mpc:rotazione', ...
        ['Le colonne di R non sono unitarie (scarto massimo %.2e).\n' ...
         'O l''integrazione sta derivando, o le colonne 7:15 non sono vec(R).'], ...
        max(abs(n1-1)));
end

%% ---------- contatto ----------
% due nozioni distinte, servono entrambe alla famiglia D:
%   contact       = contatto EFFETTIVO, dedotto dalla forza normale
%   contact_sched = contatto SCHEDULATO dalla macchina a stati dell'andatura
Fz = ws.Uout(:, 3:3:18);
run.contact = Fz > 0.5;                       % [N]

if isfield(ws,'FSMout') && ~isempty(ws.FSMout)
    F = ws.FSMout;
    % La convenzione di fcn_FSM non e' documentata: la deduciamo confrontando
    % ciascuna delle due ipotesi con il contatto effettivo e tenendo quella
    % che concorda di piu'. Se concordano entrambe poco, lo diciamo.
    ip1 = (F == 1);
    ip2 = (F ~= 1);
    a1 = mean(ip1(:) == run.contact(:));
    a2 = mean(ip2(:) == run.contact(:));
    if max(a1,a2) < 0.7
        warning('adatta_mpc:fsm', ...
            ['FSMout concorda con il contatto effettivo solo al %.0f%%.\n' ...
             'La convenzione stance/swing potrebbe essere diversa da quella\n' ...
             'ipotizzata: verificala in fcn_FSM prima di fidarti dei distacchi.'], ...
             100*max(a1,a2));
    end
    if a1 >= a2
        run.contact_sched = ip1;
    else
        run.contact_sched = ip2;
    end
end

%% ---------- riferimenti, se presenti ----------
if isfield(ws,'Xdout') && ~isempty(ws.Xdout)
    run.rif.p = ws.Xdout(:,1:3);
    run.rif.v = ws.Xdout(:,4:6);
end

%% ---------- metadati ----------
run.meta.controller = 'C3';
run.meta.task       = 'T1';
run.meta.run        = 1;
run.meta.condizione = 'nominale';
run.meta.impianto   = 'SRB';
run.meta.note       = 'famiglia C non misurabile: il modello SRB non ha giunti';

if ~isempty(p)
    if isfield(p,'vel_d'), run.meta.vel_d = p.vel_d(:).'; end
    if isfield(p,'yaw_d'), run.meta.yaw_d = p.yaw_d;      end
    if isfield(p,'gait'),  run.meta.gait  = p.gait;       end
end

f = fieldnames(meta);
for k = 1:numel(f), run.meta.(f{k}) = meta.(f{k}); end

%% ---------- riepilogo ----------
fprintf('\n[adatta_mpc] %d campioni, da %.2f a %.2f s\n', N, t(1), t(end));
fprintf('  distanza percorsa   %.3f m\n', norm(run.p(end,1:2) - run.p(1,1:2)));
fprintf('  zampe a terra medie %.2f su 6\n', mean(sum(run.contact,2)));
fprintf('  Fz negative         %d campioni-zampa', nnz(Fz < -1e-6));
if nnz(Fz < -1e-6) > 0
    fprintf(2, '   <-- il QP sta usando forze di adesione (lb = -Fzd)\n');
else
    fprintf('\n');
end
fprintf('  campi assenti: q, qd, tau (attesi: SRB senza giunti)\n\n');

end