function s_new = soglia_ottima(dati, verbose)
%SOGLIA_OTTIMA  La terna di soglie migliore, dai dati di una run gia' fatta.
%
%   [VS, dati] = valida_soglia;
%   s = soglia_ottima(dati)
%
% PERCHE' NON BASTA UN MOLTIPLICATORE
%   valida_soglia spazza un moltiplicatore della terna, cioe' muove le tre
%   soglie insieme. Ma dopo l'allineamento dello stimatore (25/9) il problema
%   e' di UN giunto solo: in volo il flag scatta per la coxa nel 39.7% dei
%   campioni, per il femore nel 10.3%, per la tibia nel 7.6%. Alzare tutte e
%   tre per sistemare la coxa peggiora il rilevamento sugli altri due - a 2x
%   l'errore in appoggio passa da 0.6% a 3.0%. Serve muoverle separatamente.
%
% COSA FA
%   1. Per ogni giunto sceglie la soglia che minimizza  falsi positivi in
%      volo + falsi negativi in appoggio, su una griglia log fra le due
%      distribuzioni.
%   2. Poi affina sulla funzione vera, che non e' per giunto: il flag e' un
%      OR sui tre, quindi tre ottimi singoli non fanno un ottimo. Tre
%      passate di discesa per coordinate sull'OR.
%   3. Confronta la terna nuova con quella in cfg, e conta i voli
%      attraversati senza che il flag torni mai basso: e' la condizione che
%      blocca il reset di z_ext in ricerca_terreno, quindi la conseguenza
%      meccanica, non solo statistica.
%
% IL LIMITE, COME SEMPRE
%   E' ricostruzione AD ANELLO APERTO: cambiando soglia cambia il flag, non
%   la traiettoria. Questa funzione produce un CANDIDATO. Vale solo dopo che
%   una run vera con quella terna lo conferma:
%       OVERRIDE_C2_SOGLIA = s;  init_gait;  [VS,dati] = valida_soglia
%   e poi spazzata_soglia per vedere se le metriche seguono.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 2 || isempty(verbose), verbose = true; end

D    = dati.D;
app  = dati.appoggio & dati.vero;     % appoggio VERO: il flag deve dire 1
vol  = dati.volo;                     % volo:          il flag deve dire 0
s_ora = reshape(dati.soglia, 1, []);

if ~any(app(:)) || ~any(vol(:))
    error('soglia_ottima:dati','Servono sia campioni in appoggio sia in volo.');
end

%% ---- 1. ottimo per giunto ----
nomi = {'coxa','femore','tibia'};
s_new = s_ora;
if verbose
    fprintf('\n---------------- SOGLIA OTTIMA, PER GIUNTO ----------------\n');
    fprintf('  %-8s %10s %10s %10s %10s\n', 'giunto', 'ora', 'ottima', 'falsi+', 'falsi-');
end
for j = 1:3
    Dj = D(:, j:3:18);
    a  = Dj(app);      % deve stare SOPRA la soglia
    v  = Dj(vol);      % deve stare SOTTO
    g  = so_griglia(a, v);
    [~, k] = min(arrayfun(@(s) mean(v > s) + mean(a <= s), g));
    s_new(j) = g(k);
    if verbose
        fprintf('  %-8s %10.4f %10.4f %9.1f%% %9.1f%%\n', nomi{j}, s_ora(j), ...
                s_new(j), 100*mean(v > s_new(j)), 100*mean(a <= s_new(j)));
    end
end

%% ---- 2. affinamento sull'OR ----
% Tre ottimi singoli non fanno l'ottimo dell'OR: basta un giunto sopra
% soglia perche' il flag scatti, quindi i falsi positivi si sommano fra
% giunti mentre i falsi negativi si compensano.
for giro = 1:3
    for j = 1:3
        Dj = D(:, j:3:18);
        g  = so_griglia(Dj(app), Dj(vol));
        best = so_costo(D, app, vol, s_new);  bestv = s_new(j);
        for s = g(:).'
            prova = s_new;  prova(j) = s;
            c = so_costo(D, app, vol, prova);
            if c < best, best = c; bestv = s; end
        end
        s_new(j) = bestv;
    end
end

%% ---- 3. confronto ----
[c_ora, fp_ora, fn_ora] = so_costo(D, app, vol, s_ora);
[c_new, fp_new, fn_new] = so_costo(D, app, vol, s_new);
sp_ora = so_voli(D, dati.volo, s_ora);
sp_new = so_voli(D, dati.volo, s_new);

if verbose
    fprintf('\n---------------- TERNA INTERA (il flag e'' un OR) ----------------\n');
    fprintf('  %-22s %14s %14s\n', '', 'in cfg', 'candidata');
    fprintf('  %-22s %14s %14s\n', 'soglia [N*m]', mat2str(s_ora,4), mat2str(s_new,4));
    fprintf('  %-22s %13.1f%% %13.1f%%\n', 'falsi positivi in volo', 100*fp_ora, 100*fp_new);
    fprintf('  %-22s %13.1f%% %13.1f%%\n', 'falsi negativi a terra', 100*fn_ora, 100*fn_new);
    fprintf('  %-22s %13.1f%% %13.1f%%\n', 'totale', 100*c_ora, 100*c_new);
    fprintf('  %-22s %10d/%-3d %10d/%-3d\n', 'voli senza reset', sp_ora.sporchi, ...
            sp_ora.n, sp_new.sporchi, sp_new.n);

    fprintf('\n  Candidata, non risultato: e'' ricostruita ad anello aperto.\n');
    fprintf('  Per confermarla serve una run vera:\n');
    fprintf('     OVERRIDE_C2_SOGLIA = %s;\n', mat2str(s_new,4));
    fprintf('     init_gait;  [VS,dati] = valida_soglia\n');
    fprintf('  e poi spazzata_soglia, per vedere se le metriche seguono.\n\n');
end
end

%% ========================= helper =========================
function g = so_griglia(a, v)
%SO_GRIGLIA  Griglia log fra le due nuvole, con un margine.
lo = max(min(v(v>0)), eps);
hi = max(a);
if ~isfinite(lo) || ~isfinite(hi) || hi <= lo, g = median([a(:);v(:)]); return; end
g = logspace(log10(lo), log10(hi), 400);
end

function [c, fp, fn] = so_costo(D, app, vol, s)
flag = false(size(D,1), 6);
for i = 1:6
    flag(:,i) = any(D(:, 3*(i-1)+(1:3)) > s, 2);
end
fp = sum(flag & vol, 'all') / sum(vol,'all');
fn = sum(~flag & app, 'all') / sum(app,'all');
c  = fp + fn;
end

function sp = so_voli(D, volo, s)
%SO_VOLI  Voli attraversati senza che il flag torni mai basso.
flag = false(size(D,1), 6);
for i = 1:6
    flag(:,i) = any(D(:, 3*(i-1)+(1:3)) > s, 2);
end
sp = struct('n',0, 'sporchi',0);
for i = 1:6
    v  = volo(:,i);
    su = find(diff([false; v]) ==  1);
    gi = find(diff([v; false]) == -1);
    for k = 1:min(numel(su), numel(gi))
        sp.n = sp.n + 1;
        if all(flag(su(k):gi(k), i)), sp.sporchi = sp.sporchi + 1; end
    end
end
end
