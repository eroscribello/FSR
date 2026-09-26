%% confronta_terreno.m - dove stanno davvero pavimento, rampa e ostacoli
%
% PERCHE' ESISTE
%   [25/9] I video del 15/9 mostrano un C2 su T6 molto migliore di quello di
%   oggi. Commentare applica_inerzie non lo riporta indietro, quindi non sono
%   le inerzie. La cronologia di git dice che dal 15/9 il .slx e' cambiato
%   OTTO volte e applica_terreno CINQUE, e che il 16/9 (c4add49) gli scenari
%   sono passati da "piazzati a mano nel modello" a "generati da script".
%   Ipotesi: il T6 del video e il T6 di oggi non sono lo stesso percorso.
%   Questa funzione la verifica leggendo le pose, invece di rilanciare
%   campagne e confrontare metriche che dipendono da mille altre cose.
%
%   Resta utile dopo: la disposizione di T6 non e' scritta da nessuna parte,
%   e in relazione va documentata.
%
% LA CONVENZIONE DEI SEGNI, CHE E' IL PUNTO DELICATO
%   [MISURATO 22/9, da applica_terreno] Tutti i Rigid Transform del terreno
%   hanno il SOLIDO sulla porta B e il MONDO sulla porta F. L'offset scritto
%   nel blocco e' quindi la posa del MONDO VISTA DAL SOLIDO, e il solido nel
%   mondo sta nella posa INVERSA:
%       p_mondo = -R' * t
%   Per le sole traslazioni vuol dire x e z col segno cambiato.
%   Questa funzione stampa ENTRAMBI: l'offset grezzo del blocco e la posa nel
%   mondo. Cosi' se un giorno la convenzione si rivelasse ancora diversa, il
%   dato grezzo e' li' e non va rilanciato niente.
%
% SOLO LETTURA. Nessun set_param, nessun save_system, su nessun modello.
%
% USO
%   clear all; bdclose all; startup_phantomx
%
%   % 1. lo stato di oggi. SERVONO TUTTI E DUE i passi prima:
%   %    applica_terreno scrive le espressioni nei blocchi,
%   %    init_gait definisce floor_off e ost_dz che quelle espressioni usano.
%   applica_terreno('T6', false);
%   init_gait
%   confronta_terreno
%
%   % 2. un altro modello (per esempio quello di una worktree)
%   confronta_terreno('phantomx_sim_zero')
%
%   % 3. il confronto fra due stati: si salva il primo e si passa al secondo
%   A = confronta_terreno;            % oggi, terreno applicato
%   ... (cd nella worktree, startup, load del modello) ...
%   B = confronta_terreno;
%   confronta_terreno(A, B)           % stampa solo le differenze
%
% ATTENZIONE
%   Le pose dipendono da applica_terreno: vanno lette DOPO averlo chiamato,
%   altrimenti si legge quello che c'e' salvato nel .slx e non quello che la
%   simulazione userebbe. Per lo stato del 15/9, in cui gli scenari erano
%   piazzati a mano nel modello, si legge senza chiamare niente - ed e'
%   proprio quella la differenza che stiamo cercando.
%
% Progetto FSR PhantomX - A. Russo

function out = confronta_terreno(a, b)

%% ---- modalita' confronto fra due tabelle gia' lette ----
if nargin == 2 && istable(a) && istable(b)
    out = ct_confronta(a, b);
    return
end

if nargin < 1 || isempty(a), a = 'phantomx_sim_zero'; end
mdl = char(a);
if ~bdIsLoaded(mdl), load_system(mdl); end

%% ---- il catalogo, con gli stessi nomi di applica_terreno ----
nomi   = [{'pavimento','rampa'}, arrayfun(@(k) sprintf('ostacolo%d',k), 1:7, 'UniformOutput',false)];
solidi = [{'Solid_Pavimento','Solid_Rampa'}, ...
          arrayfun(@(k) sprintf('Solid_Ostacolo%d',k), 1:7, 'UniformOutput',false)];

n = numel(nomi);
elemento = strings(n,1);  blocco = strings(n,1);  off_raw = strings(n,1);
rot = strings(n,1);  dim = strings(n,1);  perche = strings(n,1);
px = nan(n,1);  py = nan(n,1);  pz = nan(n,1);

for i = 1:n
    elemento(i,1) = nomi{i};
    sol = [mdl '/' solidi{i}];
    if getSimulinkBlockHandle(sol) < 0
        blocco(i,1) = "(solido assente)";
        perche(i,1) = "il blocco " + string(solidi{i}) + " non esiste in questo modello";
        continue
    end

    rt = ct_rt_collegato(mdl, solidi{i});     % stessa logica di applica_terreno

    o = '';  R = eye(3);  t = [NaN NaN NaN];  rs = '';  msg = '';
    if ~isempty(rt)
        o = ct_par(rt, 'TranslationCartesianOffset');
        [t, msg] = ct_valuta(o, mdl);
        [R, rs] = ct_rotazione(rt, mdl);
    end
    perche(i,1) = string(msg);                                      %#ok<AGROW>

    % posa nel mondo: il solido sta nella posa INVERSA dell'offset scritto
    p = [NaN NaN NaN];
    if all(isfinite(t)) && numel(t) == 3
        p = (-R.' * t(:)).';
    end

    blocco(i,1)  = string(regexprep(ct_nome(rt), '\s+', ' '));
    off_raw(i,1) = string(regexprep(o, '\s+', ' '));
    px(i,1) = p(1);  py(i,1) = p(2);  pz(i,1) = p(3);
    rot(i,1) = string(rs);
    dim(i,1) = string(ct_dimensioni(sol));
end

out = table(elemento, blocco, off_raw, px, py, pz, rot, dim, perche, ...
    'VariableNames', {'elemento','rigid_transform','offset_scritto', ...
                      'x_mondo','y_mondo','z_mondo','rotazione','geometria','perche_NaN'});

%% ---- lettura ----
fprintf('\n=========== TERRENO DI %s ===========\n', mdl);
fprintf('  offset_scritto = quello che c''e'' nel blocco\n');
fprintf('  x/y/z mondo    = dove sta il solido, con p = -R''*t\n\n');
fprintf('  %-11s %10s %9s %9s  %-22s %s\n', 'elemento','x [m]','y [m]','z [m]','rotazione','geometria');
for i = 1:height(out)
    fprintf('  %-11s %10.4f %9.4f %9.4f  %-22s %s\n', out.elemento(i), ...
            out.x_mondo(i), out.y_mondo(i), out.z_mondo(i), out.rotazione(i), out.geometria(i));
end
fprintf('\n  offset grezzi, per controllare la convenzione:\n');
for i = 1:height(out)
    if strlength(out.offset_scritto(i)) > 0
        fprintf('    %-11s %s\n', out.elemento(i), out.offset_scritto(i));
    end
end

% [25/9] Se NESSUN blocco del catalogo esiste, il modello e' di prima del
% riordino del terreno (16/9, c4add49 e d689e5f): i nomi sono altri. Invece di
% lasciare nove NaN, si elenca quello che c'e' - e' lo stesso schema che ha
% risolto i giunti, dove cercare per nome dava zero e il dump dei MaskType
% dava la risposta.
if all(contains(out.perche_NaN, 'non esiste'))
    fprintf(2, '\n  NESSUN blocco del catalogo esiste in questo modello.\n');
    fprintf(2, '  E'' una versione precedente al riordino del terreno del 16/9.\n');
    ct_inventario(mdl);
    return
end

if any(isnan(out.x_mondo))
    fprintf(2, '\n  PERCHE'' CI SONO DEI NaN:\n');
    for i = 1:height(out)
        if isnan(out.x_mondo(i)) && strlength(out.perche_NaN(i)) > 0
            fprintf(2, '    %-11s %s\n', out.elemento(i), out.perche_NaN(i));
        end
    end
    fprintf(2, ['  Se dice "undefined variable", mancano floor_off o ost_dz:\n' ...
                '  li definisce init_gait (righe 46 e 228). Lancia\n' ...
                '      applica_terreno(''T6'', false);  init_gait\n' ...
                '  e poi rilancia questa funzione.\n']);
end

fprintf('\n  Solo lettura: nessun modello e'' stato modificato.\n\n');
end

%% ================= confronto =================
function d = ct_confronta(A, B)
fprintf('\n=========== DIFFERENZE ===========\n');
fprintf('  %-11s %22s %22s\n', 'elemento', 'A (x, y, z)', 'B (x, y, z)');
n = 0;
for i = 1:height(A)
    j = find(A.elemento(i) == B.elemento, 1);
    if isempty(j), continue; end
    a = [A.x_mondo(i) A.y_mondo(i) A.z_mondo(i)];
    b = [B.x_mondo(j) B.y_mondo(j) B.z_mondo(j)];
    uguali = (all(isnan(a)) && all(isnan(b))) || (all(abs(a-b) < 1e-6));
    if uguali, continue; end
    n = n + 1;
    fprintf('  %-11s %7.3f %7.3f %7.3f   %7.3f %7.3f %7.3f\n', A.elemento(i), a, b);
end
if n == 0
    fprintf('  Nessuna differenza di posa: i due percorsi sono lo stesso.\n');
    fprintf('  Allora la causa del cambiamento e'' altrove.\n\n');
else
    fprintf(2, '\n  %d elementi in posa diversa: NON e'' lo stesso percorso.\n', n);
    fprintf(2, '  Confrontare le metriche di T6 fra i due stati non e'' un\n');
    fprintf(2, '  confronto fra controllori.\n\n');
end
d = n;
end

%% ================= helper =================
function rt = ct_rt_collegato(mdl, solido)
%CT_RT_COLLEGATO  Copiata da applica_terreno: i Rigid Transform NON seguono la
%   convenzione di nomi dei solidi, quindi si trovano seguendo il collegamento.
rt  = '';
sol = [mdl '/' solido];
if getSimulinkBlockHandle(sol) < 0, return; end
hs = get_param(sol, 'Handle');
ph = get_param(sol, 'PortHandles');
for p = [ph.LConn ph.RConn]
    l = get_param(p, 'Line');
    if l <= 0, continue; end
    h = [get_param(l,'SrcBlockHandle'), reshape(get_param(l,'DstBlockHandle'),1,[])];
    for bb = h(h > 0 & h ~= hs)
        ref = regexprep(get_param(bb, 'ReferenceBlock'), '\s+', ' ');
        nm  = regexprep(get_param(bb, 'Name'),           '\s+', ' ');
        if contains(ref, 'Rigid Transform') || startsWith(nm, 'Rigid Transform')
            rt = getfullname(bb);  return
        end
    end
end
end

function s = ct_par(blk, nome)
s = '';
try, s = get_param(blk, nome); end %#ok<TRYNC>
end

function [v, msg] = ct_valuta(espr, mdl)
%CT_VALUTA  L'offset e' un'ESPRESSIONE (es. "floor_off(:).' + [0.3 0 ost_dz]"),
%   quindi va valutata nel workspace che il modello usa davvero.
%   [25/9] Se fallisce restituisce anche il MOTIVO: la prima volta sono usciti
%   sette NaN senza spiegazione, e la causa era solo che floor_off e ost_dz
%   non erano ancora nel workspace (li definisce init_gait, righe 46 e 228).
%   Un NaN muto costringe a indovinare; un NaN che dice perche' no.
v = [NaN NaN NaN];  msg = '';
if isempty(espr), msg = 'offset vuoto'; return; end
try
    v = slResolve(espr, mdl);            % workspace del modello, poi base
catch ME1
    try
        v = evalin('base', espr);
    catch ME2
        msg = regexprep(ME2.message, '\s+', ' ');
        if isempty(msg), msg = regexprep(ME1.message, '\s+', ' '); end
        return
    end
end
v = double(v(:)).';
if numel(v) ~= 3
    msg = sprintf('l''espressione da'' %d valori invece di 3', numel(v));
    v = [NaN NaN NaN];
end
end

function [R, s] = ct_rotazione(rt, mdl)
R = eye(3);  s = 'nessuna';
met = ct_par(rt, 'RotationMethod');
if isempty(met) || contains(met, 'None'), return; end
ax = ct_par(rt, 'RotationAxisVector');
an = ct_par(rt, 'RotationAngle');
% [25/9] L'angolo della rampa usciva "8": sono GRADI, non radianti. Senza
% leggere le unita' la R veniva sbagliata e con lei la posa nel mondo.
un = ct_par(rt, 'RotationAngleUnits');
try
    a = double(ct_valuta(ax, mdl));
    th = slResolve(an, mdl);
    if ~isempty(un) && (contains(lower(un),'deg') || contains(un,'�'))
        th = deg2rad(th);
        s_un = 'deg';
    else
        s_un = 'rad';
    end
    if all(isfinite(a)) && isfinite(th)
        u = a(:)/norm(a(:));  c = cos(th);  sn = sin(th);
        K = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];
        R = eye(3)*c + sn*K + (1-c)*(u*u.');
        s = sprintf('%s  %.4g %s', mat2str(round(a(:).',3)), ...
                    ternario_ct(strcmp(s_un,'deg'), rad2deg(th), th), s_un);
    else
        s = sprintf('%s  %s', ax, an);
    end
catch
    s = sprintf('%s  %s', ax, an);
end
end

function s = ct_dimensioni(sol)
%CT_DIMENSIONI  Che forma ha il solido: il file della mesh, o le misure.
%   [25/9] La prima versione tornava sempre vuoto sui File Solid: il
%   parametro della mesh non si chiama FileName ma ExtGeomFileName. Se
%   nessuno dei nomi noti c'e', si cerca fra i parametri del blocco invece
%   di arrendersi in silenzio - un campo vuoto qui vorrebbe dire non poter
%   confrontare le geometrie fra due versioni del modello, che e'
%   esattamente la domanda aperta.
s = '';
noti = {'ExtGeomFileName','GeomFileName','FileName','GeometryLength', ...
        'Dimensions','Radius','ExtrusionCrossSection'};
for p = noti
    v = ct_par(sol, p{1});
    if ~isempty(v), s = ct_breve(v); return; end
end

try
    n = fieldnames(get_param(sol, 'DialogParameters'));
catch
    return
end
c = n(contains(n,'File','IgnoreCase',true) | contains(n,'Geom','IgnoreCase',true));
for k = 1:numel(c)
    v = ct_par(sol, c{k});
    if ~isempty(v), s = [c{k} ' = ' ct_breve(v)]; return; end
end
end

function s = ct_breve(v)
v = char(v);
[~, f, e] = fileparts(v);
if ~isempty(e), v = [f e]; end
s = regexprep(v, '\s+', ' ');
end

function ct_inventario(mdl)
%CT_INVENTARIO  Tutti i solidi del modello, con dove stanno e cosa sono.
%   Serve quando i nomi attesi non ci sono: dice come si chiamavano allora.
bb = find_system(mdl, 'LookUnderMasks','all', 'FollowLinks','on', 'Type','block');

radice = {};  annidati = 0;
for i = 1:numel(bb)
    mt = '';
    try, mt = get_param(bb{i}, 'MaskType'); end %#ok<TRYNC>
    if ~contains(mt, 'Solid', 'IgnoreCase', true), continue; end
    if strcmp(get_param(bb{i}, 'Parent'), mdl)
        radice{end+1} = bb{i};                                      %#ok<AGROW>
    else
        annidati = annidati + 1;
    end
end

fprintf('\n=========== SOLIDI AL PRIMO LIVELLO DI %s ===========\n', mdl);
fprintf('  (il terreno sta qui; i solidi del robot sono dentro i sottosistemi)\n\n');
fprintf('  %-34s %-18s %-26s %s\n', 'nome', 'tipo', 'geometria', 'offset del Rigid Transform');
for i = 1:numel(radice)
    b  = radice{i};
    nm = regexprep(get_param(b,'Name'), '\s+', ' ');
    mt = get_param(b, 'MaskType');
    g  = ct_dimensioni(b);

    % il Rigid Transform collegato, con la stessa logica del resto
    rt = ''; off = '';
    hs = get_param(b, 'Handle');  ph = get_param(b, 'PortHandles');
    for p = [ph.LConn ph.RConn]
        l = get_param(p, 'Line');
        if l <= 0, continue; end
        h = [get_param(l,'SrcBlockHandle'), reshape(get_param(l,'DstBlockHandle'),1,[])];
        for bb2 = h(h > 0 & h ~= hs)
            ref = regexprep(get_param(bb2,'ReferenceBlock'), '\s+', ' ');
            n2  = regexprep(get_param(bb2,'Name'),           '\s+', ' ');
            if contains(ref,'Rigid Transform') || startsWith(n2,'Rigid Transform')
                rt = getfullname(bb2);  break
            end
        end
        if ~isempty(rt), break; end
    end
    if ~isempty(rt)
        off = regexprep(ct_par(rt,'TranslationCartesianOffset'), '\s+', ' ');
    end

    fprintf('  %-34s %-18s %-26s %s\n', nm(1:min(34,end)), mt(1:min(18,end)), ...
            g(1:min(26,end)), off);
end
fprintf('\n  solidi al primo livello: %d;  dentro i sottosistemi: %d\n', ...
        numel(radice), annidati);
fprintf(['\n  Confronta i nomi e le geometrie con quelli di oggi: se il percorso\n' ...
         '  a ostacoli e'' fatto di blocchi diversi, il T6 del video e quello di\n' ...
         '  oggi non sono lo stesso scenario, e le metriche non si confrontano.\n\n']);
end

function s = ct_nome(rt)
if isempty(rt), s = '(nessuno)'; else, [~, s] = fileparts(rt); end
end

function v = ternario_ct(c, a, b)
if c, v = a; else, v = b; end
end
