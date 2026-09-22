function ispeziona_rampa(mdl)
%ISPEZIONA_RAMPA  Stampa la catena di blocchi dalla rampa al World Frame.
%
%   ispeziona_rampa
%
% Non simula e non modifica il modello. Serve a decidere la posa della rampa
% per T4 sapendo com'e' fatta la catena, invece di indovinare i segni come
% e' successo con gli ostacoli (x e z dei Rigid Transform opposte al mondo).
%
% Per confronto stampa anche la catena del pavimento e dell'ostacolo 1, di
% cui i segni sono gia' misurati.
%
% Per ogni blocco: nome, porta di ingresso/uscita (B = base, F = follower) e i
% parametri di posa. Per i solidi: forma e dimensioni.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(mdl), mdl = 'phantomx_sim_zero'; end
if ~bdIsLoaded(mdl), load_system(mdl); end

for s = {'Solid_Rampa', 'Solid_Pavimento', 'Solid_Ostacolo1'}
    fprintf('\n===== %s =====\n', s{1});
    catena(mdl, [mdl '/' s{1}]);
end
fprintf('\n');
end

%% -------------------------------------------------------------------------
function catena(mdl, partenza)
if getSimulinkBlockHandle(partenza) < 0
    fprintf('  (blocco non trovato)\n'); return
end
visti = get_param(partenza,'Handle');
coda  = visti;
passo = 0;
while ~isempty(coda) && passo < 12
    b = coda(1);  coda(1) = [];  passo = passo + 1;
    stampa_blocco(b);
    if contains(get_param(b,'ReferenceBlock'), 'World Frame'), continue; end
    for v = vicini(b)
        if ~ismember(v.h, visti)
            visti(end+1) = v.h;                                            %#ok<AGROW>
            coda(end+1)  = v.h;                                            %#ok<AGROW>
            fprintf('      %s --[%s -> %s]--> %s\n', nome(b), v.porta_mia, ...
                    v.porta_sua, nome(v.h));
        end
    end
end
end

function out = vicini(b)
% blocchi collegati alle porte fisiche di b, con la porta usata da ciascun lato
out = struct('h',{},'porta_mia',{},'porta_sua',{});
ph = get_param(b,'PortHandles');
mie = [ph.LConn ph.RConn];
for p = mie
    l = get_param(p,'Line');  if l <= 0, continue; end
    for q = porte_della_linea(l)
        hb = get_param(q,'Parent');  hb = get_param(hb,'Handle');
        if hb == b, continue; end
        out(end+1) = struct('h',hb, 'porta_mia',etichetta(b,p), ...
                            'porta_sua',etichetta(hb,q));               %#ok<AGROW>
    end
end
% Connection Label: il collegamento prosegue sulle etichette con lo stesso nome
lab = etichetta_nome(b);
if ~isempty(lab)
    % con un handle come radice find_system restituisce handle, non nomi
    altre = find_system(get_param(bdroot(b),'Handle'), 'LookUnderMasks','all', ...
                        'FollowLinks','on', 'Type','block');
    for k = 1:numel(altre)
        hk = altre(k);
        if hk ~= b && strcmp(etichetta_nome(hk), lab)
            out(end+1) = struct('h',hk, 'porta_mia',['label ' lab], ...
                                'porta_sua',['label ' lab]);            %#ok<AGROW>
        end
    end
end
end

function lab = etichetta_nome(b)
% nome dell'etichetta se b e' un Connection Label, altrimenti ''
lab = '';
if ~contains(get_param(b,'ReferenceBlock'), 'Connection Label') ...
        && ~contains(get_param(b,'Name'), 'Connection Label')
    return
end
try, lab = get_param(b,'Label'); catch, end
end

function pp = porte_della_linea(l)
% Le linee fisiche ramificate hanno LineChildren che possono puntare di nuovo
% al genitore: la visita tiene il conto delle linee gia' viste, altrimenti la
% ricorsione non termina (e' successo alla prima versione).
pp = [];
da_fare = l;  viste = [];
while ~isempty(da_fare)
    x = da_fare(1);  da_fare(1) = [];
    if ismember(x, viste) || x <= 0, continue; end
    viste(end+1) = x;                                                  %#ok<AGROW>
    pp = [pp get_param(x,'SrcPortHandle'), ...
             reshape(get_param(x,'DstPortHandle'),1,[])];              %#ok<AGROW>
    da_fare = [da_fare reshape(get_param(x,'LineChildren'),1,[]) ...
                       reshape(get_param(x,'LineParent'),1,[])];       %#ok<AGROW>
end
pp = unique(pp(pp > 0));
end

function e = etichetta(b, p)
% LConn/RConn e, per i blocchi di Multibody, B/F
ph = get_param(b,'PortHandles');
i = find(ph.LConn == p, 1);
if ~isempty(i), e = sprintf('L%d', i); else
    i = find(ph.RConn == p, 1);  e = sprintf('R%d', i);
end
if contains(get_param(b,'ReferenceBlock'), 'Rigid Transform')
    e = strrep(strrep(e,'L1','B'),'R1','F');
end
end

function n = nome(b)
n = regexprep(get_param(b,'Name'), '\s+', ' ');
end

function stampa_blocco(b)
ref = regexprep(get_param(b,'ReferenceBlock'), '\s+', ' ');
fprintf('  [%s]  %s   (commentato: %s)\n', nome(b), ref, get_param(b,'Commented'));
% i nomi dei parametri dei solidi cambiano fra versioni: si filtrano per
% contenuto invece di elencarli
dp = get_param(b,'DialogParameters');
if isempty(dp), return; end
f = fieldnames(dp);
tieni = ~cellfun('isempty', regexp(f, ...
    '(Translation|Rotation|Dimension|Shape|Geom|Length|Width|Height)', 'once')) ...
        & cellfun('isempty', regexp(f, ...
          '(Units|_conf|Priority|Graphic|Sequence|Arbitrary|ROffset|Theta|ZOffset)', 'once'));
tieni = tieni | strcmp(f, 'ExtGeomFileUnits');   % mm o m: cambia la scala di 1000
for k = find(tieni).'
    try
        v = get_param(b, f{k});
        if ischar(v), fprintf('        %-28s %s\n', f{k}, strtrim(v)); end
    catch
    end
end
end
