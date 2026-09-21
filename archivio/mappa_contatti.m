function mappa_contatti(mdl, piede)
%MAPPA_CONTATTI  Chi cerca cosa: etichette dei contatti e dei solidi.
%
%   mappa_contatti
%   mappa_contatti('phantomx_sim_zero','Subsystem')
%
% A COSA SERVE
%   applica_terreno assume che Spatial Contact Force(k+1) sia il contatto di
%   File Solid k. Quell'associazione non e' mai stata letta dal modello: e'
%   stata dedotta dai numeri. T6 non la mette alla prova, perche' accende
%   tutti e sette gli ostacoli insieme e qualunque permutazione funziona.
%   T5 ne accende UNO solo, quindi e' il primo caso in cui una associazione
%   sbagliata si vede - ed e' l'unico task che non compila.
%
% DUE TENTATIVI SBAGLIATI, E PERCHE'
%   v1 risaliva la rete per 4 salti: ma i solidi del terreno stanno tutti
%   sullo stesso albero, tenuti insieme dalla catena di Rigid Transform che
%   li ancora al World, quindi da ognuno si arrivava a tutte le etichette.
%   v2 aggiustava i solidi ma non i contatti: gli otto Spatial Contact Force
%   di un piede condividono la porta A sulla STESSA sfera (Spherical Solid2),
%   quindi da uno qualunque si raggiungono tutti gli altri e tutte le porte.
%
%   Qui niente attraversamenti: si guarda porta per porta, un salto solo.
%   Il percorso della geometria del terreno e'
%       Spatial Contact Force(k), porta B -> Connection Port n del piede
%       -> (livello di sopra) porta n del Subsystem -> Connection Label
%       -> etichetta -> solido
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(mdl),   mdl   = 'phantomx_sim_zero'; end
if nargin < 2 || isempty(piede), piede = 'Subsystem';         end
if ~bdIsLoaded(mdl), load_system(mdl); end

%% ---- 1. porte del piede -> etichetta, al livello di sopra ----
pcPiede = get_param([mdl '/' piede], 'PortConnectivity');
fprintf('\n=== PORTE DEL PIEDE %s -> ETICHETTA ===\n', piede);
lblPorta = containers.Map('KeyType','double','ValueType','char');
for p = 1:numel(pcPiede)
    t = pcPiede(p).Type;
    if ~ischar(t), continue; end
    n = str2double(regexp(t,'\d+$','match','once'));
    if isnan(n), continue; end
    h = [pcPiede(p).SrcBlock, pcPiede(p).DstBlock];  h = h(h > 0);
    for j = 1:numel(h)
        nome = strrep(get_param(h(j),'Name'), newline, ' ');
        if startsWith(nome, 'Connection Label')
            lab = get_param([mdl '/' get_param(h(j),'Name')], 'Label');
        else
            lab = ['?' nome];
        end
        lblPorta(n) = lab;
        fprintf('  porta %-3s  %-18s  ->  %s\n', t, nome, lab);
    end
end

%% ---- 2. contatti del piede -> porta, un salto solo ----
fprintf('\n=== CONTATTI DEL PIEDE %s (vicini diretti) ===\n', piede);
for k = 1:8
    b = sprintf('%s/%s/Spatial Contact Force%d', mdl, piede, k);
    if ~esiste(b), fprintf('  Force%-2d  (assente)\n', k); continue; end
    fprintf('  Force%-2d  commented=%s\n', k, get_param(b,'Commented'));
    pc = get_param(b,'PortConnectivity');
    for p = 1:numel(pc)
        h = [pc(p).SrcBlock, pc(p).DstBlock];  h = h(h > 0);
        for j = 1:numel(h)
            nomeRaw = get_param(h(j),'Name');
            nome    = strrep(nomeRaw, newline, ' ');
            blk     = [get_param(b,'Parent') '/' nomeRaw];
            extra   = '';
            bt = '';  try, bt = get_param(blk,'BlockType'); catch, end
            if strcmp(bt,'PMIOPort')
                n = str2double(get_param(blk,'Port'));
                if isKey(lblPorta, n), extra = sprintf('  ==> %s', lblPorta(n));
                else,                  extra = '  ==> (porta non mappata)';
                end
                nome = sprintf('%s [porta %d]', nome, n);
            end
            fprintf('      %-8s %-28s%s\n', pc(p).Type, nome, extra);
        end
    end
end

%% ---- 3. solidi del terreno -> etichetta, vicino diretto ----
fprintf('\n=== ETICHETTA FORNITA DA OGNI SOLIDO ===\n');
sol = [{'Brick Solid3','Brick Solid1','Brick Solid2','File Solid'}, ...
       arrayfun(@(k) sprintf('File Solid%d',k), 1:7, 'UniformOutput', false)];
for s = sol
    b = [mdl '/' s{1}];
    if ~esiste(b), fprintf('  %-14s (assente)\n', s{1}); continue; end
    fprintf('  %-14s label=%-12s  commented=%s\n', ...
            s{1}, vicinoDiretto(b), get_param(b,'Commented'));
end
fprintf('\n');

end

%% ================================================================
function lab = vicinoDiretto(b)
lab = '(nessuna)';
padre = get_param(b,'Parent');
try, pc = get_param(b,'PortConnectivity'); catch, return; end
for p = 1:numel(pc)
    h = [pc(p).SrcBlock, pc(p).DstBlock];  h = h(h > 0);
    for j = 1:numel(h)
        n = get_param(h(j),'Name');
        if startsWith(strrep(n,newline,' '), 'Connection Label')
            try, lab = get_param([padre '/' n],'Label'); return; catch, end
        end
    end
end
end

function t = esiste(b)
t = true;
try, get_param(b,'Name'); catch, t = false; end
end