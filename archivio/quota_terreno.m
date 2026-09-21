function q = quota_terreno(mdl)
%QUOTA_TERRENO  Quota della superficie dei due pavimenti e della base degli ostacoli.
%
%   quota_terreno
%
% PERCHE'
%   Le mesh degli ostacoli sono state modellate contro il CUBE ('imperfetto'),
%   non contro il Brick liscio. Su 'liscio' partono percio' da una quota
%   sbagliata dello scarto fra le due superfici. Serve quel numero, non una
%   correzione a occhio.
%
% COSA MISURA
%   - superficie superiore del Brick liscio:  offset del suo Rigid Transform
%     + meta' della dimensione z (l'origine del Brick sta nel centro)
%   - superficie superiore del Cube:          offset + z massima della mesh
%   - base di ogni ostacolo:                  offset + z minima della mesh
%
%   Le mesh vengono lette direttamente dal file STL, tenendo conto delle
%   unita' dichiarate nel blocco File Solid.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(mdl), mdl = 'phantomx_sim_zero'; end
if ~bdIsLoaded(mdl), load_system(mdl); end

fprintf('\n=============== QUOTE DEL TERRENO ===============\n');

%% ---- pavimento liscio: Brick Solid3 ----
dimB = param([mdl '/Brick Solid3'], {'BrickDimensions','Dimensions','Size'});
offB = param([mdl '/' rt(9)], {'TranslationCartesianOffset'});
d = valuta(dimB);  o = valuta(offB);
zLiscio = NaN;
if numel(d) >= 3 && numel(o) >= 3
    zLiscio = o(3) + d(3)/2;
end
fprintf('\nPAVIMENTO LISCIO  (Brick Solid3 + %s)\n', strrep(rt(9),newline,' '));
fprintf('  dimensioni  %-24s -> %s\n', dimB, mat2str(d,4));
fprintf('  offset      %-24s -> %s\n', offB, mat2str(o,4));
fprintf('  SUPERFICIE  z = %.5f m\n', zLiscio);

%% ---- pavimento imperfetto: il Cube ----
[fC, uC] = fileSolid([mdl '/File Solid']);
offC = param([mdl '/' rt([])], {'TranslationCartesianOffset'});
oC   = valuta(offC);
[zminC, zmaxC] = estremiZ(fC, uC);
zImperfetto = NaN;
if ~isnan(zmaxC) && numel(oC) >= 3, zImperfetto = oC(3) + zmaxC; end
fprintf('\nPAVIMENTO IMPERFETTO  (File Solid + %s)\n', strrep(rt([]),newline,' '));
fprintf('  mesh        %s   [%s]\n', fC, uC);
fprintf('  z mesh      min %.5f   max %.5f  (m)\n', zminC, zmaxC);
fprintf('  offset      %-24s -> %s\n', offC, mat2str(oC,4));
fprintf('  SUPERFICIE  z = %.5f m\n', zImperfetto);

%% ---- ostacoli ----
fprintf('\nOSTACOLI\n');
fprintf('  %-12s %-10s %-10s %-10s %-10s\n', 'solido','z base','su liscio','su imperf.','mesh');
base = nan(1,7);
for k = 1:7
    [f, u] = fileSolid([mdl '/File Solid' num2str(k)]);
    offK = param([mdl '/' rt(9+k)], {'TranslationCartesianOffset'});
    oK   = valuta(offK);
    [zmin, ~] = estremiZ(f, u);
    if ~isnan(zmin) && numel(oK) >= 3, base(k) = oK(3) + zmin; end
    fprintf('  File Solid%-2d %-10.5f %-+10.5f %-+10.5f %s\n', ...
            k, base(k), base(k)-zLiscio, base(k)-zImperfetto, f);
end

%% ---- conclusione ----
% La correzione e' lo SCARTO FRA LE DUE SUPERFICI, non la media delle basi.
% Le mesh degli ostacoli sono state posate contro il Cube: traslandole di
% quello scarto conservano la stessa quota relativa che hanno li'.
% Il segno: il Rigid Transform degli ostacoli e' percorso nel verso opposto
% a quello che verrebbe da assumere, quindi la sua z e' invertita rispetto
% alla z del mondo. Verificato in simulazione: con -0.025 l'ostacolo sale.
dz = zImperfetto - zLiscio;
fprintf('\nSCARTO FRA I DUE PAVIMENTI  %+.5f m\n', zImperfetto - zLiscio);
fprintf('CORREZIONE da applicare agli ostacoli quando il pavimento e'' LISCIO\n');
fprintf('  cfg.terreno.ost_dz = %.5f;   %% [m] (segno invertito: la z di\n', dz);
fprintf('                                %% quel Rigid Transform e'' opposta\n');
fprintf('                                %% a quella del mondo)\n');

% Quali ostacoli sono a filo del Cube e quali poggiano sui suoi rilievi.
% zmax e' il punto piu' alto dell'INTERA mesh del terreno imperfetto, non la
% quota locale sotto ciascun ostacolo: chi risulta molto sopra sta appoggiato
% a un rilievo, e su pavimento liscio restera' in aria anche dopo la
% correzione. Quelli non si possono usare su 'liscio'.
sopra = base - zImperfetto;
filo  = sopra < 5e-3;
fprintf('\nUSABILI SU PAVIMENTO LISCIO (a filo del Cube, scarto < 5 mm)\n');
fprintf('  %s\n', strjoin(compose('ost%d', find(filo)), ' '));
if any(~filo)
    fprintf('POGGIANO SUI RILIEVI del terreno imperfetto, solo per T6\n');
    for k = find(~filo)
        fprintf('  ost%d   +%.1f mm sopra il Cube\n', k, sopra(k)*1e3);
    end
end
fprintf('\n');

q = struct('z_liscio',zLiscio, 'z_imperfetto',zImperfetto, ...
           'base_ostacoli',base, 'ost_dz',dz, 'usabili_su_liscio',find(filo));
end

%% ================================================================
function v = param(blocco, candidati)
v = '';
for c = candidati
    try v = get_param(blocco, c{1}); return; catch, end
end
end

function [f, u] = fileSolid(blocco)
f = param(blocco, {'ExtGeomFileName','GeometryFileName','FileName'});
u = param(blocco, {'ExtGeomUnits','GeometryUnits','Units'});
if isempty(u), u = 'm'; end
f = strtrim(strrep(f, '''', ''));
end

function v = valuta(s)
v = [];
if isempty(s), return; end
try v = evalin('base', s); catch
    try v = str2num(s); catch, end %#ok<ST2NM>
end
v = v(:).';
end

function [zmin, zmax] = estremiZ(file, unita)
%ESTREMIZ  z minima e massima della mesh, in metri.
zmin = NaN;  zmax = NaN;
p = which(file);  if isempty(p), p = file; end
if exist(p,'file') ~= 2, return; end
try
    V = leggiSTL(p);
catch
    return
end
if isempty(V), return; end
s = fattore(unita);
zmin = min(V(:,3)) * s;
zmax = max(V(:,3)) * s;
end

function s = fattore(u)
switch lower(strtrim(u))
    case {'mm','millimeter','millimeters'}, s = 1e-3;
    case {'cm','centimeter','centimeters'}, s = 1e-2;
    case {'in','inch','inches'},            s = 0.0254;
    otherwise,                              s = 1;
end
end

function V = leggiSTL(p)
%LEGGISTL  Vertici di un STL, binario o ASCII. Solo le coordinate servono.
V = [];
fid = fopen(p,'rb');  if fid < 0, return; end
c = onCleanup(@() fclose(fid));
d = dir(p);
fseek(fid, 80, 'bof');
n = fread(fid, 1, 'uint32');
if ~isempty(n) && d.bytes == 84 + 50*n         % binario
    V = zeros(3*n, 3);
    for i = 1:n
        fread(fid, 3, 'float32');              % normale, si scarta
        V(3*i-2:3*i, :) = reshape(fread(fid, 9, 'float32'), 3, 3).';
        fread(fid, 1, 'uint16');               % attributo
    end
else                                           % ASCII
    frewind(fid);
    t = fread(fid, inf, '*char').';
    m = regexp(t, 'vertex\s+(\S+)\s+(\S+)\s+(\S+)', 'tokens');
    V = zeros(numel(m), 3);
    for i = 1:numel(m), V(i,:) = str2double(m{i}); end
end
end

function s = rt(k)
if isempty(k), s = sprintf('Rigid\nTransform');
else,          s = sprintf('Rigid\nTransform%d', k);
end
end