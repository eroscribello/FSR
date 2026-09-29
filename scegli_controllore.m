function [mdl, c2, info] = scegli_controllore(ctrl)
%SCEGLI_CONTROLLORE  Il banco di prova di un controllore, dal suo nome.
%
%   [mdl, c2] = scegli_controllore('C2')
%
%   Restituisce il modello su cui gira quel controllore e il valore da dare
%   a OVERRIDE_C2. Il nome canonico torna in info.nome, ed e' quello che
%   deve finire nel file dei risultati.
%
% PERCHE' ESISTE
%   Gli script dei task avevano il controllore come BOOLEANO:
%
%       t5_c2   = true;                 % false = C1, true = C2
%       t5_ctrl = t5_nome_ctrl(t5_c2);  % -> 'C1' oppure 'C2'
%       ...
%       writetable(t5_riga, sprintf('results/T5_%s.csv', t5_ctrl));
%
%   Con due controllori funzionava. Con tre no, e non fallisce con un
%   errore: fallisce SOVRASCRIVENDO results/T5_C2.csv con una run di C3.
%   Il nome del file esce da una variabile che non sa che C3 esiste.
%
%   Qui il nome del controllore diventa l'unica cosa che si sceglie, e il
%   resto - modello, interruttore, nome del file - ne discende.
%
% I TRE CONTROLLORI
%
%   C1   phantomx_sim_zero       c2 = false   cinematico ad anello aperto,
%                                             andatura a tripode, giunti in
%                                             posizione
%   C2   phantomx_sim_zero       c2 = true    C1 + ricerca del terreno
%                                             (Arrigoni et al. 2024)
%   C3   phantomx_sim_attitude   c2 = true    C2 + retroazione di beccheggio
%
%   Ogni controllore aggiunge UN meccanismo al precedente, e ogni confronto
%   fra due adiacenti ne misura uno solo. Se C3 girasse con c2 = false,
%   C2 e C3 differirebbero per due meccanismi insieme e nessun confronto
%   fra loro sarebbe interpretabile.
%
% PERCHE' C3 HA c2 = true, E NON E' UNA SCELTA ARBITRARIA
%   [29/9] Verificato leggendo l'XML dentro il .slx (e' uno zip) e con
%   verifica_attitude:
%
%     - phantomx_sim_attitude contiene gli STESSI due blocchi MATLAB
%       Function che chiamano ricerca_terreno, con lo stesso codice;
%     - il confronto blocco per blocco fra i due modelli non mostra NESSUNA
%       rimozione: l'assetto e' solo aggiunto;
%     - la catena e'
%           ricerca_terreno -> z_* -> Sum (+ d_z_*) -> Saturation
%                           -> Rate Limiter -> gamba
%       cioe' l'assetto sta IN SERIE, DOPO la ricerca del terreno: legge la
%       quota gia' corretta e ci somma un termine. Non si contendono il
%       segnale, che sarebbe stato il caso da evitare;
%     - il modello legge c2_par, quindi OVERRIDE_C2 e' un interruttore vivo
%       anche su C3: va acceso di proposito, non per inerzia.
%
% QUELLO CHE QUESTA FUNZIONE NON FA
%   Non carica il modello, non chiama set_param, non tocca niente. E' una
%   tabella con dei controlli, e per questo si puo' chiamare in testa a
%   ogni script senza effetti collaterali.
%
%   In particolare NON verifica che il carico ("il pacco": Brick Solid,
%   Brick Solid1, 6-DOF Joint1, Spatial Contact Force) sia commentato nel
%   modello di C3. Al 29/9 e' ATTIVO, e una campagna lanciata cosi'
%   misurerebbe C3 con il carico contro C1 e C2 scarichi. Il controllo sta
%   in verifica_attitude.
%
% USO
%   [t5_mdl, t5_c2] = scegli_controllore(t5_ctrl);
%   scegli_controllore                      % stampa la tabella e basta
%   info = scegli_controllore('c3')         % il nome non e' case sensitive
%
% Progetto FSR PhantomX - A. Russo

T = sc_tabella();

%% ---- senza argomenti: stampa e basta ----
if nargin < 1 || isempty(ctrl)
    if nargout > 0
        error('scegli_controllore:nome', ...
            'Serve il nome del controllore: %s', sc_elenco(T));
    end
    fprintf('\n  %-6s %-24s %-8s %s\n', 'nome', 'modello', 'c2', 'cos''e''');
    fprintf('  %s\n', repmat('-', 1, 78));
    for k = 1:numel(T)
        fprintf('  %-6s %-24s %-8s %s\n', T(k).nome, T(k).mdl, ...
                sc_bool(T(k).c2), T(k).cosa);
    end
    fprintf('\n');
    return
end

%% ---- normalizzazione del nome ----
% Si accettano 'c3', "C3", 'C3 ' e simili: il nome arriva spesso da una
% variabile scritta a mano in testa a uno script.
if isstring(ctrl) || ischar(ctrl)
    nome = upper(strtrim(char(ctrl)));
else
    error('scegli_controllore:tipo', ...
        ['Il controllore si indica per NOME (''C1'', ''C2'', ''C3''), non ' ...
         'come %s.\nIl booleano era la vecchia convenzione a due ' ...
         'controllori: non basta piu''.'], class(ctrl));
end

k = find(strcmp({T.nome}, nome), 1);
if isempty(k)
    error('scegli_controllore:ignoto', ...
        'Controllore ignoto: ''%s''. Quelli definiti sono %s.', nome, sc_elenco(T));
end

mdl  = T(k).mdl;
c2   = T(k).c2;
info = T(k);

%% ---- promemoria, solo per C3 ----
% Non e' un controllo: e' una riga che si legge mentre la run parte. Il
% controllo vero sta in verifica_attitude, che apre il modello.
if strcmp(nome, 'C3')
    fprintf(['  C3: controlla che il carico (Brick Solid, 6-DOF Joint1, Spatial\n' ...
             '      Contact Force) sia commentato in %s, altrimenti stai\n' ...
             '      confrontando C3 carico con C1 e C2 scarichi.\n'], mdl);
end
end

%% ================= la tabella =================
function T = sc_tabella()
%SC_TABELLA  L'unico posto dove sta scritto chi gira su cosa.
%   Aggiungere un controllore significa aggiungere una riga qui, e nient'altro.
T = struct( ...
    'nome', {'C1', 'C2', 'C3'}, ...
    'mdl',  {'phantomx_sim_zero', 'phantomx_sim_zero', 'phantomx_sim_attitude'}, ...
    'c2',   {false, true, true}, ...
    'cosa', {'cinematico ad anello aperto, tripode', ...
             'C1 + ricerca del terreno (Arrigoni 2024)', ...
             'C2 + retroazione di beccheggio, in serie'});
end

%% ================= helper =================
function s = sc_elenco(T)
s = strjoin(strcat('''', {T.nome}, ''''), ', ');
end

function s = sc_bool(b)
if b, s = 'true'; else, s = 'false'; end
end
