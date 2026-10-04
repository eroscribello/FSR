function [mdl, c2, info] = scegli_controllore(ctrl)
%   [mdl, c2] = scegli_controllore('C2')
%
%   Restituisce il modello su cui gira quel controllore e il valore da dare
%   a OVERRIDE_C2.
%
%
%   USO
%   [t5_mdl, t5_c2] = scegli_controllore(t5_ctrl);
%   scegli_controllore                      % stampa la tabella e basta
%   info = scegli_controllore('c3')         % il nome non e' case sensitive
%

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

%% ---- promemoria sul carico ----
if info.carico
    fprintf(2, ['  %s: il pacco resta ATTIVO in %s (1 kg + vassoio 0.1 kg).\n' ...
                '      Le righe non si confrontano con C1-C3: e'' un altro robot.\n'], nome, mdl);
end
end

%% ================= la tabella =================
function T = sc_tabella()
%SC_TABELLA  L'unico posto dove sta scritto chi gira su cosa.
T = struct( ...
    'nome',   {'C1', 'C2', 'C3', 'C3P'}, ...
    'mdl',    {'phantomx_sim_zero', 'phantomx_sim_zero', 'phantomx_sim_attitude', 'phantomx_sim_attitude'}, ...
    'c2',     {false, true, true, true}, ...
    'carico', {false, false, false, true}, ...
    'cosa',   {'cinematico ad anello aperto, tripode', ...
               'C1 + ricerca del terreno (Arrigoni 2024)', ...
               'C2 + retroazione di beccheggio, in serie', ...
               'C3 con il pacco ATTIVO (1 kg + vassoio 0.1 kg)'});
end

%% ================= helper =================
function s = sc_elenco(T)
s = strjoin(strcat('''', {T.nome}, ''''), ', ');
end

function s = sc_bool(b)
if b, s = 'true'; else, s = 'false'; end
end
