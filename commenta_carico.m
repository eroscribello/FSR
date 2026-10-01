function info = commenta_carico(mdl, stato)
%COMMENTA_CARICO  Toglie (o rimette) il carico dal modello di C3, in memoria.
%
%   commenta_carico                         % phantomx_sim_attitude, carico tolto
%   commenta_carico(mdl, 'off')             % lo rimette
%   info = commenta_carico(...)
%
% PERCHE' ESISTE
%   phantomx_sim_attitude porta un carico poggiato sul corpo ("il pacco":
%   Brick Solid, Brick Solid1, 6-DOF Joint1, Spatial Contact Force), usato
%   dal collega per provare l'assetto. Su disco e' ATTIVO, e una campagna
%   lanciata cosi' misura C3 carico contro C1 e C2 scarichi.
%
%   Fino all'1/10 lo si commentava a mano nella riga di comando di ogni
%   sessione: funzionava, ma nel repo non ne restava traccia, e una run
%   rifatta da qualcun altro sarebbe uscita carica senza che niente lo
%   dicesse. Qui diventa una chiamata, come applica_terreno.
%
% COME
%   set_param(..., 'Commented', ...) sui quattro blocchi, SENZA salvare il
%   modello (regola 3 di CLAUDE.md). bdclose riporta tutto com'era.
%   Il nome del contatto contiene un a capo ("Spatial" newline "Contact
%   Force"): e' il nome vero del blocco, non un errore di battitura.
%
%   Se un blocco non c'e' si ferma con un errore invece di andare avanti:
%   un carico rinominato e rimasto attivo e' esattamente il caso da evitare.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(mdl),   mdl   = 'phantomx_sim_attitude'; end
if nargin < 2 || isempty(stato), stato = 'on'; end
if ~bdIsLoaded(mdl), load_system(mdl); end

blocchi = {'Brick Solid', 'Brick Solid1', '6-DOF Joint1', ['Spatial' newline 'Contact Force']};

info = struct('mdl', mdl, 'stato', stato, 'blocchi', {strrep(blocchi, newline, ' ')});
for k = 1:numel(blocchi)
    b = [mdl '/' blocchi{k}];
    if getSimulinkBlockHandle(b) == -1
        error('commenta_carico:blocco', ...
            ['Il blocco "%s" non esiste in %s.\nIl carico potrebbe essere stato ' ...
             'rinominato: va ritrovato prima di misurare, non ignorato.'], ...
            strrep(blocchi{k}, newline, ' '), mdl);
    end
    set_param(b, 'Commented', stato);
end

if strcmp(stato, 'on')
    fprintf('  [commenta_carico] carico tolto da %s (4 blocchi, in memoria)\n', mdl);
else
    fprintf(2, '  [commenta_carico] carico RIMESSO in %s\n', mdl);
end
end
