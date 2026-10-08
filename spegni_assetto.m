function info = spegni_assetto(stato, mdl)
%SPEGNI_ASSETTO  Azzera (o ripristina) i guadagni dell'anello d'assetto.
%
%   spegni_assetto              % guadagni a zero: il modello si comporta da C2
%   spegni_assetto('off')       % li rimette com'erano
%   info = spegni_assetto(...)
%
% PERCHE' NON SI COMMENTA IL BLOCCO
%   Il delta_z nasce in 'MATLAB Function4' e arriva, via Demux9 e sei Goto,
%   a sei blocchi Sum che lo aggiungono alla z nominale prima della
%   cinematica inversa. Commentare la funzione lascia quelle sei somme con
%   un ingresso scollegato: Simulink non compila. Azzerando P, I e D dei due
%   PID l'uscita e' identicamente nulla, le somme aggiungono zero e la
%   topologia non cambia.
%
% COSA RESTA ACCESO
%   La ricerca del terreno. Il modello d'assetto legge c2_par (Constant10 e
%   Constant11), quindi con OVERRIDE_C2 = true si ottiene C2 con il carico,
%   non C1 con il carico.
%
% PERCHE' I VALORI ORIGINALI VANNO SU FILE E NON IN UNA 'persistent'
%   'clear all' cancella le variabili persistent ma NON ricarica i modelli
%   gia' aperti: i guadagni resterebbero a zero senza piu' nessuno in grado
%   di rimetterli, e C3 girerebbe come C2 senza dirlo. Il file sopravvive
%   anche alla chiusura di MATLAB.
%
% NON SALVA IL MODELLO.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(stato), stato = 'on';  end
if nargin < 2 || isempty(mdl),   mdl   = 'phantomx_sim_attitude'; end
stato = lower(char(stato));
if ~ismember(stato, {'on','off'})
    error('spegni_assetto:stato', 'Lo stato e'' ''on'' o ''off'', non ''%s''.', stato);
end
if ~bdIsLoaded(mdl), load_system(mdl); end

blocchi  = {'PID Controller', 'PID Controller1'};   % beccheggio, rollio
guadagni = {'P', 'I', 'D'};
dest     = fullfile('results', 'diagnostica');
f_bk     = fullfile(dest, sprintf('gains_assetto_%s.mat', mdl));

for k = 1:numel(blocchi)
    if getSimulinkBlockHandle([mdl '/' blocchi{k}]) == -1
        error('spegni_assetto:blocco', ...
            ['Il blocco "%s" non esiste in %s.\nL''anello d''assetto potrebbe ' ...
             'essere stato rinominato: va ritrovato prima di misurare, non ignorato.'], ...
            blocchi{k}, mdl);
    end
end

% lettura dello stato attuale, serve a entrambi i rami
ora = cell(numel(blocchi), numel(guadagni));
for k = 1:numel(blocchi)
    for g = 1:numel(guadagni)
        ora{k,g} = get_param([mdl '/' blocchi{k}], guadagni{g});
    end
end
spento_ora = all(cellfun(@(s) isequal(str2double(s), 0), ora(:)));

switch stato
    case 'on'
        if spento_ora
            fprintf(2, '  [spegni_assetto] gia'' a zero: non sovrascrivo il backup\n');
        else
            if ~isfolder(dest), mkdir(dest); end
            VAL = ora;  BLOCCHI = blocchi;  GUADAGNI = guadagni;  MDL = mdl; %#ok<NASGU>
            save(f_bk, 'VAL', 'BLOCCHI', 'GUADAGNI', 'MDL');
            fprintf('  [spegni_assetto] guadagni originali salvati in %s\n', f_bk);
        end
        for k = 1:numel(blocchi)
            for g = 1:numel(guadagni)
                set_param([mdl '/' blocchi{k}], guadagni{g}, '0');
            end
        end
        fprintf(2, ['  [spegni_assetto] anello d''assetto SPENTO in %s.\n' ...
                    '  Le run lanciate adesso sono C2 con il carico, non C3.\n'], mdl);

    case 'off'
        if ~spento_ora
            % gia' acceso: niente da fare, e nessun messaggio allarmante
            info = spegni_assetto_info(mdl, stato, blocchi, guadagni, ora);  return
        end
        if ~isfile(f_bk)
            error('spegni_assetto:backup', ...
                ['I guadagni di %s sono a zero ma %s non esiste: non so a che\n' ...
                 'valore riportarli. Vanno riletti dal modello originale (git) o\n' ...
                 'reinseriti a mano nei due blocchi PID prima di rifare C3.'], mdl, f_bk);
        end
        S = load(f_bk);
        for k = 1:numel(S.BLOCCHI)
            for g = 1:numel(S.GUADAGNI)
                set_param([S.MDL '/' S.BLOCCHI{k}], S.GUADAGNI{g}, S.VAL{k,g});
            end
        end
        fprintf('  [spegni_assetto] guadagni ripristinati in %s da %s\n', S.MDL, f_bk);
end

info = spegni_assetto_info(mdl, stato, blocchi, guadagni, ora);
end

function info = spegni_assetto_info(mdl, stato, blocchi, guadagni, ora)
info = struct('mdl', mdl, 'stato', stato, 'blocchi', {blocchi}, ...
              'guadagni', {guadagni}, 'prima', {ora});
end
