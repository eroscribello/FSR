function startup_phantomx()
% Prepara il path MATLAB per il progetto.
%
% Da lanciare una volta a ogni sessione, dalla root del progetto o da qualsiasi
% posto (si orienta da sola sulla posizione di questo file).
%
% Aggiunge:
%   common/      parametri + cinematica condivisi
%   simscape/    i modelli Simscape
%   metriche/    conversione delle run e metriche
%
% Poi verifica che di ogni file critico esista una sola copia visibile:


root = fileparts(mfilename('fullpath'));

daAggiungere = {
    'common'
    'simscape'
    'metriche'
};

fprintf('\n=== PATH PHANTOMX ===\n');
fprintf('Root: %s\n\n', root);

for k = 1:numel(daAggiungere)
    p = fullfile(root, daAggiungere{k});
    if isfolder(p)
        addpath(p);
        fprintf('  + %s\n', daAggiungere{k});
    else
        fprintf(2,'  ! manca  %s\n', daAggiungere{k});
    end
end

%% ---- guardia sui doppioni ----
critici = {'phantomx_config','inv_kyn','tripod_trajectory', ...
           'adatta_simscape','metriche','scegli_controllore'};

fprintf('\n--- controllo doppioni ---\n');
problemi = 0;
for k = 1:numel(critici)
    copie = which(critici{k}, '-all');
    copie = copie(~contains(copie,'_cestino'));

    if numel(copie) == 2 && any(contains(copie,'.mex'))
        continue
    end
    if numel(copie) > 1
        problemi = problemi + 1;
        fprintf(2,'  %s: %d copie visibili\n', critici{k}, numel(copie));
        for j = 1:numel(copie)
            fprintf('      %s\n', copie{j});
        end
    end
end

if problemi == 0
    fprintf('  ok, nessun doppione.\n');
else
    fprintf(2,['\n  %d file esistono in piu'' copie. MATLAB ne usa una sola\n' ...
               '  (la prima del path) e tu potresti stare modificando l''altra.\n' ...
               '  Risolvi prima di lanciare qualsiasi simulazione.\n'], problemi);
end


end