function startup_phantomx()
%STARTUP_PHANTOMX  Prepara il path MATLAB per il progetto FSR PhantomX.
%
% Lanciala una volta a ogni sessione, dalla root del progetto o da qualsiasi
% posto (si orienta da sola sulla posizione di questo file).
%
% Aggiunge:
%   common/                        parametri + cinematica condivisi
%   simscape/                      baseline Simscape
%   mpc_srb/ + fcns/ + fcns_MPC/   MPC su simulatore ridotto
%   third_party/qpSWIFT/           solver (il mex, non il prototipo)
%
% NON aggiunge:
%   third_party/qpSWIFT/prototype/ per non far ombra al mex
%   phantomx_description-master/   pacchetto ROS, ci arrivano solo le mesh
%                                  referenziate dal modello
%   _cestino/
%
% Poi verifica che di ogni file critico esista UNA SOLA copia visibile:
% e' l'errore che ci e' gia' costato quattro run di taratura identiche.
%
% Progetto FSR PhantomX - A. Russo

root = fileparts(mfilename('fullpath'));

daAggiungere = {
    'common'
    'simscape'
    'mpc_srb'
    fullfile('mpc_srb','fcns')
    fullfile('mpc_srb','fcns','plot')
    fullfile('mpc_srb','fcns_MPC')
    fullfile('mpc_srb','third_party','qpSWIFT')
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

% il prototipo del solver resta fuori dal path: se serve, si aggiunge a mano
proto = fullfile(root,'mpc_srb','third_party','qpSWIFT','prototype');
if isfolder(proto)
    fprintf('\n  (prototype/ NON sul path. Se ti serve il solver in puro\n');
    fprintf('   MATLAB:  addpath(''%s'') )\n', strrep(proto,'\','\\'));
end

%% ---- guardia sui doppioni ----
critici = {'phantomx_config','inv_kyn','tripod_trajectory', ...
           'get_params','MAIN','dynamics_SRB','fcn_FSM','vec','hatMap','qpSWIFT'};

fprintf('\n--- controllo doppioni ---\n');
problemi = 0;
for k = 1:numel(critici)
    copie = which(critici{k}, '-all');
    copie = copie(~contains(copie,'_cestino'));
    % un mex e il suo file di help omonimo convivono senza problemi:
    % MATLAB da' precedenza al mex
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

%% ---- promemoria sulla riproducibilita' ----
fprintf(['\n--- promemoria ---\n' ...
         '  fcn_FSM usa variabili persistent: prima di OGNI run fai\n' ...
         '      clear fcn_FSM\n' ...
         '  altrimenti la run n+1 riparte dallo stato lasciato dalla run n\n' ...
         '  e le 5 ripetizioni per cella non sono confrontabili.\n\n']);

end