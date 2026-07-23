% Trova tutti i blocchi del modello che gestiscono la visualizzazione 3D
blocchi_visual = find_system(bdroot, 'BlockType', 'SimscapeBlock');

contatore = 0;
for i = 1:length(blocchi_visual)
    % Controlliamo se il blocco ha il parametro della geometria visiva
    if isparameter(blocchi_visual{i}, 'GeometryType')
        % Forza il blocco a usare forme geometriche semplici anziché i file STL
        set_param(blocchi_visual{i}, 'GeometryType', 'GraphicPrimitive');
        contatore = contatore + 1;
    elseif isparameter(blocchi_visual{i}, 'GeometryFrom')
        % Varianti per altre versioni di MATLAB
        set_param(blocchi_visual{i}, 'GeometryFrom', 'PureGeometricPrimitive');
        contatore = contatore + 1;
    end
end

fprintf('Modificati con successo %d blocchi visivi!\n', contatore);