function n = allinea_gain_rampa(mdl, rampa_attiva)
%ALLINEA_GAIN_RAMPA  Gain di rotazione della forza di rampa nei 6 piedi, in memoria.
%
%   allinea_gain_rampa(mdl, false)   task senza rampa: Gain a 0 (elemento per elemento)
%   allinea_gain_rampa(mdl, true)    task con rampa: Gain originale 3x3 ripristinata
%
% PERCHE'
%   In ogni piede la forza della rampa passa da PS-Simulink Converter75 a una
%   Gain 3x3 (Matrix(K*u)). Nei task senza rampa applica_terreno commenta
%   Contact_rampa: il convertitore resta scollegato, Simulink ne fissa la
%   dimensione a 1 e la Gain 3x3 non compila ("Subsystem5/Gain ... dims [1]").
%   Senza rampa quella forza vale zero per costruzione, quindi una Gain a 0
%   lascia f_* = forza del pavimento, esatta. La Gain alimenta solo il blocco
%   ZMP (Force_lf..Force_rr): la dinamica non cambia.
%
%   Chiamata da applica_terreno. Nei modelli senza queste Gain (es.
%   phantomx_sim_attitude) non fa niente. NON salva il modello.
%   Stessa UserData di log_zmp ('gain_orig', 'molt_orig'): i due sono compatibili.

PIEDI = {'Subsystem','Subsystem1','Subsystem2','Subsystem3','Subsystem4','Subsystem5'};
n = 0;
for p = 1:numel(PIEDI)
    g = [mdl '/' PIEDI{p} '/Gain'];
    if getSimulinkBlockHandle(g) == -1, continue; end
    ud = get_param(g, 'UserData');
    salvato = isstruct(ud) && isfield(ud, 'gain_orig');
    if ~rampa_attiva
        if ~salvato
            set_param(g, 'UserData', struct('gain_orig', get_param(g, 'Gain'), ...
                      'molt_orig', get_param(g, 'Multiplication')));
        end
        set_param(g, 'Gain', '0', 'Multiplication', 'Element-wise(K.*u)');
        n = n + 1;
    elseif salvato
        set_param(g, 'Gain', ud.gain_orig, 'Multiplication', ud.molt_orig);
        set_param(g, 'UserData', []);
        n = n + 1;
    end
end
end
