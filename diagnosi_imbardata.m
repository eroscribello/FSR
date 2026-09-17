function D = diagnosi_imbardata(mdl)
%DIAGNOSI_IMBARDATA  Perche' l'imbardata misurata e' il 3% di quella comandata.
%
%   D = diagnosi_imbardata
%
% IL FATTO DA SPIEGARE
%   Comandando +-0.1 rad/s, script_T3 ha misurato +0.0027 e -0.0055 rad/s:
%   il SEGNO e' giusto - entrambe le celle concordano col comando - ma il
%   modulo e' fra il 3% e il 5%. Il robot cammina quasi dritto.
%
% CINQUE RUN CHE SEPARANO LE CAUSE
%   Ogni riga esclude qualcosa. Non sono cinque tentativi: sono cinque
%   domande diverse.
%
%   1. rettilineo          riferimento. Serve per sapere quanta imbardata
%                          "parassita" c'e' anche senza comandarla: se a
%                          yaw = 0 il robot gia' ruota di 0.003 rad/s, le
%                          celle di T3 non stanno misurando niente.
%
%   2. arco 0.1            la cella di adesso, ripetuta per confronto.
%
%   3. arco 0.4            LINEARITA'. Se l'imbardata misurata quadruplica,
%                          il comando funziona ma e' attenuato da un fattore
%                          costante, e a 0.1 era semplicemente sotto la
%                          soglia di aderenza. Se resta al 3%, l'attenuazione
%                          e' proporzionale e la causa e' strutturale.
%
%   4. destre specchiate   IL SOSPETTO PRINCIPALE. In inv_kyn la componente
%                          tangenziale e' moltiplicata per "side", e il
%                          fattore compare DUE volte (su y_loc e su theta).
%                          Se il specchiamento tocca anche la rotazione del
%                          passo, il delta delle tre zampe destre agisce al
%                          contrario e i due lati si cancellano: il residuo
%                          che resta e' proprio un pochi-per-cento. Questa
%                          run inverte il delta solo a destra: se
%                          l'imbardata salta al valore atteso, la causa e'
%                          quella e la correzione e' una riga.
%
%   5. rotazione sul posto CASO ESATTO. Con v = 0 tutti i sei moduli valgono
%                          omega*r_offset, quindi il passo condiviso non e'
%                          piu' un'approssimazione. omega e' scelto per dare
%                          lo STESSO passo e la stessa velocita' di piede
%                          della marcia nominale (omega = v_nom/r_offset),
%                          cosi' l'aderenza e' nelle stesse condizioni.
%                          Se questa gira e l'arco no, il problema e'
%                          l'approssimazione sul modulo. Se non gira nemmeno
%                          questa, non e' cinematica: e' aderenza.
%
% PRIMA
%   abilita_log('on')   altrimenti appoggio_medio e slip_tot restano NaN e
%                       due delle cinque righe non si possono leggere.
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(mdl), mdl = 'phantomx_sim_zero'; end

cfg   = phantomx_config();
v_nom = cfg.S / cfg.T_stance;
w_spin = v_nom / cfg.r_offset;      % stesso passo della marcia nominale
dur   = 15;

casi = {
%   etichetta              yaw            opzioni
    'rettilineo'           0              struct()
    'arco 0.1'             0.1            struct()
    'arco 0.4'             0.4            struct()
    'destre specchiate'    0.1            struct('specchia',true)
    'sul posto'            w_spin         struct('v',0)
    };

fprintf('\n============== DIAGNOSI IMBARDATA ==============\n');
fprintf('  v nominale %.4f m/s   r_offset %.3f m\n', v_nom, cfg.r_offset);
fprintf('  omega per la rotazione sul posto: %.4f rad/s\n', w_spin);
fprintf('  durata %g s, controllore C1\n\n', dur);

D = table();

for k = 1:size(casi,1)
    etich = casi{k,1};
    w     = casi{k,2};
    o     = casi{k,3};

    fprintf('[%d/%d] %-20s yaw = %+.4f ... ', k, size(casi,1), etich, w);

    try
        if w == 0 && ~isfield(o,'v')
            info = applica_imbardata(0, false, mdl);
            S_use = cfg.S;
        else
            info = applica_imbardata(w, false, mdl, o);
            S_use = info.S;
        end

        assignin('base','OVERRIDE_C2',   false);
        assignin('base','OVERRIDE_GAIT', struct('S', S_use));
        evalin('base','init_gait');

        out = sim(mdl, 'StopTime', num2str(dur));
        r = adatta_simscape(out, struct( ...
                'controller','C1', 'task','T3diag', 'run',1, ...
                'condizione', etich, ...
                'vel_d', [max(S_use/cfg.T_stance, eps) 0], ...
                'yaw_d', w));

        sel = r.t >= 2*cfg.T;
        p   = r.p(sel,1:2);
        dt  = r.t(end) - r.t(find(sel,1,'first'));
        wz = [];
        if isfield(r,'w') && ~isempty(r.w) && size(r.w,2) >= 3, wz = r.w(:,3); end
        [w_mis, rot_tot] = tasso_imbardata(r.t, r.rpy(:,3), wz);

        % arco e corda sulla STESSA finestra, altrimenti non sono
        % confrontabili: la corda calcolata sulla run intera includeva il
        % transitorio e poteva risultare MAGGIORE dell'arco, che e'
        % geometricamente impossibile.
        i0 = find(sel,1,'first');
        riga = table(string(etich), w, w_mis, ...
                     w_mis / sign0(w), ...
                     sum(vecnorm(diff(p),2,2)), ...
                     norm(r.p(end,1:2) - r.p(i0,1:2)), rot_tot/(2*pi), ...
                     S_use, ...
                     max(abs(info.delta_deg)), ...   % delta_deg e' GIA' in gradi
                     'VariableNames', {'caso','yaw_cmd','yaw_mis','rapporto', ...
                                       'arco','corda','giri','S','delta_max_deg'});

        m = metriche(r, cfg, struct('t_regime', 2*cfg.T));
        riga.appoggio_medio = m.appoggio_medio;
        riga.slip_tot       = m.slip_tot;
        riga.roll_rms       = m.roll_rms;
        riga.pitch_rms      = m.pitch_rms;

        D = [D; riga];                                                %#ok<AGROW>
        fprintf('misurata %+.5f  (%.1f%%)  arco %.2f m  piedi %.2f\n', ...
                w_mis, 100*abs(riga.rapporto), riga.arco, riga.appoggio_medio);

    catch ME
        fprintf(2,'ERRORE: %s\n', ME.message);
    end
end

%% ---- ripristino ----
applica_imbardata(0, false, mdl);
evalin('base','clear OVERRIDE_GAIT OVERRIDE_C2');
evalin('base','init_gait');

if height(D) == 0, return; end

%% ---- lettura ----
fprintf('\n');
disp(D)

fprintf('\n=============== COME SI LEGGE ===============\n');

i_ret = trova(D,'rettilineo');
i_a1  = trova(D,'arco 0.1');
i_a4  = trova(D,'arco 0.4');
i_spec= trova(D,'destre specchiate');
i_pos = trova(D,'sul posto');

if ~isnan(i_ret)
    fprintf('\n1. Imbardata parassita a comando nullo: %+.5f rad/s\n', D.yaw_mis(i_ret));
    if abs(D.yaw_mis(i_ret)) > 0.3*abs(D.yaw_mis(max(i_a1,1)))
        fprintf(2,['   E'' dello stesso ordine di quella misurata a 0.1 rad/s.\n' ...
                   '   Le celle di T3 non stanno misurando l''imbardata comandata\n' ...
                   '   ma il rumore del robot: prima va capito perche'' deriva.\n']);
    else
        fprintf('   Trascurabile rispetto alle celle comandate: il confronto regge.\n');
    end
end

if ~isnan(i_a1) && ~isnan(i_a4)
    sc = (D.yaw_mis(i_a4)/D.yaw_cmd(i_a4)) / (D.yaw_mis(i_a1)/D.yaw_cmd(i_a1));
    fprintf('\n2. Linearita'': il rapporto passa da %.1f%% a %.1f%% (fattore %.2f)\n', ...
            100*abs(D.yaw_mis(i_a1)/D.yaw_cmd(i_a1)), ...
            100*abs(D.yaw_mis(i_a4)/D.yaw_cmd(i_a4)), sc);
    if sc > 1.5
        fprintf('   Cresce: a 0.1 il comando era sotto la soglia di aderenza.\n');
        fprintf('   T3 va rifatto a un''imbardata piu'' alta.\n');
    else
        fprintf('   Costante: l''attenuazione e'' proporzionale, non una soglia.\n');
    end
end

if ~isnan(i_spec) && ~isnan(i_a1)
    g = abs(D.yaw_mis(i_spec)) / max(abs(D.yaw_mis(i_a1)), eps);
    fprintf('\n3. Destre specchiate: %+.5f contro %+.5f  (%.1f volte)\n', ...
            D.yaw_mis(i_spec), D.yaw_mis(i_a1), g);
    if g > 3
        fprintf(['   TROVATO. Il parametro "side" di inv_kyn specchia anche la\n' ...
                 '   rotazione del passo: i contributi dei due lati si\n' ...
                 '   cancellavano. Correzione in applica_imbardata: rendere\n' ...
                 '   opt.specchia = true il comportamento di DEFAULT, cioe''\n' ...
                 '   delta(i) = -delta(i) per le zampe con alpha < 0.\n']);
    else
        fprintf('   Non e'' quello: i due lati non si cancellano.\n');
    end
end

if ~isnan(i_pos)
    fprintf('\n4. Rotazione sul posto (caso esatto): %+.5f su %+.5f comandati (%.1f%%)\n', ...
            D.yaw_mis(i_pos), D.yaw_cmd(i_pos), 100*abs(D.rapporto(i_pos)));
    if abs(D.rapporto(i_pos)) > 0.5
        fprintf(['   Gira. La cinematica dell''imbardata e'' corretta, e il\n' ...
                 '   problema dell''arco e'' l''approssimazione sul modulo del\n' ...
                 '   passo. Si risolve dando a ogni zampa il suo S, che richiede\n' ...
                 '   sei blocchi di traiettoria invece di due.\n']);
    else
        fprintf(2,['   Non gira nemmeno il caso esatto: non e'' cinematica.\n' ...
                   '   Guarda appoggio_medio e slip_tot di questa riga - se i\n' ...
                   '   piedi strisciano, il passo tangenziale non fa ruotare\n' ...
                   '   niente, e il problema e'' l''aderenza, non l''imbardata.\n']);
    end
end
fprintf('\n');

end

%% ================================================================
function s = sign0(w)
if w == 0, s = NaN; else, s = w; end
end

function i = trova(D, etich)
i = find(D.caso == etich, 1);
if isempty(i), i = NaN; end
end
