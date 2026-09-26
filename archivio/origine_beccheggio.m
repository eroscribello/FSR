function B = origine_beccheggio(mdl)
%ORIGINE_BECCHEGGIO  Il beccheggio in marcia rettilinea e' oscillazione o offset?
%
%   B = origine_beccheggio
%
% IL FATTO DA SPIEGARE
%   In marcia rettilinea il corpo ha pitch_rms = 0.071 rad, cioe' 4.1 gradi,
%   contro 0.003-0.005 rad (0.2 gradi) quando il robot ruota sul posto. Un
%   fattore 18. La planarita' del corpo e' l'obiettivo dichiarato del paper e
%   tutta la famiglia B delle metriche, quindi 4 gradi in rettilineo e' un
%   numero che riguarda T1 e T2, non solo T3.
%
% ATTENZIONE: metriche.m fa l'RMS dell'ANGOLO, non della sua deviazione
%   B.pitch_rms = rms(rpy(:,2)), quindi un offset STATICO di 4 gradi e un
%   dondolio di +-5.8 gradi a media nulla danno lo stesso numero. Sono due
%   difetti diversi con due cause diverse:
%
%     OFFSET   il corpo sta inclinato e ci resta. Causa tipica: la posa di
%              equilibrio non e' quella geometrica - e' il debito noto su
%              body_z0 - oppure il carico non e' distribuito simmetricamente
%              fra zampe anteriori e posteriori.
%
%     DONDOLIO il corpo oscilla al ritmo del passo. Causa tipica: il passo
%              avanti-indietro, che sposta il punto di applicazione del
%              carico dentro il poligono di appoggio a ogni mezzo ciclo. E'
%              coerente con il fatto che ruotando sul posto - passo
%              puramente tangenziale - il beccheggio quasi scompare.
%
%   Questa funzione separa le due componenti e dice quale domina. Non e' una
%   curiosita': se e' offset si corregge con la posa, se e' dondolio si
%   corregge con l'andatura, e sono due interventi diversi.
%
% PRIMA
%   abilita_log('on')   per avere anche il carico per zampa
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1 || isempty(mdl), mdl = 'phantomx_sim_zero'; end

cfg = phantomx_config();

assignin('base','OVERRIDE_C2',   false);
assignin('base','OVERRIDE_GAIT', struct());
evalin('base','init_gait');
applica_terreno('T1', false, mdl);

out = sim(mdl, 'StopTime', num2str(12));
r   = adatta_simscape(out, struct('controller','C1', 'task','T1', 'run',1, ...
                                  'condizione','nominale', ...
                                  'vel_d',[cfg.v_nom 0]));

evalin('base','clear OVERRIDE_GAIT OVERRIDE_C2');

sel = r.t >= 2*cfg.T;
t   = r.t(sel);
roll  = r.rpy(sel,1);
pitch = r.rpy(sel,2);

serie = struct('roll', roll, 'pitch', pitch);

B = struct();
for nm = {'roll','pitch'}
    x = serie.(nm{1});
    m = mean(x);
    osc = x - m;

    s = struct();
    s.rms_totale   = sqrt(mean(x.^2));       % quello che riporta metriche
    s.offset       = m;
    s.rms_oscill   = sqrt(mean(osc.^2));
    s.picco_oscill = max(abs(osc));
    % quanto dell'RMS totale e' offset: rms^2 = offset^2 + rms_oscill^2
    s.quota_offset = m^2 / max(s.rms_totale^2, eps);

    % periodo dominante dell'oscillazione, confrontato col ciclo di andatura
    dt = median(diff(t));
    n  = numel(osc);
    if n > 32
        Y = abs(fft(osc - mean(osc)));
        Y(1) = 0;
        f  = (0:n-1)/(n*dt);
        met = 1:floor(n/2);
        [~, ip] = max(Y(met));
        s.freq_dom    = f(ip);
        s.periodo_dom = 1/max(f(ip), eps);
    else
        s.freq_dom = NaN;  s.periodo_dom = NaN;
    end
    B.(nm{1}) = s;
end

%% ---- lettura ----
fprintf('\n========== ORIGINE DEL BECCHEGGIO E DEL ROLLIO ==========\n');
fprintf('  periodo del ciclo di andatura: %.3f s   (mezzo ciclo %.3f s)\n\n', ...
        cfg.T, cfg.T/2);

fprintf('  %-8s %10s %10s %10s %10s %9s %9s\n', ...
        '', 'rms tot', 'offset', 'rms oscil', 'picco', 'T dom', '% offset');
for nm = {'roll','pitch'}
    s = B.(nm{1});
    fprintf('  %-8s %10.4f %10.4f %10.4f %10.4f %9.3f %8.0f%%\n', ...
            nm{1}, s.rms_totale, s.offset, s.rms_oscill, s.picco_oscill, ...
            s.periodo_dom, 100*s.quota_offset);
end
fprintf('\n  (radianti; 0.0175 rad = 1 grado)\n');

fprintf('\n--- verdetto ---\n');
for nm = {'roll','pitch'}
    s = B.(nm{1});
    fprintf('\n%s: ', upper(nm{1}));
    if s.quota_offset > 0.6
        fprintf(['dominato dall''OFFSET (%.0f%% dell''RMS). Il corpo sta\n' ...
                 '  inclinato di %.2f gradi e ci resta. Si corregge sulla POSA,\n' ...
                 '  non sull''andatura: guarda il debito noto su body_z0, e se il\n' ...
                 '  carico e'' distribuito in modo asimmetrico fra anteriori e\n' ...
                 '  posteriori (colonna disp_carico).\n'], ...
                100*s.quota_offset, rad2deg(s.offset));
    elseif s.quota_offset < 0.25
        fprintf(['dominato dall''OSCILLAZIONE (offset solo %.0f%%).\n' ...
                 '  Ampiezza di picco %.2f gradi, periodo dominante %.3f s.\n'], ...
                100*s.quota_offset, rad2deg(s.picco_oscill), s.periodo_dom);
        rap = s.periodo_dom / cfg.T;
        if abs(rap - 0.5) < 0.15
            fprintf(['  E'' MEZZO ciclo di andatura: e'' il cambio di tripode.\n' ...
                     '  Coerente con il fatto che ruotando sul posto il\n' ...
                     '  beccheggio quasi scompare: la'' il passo e'' tangenziale\n' ...
                     '  e non sposta il carico avanti e indietro.\n']);
        elseif abs(rap - 1) < 0.2
            fprintf('  E'' UN ciclo intero di andatura.\n');
        else
            fprintf(['  NON e'' legato al ciclo di andatura (rapporto %.2f):\n' ...
                     '  e'' un modo proprio del sistema corpo-contatto, non\n' ...
                     '  l''andatura. Guarda cfg.contact.k e lo smorzamento.\n'], rap);
        end
    else
        fprintf(['offset e oscillazione dello stesso ordine (offset %.0f%%).\n' ...
                 '  Vanno separati prima di correggere: due cause insieme.\n'], ...
                100*s.quota_offset);
    end
end

figure
subplot(2,1,1); hold on; grid on
plot(t, rad2deg(roll));  plot(t, rad2deg(pitch));
yline(rad2deg(B.roll.offset),  '--');
yline(rad2deg(B.pitch.offset), '--');
legend('roll','pitch','offset roll','offset pitch','Location','best');
xlabel('t [s]'); ylabel('[deg]'); title('Assetto del corpo, marcia rettilinea')

subplot(2,1,2); hold on; grid on
plot(t, rad2deg(pitch - B.pitch.offset));
for kk = 0:floor(t(end)/(cfg.T/2))
    xline(2*cfg.T + kk*cfg.T/2, ':');
end
xlabel('t [s]'); ylabel('beccheggio - offset [deg]');
title('Oscillazione contro i cambi di tripode (linee a mezzo ciclo)')
fprintf('\n');

end