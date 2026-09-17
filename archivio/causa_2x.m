function D = causa_2x(opt)
%CAUSA_2X  Perche' a 2x il robot cammina indietro. Misura sulle finestre COMANDATE.
%
%   D = causa_2x
%   D = causa_2x(struct('fattori',[1.0 1.25 1.5 1.75 2.0]))
%
% DOVE SIAMO, E PERCHE' LA PRIMA VERSIONE ERA CONFUSA
%   La v1 misurava lo spostamento del piede per SEGMENTO DI CONTATTO OSSERVATO.
%   Quella quantita' e' confondente: se ad alta velocita' l'appoggio si
%   frammenta, i segmenti si accorciano e lo spostamento per segmento cala
%   ANCHE SE lo scivolamento non cala. E' esattamente quello che ha dato:
%
%      1.0x  slip +12.49 mm   v 0.140 (attesa 0.145)   coerente al 3%
%      1.5x  slip  +6.12 mm   v 0.149 (attesa 0.198)   sbagliata del 33%
%      2.0x  slip  +2.19 mm   v -0.112 (attesa 0.249)  segno opposto
%
%   L'identita' Dcorpo = Dpiede_mondo - Dpiede_corpo non e' un modello da
%   verificare, e' algebra: p_piede = p_corpo + R*p_piede_corpo. Se non torna,
%   uno dei due termini e' MISURATO male. Il sospetto e' il mio, e il dato che
%   lo sostiene c'e' gia': a 2x appoggio_medio valeva 2.12, sotto 3, quindi i
%   piedi si staccano quando dovrebbero spingere.
%
% COSA CAMBIA QUI
%   1. Le finestre sono quelle COMANDATE (run.contact_sched), non quelle
%      osservate: la frammentazione del contatto non le accorcia, e diventa
%      essa stessa una misura (frazione della finestra in cui il piede tocca
%      davvero).
%   2. Si misurano SEPARATAMENTE i due termini dell'identita':
%        sweep_corpo  = Dx del piede NEL FRAME CORPO   (atteso -S)
%        slip_mondo   = Dx del piede nel frame MONDO
%      La loro differenza DEVE dare l'avanzamento del corpo. Se non lo da',
%      la misura e' rotta e lo dice invece di concludere.
%   3. Si misura se il robot e' in volo: frazione di tempo con meno di tre
%      piedi a terra. Un esapode a tripode che va sotto tre non cammina, e
%      nessuna contabilita' di scivolamento descrive cosa gli succede.
%
% LE TRE IPOTESI, E COSA DECIDONO IN RELAZIONE
%   A  lo sweep comandato NON viene realizzato (sweep_corpo molto < S):
%      i giunti non inseguono. Limite del BANCO o della catena di comando:
%      la cella 2x va esclusa, non attribuita al cinematico.
%   B  lo sweep e' realizzato ma il piede scivola indietro:
%      limite del CONTROLLORE, il tripode in posizione e' sovra-vincolato.
%      Il risultato a 2x e' valido ed e' il punto piu' forte contro C1.
%   C  il robot perde l'appoggio (meno di tre piedi, contatto frammentato):
%      il modello quasi-statico non e' piu' applicabile. Limite del
%      controllore, ma di natura diversa da B, e va scritto diversamente:
%      non "scivola", ma "non regge l'andatura".
%
% Progetto FSR PhantomX - A. Russo

if nargin < 1, opt = struct(); end
cfg = phantomx_config();

def = struct('fattori', [1.0 1.5 2.0], ...
             'nCicli',  10, ...
             'mdl',     'phantomx_sim_zero', ...
             'verbose', true);
f = fieldnames(def);
for k = 1:numel(f)
    if ~isfield(opt,f{k}), opt.(f{k}) = def.(f{k}); end
end

fprintf('\n========== CAUSA DEL FALLIMENTO A 2x ==========\n');
fprintf('  velocita'': %s   (%d cicli ciascuna, C1 in anello aperto)\n', ...
        mat2str(opt.fattori,3), opt.nCicli);
fprintf('  passo nominale S = %.1f mm\n', 1e3*cfg.S);

% Il terreno va PINNATO: una campagna e' gia' girata sul gradino di T5 senza
% che nulla nei risultati lo dicesse.
applica_terreno('T2', false, opt.mdl);

zampe = {'rf','rm','rr','lf','lm','lr'};
D = table();  G = table();

for iF = 1:numel(opt.fattori)
    fatt  = opt.fattori(iF);
    v_cmd = fatt * cfg.v_nom;
    T_i   = cfg.S / (cfg.beta_stance * v_cmd);
    T_st  = cfg.beta_stance * T_i;
    stop  = opt.nCicli * T_i;

    fprintf('\n--- %.2fx :  T = %.3f s,  appoggio comandato = %.3f s ---\n', ...
            fatt, T_i, T_st);

    try
        assignin('base','OVERRIDE_C2', false);
        assignin('base','OVERRIDE_GAIT', struct('T',T_i, 'H',cfg.H));
        evalin('base','init_gait');
        out = sim(opt.mdl, 'StopTime', num2str(stop));
        r = adatta_simscape(out, struct('task','T2', 'run',1, ...
                'condizione', sprintf('v%.2fx', fatt), ...
                'vel_d', [v_cmd 0]), struct('verbose',false));
    catch ME
        fprintf(2,'  la run e'' fallita: %s\n', ME.message);
        continue
    end

    t = r.t;
    if ~isfield(r,'pf') || isempty(r.pf)
        fprintf(2,'  senza pf non si misura nulla: serve robotModel (init_gait)\n');
        continue
    end
    pf  = r.pf;
    pb  = r.p;
    rpy = r.rpy;
    reg = t >= 2*T_i;                  % si scarta il transitorio
    i0  = find(reg, 1);

    % ---- i piedi nel frame del corpo: p_corpo = R' * (p_piede - p_corpo) ----
    % pf e' costruito come p + R*vb, quindi questa e' l'inversa esatta, non
    % un'approssimazione a piccoli angoli.
    % rotZYX e' la STESSA matrice che adatta_simscape usa per costruire pf: e'
    % replicata qui sotto invece di usare rotz/roty/rotx (che dipendono da un
    % toolbox) o eul2rotm (convenzione diversa). Se le due matrici non
    % coincidessero, l'identita' non chiuderebbe per un difetto mio.
    fb = nan(size(pf));
    for n = 1:numel(t)
        R = rotZYX(rpy(n,:));
        for i = 1:6
            c = 3*(i-1) + (1:3);
            fb(n,c) = (R.' * (pf(n,c).' - pb(n,:).')).';
        end
    end

    % ---- contatto osservato e comandato ----
    if isfield(r,'contact') && ~isempty(r.contact)
        giu = logical(r.contact);
    elseif isfield(r,'Fc') && ~isempty(r.Fc)
        giu = r.Fc(:, 3:3:18) > 0.5;
    else
        giu = false(numel(t), 6);
    end
    if isfield(r,'contact_sched') && ~isempty(r.contact_sched)
        cmd = logical(r.contact_sched);
        fonte = 'comandate (contact_sched)';
    else
        cmd = giu;
        fonte = 'OSSERVATE (contact_sched assente: misura confondibile)';
    end
    fprintf('  finestre %s\n', fonte);

    % ---- volo: quanti piedi a terra ----
    nGiu    = sum(giu(reg,:), 2);
    piediMed = mean(nGiu);
    sotto3   = mean(nGiu < 3);
    sotto1   = mean(nGiu < 1);

    % ---- avanzamento del corpo ----
    avanz   = pb(end,1) - pb(i0,1);
    durata  = t(end) - t(i0);
    v_reale = avanz / durata;

    fprintf('  corpo: %+.1f mm in %.2f s -> %+.4f m/s (comando %+.4f, rapporto %+.2f)\n', ...
            1e3*avanz, durata, v_reale, v_cmd, v_reale/v_cmd);
    fprintf('  piedi a terra: %.2f in media; sotto 3 il %.0f%% del tempo, in volo il %.0f%%\n', ...
            piediMed, 100*sotto3, 100*sotto1);

    % ---- per zampa, sulle finestre comandate ----
    for iz = 1:6
        cx = 3*(iz-1) + 1;
        [m, nFin] = perFinestra(pf(:,cx), fb(:,cx), giu(:,iz), cmd(:,iz), t, reg);

        riga = table(fatt, string(zampe{iz}), T_i, T_st, nFin, ...
                     1e3*m.sweep, 1e3*m.slip, 1e3*m.dcorpo, ...
                     m.frazione_toccata, m.frammenti, ...
            'VariableNames', {'fattore','zampa','T','T_stance','finestre', ...
                              'sweep_mm','slip_mm','dcorpo_mm', ...
                              'frazione_toccata','frammenti'});
        D = [D; riga];                                                %#ok<AGROW>
    end

    s = D(D.fattore == fatt, :);
    % L'avanzamento previsto dall'identita', da confrontare col misurato:
    % due appoggi per ciclo nel tripode.
    dc_medio = mean(s.dcorpo_mm, 'omitnan');
    v_prev   = 2 * dc_medio*1e-3 / T_i;

    fprintf('  sweep realizzato %+.2f mm (atteso %+.1f)   slip %+.2f mm\n', ...
            mean(s.sweep_mm,'omitnan'), -1e3*cfg.S, mean(s.slip_mm,'omitnan'));
    fprintf('  finestra comandata toccata al %.0f%%, %.1f frammenti per finestra\n', ...
            100*mean(s.frazione_toccata,'omitnan'), mean(s.frammenti,'omitnan'));
    fprintf('  identita'': v prevista %+.4f, misurata %+.4f\n', v_prev, v_reale);

    G = [G; table(fatt, T_i, v_cmd, v_reale, v_reale/v_cmd, v_prev, ...
            mean(s.sweep_mm,'omitnan'), mean(s.slip_mm,'omitnan'), ...
            100*mean(s.frazione_toccata,'omitnan'), ...
            mean(s.frammenti,'omitnan'), piediMed, 100*sotto3, ...
        'VariableNames', {'fattore','T','v_cmd','v_reale','rapporto','v_prevista', ...
                          'sweep_mm','slip_mm','toccata_pct','frammenti', ...
                          'piedi_giu','sotto3_pct'})];                %#ok<AGROW>
end

evalin('base','clear OVERRIDE_GAIT OVERRIDE_C2');
evalin('base','init_gait');

if height(G) == 0
    fprintf(2,'\nnessuna run completata\n');
    return
end

fprintf('\n=============== QUADRO ===============\n');
disp(G)

%% ---- il verdetto ----
fprintf('\n=============== VERDETTO ===============\n');
S_mm = 1e3*cfg.S;
fine = G(end,:);
prima = G(1,:);

% Prima di tutto: l'identita' regge? Se no, nessuna conclusione fisica vale.
scartoId = abs(fine.v_prevista - fine.v_reale) / max(abs(fine.v_reale), 1e-6);
if scartoId > 0.25
    fprintf(2,'  LA MISURA NON CHIUDE, E QUESTO VIENE PRIMA DI OGNI IPOTESI.\n');
    fprintf([ ...
      '  A %.2fx l''identita'' prevede %+.4f m/s e il corpo ne fa %+.4f: uno\n' ...
      '  scarto del %.0f%%. Dcorpo = Dpiede_mondo - Dpiede_corpo e'' algebra,\n' ...
      '  non un modello, quindi lo scarto e'' un difetto di misura, non un\n' ...
      '  risultato.\n\n' ...
      '  La causa piu'' probabile e'' che il corpo si muova FUORI dalle\n' ...
      '  finestre di appoggio, cioe'' in volo: qui i piedi a terra sono %.2f\n' ...
      '  in media e sotto tre il %.0f%% del tempo. Con meno di tre piedi\n' ...
      '  l''esapode non e'' quasi-statico e la contabilita'' dello scivolamento\n' ...
      '  non lo descrive.\n\n' ...
      '  In relazione va scritto cosi'': a %.2fx il robot NON CAMMINA, perde\n' ...
      '  l''appoggio. Non "scivola" e non "inverte": sono affermazioni\n' ...
      '  diverse, e solo questa e'' sostenuta.\n'], ...
      fine.fattore, fine.v_prevista, fine.v_reale, 100*scartoId, ...
      fine.piedi_giu, fine.sotto3_pct, fine.fattore);
    stampaGrafici(G, S_mm, cfg, opt);
    return
end

sweepMancato = abs(fine.sweep_mm) < 0.7 * S_mm;
slipIndietro = fine.slip_mm < -0.3 * S_mm;
perdeAppoggio = fine.toccata_pct < 70 || fine.sotto3_pct > 30;

% [CORRETTO] La FASE viene prima dello sweep.
% L'identita' Dcorpo = slip - sweep e' vera punto per punto, quindi chiude per
% QUALUNQUE finestra: valida il cambio di frame, non l'aggancio della fase.
% contact_sched e' agganciato massimizzando l'accordo col contatto osservato;
% se la finestra comandata e' toccata poco, quell'aggancio ha poco su cui
% appoggiarsi, e una finestra che straddia la transizione appoggio/volo somma
% un pezzo di -S e un pezzo di +S dando sweep ~ 0. Quel valore e' allora
% indistinguibile da "i giunti non inseguono", e concluderlo sarebbe un
% artefatto. E' successo: la v2 aveva dato IPOTESI A con toccata al 37%.
if fine.toccata_pct < 70 && sweepMancato
    fprintf(2,'  LA FASE NON E'' AFFIDABILE, E QUESTO VIENE PRIMA DELLO SWEEP.\n');
    fprintf([ ...
      '  A %.2fx la finestra comandata e'' toccata solo al %.0f%%, e sweep\n' ...
      '  vale %+.2f mm su %+.1f attesi. Con una fase agganciata cosi'' male,\n' ...
      '  sweep ~ 0 e'' quello che si ottiene se la finestra straddia la\n' ...
      '  transizione appoggio/volo: somma un pezzo di -S e un pezzo di +S.\n\n' ...
      '  Quindi NON si puo'' concludere che i giunti non inseguono: la misura\n' ...
      '  non distingue le due cose. Serve una misura indipendente dalla\n' ...
      '  fase.\n\n' ...
      '  Lancia  inseguimento  : misura l''escursione picco-picco del piede\n' ...
      '  nel frame corpo, che vale S qualunque sia la fase, e legge il modo\n' ...
      '  di attuazione dei giunti (se e'' in moto, un ritardo e'' impossibile\n' ...
      '  per costruzione).\n\n' ...
      '  Quello che si puo'' dire da qui: la finestra e'' toccata al %.0f%%, i\n' ...
      '  piedi a terra sono %.2f in media e sotto tre il %.0f%% del tempo.\n' ...
      '  Il robot perde l''appoggio. Se inseguimento conferma che i giunti\n' ...
      '  seguono, questa e'' la causa e va scritta come "non regge\n' ...
      '  l''andatura", non come "scivola".\n'], ...
      fine.fattore, fine.toccata_pct, fine.sweep_mm, -S_mm, ...
      fine.toccata_pct, fine.piedi_giu, fine.sotto3_pct);
    stampaGrafici(G, S_mm, cfg, opt);
    return
end

if sweepMancato
    fprintf(2,'  IPOTESI A: LO SWEEP COMANDATO NON VIENE REALIZZATO.\n');
    fprintf([ ...
      '  A %.2fx il piede percorre %+.2f mm nel frame del corpo invece di\n' ...
      '  %+.1f: il %.0f%% del comando. I giunti non inseguono, e la corsa di\n' ...
      '  appoggio parte e finisce dove non dovrebbe.\n\n' ...
      '  In relazione: LIMITE DELLA CATENA DI COMANDO, non del controllore\n' ...
      '  cinematico. La cella 2x va esclusa o rifatta, e NON va attribuita a\n' ...
      '  C1. Prossimo passo: trovare dove si perde il comando - saturazione\n' ...
      '  di coppia dei giunti, o tempo di campionamento del generatore.\n'], ...
      fine.fattore, fine.sweep_mm, -S_mm, 100*abs(fine.sweep_mm)/S_mm);
elseif slipIndietro
    fprintf(2,'  IPOTESI B: INVERSIONE DELLO SCIVOLAMENTO.\n');
    fprintf([ ...
      '  Lo sweep e'' realizzato (%+.2f mm su %+.1f attesi) e il piede\n' ...
      '  scivola INDIETRO di %.2f mm, oltre S in modulo. Dcorpo = slip + S\n' ...
      '  da'' quindi un corpo che va indietro.\n\n' ...
      '  In relazione: LIMITE DEL CONTROLLORE. Il tripode comandato in\n' ...
      '  posizione e'' sovra-vincolato e alle velocita'' alte i piedi cedono\n' ...
      '  invece di spingere. Il risultato a 2x e'' valido e va riportato.\n'], ...
      fine.sweep_mm, -S_mm, abs(fine.slip_mm));
elseif perdeAppoggio
    fprintf(2,'  IPOTESI C: PERDITA DI APPOGGIO.\n');
    fprintf([ ...
      '  Lo sweep e'' realizzato e lo scivolamento non si inverte, ma la\n' ...
      '  finestra comandata e'' toccata solo al %.0f%% e i piedi a terra sono\n' ...
      '  %.2f in media (sotto tre il %.0f%% del tempo).\n\n' ...
      '  In relazione: limite del controllore, ma DIVERSO da "scivola". A\n' ...
      '  %.2fx il tripode non regge l''andatura: il piede tocca a intermittenza\n' ...
      '  e l''appoggio non c''e'' quando serve. Va scritto con queste parole.\n'], ...
      fine.toccata_pct, fine.piedi_giu, fine.sotto3_pct, fine.fattore);
else
    fprintf('  NESSUNA DELLE TRE IN MODO NETTO.\n');
    fprintf([ ...
      '  sweep %+.2f su %+.1f, slip %+.2f, finestra toccata %.0f%%, piedi\n' ...
      '  %.2f. Nessuna soglia superata, e l''identita'' chiude: allora il\n' ...
      '  movimento indietro nasce da qualcosa che queste misure non\n' ...
      '  coprono, e il candidato resta il beccheggio - una rotazione del\n' ...
      '  corpo sposta i piedi senza che nessuno scivoli.\n'], ...
      fine.sweep_mm, -S_mm, fine.slip_mm, fine.toccata_pct, fine.piedi_giu);
end

fprintf('\n  per riferimento, a %.2fx: sweep %+.2f, slip %+.2f, toccata %.0f%%\n\n', ...
        prima.fattore, prima.sweep_mm, prima.slip_mm, prima.toccata_pct);

stampaGrafici(G, S_mm, cfg, opt);

end

%% ================================================================
function [m, nFin] = perFinestra(xw, xb, giu, cmd, t, reg)
%PERFINESTRA  Le tre quantita' dell'identita', sulle finestre COMANDATE.
%
%   xw  x del piede nel mondo      xb  x del piede nel frame corpo
%   giu contatto osservato         cmd finestra comandata
%
%   Si usano le finestre comandate perche' quelle osservate si accorciano
%   quando il contatto si frammenta, e lo spostamento per finestra cala senza
%   che lo scivolamento cali: e' il difetto che ha reso illeggibile la v1.
cmd = logical(cmd(:)) & reg(:);
ini = find(diff([false; cmd]) ==  1);
fin = find(diff([cmd; false]) == -1);
n = min(numel(ini), numel(fin));
nFin = n;

sw = nan(n,1);  sl = nan(n,1);  fr = nan(n,1);  nf = nan(n,1);
for k = 1:n
    a = ini(k);  b = fin(k);
    if b <= a, continue; end
    sw(k) = xb(b) - xb(a);        % sweep realizzato nel frame corpo (atteso -S)
    sl(k) = xw(b) - xw(a);        % scivolamento nel mondo
    g = giu(a:b);
    fr(k) = mean(g);              % quanto della finestra e' davvero toccata
    nf(k) = sum(diff([false; g(:)]) == 1);   % frammenti di contatto
end

m.sweep            = mean(sw, 'omitnan');
m.slip             = mean(sl, 'omitnan');
m.dcorpo           = m.slip - m.sweep;       % l'identita'
m.frazione_toccata = mean(fr, 'omitnan');
m.frammenti        = mean(nf, 'omitnan');
if isnan(m.sweep), m.sweep = NaN; m.slip = NaN; m.dcorpo = NaN; end
end

function R = rotZYX(a)
%ROTZYX  Copia esatta della rotazione usata da adatta_simscape per costruire pf.
cr = cos(a(1)); sr = sin(a(1));
cp = cos(a(2)); sp = sin(a(2));
cy = cos(a(3)); sy = sin(a(3));
R = [ cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr ; ...
      sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr ; ...
        -sp,            cp*sr,            cp*cr ];
end

function stampaGrafici(G, S_mm, cfg, opt)
if ~opt.verbose, return; end
figure;
subplot(3,1,1); hold on; grid on
plot(G.fattore, G.sweep_mm, 'o-', 'DisplayName','sweep nel frame corpo');
plot(G.fattore, G.slip_mm,  's-', 'DisplayName','scivolamento nel mondo');
yline(-S_mm,'r:','DisplayName','-S (sweep atteso)');  yline(0,'k--','HandleVisibility','off');
ylabel('[mm]'); legend('Location','best');
title('I due termini dell''identita'' Dcorpo = slip - sweep');

subplot(3,1,2); hold on; grid on
plot(G.fattore, G.v_reale,    'o-', 'DisplayName','misurata');
plot(G.fattore, G.v_prevista, 'x--','DisplayName','prevista dall''identita''');
plot(G.fattore, G.v_cmd,      'k:', 'DisplayName','comandata');
yline(0,'k--','HandleVisibility','off');
ylabel('velocita'' [m/s]'); legend('Location','best');
title('Se le due curve divergono, la misura non chiude');

subplot(3,1,3); hold on; grid on
plot(G.fattore, G.toccata_pct, 'o-', 'DisplayName','finestra toccata [%]');
plot(G.fattore, G.sotto3_pct,  's-', 'DisplayName','tempo sotto 3 piedi [%]');
yline(100,'k:','HandleVisibility','off');
xlabel('velocita'' [x]'); ylabel('[%]'); legend('Location','best');
title('Perdita di appoggio');
end
