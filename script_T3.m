%% script_T3.m - T3: traiettoria curva, imbardata costante  [impianto SIMSCAPE]
%
% COSA MISURA
%   Se il cinematico sa percorrere un arco di cerchio tenendo il corpo
%   orizzontale. Nel piano di confronto T3 e' classificato come NON
%   discriminante fra C1 e C2 su terreno piano - la retroazione sul contatto
%   non interviene se il terreno e' quello previsto - quindi serve a
%   caratterizzare C3 contro il cinematico, non C1 contro C2.
%
% PRIMA VIENE LA VERIFICA DEL SEGNO
%   L'imbardata si ottiene ruotando la direzione del passo di ciascuna zampa
%   (vedi applica_imbardata). Il segno di quella rotazione dipende dalla
%   convenzione dei giunti nell'URDF, che in questo progetto ha gia' prodotto
%   due errori di segno costati tempo. Percio' la prima cella NON e' una
%   misura: e' un controllo. Si comanda +yaw_d e si guarda il segno
%   dell'imbardata misurata. Se non torna, lo script si ferma e lo dice.
%
% ATTENZIONE AI NOMI: init_gait e' uno script, gira in questo workspace e
% sovrascrive k, j, L, cfg, alpha, side, offset, pen. Tutte le variabili sono
% prefissate t3_ per non finirci sotto.

% QUALI METRICHE VALGONO SU UNA TRAIETTORIA CURVA
%   La famiglia A e' definita per la marcia rettilinea, e su un arco di
%   cerchio tre delle sue colonne non vogliono dire niente:
%     err_vx_rms, err_vy_rms   confrontano la velocita' nel frame MONDO con
%                              un comando costante [v 0]. Su un arco la
%                              velocita' nel mondo ruota, quindi l'errore
%                              misura la curva, non il controllore.
%     dev_lat_rms, dev_lat_max scostamento dalla RETTA nominale: su un arco
%                              cresce per costruzione.
%     distanza                 e' la corda fra partenza e arrivo, non la
%                              lunghezza dell'arco: sottostima il percorso.
%   Restano valide: yaw_err_fin (metriche conosce meta.yaw_d), tutta la
%   famiglia B (planarita' del corpo), tutta la C, tutta la D.
%   Lo script aggiunge le tre grandezze giuste per un arco - lunghezza
%   dell'arco, velocita' media lungo il percorso, imbardata misurata - e
%   stampa solo le colonne che hanno senso.

t3_cfg  = phantomx_config();
t3_mdl  = 'phantomx_sim_zero';
t3_yaw  = t3_cfg.yaw_d;            % [rad/s] comando nominale
t3_dur  = 15;                      % [s]
t3_c2   = true;                   % false = C1, anello aperto. true = C2.
T3 = table();
t3_runs = {};
t3_etichette = {};

% Il terreno si fissa qui, non si eredita.
applica_terreno('T3', false, t3_mdl);
% [23/9] Inerzie corrette in memoria: vedi applica_inerzie e piano_confronto 9.
applica_inerzie(t3_mdl);

fprintf('\nT3: controllore %s, terreno T3 (piano liscio)\n', t3_nome_ctrl(t3_c2));

%% ================= 0. verifica del segno =================
fprintf('\n===== T3: verifica del segno dell''imbardata =====\n');

t3_info = applica_imbardata(t3_yaw);
OVERRIDE_C2   = t3_c2;                                         %#ok<NASGU>
OVERRIDE_GAIT = struct('S', t3_info.S);                        %#ok<NASGU>
init_gait

t3_out = sim(t3_mdl, 'StopTime', num2str(t3_dur));
t3_run = adatta_simscape(t3_out, struct( ...
             'controller', t3_nome_ctrl(t3_c2), 'task','T3', 'run',1, ...
             'condizione', sprintf('yaw%+.3f', t3_yaw), ...
             'vel_d', [t3_info.S/t3_cfg.T_stance 0], ...
             'yaw_d', t3_yaw));

% srotolando l'angolo: la differenza fra estremi e' limitata a +-pi/durata
[t3_yaw_mis, t3_rot, t3_iy] = tasso_imbardata(t3_run.t, t3_run.rpy(:,3), t3_wz(t3_run));
fprintf('  sorgente della misura: %s\n', t3_iy.sorgente);
fprintf('\n  comandata %+.4f rad/s   misurata %+.4f rad/s   rapporto %+.2f\n', ...
        t3_yaw, t3_yaw_mis, t3_yaw_mis / t3_yaw);
fprintf('  rotazione totale %+.3f rad = %+.2f giri  (attesa %+.2f giri)\n', ...
        t3_rot, t3_rot/(2*pi), t3_yaw*t3_dur/(2*pi));

t3_rapp = t3_yaw_mis / t3_yaw;

% [CORRETTO] IL MODULO VIENE PRIMA DEL SEGNO.
% Prima il controllo del segno stava per primo, e sign() non chiede un modulo
% minimo: con un'imbardata misurata attorno allo zero scatta sul segno del
% RUMORE. E' successo - misurata -0.0047 rad/s su +0.1 comandati, cioe' il 5% -
% e la diagnosi emessa era "segno invertito". Invertire il delta di tutte e sei
% le zampe non ha cambiato niente, come doveva essere: se invertire la causa
% non inverte l'effetto, quella non era la causa.
%
% Sotto il 5% il segno non e' un'informazione e non va nemmeno guardato.
%
% [CORRETTO] La soglia era al 20%, scelta a occhio venerdi', e avrebbe
% fermato la misura BUONA: in sessione pulita le curve rispondono al 12-15%
% del comando, con imbardata parassita a comando nullo di 5e-6 rad/s - cioe'
% un segnale tremila volte sopra il rumore. Il 5% separa i due casi osservati:
% 4.7% col segno a caso sul robot rotto, 12-15% riproducibile su quello sano.
%
% E' una soglia empirica e va detto. La protezione vera contro il rumore non
% e' questa riga: e' partire da una sessione pulita (vedi README) e la cella
% simmetrica qui sotto, che deve dare lo specchio della prima.
if abs(t3_rapp) < 0.05
    applica_imbardata(0);
    clear OVERRIDE_GAIT OVERRIDE_C2
    init_gait
    error('script_T3:autorita', ...
        ['Il robot NON IMBARDA: misurata %+.4f rad/s su %+.4f comandati,\n' ...
         'cioe'' il %.0f%%. Con un modulo cosi'' il segno e'' rumore, quindi\n' ...
         'non e'' un problema di verso.\n\n' ...
         'Lancia  diagnosi_imbardata  : cinque run che separano le cause,\n' ...
         'ed e'' scritta per questo caso. La domanda decisiva e'' la quarta,\n' ...
         '"destre specchiate": in inv_kyn il fattore side compare due volte,\n' ...
         'e se specchia anche la rotazione del passo i due lati si cancellano\n' ...
         'lasciando esattamente un residuo di pochi per cento.\n' ...
         'La quinta, rotazione sul posto, distingue cinematica da aderenza.\n\n' ...
         'Modello e alpha sono stati ripristinati.'], ...
         t3_yaw_mis, t3_yaw, 100*t3_rapp);

elseif sign(t3_yaw_mis) ~= sign(t3_yaw)
    applica_imbardata(0);
    clear OVERRIDE_GAIT OVERRIDE_C2
    init_gait
    error('script_T3:segno', ...
        ['L''imbardata misurata ha il segno OPPOSTO al comando, con modulo\n' ...
         'significativo (%.0f%%): questa volta il verso e'' davvero invertito.\n\n' ...
         'Si corregge in applica_imbardata, nel DEFAULT dell''opzione segno,\n' ...
         'non nella formula: la formula da'' il valore geometrico e l''opzione\n' ...
         'porta quello misurato. Due posti che decidono lo stesso segno sono\n' ...
         'un posto di troppo.\n' ...
         'Non toccare i sei Constant: sono sei copie della stessa scelta.\n\n' ...
         'Modello e alpha sono stati ripristinati.'], 100*abs(t3_rapp));

elseif abs(t3_rapp) < 0.5
    fprintf(2, ['\n  ATTENZIONE: imbardata misurata al %.0f%% di quella comandata.\n' ...
                '  Il segno e'' giusto ma il modulo no. Cause probabili, in ordine:\n' ...
                '    1. scivolamento: guarda slip_tot nella riga di metriche;\n' ...
                '    2. i piedi non tengono il tripode: guarda appoggio_medio;\n' ...
                '    3. l''approssimazione sul modulo del passo (%.1f%% di\n' ...
                '       dispersione fra le zampe) e'' troppo grossa per questo\n' ...
                '       raggio di curvatura: prova un yaw_d piu'' piccolo.\n'], ...
            100*abs(t3_rapp), 100*t3_info.errore_modulo);
end

T3 = [T3; t3_riga_arco(t3_run, t3_cfg, t3_yaw, t3_yaw_mis, t3_info)];
t3_runs{end+1} = t3_run;
t3_etichette{end+1} = sprintf('%+.3f rad/s', t3_yaw);

%% ================= 1. la cella simmetrica =================
% Comandare -yaw_d e verificare che il comportamento sia lo specchio. Se le
% due curve non sono simmetriche, c'e' un'asimmetria fra zampe destre e
% sinistre - il parametro "side" di inv_kyn, o il montaggio delle mesh - e
% va trovata prima di mettere T3 in relazione.
fprintf('\n===== T3: cella simmetrica =====\n');

t3_info_m = applica_imbardata(-t3_yaw);
OVERRIDE_C2   = t3_c2;                                         %#ok<NASGU>
OVERRIDE_GAIT = struct('S', t3_info_m.S);                      %#ok<NASGU>
init_gait

t3_out_m = sim(t3_mdl, 'StopTime', num2str(t3_dur));
t3_run_m = adatta_simscape(t3_out_m, struct( ...
               'controller', t3_nome_ctrl(t3_c2), 'task','T3', 'run',1, ...
               'condizione', sprintf('yaw%+.3f', -t3_yaw), ...
               'vel_d', [t3_info_m.S/t3_cfg.T_stance 0], ...
               'yaw_d', -t3_yaw));

[t3_yaw_mis_m, t3_rot_m] = tasso_imbardata(t3_run_m.t, t3_run_m.rpy(:,3), t3_wz(t3_run_m));

T3 = [T3; t3_riga_arco(t3_run_m, t3_cfg, -t3_yaw, t3_yaw_mis_m, t3_info_m)];
t3_runs{end+1} = t3_run_m;
t3_etichette{end+1} = sprintf('%+.3f rad/s', -t3_yaw);

t3_asimm = abs(abs(t3_yaw_mis_m) - abs(t3_yaw_mis)) / max(abs(t3_yaw_mis), eps);
fprintf('\n  +yaw %+.4f   -yaw %+.4f   asimmetria %.1f%%\n', ...
        t3_yaw_mis, t3_yaw_mis_m, 100*t3_asimm);
if t3_asimm > 0.15
    fprintf(2, ['  Asimmetria oltre il 15%%: le zampe destre e sinistre non si\n' ...
                '  comportano allo stesso modo. Da guardare prima di usare T3.\n']);
end

%% ================= ripristino =================
applica_imbardata(0);
clear OVERRIDE_GAIT OVERRIDE_C2
init_gait

%% ================= risultati =================
% Il nome contiene il controllore: 'T3_risultati.csv' era lo stesso file per
% C1 e C2, quindi la seconda campagna cancellava la prima in silenzio.
if ~isfolder('results'), mkdir('results'); end
t3_file = fullfile('results', sprintf('T3_%s.csv', t3_nome_ctrl(t3_c2)));
writetable(T3, t3_file);
fprintf('\nscritto  %s\n', t3_file);

% Solo le colonne che hanno senso su un arco: vedi la nota in testa al file.
disp(T3(:, {'condizione','yaw_cmd','yaw_mis','yaw_rapporto','arco', ...
            'vel_arco','yaw_err_fin','roll_rms','pitch_rms', ...
            'slip_tot','appoggio_medio','disp_carico','cot','successo'}))

fprintf(['\nNON guardare err_vx_rms, err_vy_rms, dev_lat_* e distanza su T3:\n' ...
         'sono definite per la marcia rettilinea. Stanno nel CSV per\n' ...
         'uniformita'' di tabella, non perche'' significhino qualcosa qui.\n']);

fprintf('\nNOTA per la relazione: la direzione del passo per zampa e'' esatta,\n');
fprintf('il modulo e'' condiviso e sbaglia del %.1f%% fra zampa interna ed\n', ...
        100*t3_info.errore_modulo);
fprintf('esterna alla curva. L''errore si scarica in scivolamento:\n');
if isnan(T3.slip_tot(1))
    fprintf('  slip_tot = NaN, adatta_simscape non ha fornito pf o il contatto.\n');
    fprintf('  Senza quella colonna l''approssimazione non e'' misurata: da\n');
    fprintf('  sistemare prima di mettere T3 in relazione.\n\n');
else
    fprintf('  %.1f mm su %.2f m di arco percorso (atteso ~%.1f mm/passo).\n\n', ...
            1e3*T3.slip_tot(1), T3.arco(1), 1e3*t3_info.slip_atteso);
end

figure; hold on; grid on; axis equal
for t3_j = 1:numel(t3_runs)
    plot(t3_runs{t3_j}.p(:,1), t3_runs{t3_j}.p(:,2));
end
legend(t3_etichette, 'Location','best');
xlabel('x [m]'); ylabel('y [m]'); title('T3 - traiettoria del CoM nel piano')

%% ================= helper =================
function wz = t3_wz(r)
%T3_WZ  Terza componente della velocita' angolare, se c'e'.
wz = [];
if isfield(r,'w') && ~isempty(r.w) && size(r.w,2) >= 3
    wz = r.w(:,3);
end
end

function s = t3_nome_ctrl(c2)
if c2, s = 'C2'; else, s = 'C1'; end
end

function riga = t3_riga_arco(r, cfg, yaw_cmd, yaw_mis, info)
%T3_RIGA_ARCO  La riga di metriche piu' le grandezze giuste per un arco.
%
%   Su una traiettoria curva la "distanza" di metriche e' la CORDA fra
%   partenza e arrivo, e la velocita' media ne deriva: entrambe sottostimano
%   il percorso, tanto piu' quanto piu' la curva e' chiusa. Qui si aggiunge
%   la lunghezza dell'ARCO, integrata sul cammino, e la velocita' media
%   lungo di esso.

riga = metriche(r, cfg, struct('t_regime', 2*cfg.T));

sel  = r.t >= 2*cfg.T;
p    = r.p(sel, 1:2);
arco = sum(vecnorm(diff(p), 2, 2));
i0   = find(sel, 1, 'first');
dt   = r.t(end) - r.t(i0);

riga.yaw_cmd      = yaw_cmd;
riga.yaw_mis      = yaw_mis;
riga.yaw_rapporto = yaw_mis / yaw_cmd;
riga.arco         = arco;
riga.vel_arco     = arco / max(dt, eps);
% corda sulla STESSA finestra dell'arco: calcolata sulla run intera
% includeva il transitorio e poteva superare l'arco, che e' impossibile
riga.corda        = norm(r.p(end,1:2) - r.p(i0,1:2));
riga.curvatura    = 1 - riga.corda / max(arco, eps);   % 0 = retta
riga.S_usato      = info.S;
riga.disp_moduli  = info.errore_modulo;
end
