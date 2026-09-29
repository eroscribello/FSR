# PhantomX — MPC convesso vs controllo cinematico

Progetto finale di **Field and Service Robotics** (Università Federico II, prof. Fabio Ruggiero).

Porting a esapode dell'MPC convesso *representation-free* di Ding et al. e confronto
sistematico con il controllore cinematico di Arrigoni et al. (*Robotics* 2024, 13, 142),
sulla piattaforma PhantomX AX Metal Hexapod Mark II.

---

## Requisiti

| | |
|---|---|
| MATLAB | R2021b o successivo |
| Toolbox | Simulink, Simscape, Simscape Multibody |
| Solver QP | qpSWIFT, incluso in `mpc_srb/third_party/` (mex precompilati per Windows, Linux, macOS Intel e Apple Silicon) |
| Sistema | testato su Windows 11 |

Nessuna installazione: i mex sono già compilati. Se la tua piattaforma non è coperta,
vedi `mpc_srb/third_party/qpSWIFT/Swift_make.m`.

---

## Avvio rapido

**Da fare a ogni sessione di MATLAB**, prima di qualsiasi altra cosa:

```matlab
cd <cartella del repository>
startup_phantomx
```

`startup_phantomx` aggiunge le cartelle al path, verifica che non esistano copie
duplicate dei file critici e stampa i promemoria. **Senza questo comando niente
funziona**, e gli errori che ottieni (`Unrecognized function 'qpSWIFT'`,
`Unrecognized function 'inv_kyn'`) non dicono che il problema è il path.

Poi, a seconda di cosa vuoi lanciare:

### Baseline Simscape (controllore cinematico)

```matlab
init_gait                          % carica i parametri nel base workspace
applica_terreno('T1')              % sceglie il terreno dello scenario
out = sim('phantomx_sim_zero','StopTime','10');
```

`applica_terreno` accende e spegne gli elementi del terreno **senza salvare il
modello**: vedi "Scenari di terreno".

### MPC su simulatore ridotto

```matlab
clear fcn_FSM                      % obbligatorio, vedi "Regole" sotto
MAIN
```

---

## Campagne di misura

I risultati vanno in `results/`, un CSV per task e controllore:
`results/T2_C1.csv`, `results/T2_C2.csv`, `results/T3_C1.csv`. Sono tracciati
da git — sono il risultato, non un artefatto — mentre le run grezze (`.mat`) e
le figure di lavoro no.

**Ogni campagna parte da una sessione pulita:**

```matlab
clear all; bdclose all; startup_phantomx
```

`clear all`, non `clear`: toglie anche le variabili persistenti. `init_gait`
legge dal base workspace `OVERRIDE_GAIT`, `OVERRIDE_C2`, `FORZA_STATICO`, e
quello che resta lì da run precedenti cambia il comportamento senza lasciare
traccia nel codice. È successo: una batteria di misure d'imbardata è stata
presa su un robot con mezzo piede a terra e 4° di beccheggio, e buttata. Se il
prompt è `K>>`, MATLAB è fermo in debug: prima `dbquit`.

Dal 29/9 gli script dei task leggono anche **`OVERRIDE_CTRL`**, che a differenza
delle altre **non si autocancella**: vedi sotto.

### Scegliere il controllore

Ogni `script_T*.m` ha in testa **una riga**, e da lì discendono modello,
interruttore della ricerca del terreno e nome del file dei risultati:

```matlab
t5_ctrl      = 'C2';       % 'C1' | 'C2' | 'C3'
[t5_mdl, t5_c2] = scegli_controllore(t5_ctrl);
```

`scegli_controllore.m` è **l'unico posto dove sta scritto chi gira su cosa**:

| nome | modello | `OVERRIDE_C2` | cos'è |
|---|---|---|---|
| `C1` | `phantomx_sim_zero` | `false` | cinematico ad anello aperto, tripode |
| `C2` | `phantomx_sim_zero` | `true` | C1 + ricerca del terreno (Arrigoni 2024) |
| `C3` | `phantomx_sim_attitude` | `true` | C2 + retroazione di beccheggio, **in serie** |

Aggiungere un controllore significa aggiungere una riga a quella tabella, e
nient'altro. Un nome sconosciuto è un errore immediato, non un CSV con
l'etichetta sbagliata.

> **[29/9] Perché non è più un booleano.** Era `t5_c2 = true/false`, e il nome
> del file usciva da un secondo posto. Con due controllori funzionava; con tre
> non falliva con un errore, falliva **sovrascrivendo `results/T5_C2.csv` con
> una run di C3**. Passare un booleano a `scegli_controllore` ora è un errore
> esplicito, apposta: uno script non convertito deve rompersi, non funzionare
> a metà.

`C3` tiene la ricerca del terreno (`OVERRIDE_C2 = true`) perché l'assetto sta
**in serie, dopo** di essa — verificato leggendo l'XML dentro il `.slx`:
`ricerca_terreno → z_* → Sum (+ d_z_*) → Saturation → Rate Limiter → gamba`.
Così ogni controllore aggiunge **un solo meccanismo** al precedente e ogni
confronto fra due adiacenti ne misura uno solo.

**Per una campagna su più task** senza aprire i file:

```matlab
clear all; bdclose all; startup_phantomx
OVERRIDE_CTRL = 'C3';
script_T2; script_T3; script_T5; script_T6
clear OVERRIDE_CTRL
```

`OVERRIDE_CTRL` **non viene cancellata dagli script**: se lo facesse andrebbe
riscritta prima di ogni task, che è il problema che risolve. In cambio ogni run
che la usa lo dichiara a schermo in rosso. In `script_T4`, `OVERRIDE_T4` ha
l'ultima parola — è mirata a quel task — così `script_T4_limite` non viene
dirottato da una globale dimenticata nel workspace.

**Prima di misurare C3**, `verifica_attitude` controlla che il modello regga la
catena di misura (log di Simscape, To Workspace, solidi, stimatore, `c2_par`,
`InitFcn`). Solo letture, nessun `save_system`.

### T2 — curva di velocità

```matlab
clear all; bdclose all; startup_phantomx
script_T2
```

Cinque celle, dieci cicli ciascuna, terreno T2 fissato dallo script. Dura
qualche minuto.

**Per cambiare controllore** si modifica una riga sola, in testa a
`script_T2.m`:

```matlab
t2_c2 = false;     % C1, anello aperto
t2_c2 = true;      % C2, con ricerca del terreno
```

Il nome del file segue il controllore, quindi le due campagne non si
sovrascrivono. L'interruttore vero è `c2_par(1)`, che `init_gait` assembla da
`cfg.c2.attiva`: **non** è la soglia di coppia, vedi
`docs/come_funziona_ricerca_terreno.md`.

**Le cinque velocità e cosa sono** (`cfg.t2_fattori`):

| cella | ruolo |
|---|---|
| `0.5x` | bassa velocità |
| `1.0x` | nominale |
| `1.20x` | **ultima cella con moto del corpo stabile** (`cfg.t2_limite`) — non «limite di C1», vedi sotto |
| `1.5x` | fuori inviluppo — il robot saltella |
| `2.0x` | fuori inviluppo — il robot indietreggia |

Le ultime due **falliscono per costruzione**: `frazione_task` negativa a 2× non
è un errore della campagna, è il risultato. Vanno lette con `sotto3_frac` e
`corpoZ_pp`, non con `err_vx_rms`: a quelle velocità i giunti eseguono la corsa
comandata (escursione del piede 118% di `S` a tutte le velocità), quindi non è
un errore di inseguimento — è il robot che non cammina.

**Perché 1.20× non si chiama «limite».** Il criterio aveva una condizione sul
tripode (`sotto3_frac < 5%`) tarata sulle forze *ricostruite*. Con i sensori,
che sono la fonte attuale, la stessa soglia boccia anche il nominale (11.7% a
1.0×), quindi non si trasferisce. Resta il criterio sul moto del corpo, che non
dipende dalla fonte delle forze: rimbalzo 3.4 mm a 1.20×, 9.4 a 1.30×, 19.7 a
1.40×. In relazione vanno riportati entrambi gli inviluppi — dettagli in
`common/phantomx_config.m`, sezione T2.

**Le colonne che dicono *perché* una cella fallisce** (C1, forze dai sensori):

| colonna | cosa misura | 1.0× | 2.0× |
|---|---|---|---|
| `sotto3_frac` | frazione di tempo con meno di tre piedi a terra | 0.117 | 0.536 |
| `corpoZ_pp` | rimbalzo verticale del corpo, picco-picco per ciclo | 3.2 mm | 81 mm |
| `corpoZ_vz` | rms della velocità verticale del corpo | 0.024 m/s | 0.463 m/s |

`corpoZ_pp` e `corpoZ_vz` vanno letti insieme: a 0.5× e a 1.5× il rimbalzo è
lo stesso (29 e 33 mm), ma a 0.5× il corpo dondola (0.067 m/s) e a 1.5× sbatte
(0.200 m/s). L'ampiezza da sola non distingue i due cedimenti.

**Fonte delle forze.** Dal 21/9 `adatta_simscape` legge le forze dai sensori del
modello (`Fleg`), validati allo 0.0% sul peso, e li usa da solo quando li trova.
Prima le ricostruiva dalla penetrazione. La colonna `note` di ogni riga dice
quale fonte è stata usata: CSV con fonti diverse **non** si confrontano sulle
colonne di contatto (`appoggio_medio`, `sotto3_frac`, `slip_tot`,
`disp_carico`). Le colonne di moto invece sì: sono identiche bit per bit.

`appoggio_medio` da solo non basta e non va usato per questo: una media di 3.0
vale sia per «tre piedi sempre a terra» sia per «sei e zero alternati», che
sono andature completamente diverse.

### T3 — traiettoria curva

```matlab
script_T3
```

**La prima cella non è una misura, è un controllo del segno.** Se l'imbardata
misurata ha segno opposto al comando lo script si ferma e dice quale riga di
`applica_imbardata` cambiare. Non correggere i sei `Constant` a mano: il segno
è uno solo.

Il numero da guardare è `yaw_rapporto`. La misura viene da `run.w(:,3)`, la
velocità angolare dal log; l'angolo srotolato resta come controllo incrociato e
`tasso_imbardata` avvisa se le due stime divergono oltre il 20%. Se la riga
`sorgente della misura` dice `angolo srotolato`, `run.w` non è arrivato e il
numero va guardato con sospetto — l'angolo si avvolge a ±π e su una run da 15 s
non può restituire niente fuori da ±0.209 rad/s.

`script_T3` chiama `applica_imbardata`, che fa `set_param` sui sei `Constant`
di `alpha`. **Non salva il modello, ma lo sporca**, e ripristina con
`applica_imbardata(0)` solo se arriva alla fine. Se si interrompe a metà,
lanciare `applica_imbardata(0)` a mano prima di fare qualunque cosa con il
`.slx`.

Su T3 **non** guardare `err_vx_rms`, `err_vy_rms`, `dev_lat_*` e `distanza`:
sono definite per la marcia rettilinea e su un arco misurano la curva, non il
controllore. Stanno nel CSV per uniformità di tabella.

---

## Scenari di terreno

Il terreno di ogni task si sceglie da script, prima di simulare. Il `.slx` **non
viene salvato**: il file sul disco resta identico, quindi si può lavorare in due
su task diversi senza passarsi il modello.

```matlab
init_gait
applica_terreno('T5')
out = sim('phantomx_sim_zero','StopTime','10');
```

| chiamata | terreno |
|---|---|
| `applica_terreno('T1')` | piano liscio |
| `applica_terreno('T2')` | piano liscio, tre velocità differenti |
| `applica_terreno('T3')` | piano liscio + traiettoria curva, imbardata costante||
| `applica_terreno('T4')` | piano liscio + rampa inclinata |
| `applica_terreno('T5')` | piano liscio + un ostacolo |
| `applica_terreno('T6')` | terreno imperfetto + tutti gli ostacoli |
| `applica_terreno('T7')` | piano liscio + disturbo impulsivo laterale |
| `applica_terreno('TUTTO')` | tutto acceso |

La definizione dei task è in `docs/piano_confronto.md` §6; qui c'è solo il terreno
che ciascuno richiede.

**T2 non è una singola run**: si espande nelle celle di `cfg.t2_fattori`
(`[0.5 0.75 1.0 1.5 2.0]`), una per velocità. La griglia sta **solo** lì, e la
leggono entrambi i runner — `script_T2` per l'impianto Simscape e
`esegui_misure` per quello ridotto. La variabile indipendente è la **velocità
comandata**: il periodo si ricava da `T = S/(beta_stance·v)`, così la lunghezza
del passo resta quella validata.

### Come funziona

Ogni elemento del terreno è una terna già presente nel modello: **solido +
trasformazione + un blocco di contatto in ciascuno dei sei piedi**. La funzione
commenta e scommenta solidi e contatti con `set_param`.

La corrispondenza *etichetta → blocco di contatto* viene **letta dal modello a ogni
chiamata**, piede per piede, seguendo il percorso della geometria:

```
Spatial Contact Force, porta B -> Connection Port n del piede
-> porta n del Subsystem al livello di sopra -> Connection Label
-> etichetta -> solido
```

Non è dedotta dai numeri dei blocchi, perché quei numeri **non seguono** quelli
delle etichette (`File Solid1` è servito da `Force6`, non da `Force2`). Se il
cablaggio cambia, il codice non va aggiornato; se diventa incoerente, la funzione
si ferma invece di simulare un modello sbagliato.

### Diagnostica

```matlab
mappa_contatti      % chi cerca quale etichetta e chi la fornisce
quota_terreno       % quota delle superfici e base di ogni ostacolo, dagli STL
```

### Limiti noti degli scenari

- **T4**: il cablaggio della rampa è a posto — prende in prestito lo slot
  `ostacolo7`, perché nei piedi nessun contatto cerca l'etichetta `rampa` e non le
  si può dare `pavimento` (Simscape ammette una sola geometria per linea). Resta
  sbagliata la **geometria**: il Brick della rampa è 4×4 m e a 8° compenetra il
  pavimento.
- **T5**: su pavimento liscio sono utilizzabili solo `ost1`, `ost2`, `ost3`.
  Gli altri quattro poggiano sui **rilievi** del terreno imperfetto (da +35 a
  +71 mm) e sul piano liscio restano in aria.

---

## Metriche

Una run del baseline Simscape non è direttamente leggibile da `metriche.m`:
`adatta_simscape` converte l'uscita di `sim` nella struttura che `metriche` si
aspetta, e produce la riga con tutte e cinque le famiglie (A esecuzione,
B planarità, C attuazione, D contatto, E robustezza).

```matlab
init_gait
applica_terreno('T1')
out = sim('phantomx_sim_zero','StopTime','10');
run = adatta_simscape(out);
riga = metriche(run);
```

`adatta_simscape` **si rifiuta di produrre una riga** se il log di Simscape non
copre tutta la simulazione, e blocca la conversione se gli angoli di giunto
superano i limiti meccanici (è la spia degli angoli letti in gradi invece che in
radianti — vedi "Regole").

Verifiche di supporto:

```matlab
complanarita        % quota assoluta dei sei piedi lungo il passo
verifica_marcia     % andatura a regime
verifica_ik         % escursione di una zampa
```

---

## Struttura

| cartella | contenuto |
|---|---|
| `common/` | `phantomx_config.m` (parametri condivisi), `inv_kyn.m` (IK di gamba), `tripod_trajectory.m` |
| `simscape/` | modello Simscape Multibody, il suo script di inizializzazione e `props/` (STL del terreno) |
| `mpc_srb/` | MPC convesso sul modello a corpo rigido singolo, più `fcns/`, `fcns_MPC/` e il solver |
| `phantomx_description-master/` | pacchetto ROS originale: mesh STL e URDF. **Non modificare** |
| `docs/` | `piano_confronto.md` (controllori, metriche, task, calendario, debiti), `README.md` (porting dell'MPC da quadrupede a esapode), paper di riferimento |
| `grafici/` | figure per la relazione |

Utility nella radice:

| gruppo | file |
|---|---|
| controllori | `scegli_controllore` (nome → modello + `OVERRIDE_C2`), `allinea_stimatore`, `verifica_attitude` |
| terreno | `applica_terreno`, `quota_terreno`, `mappa_contatti` |
| metriche | `adatta_simscape`, `metriche`, `complanarita`, `verifica_marcia`, `verifica_ik` |
| modello | `setup_modello`, `setup_terreno_param`, `audit_mesh`, `fix_mesh_paths`, `trova_nel_modello` |
| diagnostica | `valida_soglia`, `soglia_ottima`, `fattibilita`, `estrai_funzioni` |
| varie | `pulizia`, `pulizia2` |

---

## I due simulatori

| | baseline Simscape | simulatore ridotto |
|---|---|---|
| modello | multibody completo, 25 corpi | corpo rigido singolo (SRB) |
| attuazione | giunti in **posizione** | forze di contatto ottimizzate |
| contatto | modello a penalità, attrito | vincoli di cono d'attrito nel QP |
| entry point | `phantomx_sim_zero.slx` | `MAIN.m` |
| parametri | `init_gait.m` → `phantomx_config` | `get_params.m` → `phantomx_config` |

Entrambi leggono **`common/phantomx_config.m`**. È l'unico posto dove si modificano
masse, geometria, tempi dell'andatura, attrito e limiti degli attuatori.

### I modelli Simscape

Un simulatore, più `.slx` in `simscape/`. Non si scelgono a mano: li dà
`scegli_controllore`.

| file | a cosa serve |
|---|---|
| `phantomx_sim_zero.slx` | l'impianto di riferimento, C1 e C2 |
| `phantomx_sim_attitude.slx` | C3: lo stesso impianto più la retroazione di beccheggio |
| `phantomx_sim_mpc.slx` | ponte verso l'MPC, non usato nella campagna |

Gli altri `.slx` della cartella sono varianti di lavoro e backup datati.

> **[29/9]** `phantomx_sim_zero` è arrivato una volta sul remoto con sei blocchi
> `From` (`d_z_*`) **senza i `Goto` corrispondenti**, residuo della costruzione
> del modello d'assetto: non compilava. Un `.slx` è binario e un diff non lo
> mostra, ma è uno zip e l'XML dentro si legge. Se un modello smette di
> compilare dopo un `pull`, `git log -- simscape/<file>.slx` e
> `git checkout <sha> -- simscape/<file>.slx` sono la via più corta.

---

## Regole

Sono i tranelli che ci sono già costati tempo. Valgono per chiunque lavori al repo.

| regola | perché |
|---|---|
| **`startup_phantomx` a ogni sessione** | senza, il path non c'è e gli errori non lo dicono |
| **`clear fcn_FSM` prima di ogni run dell'MPC** | `fcn_FSM` usa variabili `persistent`: la run *n+1* riparte dallo stato della *n*, e due run identiche danno risultati diversi |
| **Non salvare il `.slx` dopo `applica_terreno`** | la funzione modifica il modello **in memoria**, apposta perché il file sul disco resti identico e non si generino conflitti. Salvarlo vanifica tutto |
| **Non commentare i `Rigid Transform` del terreno** | fanno parte della catena che ancora il ramo al World, non del singolo elemento: commentarne uno stacca tutto quello che pende da lì e il pavimento cade come corpo libero, trascinando giù il robot. Si commentano solo solidi e contatti |
| **Non usare variabili con i nomi dell'`InitFcn`** | l'`InitFcn` del modello è uno *script* che condivide il base workspace e definisce fra l'altro `k`, `j`, `cfg`, `A`, `B`, `C`, `D`, `ss`. Una variabile `ss` in uno snippet maschera la funzione `ss()` e rompe il modello, con un errore che non nomina la variabile |
| **Gli angoli del log di Simscape escono in GRADI** | letti come radianti davano energia 3463 J, CoT 159 e 215 m di scivolamento su 1,4 m percorsi: tutti plausibili a prima vista. `adatta_simscape` chiede sempre l'unità esplicita e ha una guardia sui limiti di giunto |
| **`SimscapeLogLimitData` deve stare su `off`** | con `on` il log tiene solo gli ultimi 5000 punti e la run parte a metà, in silenzio |
| **Non duplicare `phantomx_config.m`** | due copie divergono in silenzio, senza che nessun errore lo segnali. `startup_phantomx` controlla e avvisa |
| **Non scambiare le mesh `_l` con le `_r`** | l'URDF usa `thigh_l.STL` e `tibia_l.STL` per tutte e sei le zampe: i frame dei link destri sono già ruotati. Le `_r` esistono in `meshes/` ma appartengono a un'altra convenzione, e usarle scompone le zampe destre |
| **Non modificare `phantomx_description-master/`** | è il pacchetto originale. L'unica eccezione è `fix_mesh_paths`, che tocca il `.slx`, non il pacchetto |
| **Concordate chi modifica e salva il `.slx`** | i file `.slx` sono binari: git non sa fonderli, quindi due modifiche parallele generano un conflitto irrisolvibile. Da quando il terreno si sceglie da script questo vincolo riguarda solo le modifiche **strutturali** allo schema |

---

## Mesh e percorsi

I blocchi `File Solid` del modello referenziano gli STL con percorsi **relativi alla
radice del repository** (`phantomx_description-master/meshes/...`). Simscape li risolve
rispetto al *current folder* di MATLAB, non alla posizione del `.slx`: per questo
`startup_phantomx` va lanciato **dalla radice**, ed è anche da lì che va lanciata la
simulazione.

Per verificare in qualsiasi momento:

```matlab
audit_mesh
```

Elenca i 25 blocchi, dice se ogni file è raggiungibile e se il percorso è assoluto o
relativo. Se qualcosa non torna, `fix_mesh_paths` (dry run di default) lo sistema.

> **Non convertire i percorsi in assoluti in un repository condiviso.** Funzionerebbe
> sulla tua macchina e si romperebbe su quella dell'altro, e siccome il `.slx` è
> binario ogni conversione produce un conflitto git. Se ti serve la modalità assoluta
> per un test locale, ricordati di tornare a `fix_mesh_paths repo` prima del commit.

---

## Stato del progetto

Il piano di confronto completo — tre controllori, cinque famiglie di metriche, sette
task, calendario — è in **`docs/piano_confronto.md`**.

| | controllore | stato |
|---|---|---|
| **C1** | NUKE feed-forward (baseline del paper) | funzionante |
| **C2** | Arrigoni closed-loop: rilevazione contatto da coppia | campagna completa |
| **C3** | C2 + retroazione di beccheggio (`phantomx_sim_attitude`) | implementato, **campagna assente** |
| ~~C3~~ | ~~MPC convesso~~ | gira sul simulatore ridotto, **fuori dal confronto** per scelta: modello a corpo singolo, sarebbe un confronto fra due impianti |

L'interruttore fra C1 e C2 non richiede blocchi aggiuntivi: la retroazione blocca la
zampa quando `|tau_misurata − tau_attesa|` supera una soglia su almeno uno dei tre
giunti, con `tau_attesa` calcolata dal blocco `Inverse Dynamics`.

> **[CORRETTO 25/9]** Qui era scritto `|tau|` **senza la differenza**, e in
> `abilita_log.m` lo stesso. La regola vera è stata letta nel modello risalendo il
> collegamento del `Constant c2_soglia`. Non è una precisazione formale: il flag
> dipende dall'accuratezza del modello dello stimatore, ed è per questo che
> correggere le inerzie del solo robot lo ha reso inutilizzabile. Vedi
> `allinea_stimatore` e `docs/piano_confronto.md` §11.

Con soglia infinita il confronto è sempre falso — ma **questo non dà l'anello
aperto**: l'interruttore vero è `cfg.c2.attiva`, vedi
`docs/come_funziona_ricerca_terreno.md` §6.

```matlab
cfg.c2.attiva = false;   % C1, anello aperto  (c2_soglia = inf)
cfg.c2.attiva = true;    % C2                 (c2_soglia = cfg.c2.soglia_tau)
```

### Debiti noti

Elencati per esteso in `docs/piano_confronto.md` §9. I due che vanno decisi **prima**
di far partire la campagna di misura:

- **Tibia 0,12 o 0,153 m.** Oggi il simulatore è internamente coerente su 0,12 (l'IK e
  la sfera di contatto concordano), ma il robot vero misura 0,153. Cambiarla richiede
  tre modifiche simultanee più una ritaratura di `z0`.
- **`body_z0 = 0,25 m`** contro gli 0,11 della derivazione geometrica: all'istante zero
  i piedi partono 14 cm in aria. `init_gait` lo segnala a ogni lancio.

Aperti sul banco di prova:

- **C3 non ha righe di campagna.** Il controllore esiste e funziona, ma non è
  passato per i sette task con lo stesso protocollo. Tre cose da sistemare prima:
  - il **carico** ("il pacco": `Brick Solid`, `Brick Solid1`, `6-DOF Joint1`,
    `Spatial Contact Force`) è **attivo** nel modello d'assetto. Una campagna
    lanciata così misura C3 carico contro C1 e C2 scarichi;
  - le posizioni delle zampe in `calcola_delta_z` sono **scritte a mano**, non
    prese da `cfg` — il commento nel codice lo dice. Se non sono quelle vere, il
    guadagno efficace dell'anello è sbagliato di un fattore, e l'anello funziona
    lo stesso;
  - il regolatore è un **proporzionale puro** (`P = 30`, `I = 0`, `D = 0`)
    nonostante il commento parli di PI: resta un errore di beccheggio a regime,
    proprio nel caso — la rampa — in cui serve.
- **T6 esaurisce il pavimento** (8 × 8 m). Gestito troncando le run al bordo, ma
  un controllore più veloce esce dal mondo: o si allarga il pavimento o si
  accorcia la run, **prima** di misurare C3.
- **`rumore_metriche` non passa da `scegli_controllore`**: si costruisce le run e
  fa `OVERRIDE_C2 = strcmp(rm_ctrl,'C2')`. Per il pavimento di rumore di C3 va
  passato anche il modello.
- **Geometria della rampa (T4)**: da ridimensionare e riposizionare, non da ricablare.
- **~~Filtri `sys_filter`~~ — CHIUSO.** `sys_filter` esiste, ha 18 stati e i suoi poli
  seguono `tau` alla cifra, ma **nessuno dei 2085 blocchi del modello lo legge**:
  l'`InitFcn` lo costruisce e il modello lo ignora. Non c'entra con la cella 2×, che
  fallisce perché il tripode non regge l'andatura (vedi `archivio/README.md`). Il
  filtro resta lì come residuo: si può togliere dall'`InitFcn`, ma non è urgente.
- **La tabella delle metriche ha troppe colonne** (48) ed è diventata difficile da
  leggere. Da affrontare, ma con una distinzione: **il CSV deve restare completo** — è
  il record, e una colonna tolta dal record non si recupera. Quello che va ridotto è la
  *vista*: una funzione che seleziona il sottoinsieme giusto per task, usata sia dal
  `disp` degli script sia per esportare accanto al CSV completo una tabella corta,
  quella che va in relazione. Togliere colonne dal CSV per guadagnare leggibilità
  sarebbe l'errore: le colonne che oggi sembrano di troppo sono quelle che hanno
  spiegato il fallimento a 2×.
- **Due `Data Store Write`** su `j_c1_rr` e `j_thigh_rr` scrivono nella stessa memoria
  senza ordine garantito.
- **Semantica di `c_*` e `z_*`** nei blocchi To Workspace: da chiarire se siano forze o
  flag di contatto.
- **Il repository sta su Google Drive**: la sincronizzazione tiene aperti i file binari
  e fa fallire `git pull` con *"unable to create file … File exists"*. Da spostare.

---

## Riferimenti

- S. Arrigoni, M. Zangrandi, G. Bianchi, F. Braghin, *Control of a Hexapod Robot
  Considering Terrain Interaction*, Robotics 2024, 13, 142.
- Ding et al., MPC convesso representation-free per robot legged.
- [PhantomX AX Metal Hexapod Mark II](https://www.trossenrobotics.com) — Trossen Robotics.
- [qpSWIFT](https://github.com/qpSWIFT/qpSWIFT) — solver QP, incluso in `third_party/`.