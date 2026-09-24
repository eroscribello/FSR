# Piano: porting dell'MPC (C3) su Simscape

Progetto FSR PhantomX · 22/9/2026 · stato: **proposta, nessun codice scritto**

Obiettivo: far girare C3 **sullo stesso impianto** di C1 e C2 (opzione A di
`piano_confronto.md` §4), così che le differenze in tabella siano del
controllore e non del simulatore.

---

## 1. Da dove si parte (letto dal codice, non supposto)

| cosa | dove | stato |
|---|---|---|
| RF-MPC a 6 zampe, tripode | `mpc_srb/` (`mpc_loop`, `fcn_FSM`, `fcn_get_QP_form_eta`, `fcn_get_ABD_eta`) | gira sul modello a corpo rigido (SRB), integrato con `ode45` |
| QP | qpSWIFT, mex già compilato per Windows (`qpSWIFT.mexw64`) | `ExitFlag` ora controllato in `mpc_loop` |
| passo di ricalcolo | `simTimeStep = 0.01 s` (100 Hz) | orizzonte 6 passi × `Tmpc = 0.08 s` = 0.48 s ≈ un ciclo |
| stato | `[p, v, R(9), ω, piedi(18)]`, 36 elementi | lo stato del QP è ridotto a 12 (p, v, η, ω) |
| andatura | `fcn_FSM`: stesso `T` e stesse fasi di C1 (da `cfg`), piede in volo con *capture point* | differisce da C1 nell'atterraggio e nell'appoggio, vedi §2 |
| giunti Simscape | 18 Revolute, **`InputMotion` / `ComputedTorque`** | l'MPC ha bisogno dell'inverso: `InputTorque` |
| sensori già nel modello | posizione, velocità e coppia dei 18 giunti; Transform Sensor sul corpo | lo stato per l'MPC c'è già |
| solutore | `daessc`, passo variabile | un controllore discreto a 100 Hz ci convive senza problemi |

Conseguenza: **non si riscrive l'MPC**. Si cambia l'impianto sotto di lui e si
aggiunge lo strato che trasforma forze in coppie.

---

## 2. Architettura proposta

```
 Simscape (giunti in coppia)
   │  q, q̇ (18)   posa e twist del corpo
   ▼
 [stima stato]  p, v, R, ω  +  piedi nel mondo (FK da q e posa)
   │
   ▼
 [orologio andatura]  LO STESSO di C1: T, fase, stance/swing per zampa
   │
   ├── zampe in appoggio ──► [MPC, 100 Hz]  forze f_i (3 × 6)
   │                           │
   │                           ▼
   │                  τ_i = − J_i(q)ᵀ · Rᵀ · f_i
   │
   └── zampe in volo ─────► q_d da inv_kyn (stessa traiettoria di C1)
                              τ_i = Kp (q_d − q) + Kd (q̇_d − q̇)
   │
   ▼
 somma → 18 coppie → Simscape
```

Il segno di `τ = −Jᵀ Rᵀ f` va **misurato**, non scelto (vedi S0): `f` è la
forza che il terreno esercita sul robot, la zampa spinge il terreno con `−f`.
È esattamente il tipo di segno che con `applica_imbardata` ci è costato tre
giorni.

### Scelta di disegno: piede in volo come C1, capture point dopo

[CORRETTO 22/9] L'orologio **è già lo stesso**: `get_params` prende `T` e le
fasi da `cfg` e blocca se divergono. Le differenze reali fra `fcn_FSM` e C1:

| | C1 | `fcn_FSM` |
|---|---|---|
| durata appoggio | fissa | si accorcia se `v > v_nom` (`Tst = min(Tst, S/v)`) |
| piede in appoggio | comandato in posizione | fermo nel mondo, il corpo si muove per le forze |
| atterraggio | fisso rispetto al corpo | capture point (Raibert): si sposta con l'errore di velocità |

**Deciso:** prima C3 con il piede in volo come C1, così C1 → C3 cambia solo la
spinta in appoggio. Il capture point dopo, come variante.

### Limite noto: contatto pianificato, non rilevato

Le zampe «in volo» per l'orologio hanno `Fzd = 0`, quindi `lb = ub = 0`: forza
nulla per costruzione. L'MPC non usa sensori di contatto. Conseguenze attese:

- terreno **più basso** del previsto (T4D spigoli e discesa, T6): zampa a terra
  per l'orologio ma in aria, forza pianificata che non arriva;
- terreno **più alto** (T5, T6, salita T4): piede che tocca durante il volo,
  forza reale che l'MPC non conosce.

C2 invece rileva il contatto. Da dichiarare nel confronto; una variante di C3
con il contatto rilevato è un'estensione possibile.

---

## 3. La scala di validazione

Ogni gradino ha un **criterio di passaggio scritto prima** di lanciare. Non si
sale al gradino dopo finché quello sotto non passa.

| gradino | cosa gira | criterio di passaggio | cosa verifica |
|---|---|---|---|
| **S0** | ~~giunti in coppia, niente MPC~~ | ~~corpo fermo: deriva < 5 mm e assetto < 1° in 2 s~~ | **[RITIRATO 24/9]** criterio fisicamente irraggiungibile, vedi §3b. Quello che doveva verificare è verificato per altra via |
| **S1** | MPC, **tutte le zampe in appoggio** (`gait = −1`), `v = 0` | tiene la posa; dopo una spinta sul corpo torna entro 1 s | stima dello stato, chiusura dell'anello, tempi di calcolo |
| **S2** | tripode **sul posto** (`v = 0`), volo con PD di giunto | 10 s senza cadere, `ExitFlag` sempre 0, deriva < 5 cm | passaggio stance/swing, guadagni del PD in volo |
| **S3** | tripode a `v_nom` su piano (T1) | stessa riga di metriche di C1; distanza entro il 20% di quella comandata | è C3 |
| **S4** | campagna T2–T6 con gli script esistenti | — | il confronto |

---

## 3b. [RITIRATO 24/9] S0 non era un test sbagliato: era un test impossibile

S0 chiedeva al robot di **restare fermo con coppie costanti in anello aperto**.
Nessun vettore di coppie può farlo, nemmeno quello esatto.

### Perché, in una riga

A coppie di giunto fissate, la forza al piede è `f = −(Jᵀ)⁻¹ τ`, e `J` dipende
dalla configurazione. Se il corpo **sale**, la zampa si distende, il braccio si
accorcia e la stessa coppia dà **più** spinta: sale ancora. Se **scende**, la
zampa si piega, il braccio si allunga e la stessa coppia dà **meno** spinta:
scende ancora. È retroazione positiva in tutte e due le direzioni. Lo
smorzamento (0.265 N·m·s/rad, la retta coppia-velocità dell'AX-12A) rallenta
la divergenza ma non la inverte: combatte la velocità, non il segno del
guadagno. È anche il motivo per cui l'MPC esiste.

### Le prove, in ordine

| # | prova | esito |
|---|---|---|
| 1 | `tau_statico`, coppie analitiche, a gradino | sale, muore a 0.17 s |
| 2 | segno invertito | scende, muore a 0.07 s |
| 3 | rampa 0.5 s | scende, muore a 0.20 s — **rampa ritirata**: la gravità resta piena, quindi durante la rampa il robot è sotto-coppia per costruzione |
| 4 | smorzamento ai giunti | sale lo stesso, 0.16 s |
| 5 | `tau_misurato`: coppie **lette dal modello base** che sta fermo davvero | sale lo stesso, 0.16 s |

Cinque ingressi diversi, sempre `degenerate mass distribution` sul 6-DOF Joint.
Quando l'errore non dipende dall'ingresso, non è l'ingresso.

### Cosa è stato escluso, e come

| ipotesi | strumento | esito |
|---|---|---|
| modello degenere di suo | `corpi_degeneri` | 0 corpi problematici |
| cablaggio della copia diverso dal base | `controlla_copia` | 18/18 giunti identici |
| massa degenere a prescindere dalle forze | `s0_vuoto` | 3 s puliti a gravità zero, **con le zampe che percorrono mezza corsa**: la massa non è degenere né in posa nominale né in un ampio intorno |
| segno globale, mirroring, ordine dei giunti | `tau_misurato` | escluse tutte e tre: sei zampe con gli stessi numeri a 4 decimali |
| ordine / segno / unità fra `Constant` e giunti | `prova_coppie` | rapporto arrivato/comandato **1.000 su tutti e 18**, scarto max 0.0000 N·m |
| posa di partenza diversa nella copia | `posa_iniziale` | **identica al bit**: stessi angoli, stessi piedi (6/6 a terra, 2.788 N ciascuno), stessa quota 154.07 mm |

L'ultima riga chiude il cerchio: base e copia partono dallo stesso stato con le
stesse coppie, quindi a `t = 0` l'equilibrio è esatto. Quello che segue è
divergenza da un equilibrio instabile, non un difetto.

> **Errore mio, da non ripetere.** La prova 1 di `s0_vuoto` era stata dichiarata
> "prova nulla": gravità zero, coppia zero, niente può muovere il robot. Falso.
> Restava il **precarico del contatto** — `cfg.body_z0` parte 2.1 mm dentro il
> pavimento — e le molle hanno spinto le zampe fino a richiuderle. Se ne è
> accorto A. guardando l'animazione, non io leggendo i numeri.

### Cosa resta di S0

S0 doveva verificare tre cose. Tutte e tre sono verificate, meglio di come le
avrebbe verificate S0:

| cosa | verificato da | evidenza |
|---|---|---|
| ordine delle zampe (CAN contro Mux) | `prova_coppie` | 18 rapporti a 1.000 |
| segno della mappa | `tau_misurato` | segni concordi su tutte e sei le zampe |
| Jacobiano | `tau_misurato` | struttura confermata; i moduli differiscono (femore ×1.26, tibia ×2.36) perché con sei piedi a terra e giunti in posizione il sistema è **iperstatico** e la forza al piede non è verticale |

**Il gradino S0 è ritirato. Si passa a S1.** La scala non salta un controllo:
salta un controllo che era stato scritto male.

---

## 4. Cose da sistemare nell'MPC prima di S1

Già elencate in `piano_confronto.md` §9, qui in ordine di esecuzione.

| # | cosa | esito del 24/9 |
|---|---|---|
| 1 | ~~`lb = −Fzd` → `lb = 0`~~ | **RITIRATO: il codice è già corretto.** La variabile del QP è `U − Ut` e i vincoli sono scritti attorno a `Ud`: la riga `−lb + Ut − Ud` dà `U_z ≥ lb + Ud_z`, che con `lb = −Fzd = −Ud_z` vale `U_z ≥ 0`. Nessun piede può tirare il terreno. Cambiarlo in `lb = 0` imporrebbe `U_z ≥ Ud_z`, cioè il contrario di quello che serve |
| 2 | `cfg.J` dall'URDF (`importrobot`) | **NON si può**, e non serve. Le inerzie dell'URDF sono quelle sbagliate di ~1000 volte (§9 di `piano_confronto`): `importrobot` restituirebbe proprio quelle. `calcola_J.m` la verifica invece componendo i valori corretti con gli assi paralleli: `cfg.J` sta dentro la forchetta fra i due casi limite e ~15% sotto il modello equispaziato. **Si tiene** |
| 3 | disturbo spento e documentato | già spento in `mpc_loop` |
| 4 | `clear fcn_FSM` all'avvio | **già fatto**: `mpc_loop.m` riga 57, con il commento che spiega perché |
| 5 | massa delle zampe | **misurato: 38%** della massa totale (24 link da 24.4 g + 6 piedi, su 1.585 kg). Il modello SRB le considera **senza massa**: è l'errore di modello dominante dell'MPC, molto più grande del 15% su `J`. Va dichiarato in relazione |

**[24/9] Il giorno 1 era quasi tutto già fatto o sbagliato.** Tre voci su cinque
cadono, la quarta è la verifica di `J` (fatta, `calcola_J.m`), la quinta è un
numero da scrivere in relazione, non un lavoro. Si passa direttamente a
`crea_modello_mpc.m` e a S0.

---

## 5. [RITIRATO 24/9] I limiti di coppia non sono l'argomento che credevamo

Questa sezione diceva: *«in T4, T4D, T5 e T6 C2 arriva a 13–18 volte il
datasheet dell'AX-12A»*, e da lì faceva discendere che il vincolo `|τ| ≤ τ_max`
nel QP fosse *«l'unico confronto in cui l'MPC può rivendicare qualcosa che i
cinematici non possono fare»*.

**Quel numero è del 22/9, cioè PRIMA della correzione delle inerzie.** Riletti
i CSV della campagna rifatta, non regge.

### `tau_max` misurato, inerzie corrette (datasheet 1.5 N·m)

| task | C1 | C2 | × datasheet (C2) |
|---|---|---|---|
| T2 v1.00x | 1.87 | 1.90 | 1.3× |
| T2 v1.20x | 2.38 | 2.90 | 1.9× |
| T2 v2.00x | 2.87 | 3.64 | 2.4× |
| T2 v0.50x | 3.64 | 5.86 | 3.9× |
| T2 v1.50x | **14.58** | 14.09 | 9.4× |
| T4 rampa | 2.10 | 2.52 | 1.7× |
| T4D dosso | 2.58 | 4.27 | 2.8× |
| T5 ostacolo | 4.46 | 10.82 | 7.2× |
| T6 ostacoli | 7.51 | 9.46 | 6.3× |

Il massimo assoluto è ora 14.6 N·m, **su C1**, ed è un valore isolato fra
vicini a 1.87 e 2.87: un picco da impatto, un campione. È esattamente il tipo
di numero che il pavimento di rumore aveva già bocciato — `tau_max` non
discrimina in **nessun** task (`piano_confronto.md`, 23/9). Usarlo come titolo
avrebbe contraddetto un criterio che avevamo scritto noi.

Sulle colonne che sopravvivono:

| | C1 | C2 | vs datasheet |
|---|---|---|---|
| `tau_rms`, tutti i task | 0.38–0.55 | 0.37–0.50 | 25–37% |
| `tau_rms_giunto_peggiore`, peggior caso | 1.11 | 1.01 | 74% |

**In RMS, giunto peggiore compreso, il robot sta dentro il datasheet su ogni
task.** Il vincolo di coppia nel QP risolverebbe un problema che non esiste.

### Cosa cade con questa sezione

| affermazione | stato |
|---|---|
| «C2 arriva a 13–18× il datasheet» | **ritirata**: pre-inerzie |
| «C2 costa più coppia di C1» | **non sostenibile**: in RMS sono identici, e in T2 v1.50x C1 supera C2 |
| «C3τ con i limiti di coppia è il confronto che vale» | **ritirata**: niente da vincolare |

Resta valido solo il fatto tecnico: a `q` fissato `τ = −Jᵀ Rᵀ f` è lineare in
`f`, quindi `|τ| ≤ τ_max` **sarebbe** un vincolo lineare facile da aggiungere.
Se un giorno servisse, si sa come. Oggi non serve.

### Dove si è spostato l'argomento

Quello che un controllore in forza può fare e uno cinematico no, e che è
misurabile con le colonne sopravvissute al pavimento di rumore:

| capacità | perché il cinematico non può | colonna che lo misura |
|---|---|---|
| **ripartire il carico** fra le zampe | non conosce le forze; con sei piedi a terra e giunti in posizione nascono forze interne (misurate il 24/9: femore ×1.26, tibia ×2.36 rispetto al solo peso) | `cot`, `energia` |
| **adattarsi al terreno senza soglia** | C2 ha bisogno di una ricerca esplicita con soglia tarata a mano e mai validata | `frazione_task`, `z_media` su T6 |
| **incassare un disturbo** | nessun margine di forza da redistribuire | `frazione_task` (T7, mai eseguito) |
| **reggere massa sottostimata del 10%** | non ha un modello da correggere | `cot`, `z_media` |

La prima riga è la più promettente e nasce da una misura che abbiamo già in
mano: le forze interne esistono, sono grandi, e un controllore che le
minimizza dovrebbe pagarle meno in `cot`. È un'ipotesi, e va misurata.

**Vincoli nel QP: `f_z ≥ 0` resta necessario** (una pseudo-inversa può chiedere
a un piede di tirare il terreno). I coni d'attrito con `mu = 0.9` e pendenza 8°
(`tan 8° = 0.14`) non sono vincolanti. Quindi il QP va introdotto quando si
misura che `f_z < 0` accade, non prima.

---

## 6. Il modello: copia generata da script

Cambiare la modalità dei giunti cambia le porte (sparisce l'ingresso di
posizione, compare quello di coppia): **i collegamenti di C1 si rompono**, quindi
lo stesso `.slx` non può servire tutti e tre.

| opzione | come | pro | contro |
|---|---|---|---|
| **A. copia generata da script** (proposta) | `crea_modello_mpc.m` parte da `phantomx_sim_zero.slx` e salva `phantomx_sim_mpc.slx`: cambia la modalità dei 18 giunti, toglie il blocco di C1, aggiunge quello di C3 | riproducibile: se il collega cambia il modello di base, si rilancia lo script. Terreno, contatti e sensori identici per costruzione | lo script è delicato da scrivere (cablaggio da codice) |
| B. copia fatta a mano | salva con nome e modifica in Simulink | più veloce da fare la prima volta | due modelli che divergono in silenzio a ogni modifica del collega |
| C. Variant Subsystem nello stesso modello | un solo `.slx` con due varianti | un file solo | modifica pesante del modello del collega |

`applica_terreno` accetta già il nome del modello come terzo argomento, quindi
i task funzionano sulla copia senza modifiche.

### [24/9] Modifiche alla copia fatte FUORI da `crea_modello_mpc`

L'opzione A vale solo se la copia resta rigenerabile. Ogni modifica fatta
altrove va scritta qui e prima o poi riportata dentro `crea_modello_mpc`,
altrimenti un `crea_modello_mpc('rigenera')` le cancella in silenzio.

| # | modifica | dove è ora | perché | da riportare? |
|---|---|---|---|---|
| 1 | `PositionTargetPriority = 'Low'` sui 18 giunti | fatta a mano | con `'High'` i 18 target sovra-vincolano l'assemblaggio e il robot spariva al primo istante. `posa_iniziale` dimostra che con `'Low'` la copia parte comunque **identica al base**, quindi la priorità bassa non costa niente | **sì**, va messa in `crea_modello_mpc` passo 5 |
| 2 | `Constant tau_S0` → `From Workspace` che legge `tau_ts` | `prova_S0.m`, una volta sola, con `save_system` sulla **sola copia** | serve un ingresso variabile nel tempo, non un numero | no: è un blocco di prova, sparisce quando entra il blocco C3 |
| 3 | smorzamento `DampingCoefficient` ai giunti | `prova_S0.m`, **solo in memoria** | è la retta coppia-velocità dell'AX-12A (`τ_max/qd_max` = 0.265 N·m·s/rad), quindi è fisico e non un numero di comodo. Non sposta l'equilibrio statico: a velocità nulla dà coppia nulla | **da decidere**: se resta, va in `crea_modello_mpc`; se no, va tolto anche da `prova_S0` |

Il modello del collega `phantomx_sim_zero.slx` non è mai stato aperto in
scrittura: nessuno di questi script chiama `save_system` su di lui.

---

## 7. Calendario e punto di ripiego

| giorno | lavoro | esce |
|---|---|---|
| 1 | punti 1–5 della §4; `crea_modello_mpc`; ~~**S0**~~ | copia generata e verificata; **S0 ritirato** (§3b) |
| 2 | blocco C3 nel modello; **S1** | MPC in anello chiuso |
| 3 | volo con PD; **S2**, poi **S3** | C3 cammina |
| 4 | campagna T2–T6 (gli script ci sono) | tabelle C3 |
| 5 | C3τ, se c'è margine | il confronto sui limiti di coppia |

**Punto di ripiego, deciso adesso:** se a fine giorno 2 S1 non passa, si
confronta C3 sul simulatore SRB (opzione B di §4, già funzionante), dichiarando
in relazione che gli impianti sono diversi. È il punto 3 della scaletta di
ripiego (§8): lo si attiva a una data, non quando si è stanchi.

---

## 8. Decisioni (22/9)

| # | domanda | deciso |
|---|---|---|
| 1 | piede in volo | come C1; capture point come variante dopo |
| 2 | modello | copia generata da script (`crea_modello_mpc.m`), da dire al collega |
| 3 | C3τ con limiti di coppia | se ne riparla dopo S3 |
| 4 | ripiego a fine giorno 2 | confermato |
