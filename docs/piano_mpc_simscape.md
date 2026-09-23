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
| **S0** | giunti in coppia, **niente MPC**: `τ = −Jᵀ Rᵀ f` con `f = m g / 6` verticale su tutte le zampe | corpo fermo: deriva di quota < 5 mm e assetto < 1° in 2 s | Jacobiano, segno, ordine delle zampe (CAN contro Mux) |
| **S1** | MPC, **tutte le zampe in appoggio** (`gait = −1`), `v = 0` | tiene la posa; dopo una spinta sul corpo torna entro 1 s | stima dello stato, chiusura dell'anello, tempi di calcolo |
| **S2** | tripode **sul posto** (`v = 0`), volo con PD di giunto | 10 s senza cadere, `ExitFlag` sempre 0, deriva < 5 cm | passaggio stance/swing, guadagni del PD in volo |
| **S3** | tripode a `v_nom` su piano (T1) | stessa riga di metriche di C1; distanza entro il 20% di quella comandata | è C3 |
| **S4** | campagna T2–T6 con gli script esistenti | — | il confronto |

S0 è il gradino che conta di più: se il segno o lo Jacobiano sono sbagliati, da
S1 in poi si vedrebbe solo «un robot che si comporta male» senza sapere perché.

---

## 4. Cose da sistemare nell'MPC prima di S1

Già elencate in `piano_confronto.md` §9, qui in ordine di esecuzione.

| # | cosa | perché | costo |
|---|---|---|---|
| 1 | `lb = −Fzd` → `lb = 0` | oggi il QP ammette piedi che **tirano** il terreno | 1 riga + verifica di ammissibilità |
| 2 | `cfg.J` dall'URDF (`importrobot`) | è stimata, ed entra nella predizione | ½ ora |
| 3 | disturbo spento e documentato | `fcn_get_disturbance` è ancora quello del quadrupede | già spento in `mpc_loop` |
| 4 | `clear fcn_FSM` all'avvio di ogni simulazione | variabili `persistent`: senza azzerarle due run uguali danno risultati diversi | 1 riga in `init_gait` |
| 5 | massa delle zampe | il modello SRB le considera senza massa: va misurata la quota di massa nelle zampe per sapere quanto errore di modello aspettarsi | ½ ora |

---

## 5. Un'aggiunta che vale la relazione: limiti di coppia nel QP

In T4, T4D, T5 e T6 C2 arriva a 13–18 volte il datasheet dell'AX-12A, e
abbiamo scritto più volte *«è un argomento per l'MPC, che può mettere i limiti
di coppia nel problema»*. Si può fare davvero: a `q` fissato, `τ = −Jᵀ Rᵀ f` è
**lineare** in `f`, quindi `|τ| ≤ τ_max` è un vincolo lineare in più nel QP,
ricalcolato a ogni passo con la `q` corrente.

Proposta: **C3 base** senza il vincolo, e **C3τ** con il vincolo, dopo S3. È
l'unico confronto in cui l'MPC può rivendicare qualcosa che i cinematici non
possono fare per costruzione.

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

---

## 7. Calendario e punto di ripiego

| giorno | lavoro | esce |
|---|---|---|
| 1 | punti 1–5 della §4; `crea_modello_mpc`; **S0** | robot fermo in coppia |
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
