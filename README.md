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

La definizione dei task è in `docs/piano_confronto.pdf`; qui c'è solo il terreno
che ciascuno richiede.

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
| `docs/` | piano di confronto, changelog, paper di riferimento |
| `grafici/` | figure per la relazione |

Utility nella radice:

| gruppo | file |
|---|---|
| terreno | `applica_terreno`, `quota_terreno`, `mappa_contatti` |
| metriche | `adatta_simscape`, `metriche`, `complanarita`, `verifica_marcia`, `verifica_ik` |
| modello | `setup_modello`, `setup_terreno_param`, `audit_mesh`, `fix_mesh_paths`, `trova_nel_modello` |
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
task, calendario — è in **`docs/piano_confronto.pdf`**.

| | controllore | stato |
|---|---|---|
| **C1** | NUKE feed-forward (baseline del paper) | funzionante |
| **C2** | Arrigoni closed-loop: rilevazione contatto da coppia | implementato nel modello |
| **C3** | MPC convesso | funzionante sul simulatore ridotto |

L'interruttore fra C1 e C2 non richiede blocchi aggiuntivi: la retroazione blocca la
zampa quando `|tau|` supera una soglia, quindi con soglia infinita il confronto è
sempre falso e il comportamento torna quello ad anello aperto.

```matlab
cfg.c2.attiva = false;   % C1, anello aperto  (c2_soglia = inf)
cfg.c2.attiva = true;    % C2                 (c2_soglia = cfg.c2.soglia_tau)
```

### Debiti noti

Elencati per esteso in `docs/piano_confronto.pdf` §9. I due che vanno decisi **prima**
di far partire la campagna di misura:

- **Tibia 0,12 o 0,153 m.** Oggi il simulatore è internamente coerente su 0,12 (l'IK e
  la sfera di contatto concordano), ma il robot vero misura 0,153. Cambiarla richiede
  tre modifiche simultanee più una ritaratura di `z0`.
- **`body_z0 = 0,25 m`** contro gli 0,11 della derivazione geometrica: all'istante zero
  i piedi partono 14 cm in aria. `init_gait` lo segnala a ogni lancio.

Aperti sul banco di prova:

- **Geometria della rampa (T4)**: da ridimensionare e riposizionare, non da ricablare.
- **Filtri `sys_filter`**: 18 filtri del primo ordine con `tau = 0,05 s` sui comandi di
  giunto, nell'`InitFcn`. A doppia velocità lo swing dura 200 ms e il filtro ne taglia
  il 25%: va escluso che il fallimento della cella 2× sia del banco e non del
  controllore.
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