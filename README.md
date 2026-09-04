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
open simscape/phantomx_sim_zero.slx
sim('phantomx_sim_zero')
```

### MPC su simulatore ridotto

```matlab
clear fcn_FSM                      % obbligatorio, vedi "Regole" sotto
MAIN
```

---

## Struttura

| cartella | contenuto |
|---|---|
| `common/` | `phantomx_config.m` (parametri condivisi), `inv_kyn.m` (IK di gamba), `tripod_trajectory.m` |
| `simscape/` | modello Simscape Multibody e il suo script di inizializzazione |
| `mpc_srb/` | MPC convesso sul modello a corpo rigido singolo, più `fcns/`, `fcns_MPC/` e il solver |
| `phantomx_description-master/` | pacchetto ROS originale: mesh STL e URDF. **Non modificare** |
| `docs/` | piano di confronto, changelog, paper di riferimento |
| `grafici/` | figure per la relazione |

Utility nella radice: `audit_mesh`, `fix_mesh_paths`, `trova_nel_modello`,
`pulizia`, `pulizia2`.

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
| **Non duplicare `phantomx_config.m`** | due copie divergono in silenzio, senza che nessun errore lo segnali. `startup_phantomx` controlla e avvisa |
| **Non scambiare le mesh `_l` con le `_r`** | l'URDF usa `thigh_l.STL` e `tibia_l.STL` per tutte e sei le zampe: i frame dei link destri sono già ruotati. Le `_r` esistono in `meshes/` ma appartengono a un'altra convenzione, e usarle scompone le zampe destre |
| **Non modificare `phantomx_description-master/`** | è il pacchetto originale. L'unica eccezione è `fix_mesh_paths`, che tocca il `.slx`, non il pacchetto |
| **Un solo `.slx` alla volta aperto in MATLAB** | i file `.slx` sono binari: git non sa fonderli, quindi due modifiche parallele generano un conflitto irrisolvibile. Concordate chi tocca il modello |

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
| **C2** | Arrigoni closed-loop: rilevazione contatto da coppia | da implementare |
| **C3** | MPC convesso | funzionante sul simulatore ridotto |

### Debiti noti

Elencati per esteso in `docs/piano_confronto.pdf` §9. I due che vanno decisi **prima**
di far partire la campagna di misura:

- **Tibia 0,12 o 0,153 m.** Oggi il simulatore è internamente coerente su 0,12 (l'IK e
  la sfera di contatto concordano), ma il robot vero misura 0,153. Cambiarla richiede
  tre modifiche simultanee più una ritaratura di `z0`.
- **`body_z0 = 0,25 m`** contro gli 0,11 della derivazione geometrica: all'istante zero
  i piedi partono 14 cm in aria. `init_gait` lo segnala a ogni lancio.

---

## Riferimenti

- S. Arrigoni, M. Zangrandi, G. Bianchi, F. Braghin, *Control of a Hexapod Robot
  Considering Terrain Interaction*, Robotics 2024, 13, 142.
- Ding et al., MPC convesso representation-free per robot legged.
- [PhantomX AX Metal Hexapod Mark II](https://www.trossenrobotics.com) — Trossen Robotics.
- [qpSWIFT](https://github.com/qpSWIFT/qpSWIFT) — solver QP, incluso in `third_party/`.
