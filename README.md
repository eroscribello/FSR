# PhantomX — confronto fra tre controllori in Simscape

Progetto finale di **Field and Service Robotics** (Università Federico II).

Confronto sistematico fra tre controllori per l'esapode **PhantomX AX Metal Hexapod
Mark II**, sullo stesso impianto Simulink/Simscape Multibody, contro il lavoro di
Arrigoni et al. (*Control of a Hexapod Robot Considering Terrain Interaction*,
Robotics 2024, 13, 142).

| | |
|---|---|
| **C1** | cinematico ad anello aperto, andatura a tripode, giunti in posizione |
| **C2** | C1 + **ricerca del terreno**: il piede scende finché la coppia non segnala il contatto |
| **C3** | C2 + retroazione sull'assetto del corpo, beccheggio e rollio |

Ogni controllore aggiunge **un solo meccanismo** al precedente: ogni confronto fra
due adiacenti ne misura uno solo.

---

## Requisiti

| | |
|---|---|
| MATLAB | R2021b o successivo |
| Toolbox | Simulink, Simscape, Simscape Multibody |
| Toolbox | Robotics System Toolbox (`importrobot`, `rigidBodyTree`) |
| Sistema | testato su Windows 11, MATLAB R2025b |

---

## Avvio rapido

**Da fare a ogni sessione di MATLAB**, prima di qualsiasi altra cosa:

```matlab
cd <cartella del repository>
startup_phantomx
```

`startup_phantomx` aggiunge le cartelle al path, verifica che non esistano copie
duplicate dei file critici e stampa i promemoria. **Senza questo comando niente
funziona**, e l'errore che ottieni (`Unrecognized function 'inv_kyn'`) non dice
che il problema è il path.

Poi, a seconda di cosa vuoi lanciare:

### Baseline Simscape (controllore cinematico)

```matlab
init_gait                          % carica i parametri nel base workspace
applica_terreno('T1')              % sceglie il terreno dello scenario
out = sim('phantomx_sim_zero','StopTime','10');
```

`applica_terreno` accende e spegne gli elementi del terreno **senza salvare il
modello**: vedi "Scenari di terreno".

---

## Campagne di misura

I risultati vanno in `results/`, un CSV per task e controllore:
`results/T2_C1.csv`, `results/T2_C2.csv`, `results/T3_C1.csv`.

**Ogni campagna parte da una sessione pulita:**

```matlab
clear all; bdclose all; startup_phantomx
```

`clear all`, non `clear`: toglie anche le variabili persistenti. 
`init_gait` legge dal base workspace `OVERRIDE_GAIT`, `OVERRIDE_C2`, `FORZA_STATICO`, e
quello che resta lì da run precedenti cambia il comportamento senza lasciare
traccia nel codice.

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
| `C2` | `phantomx_sim_zero` | `true` | C1 + ricerca del terreno |
| `C3` | `phantomx_sim_attitude` | `true` | C2 + retroazione di beccheggio, **in serie** |

Aggiungere un controllore significa aggiungere una riga a quella tabella, e
nient'altro. Un nome sconosciuto è un errore immediato, non un CSV con
l'etichetta sbagliata.

**Per una campagna su più task** senza aprire i file:

```matlab
clear all; bdclose all; startup_phantomx
OVERRIDE_CTRL = 'C3';
script_T2; script_T3; script_T5; script_T6
clear OVERRIDE_CTRL
```

`OVERRIDE_CTRL` **non viene cancellata dagli script**, ogni run
che la usa lo dichiara a schermo in rosso. 

### T2 — curva di velocità

```matlab
clear all; bdclose all; startup_phantomx
script_T2
```

Cinque celle, dieci cicli ciascuna, terreno T2 fissato dallo script.

**Le cinque velocità e cosa sono**:

| cella | ruolo |
|---|---|
| `0.5x` | bassa velocità |
| `1.0x` | nominale |
| `1.20x` | **ultima cella con moto del corpo stabile**|
| `1.5x` | fuori inviluppo — il robot saltella |
| `2.0x` | fuori inviluppo — il robot indietreggia |

Le ultime due **falliscono per costruzione**.

**Le colonne che dicono *perché* una cella fallisce** (C1, forze dai sensori):

| colonna | cosa misura | 1.0× | 2.0× |
|---|---|---|---|
| `sotto3_frac` | frazione di tempo con meno di tre piedi a terra | 0.117 | 0.536 |
| `corpoZ_pp` | rimbalzo verticale del corpo, picco-picco per ciclo | 3.2 mm | 81 mm |
| `corpoZ_vz` | rms della velocità verticale del corpo | 0.024 m/s | 0.463 m/s |

`corpoZ_pp` e `corpoZ_vz` vanno letti insieme: a 0.5× e a 1.5× il rimbalzo è
lo stesso (29 e 33 mm), ma a 0.5× il corpo dondola (0.067 m/s) e a 1.5× sbatte
(0.200 m/s). L'ampiezza da sola non distingue i due cedimenti.

**Fonte delle forze.** `adatta_simscape` legge le forze dai sensori del
modello (`Fleg`), validati allo 0.0% sul peso, e li usa da solo quando li trova.

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
| `applica_terreno('T4D')` | piano liscio + dosso |
| `applica_terreno('T5')` | piano liscio + un ostacolo |
| `applica_terreno('T6')` | piano liscio + serie di ostacoli |
| `applica_terreno('T7')` | piano liscio + disturbo impulsivo laterale |

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
| `phantomx_description-master/` | pacchetto ROS originale: mesh STL e URDF. **Non modificare** |
| `docs/` | `piano_confronto.md` (controllori, metriche, task, debiti), `stato_progetto.md`, `andatura.md` (generatore, traiettoria del piede, provenienza dei parametri), paper e datasheet di riferimento |
| `archivio/` | indagini chiuse, con l'esito registrato in `archivio/README.md`. **Fuori dal path** |
| `grafici/` | figure per la relazione |

Utility nella radice:

| gruppo | file |
|---|---|
| controllori | `scegli_controllore` (nome → modello + `OVERRIDE_C2`), `allinea_stimatore`, `verifica_attitude` |
| terreno | `applica_terreno`, `quota_terreno`, `mappa_contatti` |
| metriche | `adatta_simscape`, `metriche`, `complanarita`, `verifica_marcia`, `verifica_ik` |
| modello | `setup_modello`, `setup_terreno_param`, `audit_mesh`, `fix_mesh_paths`, `trova_nel_modello` |
| diagnostica | `fattibilita`, `diagnosi_appoggio`, `stabilita_zmp`, `soglia_deriva_T6`, `estrai_funzioni` |
| risultati | `tabella_confronti` (genera `results/tabella_confronti.md`), `rumore_metriche` |
| log | `abilita_log`, `cerca_log` |

---

## L'impianto

| | |
|---|---|
| modello | multibody completo, 25 corpi |
| attuazione | giunti in **posizione** |
| contatto | modello a penalità, con attrito |
| entry point | `phantomx_sim_zero.slx`, `phantomx_sim_attitude.slx` |
| parametri | `init_gait.m` → `phantomx_config` |

Tutto legge **`common/phantomx_config.m`**. È l'unico posto dove si modificano masse,
geometria, tempi dell'andatura, attrito e limiti degli attuatori.

### I modelli Simscape

Un simulatore, più `.slx` in `simscape/`. Non si scelgono a mano: li dà
`scegli_controllore`.

| file | a cosa serve |
|---|---|
| `phantomx_sim_zero.slx` | l'impianto di riferimento, C1 e C2 |
| `phantomx_sim_attitude.slx` | C3: lo stesso impianto più la retroazione di beccheggio |

Gli altri `.slx` della cartella sono varianti di lavoro e backup datati.

---

## Regole

| regola | perché |
|---|---|
| **`startup_phantomx` a ogni sessione** | senza, il path non c'è e gli errori non lo dicono |
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

## Riferimenti

- S. Arrigoni, M. Zangrandi, G. Bianchi, F. Braghin, *Control of a Hexapod Robot
  Considering Terrain Interaction*, Robotics 2024, 13, 142.
- [PhantomX AX Metal Hexapod Mark II](https://www.trossenrobotics.com) — Trossen Robotics.
- [ROBOTIS AX-12A](https://emanual.robotis.com) — datasheet dei servomotori, in `docs/`.