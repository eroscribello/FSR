# CLAUDE.md — istruzioni per una sessione su questo repository

Progetto FSR (Università Federico II, prof. Ruggiero). Confronto fra tre
controllori per l'esapode **PhantomX** in Simulink/Simscape Multibody, contro
Arrigoni et al., *Control of a Hexapod Robot Considering Terrain Interaction*,
Robotics 2024, 13, 142.

| | |
|---|---|
| **C1** | cinematico ad anello aperto, andatura a tripode, giunti in posizione |
| **C2** | C1 + **ricerca del terreno**: il piede scende finché la coppia non segnala il contatto |
| **C3** | C2 + retroazione sull'assetto del corpo: **beccheggio e rollio** (il rollio dall'1/10) |

Due persone: **Andrea** (misure, diagnostica, documentazione) e un **collega**
(il modello d'assetto, `phantomx_sim_attitude.slx`). Il `.slx` è suo.

---

## Da leggere prima di toccare qualcosa

In quest'ordine. Non serve leggere tutto: serve sapere che c'è.

| file | quando |
|---|---|
| `README.md` | sempre. Regole del repo, campagne, metriche, struttura |
| `docs/stato_progetto.md` | sempre. Dove siamo, cosa si può dichiarare, cosa è aperto |
| `docs/piano_confronto.md` | prima di discutere una misura. §9 inerzie, §11 il difetto dello stimatore |
| `docs/come_funziona_ricerca_terreno.md` | prima di toccare C2 |
| `archivio/README.md` | prima di riaprire un'indagine: 23 sono già chiuse, con l'esito |
| `commit_msg_*.txt` | la narrazione di ogni cambiamento, col perché |

Ogni script ha in testa **la domanda, la soglia e le conclusioni possibili**,
scritte prima di lanciarlo. Leggere l'header prima del codice.

---

## Regole di lavoro — non negoziabili

1. **I comandi git li lancia Andrea.** Mai eseguirli al posto suo.
2. **Nei messaggi di commit non compare Claude.** Niente `Co-Authored-By`,
   niente link di sessione: il lavoro lo firma lui.
3. **Mai `save_system` sul modello.** Tutto in memoria con `set_param`, come
   fanno `applica_terreno`, `applica_inerzie`, `allinea_stimatore`. Il `.slx`
   è binario e condiviso: salvarlo genera conflitti irrisolvibili.
4. **I file li scrive Claude, direttamente nella cartella del progetto.**
   Andrea non fa copia-incolla di codice.
5. **Risposte schematiche** — tabelle ed elenchi, non prosa lunga.
6. **Quando si revisiona un documento, dire sempre cosa è cambiato e perché**,
   non solo consegnare il risultato.
7. **Il criterio si scrive prima di lanciare.** Domanda, soglia e conclusioni
   possibili vanno nell'header *prima* di vedere i numeri.
8. **Le smentite restano scritte**, marcate `[RITIRATO <data>]` dove stavano,
   col motivo. Su questo progetto un'affermazione sbagliata è sopravvissuta
   tre giorni perché era plausibile.
9. **Non duplicare un file per cambiarne il default.** `applica_terreno_attitude.m`
   differisce da `applica_terreno.m` in due righe: è il modo esatto in cui due
   copie divergono in silenzio.

---

## Come si lancia una campagna

Ogni campagna parte da una sessione pulita:

```matlab
clear all; bdclose all; startup_phantomx
```

`clear all`, non `clear`: toglie anche le `persistent`. `init_gait` legge dal
base workspace `OVERRIDE_GAIT`, `OVERRIDE_C2`, `FORZA_STATICO`, e quello che
resta lì da run precedenti cambia il comportamento **senza lasciare traccia nel
codice**.

Il controllore si sceglie **per nome**, in una riga in testa a ogni `script_T*.m`:

```matlab
t5_ctrl      = 'C2';       % 'C1' | 'C2' | 'C3'
[t5_mdl, t5_c2] = scegli_controllore(t5_ctrl);
```

Oppure dal workspace, per più task di fila:

```matlab
OVERRIDE_CTRL = 'C3';
script_T2; script_T3; script_T4; script_T4D; script_T5; script_T6; script_T7
clear OVERRIDE_CTRL
```

`OVERRIDE_CTRL` **non si autocancella** (se lo facesse andrebbe riscritta prima
di ogni task). Ogni run che la usa lo dichiara a schermo in rosso.

`scegli_controllore.m` è l'unico posto dove sta scritto chi gira su quale
modello. Aggiungere un controllore = aggiungere una riga lì.

### Da shell (modalità agente)

```
matlab -batch "clear all; bdclose all; startup_phantomx; OVERRIDE_CTRL='C3'; script_T5"
```

In `-batch` non c'è finestra: le figure si salvano, non si mostrano
(`salva_grafico` scrive in `grafici/`). I risultati finiscono in `results/*.csv`
e si leggono direttamente da lì.

---

## C3, com'è fatto davvero

Verificato leggendo l'XML dentro il `.slx` (è uno zip), non dalla documentazione.

```
ricerca_terreno -> z_*  ->  Sum (+ d_z_*)  ->  Saturation  ->  Rate Limiter  ->  gamba
```

L'assetto sta **in serie, dopo** la ricerca del terreno: legge la quota già
corretta e ci somma un termine. Per questo `scegli_controllore` dà a C3
`OVERRIDE_C2 = true`: C3 *contiene* C2, e ogni controllore aggiunge un solo
meccanismo al precedente.

```matlab
delta_z = x_i * u_theta + y_i * u_phi;      % MATLAB Function4, calcola_delta_z
```

| | valore | note |
|---|---|---|
| PID beccheggio | **P = 10, I = 1, D = 0** | discreto a 1 ms, riferimento 0. Era P = 30, I = 0 fino al 30/9 |
| PID rollio | **P = 5, I = 0.5, D = 0** | aggiunto l'1/10 |
| `Saturation` | `LowerLimit = 0.07` | upper al default |
| `Rate Limiter` | ±0.3 m/s | |

**Con P = 30 e I = 0 l'anello faceva vibrare i piedi**: andatura visivamente
corretta, ma contatti frammentati sotto i 0.125 s, nessun appoggio riconosciuto
e `script_T4` che si fermava con un errore di indice. Diagnosticato con
`diagnosi_appoggio`, risolto abbassando il guadagno. Se ricompare un
comportamento del genere, il primo sospetto è lì.

**`x_i` e `y_i` non sono una misura geometrica.** Sono una matrice di
distribuzione dell'uscita sui sei piedi, scritta a mano. I segni e l'ordine
sono giusti (sequenza `lf, lm, lr, rr, rm, rf`, verificata contro i `Goto
d_z_*`), ma i moduli stanno **~1.8× sotto** i valori veri di `cfg.pf_nom`
(x ±0.224 contro ±0.12, y ±0.161/±0.243 contro ±0.10/±0.14). Il fattore di
scala è **assorbito in `P`**, che è stato tarato sperimentalmente con questi
numeri dentro: l'anello funziona. Correggerli moltiplicherebbe il guadagno per
1.8 e richiederebbe di ritarare. Resta un errore del 7% nel rapporto fra zampe
centrali e d'angolo sul rollio.

**Il carico** ("il pacco": `Brick Solid`, `Brick Solid1`, `6-DOF Joint1`,
`Spatial Contact Force`) sta nel modello d'assetto. Va commentato prima di
qualunque campagna, altrimenti si misura C3 carico contro C1 e C2 scarichi.

---

## Cosa NON si può misurare — i limiti del banco

Questa sezione evita di "scoprire" risultati che sono artefatti. **Leggerla
prima di interpretare qualunque numero.**

- **I sensori di forza vedono solo il pavimento, non rampe e ostacoli.** Su
  T4, T4D, T5 e T6 un piede appoggiato sull'ostacolo è carico e il sensore
  segna zero (chiusura al 41–65% del peso). Tutte le colonne di contatto di
  quei task — `appoggio_medio`, `slip_tot`, `distacchi`, `frazione_persa`,
  `disp_carico` — sono **indisponibili per costruzione**, e gli script le
  mettono a NaN. Anche `zampe a terra medie` e `contact_sched` stampati da
  `adatta_simscape` non sono leggibili lì.
  → per l'appoggio su quei task si usa `appoggio_cinematico` (piede fermo nel
  mondo), mai le forze.
- **Le metriche fini di T6 non vanno citate.** I controllori seguono
  traiettorie diverse e non incontrano gli stessi ostacoli. Di T6 vale solo
  l'esito binario (arriva / non arriva). Il confronto fine lo danno T5 e T4D.
- **Una differenza si dichiara solo se vale ≥ 3× il rumore misurato**, e il
  rumore va misurato **sullo stesso controllore**. Applicarlo con il pavimento
  di un controllore diverso ci ha fatto credere per un giorno che non fosse
  dichiarabile niente.
- **La campagna gira ad attuatore ideale.** Il limite del servo è 1.5 N·m; la
  fattibilità è misurata a parte (`fattibilita.m`): il giunto peggiore sta al
  41–46% del limite, i picchi d'urto lo superano su <1.1% dei campioni.
- **Un parametro è stato tarato da noi**: `cfg.c2.soglia_tau`, da `[0.04 0.1 0.1]`
  a `[0.385 0.136 0.105]`. La terna nuova **esclude di fatto la coxa**, che
  ruota attorno all'asse verticale e dal contatto verticale non riceve coppia.
  È un **cambio di regola**, non una ritaratura, e va dichiarato in relazione.

---

## Tranelli che sono già costati tempo

| | |
|---|---|
| **`applica_inerzie` e `allinea_stimatore` vanno sempre in coppia** | le inerzie dell'URDF sono ~1000× troppo grandi; correggerle solo sul robot e non sul `rigidBodyTree` dello stimatore rende `\|τ_mis − τ_att\|` grande ovunque, il flag di contatto resta incollato a 1 e **C2 smette di cercare il terreno**. È successo dal 22 al 25 settembre senza che nulla lo segnalasse |
| **I due modelli non hanno le stesse inerzie SU DISCO** | dal 30/9 `phantomx_sim_attitude` è stato salvato con le inerzie **già corrette** (`applica_inerzie` riporta 25/25 già corretti), mentre `phantomx_sim_zero` tiene ancora quelle dell'URDF e viene corretto a ogni run (0/25). Non cambia i risultati — `applica_inerzie` è idempotente — ma uno script che **dimentica** di chiamarla ottiene inerzie giuste su C3 e sbagliate su C1/C2, e non si vede. `taratura_T2.m` e `script_T4_limite.m` sono già in quella condizione |
| **Il `.slx` è binario ma è uno zip** | `unzip` e si legge l'XML: `simulink/systems/*.xml` per i blocchi e i collegamenti, `simulink/stateflow/chart_*.xml` per il codice delle MATLAB Function. È così che si è trovato un modello arrivato dal remoto con sei `From` senza `Goto`, che non compilava |
| **`estrai_funzioni.m`** | estrae il codice delle MATLAB Function in file di testo, per diffare due versioni del modello |
| **Chiudere i modelli in Simulink prima di `git pull`/`rebase`** | git riscrive il file sotto i piedi di MATLAB; se poi si risponde "salva" si sovrascrive la versione appena arrivata |
| **Google Drive** | tiene aperti i file binari e fa fallire `git pull` con *"unable to create file … File exists"*. `git status` può anche marcare file sporchi per stat mentre `git diff` è vuoto: non committarli |
| **`init_gait` è uno script**, non una funzione | condivide il base workspace e definisce fra l'altro `k`, `j`, `cfg`, `A`, `B`, `C`, `D`, `ss`. Tutte le variabili degli script di task sono prefissate `t2_`, `t5_`, … per questo |
| **Gli angoli del log di Simscape escono in GRADI** | letti come radianti davano energia 3463 J e 215 m di scivolamento su 1.4 m percorsi, tutti plausibili a prima vista |
| **I `.mat` non sono tracciati** | una run non salvata non si recupera: per rifare un calcolo su una run vecchia bisogna rilanciare la simulazione |
| **I warning di `find_system` sui Variant Subsystem** | sono rumore di MATLAB, nel progetto non ci sono varianti |

---

## Stato al 1 ottobre 2026

| | |
|---|---|
| righe di campagna **C1** | complete, verificate non toccate dalle correzioni |
| righe di campagna **C2** | rifatte tutte dopo la correzione dello stimatore |
| righe di campagna **C3** | **assenti** — è il lavoro in corso |
| pavimento di rumore | misurato per C1 e C2 su quattro task. **Per C3 manca** |
| fattibilità attuatori | misurata su tutte le celle nominali |

### Da fare, in ordine

1. **Prima della campagna C3**, tre precondizioni:
   - commentare il **carico** ("il pacco": `Brick Solid`, `Brick Solid1`,
     `6-DOF Joint1`, `Spatial Contact Force`) in `phantomx_sim_attitude`,
     altrimenti si misura C3 carico contro C1 e C2 scarichi;
   - decidere il **pavimento di T6** (8×8 m, il percorso lo esaurisce): o si
     allarga o si accorcia la run;
   - cancellare `applica_terreno_attitude.m`.
2. **Verificare `script_T4` su C3.** Non produceva grafici né CSV perché
   l'anello d'assetto faceva vibrare i piedi e `appoggio_cinematico` non
   trovava un solo appoggio. Il guadagno è stato abbassato: rilanciare e
   controllare con `diagnosi_appoggio` prima di modificare codice. Resta da
   mettere la **guardia** in `t4_salita`: con meno di tre piedi in appoggio
   deve dire che il robot non cammina, non dare un errore di indice.
   *(Il guadagno è stato abbassato l'1/10 e il tremolio è sparito: la verifica
   serve a confermarlo, non c'è detto che ci sia codice da cambiare.)*
3. **Campagna C3** sui sette task.
4. **Pavimento di rumore per C3**, quattro task × 3 run. Senza, nessuna
   differenza C2–C3 è dichiarabile. `rumore_metriche` non passa da
   `scegli_controllore`: va esteso al modello di C3 prima di usarlo.
5. Pulizia del repo e commenti.
6. Relazione e presentazione.

### Strumenti di diagnostica recenti

| | |
|---|---|
| `verifica_attitude.m` | il modello di C3 regge la catena di misura? Sette controlli, sola lettura |
| `diagnosi_appoggio.m` | perché `appoggio_cinematico` non trova appoggi: velocità del piede, tratti fermi, spettro |
| `stabilita_zmp.m` | ZMP, baricentro completo e margine sul poligono d'appoggio |
| `fattibilita.m` | coppia richiesta contro il limite del servo, dai CSV esistenti |
| `valida_soglia.m` | misura il flag di contatto contro la forza vera, una run sola |

---

## Una nota sul metodo, che è il contributo trasferibile

Tre regole adottate dopo aver sbagliato almeno una volta per ciascuna: il
**pavimento di rumore** con la soglia di leggibilità a 3×, il **criterio
scritto prima di lanciare**, le **smentite che restano scritte**. Non sono
formalità: la parte più consistente del lavoro è stata trovare e correggere
due difetti del banco di prova, entrambi capaci di invertire le conclusioni, e
nessuno dei due si segnalava da solo — il robot camminava, l'animazione era
plausibile, le metriche uscivano.
