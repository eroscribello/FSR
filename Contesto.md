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
| `git log` | la narrazione di ogni cambiamento, col perché. *[2/10] Erano anche i `commit_msg_*.txt` nella radice: cancellati dal disco, il testo è tutto nella storia git* |

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
   *Eccezione deliberata, 1/10 sera:* Andrea e il collega hanno salvato i due
   modelli con lo slew rate. Se si rifà, la procedura è: MATLAB riaperto,
   `bdclose all; startup_phantomx; open_system(...)`, **nessuno script di task
   prima di salvare**, poi verifica sull'XML che sia cambiato solo quello che si
   voleva (confronto con la versione precedente estratta dallo zip).
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

~~In `-batch` non c'è finestra: le figure si salvano, non si mostrano.~~
**[RITIRATO 1/10]** *Il Mechanics Explorer con l'animazione si apre anche in
`-batch`. Non chiuderlo mentre lo script sta ancora salvando il grafico: il
1/10 una sessione è rimasta bloccata a quel passo e va terminata a mano.* Le
figure le scrive `salva_grafico` in `grafici/`. I risultati finiscono in
`results/*.csv` e si leggono direttamente da lì.

**Una sessione per run, se le run sono più d'una e devono partire pulite.**
`startup_phantomx` svuota il workspace: un ciclo `for` in una sola sessione che
lo richiama a ogni giro perde la sua stessa variabile. Si lanciano più
`matlab -batch` in fila.

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

**[SUPERATO 1/10 sera]** *Dal 1/10 sera il Rate Limiter è in **tutti e due** i
modelli, ±0.4 m/s, uno per piede, tra `Sum` e gamba (in `phantomx_sim_zero`
`Sum2 → RR`, `Sum3 → MR`, `Sum4 → FR`, `Sum5 → RL`, `Sum6 → ML`, `Sum7 → FL`,
verificato sull'XML). La saturazione resta solo in C3, ma su C2 non
interverrebbe: C2 → C3 è di nuovo **un meccanismo solo**. Il paragrafo sotto
descrive com'era prima, e resta come storia.*

**[1/10] Ma il meccanismo di C3 ha tre pezzi, non uno.** `Saturation` e
`Rate Limiter` (sei per tipo, uno per piede) ci sono **solo** in
`phantomx_sim_attitude`: in `phantomx_sim_zero` non ce n'è nessuno, verificato
sull'XML. Non sono un'aggiunta indipendente — esistono *per* l'anello: la
saturazione limita la quota comandata perché l'uscita dei PID non ha limiti,
lo slew rate (29/9, commit "aggiunto slew rate…") toglie il tremolio che
l'anello induceva. Quindi C2 → C3 misura **l'anello d'assetto insieme ai suoi
limitatori**, e un effetto C2 → C3 non si attribuisce al solo anello finché non
si misura C2 + Rate Limiter da solo. La saturazione no: agisce sulla somma
`z_* + d_z_*` con limite inferiore 0.07, e la quota di C2 da sola sta fra
0.09 (z0 − `cfg.H`) e 0.17 (z0 + `cfg.c2.z_ext_max`), quindi su C2 non
interviene mai. Il Rate Limiter invece limita **ogni** comando di quota,
compreso quello della ricerca del terreno.

```matlab
delta_z = x_i * u_theta + y_i * u_phi;      % MATLAB Function4, calcola_delta_z
```

| | valore | note |
|---|---|---|
| PID beccheggio | **P = 10, I = 1, D = 0** | discreto a 1 ms, riferimento 0. Era P = 30, I = 0 fino al 30/9 |
| PID rollio | **P = 5, I = 0.7, D = 0** | aggiunto l'1/10 con I = 0.5, portato a 0.7 lo stesso giorno (commit "aggiornato attitude control e box"). Con 0.7 `diagnosi_appoggio` su T4 non trova tremolio |
| `Saturation` | `LowerLimit = 0.07` | upper al default |
| `Rate Limiter` | **±0.4 m/s** | era ±0.3 fino al 1/10 sera: tagliava il 32% del volo nominale (picco verticale del piede 0.375 m/s). A ±0.4 l'andatura nominale non viene toccata; agisce sulle celle di T2 da 1.2× in su. Uguale in `phantomx_sim_zero` |

**Con P = 30 e I = 0 l'anello faceva vibrare i piedi**: andatura visivamente
corretta, ma contatti frammentati sotto i 0.125 s, nessun appoggio riconosciuto
e `script_T4` che si fermava con un errore di indice. *[1/10 sera] Il tremolio
era stato notato per primo su **T6**, nell'animazione (Andrea); T4 è dove si è
manifestato come errore. Con slew ±0.4 `diagnosi_appoggio` su T6, C3: nessun
tremolio.* Diagnosticato con
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
`Spatial Contact Force`) sta nel modello d'assetto ed è **attivo su disco**.
Va tolto prima di qualunque misura, altrimenti si misura C3 carico contro C1 e
C2 scarichi: `commenta_carico` lo fa in memoria, e si ferma se un blocco è
stato rinominato. L'attrito cambiato il 1/10 (0.8/0.75) è quello del contatto
**del carico**: i 54 contatti dei piedi hanno 0.9/0.4 in tutti e due i
modelli, verificato sull'XML.
Dall'1/10 lo chiamano tutti gli `script_T*` (dopo `allinea_stimatore`, solo
sul modello di C3) e `rumore_metriche`. La campagna C3 dell'1/10 è stata fatta
prima, con il carico tolto dalla riga di comando: stesso effetto.

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
  **[1/10] E l'esito binario vale solo con `dev_lat_max` < 0.183 m**
  (`soglia_deriva_T6`, dalla geometria): oltre, il robot può essere passato
  di fianco agli ostacoli 3–4 invece che sopra. È quello che fa C2 (0.296 m).
- **Una differenza si dichiara solo se vale ≥ 3× il rumore misurato**, e il
  rumore va misurato **sullo stesso controllore**. Applicarlo con il pavimento
  di un controllore diverso ci ha fatto credere per un giorno che non fosse
  dichiarabile niente.
- **La campagna gira ad attuatore ideale.** La coppia di stallo dell'AX-12A è
  1.5 N·m (manuale ROBOTIS in `docs/`, verificato il 1/10: prima era scritto a
  memoria). **Il limite che conta per la marcia è 1/5 dello stallo, 0.3 N·m**
  (nota del costruttore, `cfg.tau_lavoro`): il giunto peggiore sta a **1.9–2.3
  volte** quel valore, per tutti e tre. Sotto lo stallo sì (39–46%), "con
  margine" no. I picchi d'urto superano lo stallo su <1% dei campioni.
- **Lo slew rate ±0.4 m/s è un pezzo dell'impianto comune, aggiunto da noi**
  (1/10 sera): non c'è nel paper. Va dichiarato. Senza, C2 deriva su T6.
- **Un parametro è stato tarato da noi**: `cfg.c2.soglia_tau`, da `[0.04 0.1 0.1]`
  a `[0.385 0.136 0.105]`. La terna nuova **esclude di fatto la coxa**, che
  ruota attorno all'asse verticale e dal contatto verticale non riceve coppia.
  È un **cambio di regola**, non una ritaratura, e va dichiarato in relazione.

---

## Tranelli che sono già costati tempo

| | |
|---|---|
| **`applica_inerzie` e `allinea_stimatore` vanno sempre in coppia** | le inerzie dell'URDF sono ~1000× troppo grandi; correggerle solo sul robot e non sul `rigidBodyTree` dello stimatore rende `\|τ_mis − τ_att\|` grande ovunque, il flag di contatto resta incollato a 1 e **C2 smette di cercare il terreno**. È successo dal 22 al 25 settembre senza che nulla lo segnalasse |
| **I due modelli non hanno le stesse inerzie SU DISCO** | **[SUPERATO 1/10 sera]** *salvando lo slew rate, anche `phantomx_sim_zero` è finito su disco con le inerzie già corrette (26 solidi): ora i due modelli sono uguali, e `applica_inerzie` le trova giuste su tutti e due. Resta obbligatoria per sicurezza: un pull di una versione vecchia le riporterebbe sbagliate.* Com'era: dal 30/9 `phantomx_sim_attitude` è stato salvato con le inerzie **già corrette** (`applica_inerzie` riporta 25/25 già corretti), mentre `phantomx_sim_zero` tiene ancora quelle dell'URDF e viene corretto a ogni run (0/25). Non cambia i risultati — `applica_inerzie` è idempotente — ma uno script che **dimentica** di chiamarla ottiene inerzie giuste su C3 e sbagliate su C1/C2, e non si vede. `taratura_T2.m` e `script_T4_limite.m` sono già in quella condizione |
| **Il `.slx` è binario ma è uno zip** | `unzip` e si legge l'XML: `simulink/systems/*.xml` per i blocchi e i collegamenti, `simulink/stateflow/chart_*.xml` per il codice delle MATLAB Function. È così che si è trovato un modello arrivato dal remoto con sei `From` senza `Goto`, che non compilava |
| **`estrai_funzioni.m`** | estrae il codice delle MATLAB Function in file di testo, per diffare due versioni del modello |
| **Chiudere i modelli in Simulink prima di `git pull`/`rebase`** | git riscrive il file sotto i piedi di MATLAB; se poi si risponde "salva" si sovrascrive la versione appena arrivata |
| **Google Drive** | tiene aperti i file binari e fa fallire `git pull` con *"unable to create file … File exists"*. `git status` può anche marcare file sporchi per stat mentre `git diff` è vuoto: non committarli |
| **Google Drive, la variante dell'1/10: dimensioni sbagliate nell'indice** | dopo un checkout git può registrare nell'indice una dimensione **tonda e sbagliata** (16384 B per un `.fig` da 433 kB, 2113536 per il `.slx`), come se Drive gli avesse riportato il file a metà scrittura. Sintomi: ` M` in `git status`, `git diff` vuoto, `update-index --refresh` che dice *needs update*, `rebase --continue` che risponde *"You must edit all merge conflicts"* senza conflitti, `reset --hard` che non pulisce. Rimedio: **`git add <quei file>`** — rilegge il contenuto, che è identico, e corregge solo la dimensione. Diagnosi: confrontare l'hash dei file col blob dell'indice. Ci è costato un pomeriggio |
| **I default dei modelli negli helper** | `applica_imbardata`, `applica_disturbo`, `abilita_log` hanno `phantomx_sim_zero` come modello di default. Uno script che non passa il modello funziona su C1 e C2 e su C3 **agisce sul modello sbagliato senza errori**: è successo in `script_T3` (C3 andava dritto) e sarebbe successo in `rumore_metriche`. Passare sempre il modello |
| **`init_gait` è uno script**, non una funzione | condivide il base workspace e definisce fra l'altro `k`, `j`, `cfg`, `A`, `B`, `C`, `D`, `ss`. Tutte le variabili degli script di task sono prefissate `t2_`, `t5_`, … per questo |
| **Gli angoli del log di Simscape escono in GRADI** | letti come radianti davano energia 3463 J e 215 m di scivolamento su 1.4 m percorsi, tutti plausibili a prima vista |
| **I `.mat` non sono tracciati** | una run non salvata non si recupera: per rifare un calcolo su una run vecchia bisogna rilanciare la simulazione |
| **I warning di `find_system` sui Variant Subsystem** | sono rumore di MATLAB, nel progetto non ci sono varianti |

---

## Stato al 1 ottobre 2026, sera

| | |
|---|---|
| impianto | **Rate Limiter ±0.4 m/s in entrambi i `.slx`** (salvato il 1/10 sera, verificato sull'XML) |
| campagna | **7 task × 3 controllori, rifatta tutta con il limitatore** il 1/10 sera. La precedente in `results/storico/pre_slew_20261001/` (con `LEGGIMI.md`) |
| righe di **C1** | identiche byte per byte a quelle senza limitatore, salvo le celle veloci di T2 |
| pavimento di rumore | **quarto giro**, tutti e tre, `results/diagnostica/rumore_slew.csv` (36 run). Solo questo vale |
| tabella dei confronti | `results/tabella_confronti.md`, generata da `tabella_confronti` (legge `rumore_slew`). Non si modifica a mano |
| fattibilità attuatori | tutti e tre: giunto peggiore 39–46% dello stallo = **1.9–2.3× il carico raccomandato** da ROBOTIS |
| non rifatto | `script_T4_limite` (righe pre-slew in `results/storico/pre_slew_20261001/T4_limite/`; fino al 2/10 stavano in `results/T4_limite/`, fra i risultati validi) |

### Fatto il 1/10

- ~~cancellare `applica_terreno_attitude.m`~~ — cancellato
- ~~verificare `script_T4` su C3~~ — passa; `diagnosi_appoggio`: tutte le
  zampe hanno tratti fermi validi, nessun tremolio
- ~~decidere il pavimento di T6~~ — resta 8 × 8: nessuno dei tre arriva al bordo
- ~~commentare il carico~~ — `commenta_carico.m`
- ~~campagna C3~~ — sette task
- ~~pavimento di rumore per C3~~ — `rumore_metriche` esteso (modello da
  `scegli_controllore`, coppia configurabile)
- ~~`commenta_carico` negli `script_T*`~~ — tutti e sette
- ~~`fattibilita.m` su C3~~ — generalizzato a `opt.ctrl`; sezione in
  `tabella_confronti.md`
- ~~fonte di `tau_max`~~ — manuale ROBOTIS in `docs/`; aggiunta la soglia del
  costruttore (1/5 dello stallo) a `fattibilita`
- ~~slew rate su tutti i modelli~~ — ±0.4, salvato; campagna, rumore (quarto
  giro), fattibilità, tabella e documenti rifatti

### Da fare, in ordine

1. **La guardia in `t4_salita`**: con meno di tre piedi in appoggio deve dire
   che il robot non cammina, non dare un errore di indice. Oggi non scatta,
   ma il difetto c'è.
2. **`verifica_attitude`**: stampa ancora "il nome del controllore è binario"
   (superato da `scegli_controllore`) e non controlla il carico, mentre
   `scegli_controllore` dice che lo fa.
3. **`script_T4_limite` con il limitatore**, se la pendenza limite serve
   ancora in relazione. Attenzione: è uno degli script che non chiamavano
   `applica_inerzie` (tranello sopra) — da controllare prima di lanciarlo.
4. Aperti in `stato_progetto.md` §7.6 — **non** avviarli senza che Andrea lo
   chieda. Deriva di C2 ed energia di C3 sono chiusi dallo slew rate.
5. Pulizia del repo e commenti.
6. Relazione e presentazione.

### Strumenti di diagnostica recenti

| | |
|---|---|
| `verifica_attitude.m` | il modello di C3 regge la catena di misura? Sette controlli, sola lettura |
| `diagnosi_appoggio.m` | perché `appoggio_cinematico` non trova appoggi: velocità del piede, tratti fermi, spettro |
| `stabilita_zmp.m` | ZMP, baricentro completo e margine sul poligono d'appoggio |
| `fattibilita.m` | coppia richiesta contro il limite del servo, dai CSV esistenti |
| `valida_soglia.m` | misura il flag di contatto contro la forza vera, una run sola. **In `archivio/`** dal terzo giro (2/10): fuori dal path |
| `commenta_carico.m` | toglie (o rimette) il carico dal modello di C3, in memoria |
| `soglia_deriva_T6.m` | la deriva laterale che basta a mancare un ostacolo di T6, dalla geometria. Sola lettura |
| `tabella_confronti.m` | C1–C2–C3 per task con il verdetto sul rumore, dai CSV → `results/tabella_confronti.md` |
| `rumore_metriche.m` | il pavimento di rumore; dall'1/10 per qualunque controllore e coppia (`rm_coppia`) |

---

## Una nota sul metodo, che è il contributo trasferibile

Tre regole adottate dopo aver sbagliato almeno una volta per ciascuna: il
**pavimento di rumore** con la soglia di leggibilità a 3×, il **criterio
scritto prima di lanciare**, le **smentite che restano scritte**. Non sono
formalità: la parte più consistente del lavoro è stata trovare e correggere
due difetti del banco di prova, entrambi capaci di invertire le conclusioni, e
nessuno dei due si segnalava da solo — il robot camminava, l'animazione era
plausibile, le metriche uscivano.
