# Piano di confronto MPC vs cinematico — PhantomX

**Progetto:** FSR 2025–26, categoria legged robots
**Aggiornato:** 17 settembre 2026 · orizzonte: 2 settimane
**Riferimenti:** Ding et al. (MPC convesso representation-free, hw4) · Arrigoni, Zangrandi, Bianchi, Braghin, *Control of a Hexapod Robot Considering Terrain Interaction*, Robotics 2024, 13, 142

> **Revisione del 17 settembre.** Due modifiche rispetto alla versione del 3
> settembre, entrambe in §6: **T2 passa da tre a cinque punti** di velocità, e
> la definizione del task viene fissata in `cfg.t2_fattori` invece di essere
> ripetuta negli script. La motivazione è una misura, ed è riportata in §6.
> Cambia di conseguenza l'ordine di ripiego in §8.

> **Correzione rispetto alla revisione del 2 settembre.** La versione precedente
> descriveva il controllore cinematico puro come «reimplementazione fedele del
> paper». **È sbagliato**: quello è il controllore NUKE di serie, cioè la
> *baseline* contro cui il paper misura sé stesso, non l'architettura proposta
> dagli autori. La §2 è stata riscritta di conseguenza, e la correzione si
> propaga alla matrice dei task (§5) e al calendario (§6).

---

## 1. Dove siamo davvero

| voce del piano | stato |
|---|---|
| G1–2 · estensione SRB + MPC a 6 zampe, simulazione ridotta su piano | **fatto**, gira |
| G3–5 · baseline cinematico su Simscape | **fatto**, e più solido del previsto |
| — · riordino del repository e allineamento dei due simulatori | **fatto** (non era a piano) |
| G6–8 · robustezza e definizione metriche | **fatto** — `metriche.m` + i due adattatori |
| — · scenari di terreno da script (`applica_terreno`) | **fatto** (non era a piano): T1, T5, T6 verificati |
| G9–11 · Simscape ad alta fedeltà per la demo | **decisione aperta**, vedi §4 |
| G12–14 · plot, relazione, video, slide | da fare |

Il riordino non era previsto ma ha pagato: ha fatto emergere che l'allineamento
fra i due simulatori non era mai stato salvato su disco. Le lunghezze dei link,
i tempi dell'andatura e la tibia corretta esistevano solo nella cronologia della
chat. Oggi c'è `phantomx_config.m`, letto sia dall'MPC sia dal baseline, con
controlli automatici che bloccano l'esecuzione se i due divergono. Se la cosa
fosse emersa durante la campagna di misura, le run già fatte sarebbero da
buttare.

---

## 2. Che cosa fa davvero il paper di riferimento

L'architettura di Arrigoni et al. ha tre pezzi:

1. **IK estesa (§3).** La formulazione di NUKE viene ampliata perché accetti
   *orientamento e quota del corpo* come ingressi aggiuntivi, oltre alla
   posizione degli end-point delle zampe. È ciò che permette di tenere il corpo
   orizzontale su terreno inclinato *noto*.
2. **Modello dinamico leggero (§4).** Stima le coppie attese ai diciotto servo,
   con un costo computazionale compatibile con l'hardware di bordo.
3. **Controllo in retroazione (§5).** Confronta la **coppia attesa dal modello**
   con la **coppia misurata dai servo**. Quando le due coincidono, la zampa è
   considerata a terra e il suo movimento viene interrotto.

Il punto 3 è il contributo. Non è un controllore d'assetto retroazionato: è un
**rilevatore di contatto basato sulla coppia**, e serve a evitare due fallimenti
opposti — la zampa che resta appesa perché il terreno è più basso del previsto,
e l'*overstepping* che destabilizza il corpo quando è più alto. Gli autori
osservano che entrambi accadono **solo nella fase di abbassamento del piede**,
quindi la retroazione va invocata solo in quella finestra. Notevole: tutto questo
senza sensori di forza ai piedi e senza IMU, sfruttando solo il *load feedback*
dei Dynamixel AX-12A.

Va citata anche la loro §7, perché è precisa e ci riguarda:

> *«the current architecture lacks a localization and feedback control that
> considers the robot's position on the terrain map, resulting in an open-loop
> trajectory execution»*

Cioè: **la traiettoria è eseguita in anello aperto; è la rilevazione del contatto
a essere in anello chiuso.** È una distinzione che va riportata con precisione
nella relazione, perché delimita esattamente ciò che l'MPC può rivendicare.

Le figure 15 e 18 del paper confrontano NUKE di serie contro la logica in
retroazione. Quel confronto è la loro tesi — ed è la ragione per cui non
possiamo usare NUKE di serie come nostro unico termine di paragone.

---

## 3. Tre controllori

| | controllore | cosa fa | ruolo nel confronto |
|---|---|---|---|
| **C1** | NUKE feed-forward | IK analitica + gait tripode, giunti in posizione, nessuna retroazione | la **baseline del paper**, non il paper |
| **C2** | Arrigoni closed-loop | C1 + posa del corpo negli ingressi dell'IK + rilevazione del contatto dalla coppia in fase di abbassamento | **la reimplementazione del paper**: il vero termine di paragone |
| **C3** | MPC convesso | ottimizzazione delle GRF su orizzonte recedente, mappatura `τ = Jᵀf` in coppie di giunto | il contributo del progetto |

Perché tre e non due. Con il solo C1 si confronterebbe l'MPC contro la baseline
*che il paper stesso batte*: un uomo di paglia, e il primo rilievo che
arriverebbe in discussione. Con C1 e C2 entrambi presenti, invece, la scala
diventa leggibile e separa due contributi che altrimenti restano confusi:

- **C1 → C2** misura quanto vale la **retroazione sul contatto**, a parità di
  architettura cinematica;
- **C2 → C3** misura quanto vale l'**ottimizzazione**, a parità di informazione
  disponibile.

La domanda a cui un confronto a due controllori non sa rispondere — *l'MPC vince
perché è ottimo, o perché è l'unico che guarda lo stato del corpo?* — con C2 in
mezzo diventa misurabile.

### Nota implementativa su C2

In Simscape C2 è **più semplice** che sull'hardware reale: la coppia ai giunti si
legge direttamente dai sensori di giunto, quindi non serve il modello dinamico
di stima del punto 2. Resta la logica: durante l'abbassamento del piede, quando
la coppia al femore supera una soglia, la zampa è a terra e si blocca lì.

La semplificazione va **dichiarata** nella relazione: noi misuriamo la coppia,
loro la stimano. È a nostro vantaggio e va detto, non nascosto.

**Interruttore C1/C2**, senza blocchi aggiuntivi: con soglia infinita il
confronto è sempre falso, la zampa non viene mai bloccata e il comportamento
torna quello in anello aperto.

```matlab
cfg.c2.attiva = false;   % C1  ->  c2_soglia = inf
cfg.c2.attiva = true;    % C2  ->  c2_soglia = cfg.c2.soglia_tau
```

### Il PD d'assetto

Il PD su rollio e beccheggio sviluppato durante il debug del baseline
(`attitude_hold.m`) **non fa parte del paper**. Resta disponibile come variante
opzionale, ma non va presentato come "il controllore di Arrigoni". Se il tempo
lo consente può diventare un C2-bis interessante; altrimenti si accantona senza
danno.

---

## 4. La decisione aperta: su quale impianto si confronta

La proposta prevedeva l'MPC sul simulatore ridotto a corpo rigido e il baseline
su Simscape. Sono **due impianti diversi**: qualunque differenza di prestazione
sarebbe ambigua fra effetto del controllore ed effetto del simulatore.

Ostacolo tecnico: i giunti Simscape sono oggi **attuati in posizione**, mentre
l'MPC ha bisogno di **coppia in ingresso**. Sono modalità mutuamente esclusive
sullo stesso giunto, quindi serve una variante del modello.

**Opzione A — entrambi in Simscape (consigliata).** Si duplica il modello in una
variante a giunti attuati in coppia, si aggiunge la mappatura `τ = Jᵀf`, si
riusa lo schedule a tripode già validato. Lo stato del corpo per l'MPC arriva dal
`Transform Sensor` già collegato. Costo: 3–4 giorni. Risultato: confronto a
parità di terreno, contatto, attrito e condizioni iniziali. Nessun asterisco
nella relazione.

**Opzione B — impianti separati.** Più veloce, ma il confronto va dichiarato
qualitativo e il limite scritto esplicitamente. Praticabile solo come ripiego.

**Raccomandazione: opzione A.**

---

## 5. Metriche

Cinque famiglie. Ogni run produce la stessa riga di tabella, generata da
`metriche.m` — non a mano. I due simulatori scrivono nella stessa struttura
normalizzata (`run_vuoto.m` è il contratto) tramite `adatta_simscape` e
`adatta_mpc`: le metriche conoscono un solo formato.

**A. Esecuzione del task.** Errore RMS della velocità longitudinale rispetto al
comando `[m/s]`; deviazione laterale accumulata dalla retta nominale `[m]`;
errore di imbardata finale `[rad]`; distanza percorsa in un tempo fisso `[m]`;
frazione del task eseguita **con segno** lungo la direzione comandata;
**tasso di successo** su N run, dove ribaltamento, arresto o avanzamento sotto
`opt.frac_min` contano come fallimento.

**B. Planarità del corpo.** È l'obiettivo dichiarato del paper, quindi la
metrica su cui il cinematico gioca in casa. RMS di rollio e beccheggio `[rad]`;
picco di |rollio| e |beccheggio| `[rad]`; RMS della deviazione di quota `[m]`.

**C. Sforzo di attuazione.** Coppia RMS e di picco per giunto `[N·m]`; frazione
di tempo oltre il limite nominale dell'AX-12A `[%]`; energia meccanica
`∫Σ|τ·ω|dt` `[J]`; **cost of transport** `E/(mgd)`, adimensionale e
confrontabile con la letteratura.

**D. Qualità del contatto.** Scivolamento cumulato dei piedi in appoggio `[m]`;
eventi di distacco non previsti; picco di forza normale normalizzato al peso;
**dispersione del carico** fra le sei zampe. Per C2 si aggiunge una metrica sua:
**latenza di rilevazione del contatto**, cioè il ritardo fra tocco reale e
blocco della zampa.

**E. Robustezza.** Le famiglie A–D ripetute in condizioni degradate: massa
sottostimata del 10% (richiesta esplicita del template Progetto 3); forza
esterna costante di 1 N lungo x; gradino non modellato di 1, 2, 4 cm; attrito
statico ridotto da 0,9 a 0,4.

---

## 6. Matrice dei task

| | task | terreno | discrimina C1 / C2 ? | perché c'è |
|---|---|---|---|---|
| **T1** | piano, rettilineo, velocità nominale | liscio | **no** | riferimento di base |
| **T2** | piano, rettilineo, **cinque velocità** | liscio | **da riverificare** | curva prestazione–velocità: dove il cinematico cede e l'MPC no |
| **T3** | traiettoria curva, imbardata costante | liscio | **no** | il cinematico gestisce l'imbardata per costruzione |
| **T4** | rampa, salita e discesa | liscio + rampa | **sì** | scenario del paper: terreno noto, corpo da tenere orizzontale |
| **T5** | ostacolo singolo non modellato | liscio + 1 ostacolo | **sì, molto** | scenario del paper: terreno **ignoto** |
| **T6** | terreno irregolare, ostacoli multipli | imperfetto + 7 ostacoli | **sì** | stress test, corrisponde alla loro fig. 19b |
| **T7** | disturbo impulsivo laterale | liscio | parziale | recupero dopo perturbazione |

Su terreno piano la retroazione del paper non interviene mai: C1 e C2 danno la
stessa traiettoria, e T1–T3 servono a caratterizzare C3 contro il cinematico. Il
confronto fra i due cinematici vive su **T4, T5 e T6** — che sono poi esattamente
gli scenari sperimentali degli autori. Se il tempo obbligasse a tagliare, si
tagliano task piani, non quelli accidentati.

### T2: definizione canonica

**La griglia sta in `cfg.t2_fattori`, e in nessun altro posto.** Prima era
scritta due volte e in modo incompatibile — tre fattori variando la velocità in
`esegui_misure`, cinque variando il periodo in `script_T2` — quindi i due CSV
non erano confrontabili e la curva non si poteva disegnare.

```matlab
cfg.t2_fattori = [0.5 0.75 1.0 1.5 2.0];
```

**La variabile indipendente è la velocità comandata**, non il periodo. Due
ragioni: tutte le metriche della famiglia A sono definite *rispetto* al comando
(`meta.vel_d`, `err_vx_rms`, `avanzamento`, `frazione_task`), e la velocità è
l'unica grandezza comune ai tre controllori — C1 e C2 accettano un periodo, C3
accetta una velocità. Il periodo si ricava: `T = S / (beta_stance · v)`, così la
lunghezza del passo resta quella validata e la differenza fra le celle è
attribuibile alla velocità e non a un diverso punto di lavoro della gamba.

**Cinque punti, non tre.** Con 0,5× / 1× / 2× la misura si legge come «degrada
ad alta velocità». Con i punti intermedi si vede il risultato vero: l'ottimo è
**stretto** e il cedimento è di **due tipi opposti**.

| cella | frazione del task | CoT | tripode | dispersione carico |
|---|---|---|---|---|
| 0,5× | 86% | 1,27 | 36% | 16% |
| 1,0× | 116% | 1,12 | 95% | 6% |
| 2,0× | 18% | 20,27 | 6% | — (11% del tempo in volo) |

A bassa velocità il corpo si assesta dentro la cedevolezza del contatto trovando
un equilibrio asimmetrico: i sei piedi si sfalsano di 2,70 mm contro 0,32 a
velocità nominale, e una zampa per terna resta scarica. Ad alta velocità è
perdita di appoggio. In entrambi i casi la causa è la stessa: il controllore
comanda posizioni e non ha alcuna informazione sul carico.

**Conseguenza sulla colonna «discrimina C1 / C2»:** T2 era classificato come non
discriminante. La rilevazione del contatto dalla coppia interverrebbe proprio sul
meccanismo osservato a 0,5×, quindi la classificazione va **riverificata** con
C2 attivo. Se confermata, T2 smette di essere un task solo descrittivo.

### Rigore statistico

Ogni cella va ripetuta **N = 5 volte** con fase iniziale dell'andatura
randomizzata e piccola perturbazione sulla posa di partenza, riportando media e
deviazione standard. Un run singolo è un aneddoto; cinque sono una misura.

Griglia: 3 controllori × (6 task + 5 celle di T2) × 5 run = **165 simulazioni**
nominali, più le condizioni di robustezza. Con uno script di campagna in batch è
una notte di calcolo, non una settimana di lavoro manuale.

> **Attenzione alla ripetibilità.** `fcn_FSM` usa variabili `persistent`: senza
> `clear fcn_FSM` prima di ogni run, la run *n+1* riparte dallo stato lasciato
> dalla *n* e le cinque ripetizioni non sono confrontabili. È la prima riga da
> scrivere nello script di campagna.

---

## 7. Calendario rivisto

| giorni | attività | esito |
|---|---|---|
| **G1** | script di raccolta metriche; congelare le versioni dei file | ogni run produce una riga di tabella — **fatto** |
| **G2** | **C2: rilevazione del contatto da coppia** + posa del corpo nell'IK | i tre controllori esistono — **fatto nel modello** |
| **G3–5** | variante Simscape a giunti in coppia; mappatura `τ = Jᵀf`; MPC in the loop su piano | C3 gira sullo stesso impianto di C1 e C2 |
| **G6–7** | tuning dell'MPC su rampa e ostacolo | T4 e T5 girano per tutti e tre |
| **G8–9** | campagna completa in batch | tabelle grezze |
| **G10** | analisi e grafici | figure della relazione |
| **G11–13** | relazione (max 25 pp, inglese) | testo consegnabile |
| **G14** | video e slide | pacchetto completo |

**Lo script di metriche a G1 è ciò che fa la differenza.** Se ogni simulazione
scrive da sola la sua riga, la campagna di G8–9 è un ciclo `for`. Se le metriche
si estraggono a mano dai grafici, la campagna non si chiude in due giorni e i
numeri non saranno confrontabili fra loro.

---

## 8. Scaletta di ripiego, in ordine di rinuncia

1. **T7 e T3** — disturbo impulsivo e traiettoria curva. Belli da avere, non
   necessari alla tesi.
2. **I punti intermedi di T2** — si tolgono 0,75× e 1,5× e si torna a tre celle.
   Costa il risultato sulla *strettezza* dell'ottimo, non la curva: è una
   rinuncia parziale, ed è il motivo per cui T2 non è più il secondo intero da
   sacrificare.
3. **L'opzione A** — la variante Simscape dell'MPC. Si ripiega sul confronto a
   impianti separati, dichiarando il limite in relazione e in discussione.
4. **C2** — ultimo da toccare. Rinunciarci significa confrontare l'MPC contro la
   baseline che il paper stesso supera, e va scritto a chiare lettere.

---

## 9. Stato tecnico e cose aperte

Non sono voci di piano, sono debiti noti. Elencati perché non si perdano.

**Sul modello Simscape**

- `body_z0 = 0.25 m` contro i `0.11 m` della derivazione geometrica: all'istante
  zero i piedi partono 14 cm in aria e atterrano a 1,7 m/s. Da provare la
  quota geometrica e guardare i primi 0,5 s.
- **Tibia 0,12 o 0,153 m.** Oggi il simulatore è internamente coerente su 0,12
  (l'IK e la sfera di contatto concordano), ma il robot vero misura 0,153.
  Passare al valore reale richiede tre modifiche simultanee — `lt` in `inv_kyn`,
  `cfg.lt`/`cfg.foot_offset`, la traslazione delle sei sfere — più una
  ritaratura di `z0`. **Va deciso prima della campagna, non durante**, e va
  scritto in relazione come scelta di modellazione.
- **Geometria della rampa (T4)**: il cablaggio è risolto (la rampa prende in
  prestito lo slot `ostacolo7`), ma il Brick è 4×4 m e a 8° compenetra il
  pavimento. Da ridimensionare e riposizionare, non da ricablare.
- **Ostacoli 4–7**: poggiano sui rilievi del terreno imperfetto (da +35 a
  +71 mm). Su pavimento liscio restano in aria: in T5 sono utilizzabili solo
  `ost1`, `ost2`, `ost3`.
- Due `Data Store Write` scrivono nella stessa memoria a `t = 0` senza ordine
  garantito: non determinismo in un modello che deve produrre misure ripetibili.
- `sys_filter`, filtri del primo ordine con `τ = 0,05 s` sui comandi di giunto,
  con nomenclatura ereditata da un esperimento di reinforcement learning
  abbandonato. A doppia velocità lo swing dura 200 ms e il filtro ne taglia il
  25%: va escluso che il fallimento della cella 2× sia del banco di prova
  invece che del controllore.
- Semantica dei To Workspace `c_*` e `z_*`: da chiarire se siano forze o flag.

**Sull'MPC**

- `lb = -Fzd` nel QP permette **forze verticali negative**, cioè piedi che
  tirano il terreno. Su un quadrupede con due zampe in appoggio è poco
  sfruttabile; con sei zampe la ridondanza è molto maggiore. Da portare a zero e
  verificare che il QP resti ammissibile.
- `fcn_get_disturbance` è ancora quello del quadrupede: ampiezze tarate su un
  robot del MIT (22 N su un robot che ne pesa 15,3 di peso proprio), istanti
  hardcoded su una run da 4,5 s, punto di applicazione allo spigolo di un
  rettangolo che non esiste. E comunque il disturbo è azzerato due volte in
  `MAIN`. Da riscrivere prima dei task di robustezza.
- L'`ExitFlag` di qpSWIFT viene catturato e mai controllato: un QP che fallisce
  restituisce forze qualunque e si vede solo un robot che si comporta male.
- `R` pesa più che su un quadrupede: con sei zampe la distribuzione delle forze
  è sovra-determinata nelle fasi a supporto multiplo, e `R` è l'unica cosa che
  sceglie fra le combinazioni equivalenti. Da giustificare, non da subire.

**Aperti in generale**

- `attitude_hold.m` non è mai arrivato su disco e i suoi due segni non sono mai
  stati determinati sperimentalmente.
- `cfg.J`, l'inerzia del corpo rigido singolo, è stimata e non calcolata dalle
  mesh. Entra direttamente nel modello di predizione dell'MPC: vale mezz'ora
  ricalcolarla con `importrobot` sull'URDF.
- Il progetto vive su Google Drive, con percorsi contenenti spazi. La cache di
  compilazione viene sincronizzata inutilmente, un `.slx` toccato dal sync
  durante un salvataggio può corrompersi, e `git pull` fallisce con
  *«unable to create file … File exists»*. Consigliato spostare il lavoro in
  locale e usare Drive solo per la condivisione.