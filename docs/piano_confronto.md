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

**Interruttore C1/C2.** L'interruttore è `c2_par(1)`, che `init_gait` assembla
da `cfg.c2.attiva`, e in pratica si usa `OVERRIDE_C2`:

```matlab
OVERRIDE_C2 = false;  init_gait     % C1, anello aperto vero
OVERRIDE_C2 = true;   init_gait     % C2
clear OVERRIDE_C2                   % torna a cfg.c2.attiva
```

> **[CORRETTO]** Questo paragrafo diceva che bastava portare la soglia di
> coppia a infinito. **È falso e fa il contrario.** Con la soglia infinita il
> flag di contatto è sempre 0, e con il flag a 0 la ricerca entra nel ramo di
> **discesa** a ogni appoggio — `z0 = 0.140` contro la soglia `0.138` — quindi
> la zampa scende di 3 cm a 0,06 m/s. È la ricerca al massimo della sua
> autorità, non l'anello aperto. Dettagli in
> `docs/come_funziona_ricerca_terreno.md`.

### [MISURATO] Su terreno piano C2 non serve, e si fa sentire

*Numeri del 21/9, forze dai sensori (vedi §9). La prima versione di questa
sezione usava le forze ricostruite e ne traeva una conclusione sul contatto che
con i sensori **non regge**: è corretta qui sotto.*

T2 era classificato come **non discriminante** fra C1 e C2, con questa
motivazione: su terreno piano la retroazione sul contatto non ha niente da
trovare, quindi non interviene. **La prima metà è confermata, la seconda no.**

Avanzamento, a parità di cella (`frazione_task`):

| | 0.50× | 1.00× | 1.20× | 1.50× | 2.00× |
|---|---|---|---|---|---|
| C1 | 0.847 | 1.163 | 1.119 | 0.826 | −0.482 |
| C2 | 0.845 | 1.140 | 1.112 | **0.910** | −0.371 |

Entro il 2% nelle prime tre celle: sull'avanzamento T2 non discrimina, come
previsto. Ma C2 **interviene**, e si vede nel moto e nell'energia:

| C2 rispetto a C1 | 0.50× | 1.00× | 1.20× |
|---|---|---|---|
| `z_media` | +6.6 mm | +5.5 mm | +4.6 mm |
| `cot` | 2.21 → **4.39** | 2.32 → 2.64 | 3.04 → 3.18 |
| `distacchi` | 38 → **155** | 53 → 68 | 46 → 52 |
| `potenza_max` | 29 → 195 W | 13 → 48 W | 23 → 57 W |

**Il meccanismo si legge in `z_media`:** con C2 il corpo sta 5–7 mm più in alto
a ogni velocità. La ricerca estende le zampe verso il basso, quindi il corpo si
alza — su un terreno dove non c'è niente da cercare.

Perché si attiva: la ricerca parte quando `c(i) == 0` e il comando vuole la
zampa a terra. All'istante del touchdown la forza è sotto la soglia per qualche
decina di ms, quindi `c = 0` e la ricerca **parte a ogni appoggio**, estendendo
di qualche mm. Nell'articolo la ricerca serve a una zampa che **non trova** il
terreno, non al transitorio normale di contatto.

È un difetto di implementazione, non del metodo, e la correzione naturale è un
**ritardo di innesco**: non cercare finché non è passato un tempo minimo dal
touchdown comandato. Da provare, non ancora provato.

> **[CORRETTO] Il contatto non peggiora ovunque.** Con le forze ricostruite
> questa sezione diceva che C2 peggiora la qualità del contatto a tutte le
> velocità (`sotto3_frac` 0.004 → 0.169 a 1×). Con i sensori:
>
> | `sotto3_frac` | 0.50× | 1.00× | 1.20× |
> |---|---|---|---|
> | C1 | 0.249 | 0.117 | 0.310 |
> | C2 | **0.114** | 0.193 | **0.220** |
>
> C2 è peggio solo a 1×, meglio a 0.5× e a 1.2×. L'affermazione sul contatto
> era un artefatto della misura. Quelle su moto ed energia, sopra, non
> dipendono dalla fonte delle forze e restano.

**Vicino al bordo invece C2 aiuta.** A 1.50× l'avanzamento passa da 0.826 a
0.910 e la potenza di picco si **dimezza**, da 285.7 a 139.3 W, con `cot` da
7.48 a 6.51. Lì le zampe perdono davvero l'appoggio, quindi la ricerca ha
qualcosa da trovare e il suo costo è ripagato. A 2.00× non basta più: il robot
indietreggia meno (−0.371 contro −0.482) ma consuma quasi il doppio
(`cot` 30.9 contro 18.0).

Da riportare così: **C2 sposta il costo, non lo elimina.** Paga in **energia e
distacchi** dove il terreno è noto — fino al doppio del cost of transport a
0.5× — e recupera dove il terreno sorprende. È il compromesso che un MPC con
orizzonte non dovrebbe dover fare, ed è il confronto da impostare con C3.

Aperto: `pitch_rms` con C2 vale ~0.03 rad a tutte le velocità basse, quasi
costante, contro 0.001 di C1. Un valore costante somiglia a un **offset** più
che a un'oscillazione, e `pitch_rms` è l'RMS dell'angolo, quindi le due cose si
confondono. `origine_beccheggio` le separa e non è ancora stato lanciato:
finché non lo si fa, «C2 fa beccheggiare il robot di 2°» non è sostenuto.

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
| **T2** | piano, rettilineo, **cinque velocità** | liscio | sull'avanzamento **no**, su energia e distacchi **sì** (§3) | curva prestazione–velocità: dove il cinematico cede e l'MPC no |
| **T3** | traiettoria curva, imbardata costante | liscio | poco: C2 +3 punti d'imbardata, stessi costi di T2 (§9) | C1 realizza il **15%** dell'imbardata comandata (§9): caratterizza C3 contro il cinematico |
| **T4** | rampa, salita e discesa | liscio + rampa | **sì** | scenario del paper: terreno noto, corpo da tenere orizzontale |
| **T5** | ostacolo singolo non modellato | liscio + 1 ostacolo | **sì, molto** — misurato: rollio −52%, deriva annullata (§9) | scenario del paper: terreno **ignoto** |
| **T6** | terreno irregolare, ostacoli multipli | imperfetto + 7 ostacoli | **sì** | stress test, corrisponde alla loro fig. 19b |
| **T7** | disturbo impulsivo laterale | liscio | parziale | recupero dopo perturbazione |

Su terreno piano la retroazione del paper **non dovrebbe** intervenire, e
sull'avanzamento infatti C1 e C2 coincidono. Ma interviene lo stesso, a ogni
touchdown, e costa energia (§3): T1–T3 servono soprattutto a caratterizzare C3
contro il cinematico. Il
confronto fra i due cinematici vive su **T4, T5 e T6** — che sono poi esattamente
gli scenari sperimentali degli autori. Se il tempo obbligasse a tagliare, si
tagliano task piani, non quelli accidentati.

### T2: definizione canonica

**La griglia sta in `cfg.t2_fattori`, e in nessun altro posto.** Prima era
scritta due volte e in modo incompatibile — tre fattori variando la velocità in
`esegui_misure`, cinque variando il periodo in `script_T2` — quindi i due CSV
non erano confrontabili e la curva non si poteva disegnare.

```matlab
cfg.t2_fattori = [0.5 1.0 1.20 1.5 2.0];
```

Il terzo punto era 0.75×; ora è 1.20×, l'ultima cella con moto del corpo
stabile. Tre punti di confronto (0.5, 1.0, 1.20) e due fuori inviluppo (1.5,
2.0). Perché 1.20× non si chiama «limite», e i due inviluppi da riportare: vedi
`common/phantomx_config.m`, sezione T2.

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

### Risultato chiuso: C1 scivola, e si vede nella velocità

Misurato in C1 su T1, terreno piano, 10 s. Tre grandezze indipendenti che
concordano entro l'1,6%:

| grandezza | valore |
|---|---|
| spostamento del piede **in avanti** durante l'appoggio | +9,54 mm (media su 60 appoggi) |
| avanzamento del corpo previsto: `2·(S + slip)` | 2·(60 + 9,54) = 139 mm/ciclo |
| avanzamento misurato | 136,8 mm/ciclo |
| velocità comandata vs misurata | 0,120 → 0,137 m/s, **+14%** |
| dispersione del carico fra le sei zampe | 27% |

La relazione è `Δcorpo = Δpiede − Δ(piede nel frame corpo)`, e con
`Δ(piede nel frame corpo) = −S` i due contributi si **sommano**: un piede che
striscia in avanti regala avanzamento. Il robot va più veloce di quanto gli si
chieda *perché* i piedi slittano.

**Interpretazione.** Il tripode comandato in posizione è sovra-vincolato: tre
piedi a terra impongono sei vincoli orizzontali su tre gradi di libertà nel
piano. Le zampe si contrastano, le forze interne crescono oltre l'attrito
disponibile e i piedi cedono. Il controllore comanda posizioni e non ha modo di
sapere che stanno slittando — è il limite strutturale che l'MPC, distribuendo
le forze dentro il cono d'attrito, può rimuovere.

**Perché è una misura e non un artefatto.** Le posizioni dei piedi vengono
dalla cinematica diretta sugli angoli di giunto; se quella catena fosse
sfasata, il conto sopra non chiuderebbe all'1,6%. La chiusura valida `pf`.

**Conseguenza sulle metriche.** `err_vx_rms` confronta la velocità misurata con
quella comandata: su T1 e T2 quel numero contiene il +14% di scivolamento, non
un errore di inseguimento. Va riportato insieme a `slip_tot`, altrimenti si
attribuisce al controllore un errore di velocità che è invece perdita di
aderenza.

### T2 non va ritarato: H è un vincolo, non un parametro libero

Campagna di taratura in C1 su T1/T2, 26 run: `z0` su 0,140 e 0,142, `H` da
0,020 a 0,065, velocità 0,5× 1,0× 2,0×.

**`z0` è insensibile.** Fra 0,140 e 0,142 la dispersione del carico cambia dello
0,2%. Resta 0,14.

**`H` ha un ottimo degenere su terreno piano.** Tutte le metriche migliorano
monotonicamente al calare di `H`, fino al bordo della griglia, due volte di
seguito. Il motivo si legge in `appoggio_medio`, che a 1,0× passa da 3,42 con
`H` = 0,035 a **3,64** con `H` = 0,020: tende a 6. Il minimo della dispersione
del carico si ottiene con i piedi che **non si alzano**, strisciano, e il carico
è perfettamente distribuito perché tutte e sei le zampe sono sempre a terra.
Il criterio premiava l'andatura che non cammina.

`H` serve a scavalcare. Su terreno piano non c'è niente da scavalcare, quindi
il suo ottimo lì è zero per costruzione, e usare quel valore su T5/T6 farebbe
inciampare il robot. Il valore si deriva dal requisito di franco:

| vincolo | quota |
|---|---|
| penetrazione del contatto | 4,3 mm |
| oscillazione verticale del corpo | 8,6 mm |
| franco minimo su piano | ~15 mm |
| ostacolo di T5 | 45 mm |
| **franco minimo per T5/T6** | **~55 mm** |

`cfg.H = 0,05` è quindi giustificato dai task accidentati, e va dichiarato come
**vincolo di franco**, non come taratura. Una sola taratura per tutte le
velocità e per tutti i task.

**Il costo del franco, quantificato.** A 1,0× il cost of transport passa da
1,43 con `H` = 0,05 a **0,94** con `H` = 0,02: il **34% di energia in più**
speso a portare in giro una capacità di scavalcamento che su terreno piano non
serve. È un limite strutturale del cinematico — l'altezza di volo è una
costante dell'andatura e il controllore non sa che il terreno è piatto — e un
punto su cui un MPC che pianifica sul terreno noto ha qualcosa da rivendicare.

**La cella 2× fallisce per inversione, non per taratura.** In tutte e sei le
combinazioni `frazione_task` è NEGATIVA (da −26% a −69%): il robot cammina
indietro, e nessun valore di `z0` o `H` la recupera.

**[CHIUSO] La causa: il tripode non regge l'andatura.** I sospetti sono stati
esclusi uno per uno: `sys_filter` è un residuo che nessun blocco legge, e i
giunti eseguono la corsa comandata a tutte le velocità (escursione del piede
71–72 mm, identica a 1×, 1.5× e 2×). Quello che cede è il corpo: a 2× rimbalza
di 81 mm su 154 di altezza di appoggio e perde l'aggancio di fase col contatto
(accordo 51%). Lo scivolamento a −30 mm è la conseguenza, non la causa. È un
limite del controllore, quindi la cella resta in relazione, letta con
`corpoZ_pp`/`corpoZ_vz` e non con `err_vx_rms`. Percorso completo e lezioni di
metodo in `archivio/README.md`.

(La prima versione di questa sezione parlava di «contatto eccellente, 3,46
piedi a terra»: veniva dalle forze ricostruite, ed è una delle ragioni per cui
la perdita di appoggio era stata esclusa troppo presto. Con i sensori, a 2× i
piedi a terra sono 2.33 e sotto tre il 54% del tempo.)

### [MISURATO] T3 con C1: il 15% dell'imbardata comandata

Sessione pulita, forze dai sensori, `yaw_d = ±0.1 rad/s`, 15 s:

| | +0.1 | −0.1 |
|---|---|---|
| `yaw_mis` [rad/s] | +0.0147 | −0.0153 |
| `yaw_rapporto` | 14.7% | 15.3% |
| `roll_rms` / `pitch_rms` [rad] | 0.0043 / 0.0018 | 0.0044 / 0.0015 |

Asimmetria fra i due versi **3.9%**, corpo orizzontale, imbardata parassita a
comando nullo 5·10⁻⁶ rad/s — il segnale è tremila volte sopra il rumore. Il
raggio effettivo è v/ω ≈ 0.139/0.0147 ≈ **9.4 m** contro gli 1.2 comandati.

Caratterizzazione da `diagnosi_imbardata`:

| caso | realizzato | lettura |
|---|---|---|
| arco 0.1 | 14.7% | la cella di campagna |
| arco 0.4 | 23.8% | cresce col comando: c'è una componente a soglia |
| destre specchiate | ≈ 0 | i due lati contribuiscono uguale: nessuna asimmetria destra/sinistra |
| **sul posto** (caso esatto) | **56%** | senza approssimazioni sul passo |

Due limiti diversi, da tenere separati in relazione:

- **56% sul posto** è il limite di aderenza del tripode comandato in posizione:
  lo stesso scivolamento che in rettilineo lo fa andare *più* veloce del
  comando (116%) qui gli toglie metà della rotazione. È del metodo.
- **15–24% sull'arco** è molto sotto il 56% perché le sei zampe condividono una
  sola lunghezza di passo, mentre in curva l'interna e l'esterna ne vorrebbero
  due diverse, e si contrastano. È della **nostra implementazione** — due blocchi
  di traiettoria invece di sei — e va dichiarato accanto al confronto con C3,
  altrimenti si attribuisce all'MPC un vantaggio che è in parte nostro.

`yaw_d` resta 0.1 per la campagna: il segnale è già ampiamente sopra il rumore,
e alzare il comando porta il `delta` per zampa a 26°, dove l'approssimazione sul
passo pesa di più.

**Tre errori di metodo, tutti costati giorni, da non ripetere:**

1. **Stato residuo della sessione.** Tutte le misure d'imbardata prima del 21/9
   sono da scartare: erano prese in una sessione MATLAB con stato residuo nel
   workspace, dove il robot in rettilineo aveva 0.455 piedi a terra e 4° di
   beccheggio. In sessione pulita, stesso codice: 2.90 e 0.001. **Ogni campagna
   parte da `clear all; bdclose all; startup_phantomx`.**
2. **Il segno giudicato sul rumore.** Con quella misura `applica_imbardata` è
   stato portato a `segno = −1`, e ci è rimasto tre giorni. Invertire il segno
   non cambiava niente — e andava letto come «la misura non vale», non come «il
   segno è giusto comunque». In sessione pulita −1 faceva girare il robot al
   contrario; +1, il valore geometrico, è quello giusto.
3. **Il segno prima del modulo.** Il controllo in `script_T3` guardava `sign()`
   senza chiedere un modulo minimo, quindi scattava sul rumore. Ora il modulo
   viene prima (soglia 5%, empirica: separa il 4.7% casuale del robot rotto dal
   15% riproducibile di quello sano).

**T3 con C2** — stessa sessione, stessa fonte delle forze:

| | C1 +0.1 | C1 −0.1 | C2 +0.1 | C2 −0.1 |
|---|---|---|---|---|
| `yaw_rapporto` | 14.7% | 15.3% | **17.2%** | **18.3%** |
| `z_media` [m] | 0.1520 | 0.1519 | 0.1575 | 0.1575 |
| `pitch_rms` [rad] | 0.0018 | 0.0015 | 0.0349 | 0.0351 |
| `cot` | 2.45 | 2.44 | 2.80 | 2.80 |
| `tau_max` [N·m] | 1.67 | 1.62 | 11.4 | 11.4 |

Asimmetria di C2 fra i due versi 6.8%, sotto il 15%. C2 realizza un po' più
imbardata di C1: +2.5 e +3.0 punti, nello stesso senso in entrambi i versi. È
coerente con una ricerca che tiene i piedi in contatto in curva, ma sono due
run: va detto come «leggermente maggiore», non come un guadagno misurato.

La cosa più solida è un'altra: **le firme di C2 misurate in T2 ricompaiono in
T3 con gli stessi valori.** Corpo 5.6 mm più alto (5.5 in T2 a 1×), `cot`
+14.5% (+14% in T2), `pitch_rms` ~0.035 rad (0.034 in T2). Due task diversi,
stesso meccanismo, stessa ampiezza: il difetto d'innesco della ricerca non è un
caso di una campagna.

**Da dichiarare: i picchi di coppia di C2.** `tau_max` arriva a 11.4 N·m, in T3
come in T2 (10.4 a 1×), contro gli 1.5 N·m del datasheet dell'AX-12A e 1.6–2.0
di C1. `frazione_saturo` resta piccola (1.3% del tempo), quindi sono picchi
brevi, ma sul robot vero un attuatore saturerebbe lì dove il simulatore, con
attuatori ideali, non lo fa. È un vantaggio che il banco concede a C2 e va
scritto.

**T3 è chiuso**, su C1 e C2. Resta aperta per entrambi i task solo l'origine
del `pitch_rms` di C2 (offset o oscillazione), che `origine_beccheggio` separa.

### [MISURATO] T5: sull'ostacolo C2 dimezza il rollio e annulla la deriva

Ostacolo 1, ~30 mm, spostato 0.5 m in avanti rispetto al `.slx` (vedi sotto),
velocità nominale, 20 s. `script_T5`, una run per controllore.

| | C1 | C2 | |
|---|---|---|---|
| escursione di rollio | 8.2° | **3.9°** | −52% |
| escursione di beccheggio | 8.6° | **6.9°** | −20% |
| corpo in z sull'ostacolo (pp) | 60 mm | **34 mm** | −44% |
| deviazione laterale massima | 0.25 m | **0.009 m** | |
| imbardata finale | −9.8° | **−0.03°** | |
| tempo sull'ostacolo | 11.2 s | 7.7 s | |
| frazione del task | 104% | 111% | |
| `tau_max` | 6.7 N·m | **20.0 N·m** | |
| `potenza_max` | 50 W | 166 W | |
| `cot` | 2.97 | 3.31 | +11% |

È il primo task in cui la ricerca del terreno fa quello per cui esiste. C1
esce dall'ostacolo girato di 10° e spostato di 25 cm di lato, e oscilla per
altri cinque secondi; C2 esce dritto e torna subito all'assetto di prima.

**Il prezzo va scritto accanto al guadagno.** `tau_max` di C2 arriva a
20 N·m, **13 volte** il datasheet dell'AX-12A (1.5 N·m). Il banco ha attuatori
ideali e lo concede; un robot vero saturerebbe proprio dove C2 si guadagna il
vantaggio. Formulazione per la relazione: *C2 dimezza il rollio e annulla la
deriva sull'ostacolo; su attuatori reali il vantaggio è da verificare.* È anche
un argomento per l'MPC, che può mettere i limiti di coppia nel problema.

**Come si misura** (dettagli in testa a `script_T5.m`):

- il passaggio si riconosce da un piede **fermo nel mondo** e più alto del
  pavimento di 15 mm, non dai sensori di forza — che vedono solo il pavimento
  (chiudono al 79–83% del peso in T5), quindi le colonne di contatto sono a
  `NaN`;
- l'assetto è un'**escursione** rispetto alla media in piano, presa fuori dalla
  finestra: così l'offset di −2° di C2 non lo penalizza né lo favorisce.

**Limiti:** una run per controllore, quindi nessuna stima di variabilità; e le
finestre non coincidono (C2 riconosciuto sull'ostacolo a 0.61 m, C1 a 0.74 m).
Le escursioni sono massimi, e la finestra di C1 è più lunga per le oscillazioni
dopo la discesa, che sono comportamento suo: il confronto non sfavorisce C2.

**Lo spostamento dell'ostacolo.** Nel `.slx` l'ostacolo 1 stava a 16 cm dalla
partenza: il robot ci arrivava a 1.2 s, prima della fine del transitorio.
`cfg.terreno.ost_dx.T5 = [-0.5 0 ...]` lo sposta di 0.5 m in avanti a runtime,
senza toccare il modello; ora il robot ci arriva a ~5 s. Il segno è negativo
perché i Rigid Transform degli ostacoli hanno x e z opposte al mondo (catena
del pavimento ruotata di 180° attorno a Y) — misurato: con +0.5 l'ostacolo si
avvicinava. Le escursioni di C1 non cambiano con la posizione (8.0°/8.7° prima,
8.2°/8.6° dopo).

**Un difetto trovato per strada.** La riscrittura di `applica_terreno` cercava
i Rigid Transform per nome (`Rigid Transform_Ostacolo1`), ma nel modello si
chiamano `Rigid⏎Transform10` — con un a capo dentro il nome. `set_param`
falliva e un `catch` vuoto lo nascondeva: le sezioni sulla posa degli ostacoli
non hanno mai applicato niente. Ora il blocco si trova seguendo il collegamento
del solido, e ogni fallimento viene segnalato anche senza `verbose`. **La
rampa ha lo stesso difetto ed è stata lasciata com'è di proposito**: la posa
di `cfg` non è mai stata applicata, e agganciarla sposterebbe la rampa in una
posizione mai verificata. Da decidere prima di T4.

### Come sono ottenute le forze di contatto

**Dal 21/9 dai sensori del modello** (`Fleg`), aggiunti dal collega nel commit
del 17/9. `adatta_simscape` li usa da solo quando li trova.

Validazione, C1 a 1.0× per 10 s: forza verticale media **15.55 N contro 15.55 N
di peso, 0.0%**. È una verifica forte, perché nessuno ha imposto il vincolo: su
un ciclo periodico la reazione verticale media deve valere il peso, e una misura
fisica lo rispetta da sola.

`Fleg` è il **modulo** della forza, messo nella componente z. Che la media
chiuda allo 0.0% dice che la componente tangenziale è trascurabile nel modulo —
altrimenti la media starebbe sopra il peso. Le componenti tangenziali restano
comunque non disponibili separatamente, e `Fz_max_norm` è un limite superiore.

**Prima** le forze venivano ricostruite dalla penetrazione, con la stessa legge
dei blocchi di contatto:

```
delta = max(0, z_terreno − z_piede)
Fz    = (k·delta + c·delta_punto) · rampa(delta/w)
```

con la quota del terreno ricavata imponendo che la somma delle sei forze valga
in media il peso. La sua verifica (1.1%) era debole proprio per questo: la
chiusura sul peso era in parte imposta. La ricostruzione resta in
`adatta_simscape` come ripiego quando i sensori mancano.

**Conseguenze sulle misure:**

- la colonna `note` di ogni riga dice quale fonte è stata usata;
- CSV con fonti diverse **non si confrontano sulle colonne di contatto**
  (`appoggio_medio`, `sotto3_frac`, `slip_tot`, `disp_carico`). Con i sensori
  il contatto è più severo: C1 a 1.0× sta sotto tre piedi l'11.7% del tempo,
  contro lo 0.4% della ricostruzione;
- le colonne di **moto** sono identiche bit per bit fra le due fonti
  (`frazione_task` a 1.0× coincide all'ottava cifra): il robot è lo stesso, è
  cambiata solo la misura del contatto;
- i due CSV di T2 sono stati rifatti con i sensori, e tutte le tabelle di
  questo documento usano quella fonte salvo dove è detto.

Storia, perché spiega le scelte precedenti: fino al 17/9 le forze **non erano
ottenibili** dal modello. I dodici `From` di `Fleg` non avevano un `Goto`, il
log di Simscape conteneva solo i giunti, e `LogSimulationData` sui blocchi di
contatto è rifiutato da Simulink.

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
- **Rampa (T4)**: rifatta dal collega (commit del 17 e 20/9), con blocchi e
  contatti dedicati. Da verificare: `rampa_pos` è passato a `[-0.8, 0, 0.1]`,
  mentre il commento in `phantomx_config` dice ancora «riposizionata davanti»
  — con il robot che avanza verso +x, −0.8 è dietro.
- **Ostacoli 4–7**: poggiano sui rilievi del terreno imperfetto (da +35 a
  +71 mm). Il vecchio T6 usava il pavimento imperfetto per questo; il nuovo
  `applica_terreno` usa `pavimento` per tutti i task. **Da verificare a occhio**
  che in T6 gli ostacoli 4–7 non galleggino.
- Due `Data Store Write` scrivono nella stessa memoria a `t = 0` senza ordine
  garantito: non determinismo in un modello che deve produrre misure ripetibili.
- ~~`sys_filter`~~ — **chiuso**: l'`InitFcn` lo costruisce, i poli seguono `τ`
  alla cifra, ma nessuno dei 2085 blocchi del modello lo legge. È un residuo e
  non c'entra con la cella 2× (vedi `archivio/README.md`).
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
