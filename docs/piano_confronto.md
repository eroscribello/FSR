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

### [MISURATO 23/9] Su terreno piano C2 non serve, e si fa sentire

*Riscritta il 23/9 con le inerzie corrette (§9) e filtrata con il pavimento di
rumore misurato sullo stesso task. Rispetto alla versione del 21/9 cambiano
tutti i valori assoluti di energia e coppia; la tesi resta. Sono state
**ritirate** le affermazioni su `distacchi` e `potenza_max`, che non superano il
rumore, e quella sui picchi di coppia di C2 (vedi sotto).*

Avanzamento, a parità di cella (`frazione_task`):

| | 0.50× | 1.00× | 1.20× | 1.50× | 2.00× |
|---|---|---|---|---|---|
| C1 | 0.894 | 1.122 | 1.087 | 0.844 | −0.453 |
| C2 | 0.941 | 1.101 | 1.074 | 0.850 | −0.452 |

A 2.00× entrambi falliscono per avanzamento insufficiente, come prima. La
velocità satura fra 1.2× e 1.5× (0.157 → 0.153 m/s con C1): **1.20× resta
l'ultima cella sana**.

**Il risultato, alla cella nominale 1.00×**, con il rapporto fra differenza e
rumore accanto a ogni riga (criterio: ≥ 3 per poter concludere):

| | C1 | C2 | diff. / rumore | |
|---|---|---|---|---|
| `cot` | 0.888 | 1.028 | **7.6** | +15.8% |
| `energia` [J] | 18.6 | 21.1 | **7.6** | |
| `z_media` [m] | 0.1521 | 0.1611 | **7.6** | corpo +9.0 mm |
| `vel_media` [m/s] | 0.1349 | 0.1320 | **6.8** | C2 un 2% più lento |
| `frazione_task` | 1.122 | 1.101 | **9.2** | |
| `pitch_max` [rad] | 0.0090 | 0.0704 | **9.0** | |
| `tau_max` [N·m] | 1.87 | 1.90 | 0.8 | non concludente |
| `roll_max` [rad] | 0.0036 | 0.0186 | 1.7 | non concludente |
| `dev_lat_max` [m] | 0.0043 | 0.0133 | 2.9 | non concludente |

**Il meccanismo si legge in `z_media`:** con C2 il corpo sta **9 mm più in
alto** su un terreno dove non c'è niente da cercare. La ricerca estende le
zampe verso il basso a ogni appoggio, il corpo si alza, e quel movimento costa:
+15.8% di cost of transport per fare la stessa strada un 2% più piano.

Perché si attiva: la ricerca parte quando `c(i) == 0` e il comando vuole la
zampa a terra. All'istante del touchdown lo scarto fra coppia attesa e misurata
è sotto la soglia per qualche decina di ms, quindi `c = 0` e la ricerca **parte
a ogni appoggio**. Nell'articolo serve a una zampa che **non trova** il terreno,
non al transitorio normale di contatto. È un difetto di implementazione, non del
metodo: la correzione naturale è un **ritardo di innesco** — non cercare finché
non è passato un tempo minimo dal touchdown comandato. Da provare, non ancora
provato.

Con le inerzie corrette la firma è più marcata di prima (+9 mm contro i +5.5
misurati il 21/9): le coppie in volo sono più basse, il flag di contatto resta
falso più a lungo, la ricerca scende di più.

**Alle velocità estreme il quadro si rovescia**, ma con cautela:

| `cot` | 0.50× | 1.00× | 1.20× | 1.50× |
|---|---|---|---|---|
| C1 | 2.00 | 0.888 | 0.887 | 3.09 |
| C2 | 2.60 | 1.028 | 0.879 | 2.83 |

A 0.5× C2 costa il **30%** in più; a 1.2× i due sono indistinguibili (0.008 di
differenza contro 0.018 di rumore); a 1.5× **C2 costa il 9% in meno**. Lì le
zampe perdono davvero l'appoggio e la ricerca ha qualcosa da trovare. Il
pavimento di rumore però è misurato **solo a 1.00×**: nelle celle a 1.5× e 2×
compaiono picchi d'urto (`tau_max` ~14 N·m con entrambi) e la loro variabilità
non è nota. Da riportare come indicazione, non come misura.

> **[RITIRATO 23/9] I picchi di coppia di C2.** Le versioni precedenti di
> questa sezione riportavano `tau_max` di C2 a 10–11 N·m contro 1.5 di
> datasheet. Con le inerzie corrette scende a 1.90 N·m contro 1.87 di C1 — e
> soprattutto **`tau_max` non discrimina in nessun task**: il suo rumore (0.033
> in piano) è più grande della differenza fra i controllori (0.027). Il massimo
> di un segnale rumoroso è instabile per costruzione. Per parlare di coppie
> serve `tau_rms`, e anche quello qui sta a 2.4, sotto la soglia di 3.

**Da riportare così:** su terreno piano C2 non ha niente da cercare, cerca lo
stesso, alza il corpo di 9 mm e paga il 16% di cost of transport. È il
compromesso che un MPC con orizzonte non dovrebbe dover fare, ed è il confronto
da impostare con C3.

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
| **T2** | piano, rettilineo, **cinque velocità** | liscio | sull'avanzamento **no**, su energia e quota del corpo **sì** (§3) | curva prestazione–velocità: dove il cinematico cede e l'MPC no |
| **T3** | traiettoria curva, imbardata costante | liscio | su energia e quota del corpo **sì**, come T2 (§9) | C1 realizza il **22%** dell'imbardata comandata (§9): caratterizza C3 contro il cinematico |
| **T4** | rampa di 8° in salita; **T4D**: dosso 14 cm, salita e discesa | liscio + rampa / dosso | a regime **no**; su energia e quota del corpo **sì**; sui massimi d'assetto **non concludente** (§9) | nel paper la rampa è terreno noto e il corpo resta orizzontale (IK estesa, non implementata qui); da noi è ignota a entrambi |
| **T5** | ostacolo singolo non modellato | liscio + 1 ostacolo | solo sulla **quota del corpo** (+19 mm); tutto il resto sotto il rumore (§9) | scenario del paper: terreno **ignoto** |
| **T6** | terreno irregolare, ostacoli multipli | pavimento + 7 ostacoli | **sì**, ma a favore di C2 solo sull'energia: `cot` −32% (§9); i due non percorrono lo stesso tratto | stress test, corrisponde alla loro fig. 19b |
| **T7** | disturbo impulsivo laterale | liscio | parziale | recupero dopo perturbazione |

Su terreno piano la retroazione del paper **non dovrebbe** intervenire, e
sull'avanzamento infatti C1 e C2 coincidono. Ma interviene lo stesso, a ogni
touchdown, e costa energia (§3): T1–T3 servono soprattutto a caratterizzare C3
contro il cinematico. Il confronto fra i due cinematici vive su **T4, T5 e T6** —
che sono poi esattamente gli scenari sperimentali degli autori — ma **[23/9]**
proprio lì i massimi d'assetto e le coppie di picco non superano il pavimento di
rumore (§9): su terreno accidentato il confronto regge su quota del corpo ed
energia, non sull'assetto. Se il tempo obbligasse a tagliare, si
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

### [MISURATO 22/9] Le inerzie del modello sono ~1000 volte troppo grandi

Nel `.slx` i momenti d'inerzia vengono dall'URDF e sono letti come kg·m², ma
valgono circa mille volte il vero. Le **masse sono giuste**.

| corpo | massa | nel modello | valore fisico plausibile |
|---|---|---|---|
| `MP_BODY` | 0.976 kg | [3.11, 6.38, 5.33] kg·m² | `cfg.I_body` = [3.6, 5.2, 8.6]·10⁻³ |
| ciascuno dei 24 link | 0.024 kg | [5.1, 8.2, 1.1]·10⁻³ | ~10⁻⁵ |

Il corpo si comporta come se la sua massa stesse a 1.7 m dal centro, ogni link
come un'asta da mezzo metro. Per il corpo il valore plausibile torna con il
conto a mano di un parallelepipedo da 1 kg e 25 × 20 × 5 cm.

#### [25/9] La prova, che qui mancava

Fino a oggi questa sezione **affermava** il fattore mille senza mostrarlo, e la
domanda è arrivata giusta: *«siamo sicuri? le avevo prese da un GitHub
ufficiale»*. La provenienza è vera e non è in discussione — mesh, cinematica,
struttura e **masse** di quell'URDF sono buone. Ma la provenienza non rende
fisici i numeri. Due prove indipendenti, entrambe rifacibili in tre righe.

**Prima prova: il raggio di girazione.** Da `I = m·r²`, un corpo rigido non può
avere `r` maggiore della propria dimensione massima. È geometria, non
convenzione né unità di misura.

| corpo | `I_xx` URDF | massa | **raggio di girazione** | dimensione vera |
|---|---|---|---|---|
| `MP_BODY` | 3.108 kg·m² | 0.976 kg | **1.79 m** (su `I_yy`: 2.56 m) | 25 cm |
| ogni link | 5.14·10⁻³ kg·m² | 0.0244 kg | **0.46 m** (su `I_yy`: 0.58 m) | 2–12 cm |

Letti in kg·m², che è ciò che lo standard URDF impone, quei numeri descrivono un
oggetto che non esiste.

**Seconda prova: i 24 link sono identici.**

```
ixx=0.0051411124   iyy=0.0081915737   izz=0.0011379812      × 24, uguali
```

Coxa (`c1`, ~2 cm), femore (`thigh`, ~7 cm) e tibia (~12 cm) hanno forma e
lunghezza diverse: non possono avere la stessa inerzia. È un **valore
segnaposto copiato**, non una misura. Negli URDF di comunità, nati per
visualizzazione e cinematica, è il campo che più spesso resta al default.

**Le masse invece sono giuste** (0.976 kg il corpo, 24.4 g per link, plausibili
e coerenti): chi ha esportato l'URDF aveva la geometria, è sbagliato solo il
blocco `<inertia>`. Dividendo per 1000 tutto rientra nell'ordine di grandezza
giusto — la firma dell'errore di esportazione CAD con la massa in **grammi**
nel calcolo dell'inerzia.

**Controprova sui valori sostitutivi.** `cfg.I_body` coincide con il calcolo a
mano di un parallelepipedo da 1 kg e 25 × 20 × 5 cm entro il 3%:

| | `cfg.I_body` | scatola a mano |
|---|---|---|
| `I_xx` | 3.557·10⁻³ | 3.455·10⁻³ |
| `I_yy` | 5.154·10⁻³ | 5.284·10⁻³ |
| `I_zz` | 8.565·10⁻³ | 8.333·10⁻³ |

**Cosa resta incerto, e va detto.** Che l'URDF sia sbagliato è certo. Che
`cfg.I_c1`, `I_c2`, `I_thigh`, `I_tibia` siano i valori *esatti* no: sono stime
per pezzo, non misure. Sono però certamente più vicine del vero di un unico
numero uguale per tre pezzi diversi e sbagliato di mille volte.

**Verifica in tre righe**, senza fidarsi di questo documento:

```matlab
rm = importrobot("phantomx_description-master\urdf\phantomx.urdf");
for k = [1 2 25]
    b = rm.Bodies{k};
    fprintf('%-10s m=%.4f kg   r_girazione = %.2f m\n', b.Name, b.Mass, sqrt(b.Inertia(1)/b.Mass));
end
```

**Perché non si vedeva:** in C1 e C2 i giunti sono comandati **in posizione**.
Il simulatore impone la traiettoria qualunque sia l'inerzia, quindi l'andatura
in animazione è corretta. Cambiano le coppie registrate e la reazione del corpo.

**Due prove, inerzie originali contro corrette** (`prova_inerzie.m`,
`results/prova_inerzie*.csv`, una run per caso):

| | T2 1× C1 | T6 C1 | T6 C2 |
|---|---|---|---|
| `tau_max` | 1.96 → 1.87 | 6.98 → 7.51 | **26.5 → 9.46** |
| energia | 50.4 → 18.6 J | 196 → 109 J | 303 → 104 J |
| `cot` | 2.32 → 0.89 | 3.93 → 2.86 | **5.30 → 1.95** |
| beccheggio max | 0.17° → 0.51° | **45.1° → 14.7°** | 14.4° → 22.6° |
| rollio max | 0.39° → 0.21° | **30.9° → 7.0°** | 8.1° → 9.4° |
| velocità media | 0.140 → 0.135 | 0.106 → 0.079 | 0.122 → 0.114 |

**Le inerzie cambiano le conclusioni, non solo i valori assoluti:**

- C1 in T6 **non supera più** la soglia di assetto dei 30°;
- il vantaggio di C2 sull'assetto in T6 **si inverte** (22.6° contro 14.7°);
- `tau_max` di C2 passa da 18 a ~6 volte il datasheet: **ritirato** quanto
  scritto nelle sezioni T4–T6 sul «13–18 volte»;
- il rapporto `cot` C2/C1 in T6 passa da **1.35 a 0.68**: con le inerzie
  corrette C2 consuma **meno** di C1, non di più.
- **[23/9]** il discorso sui picchi di coppia è chiuso da un'altra parte:
  `tau_max` non discrimina in **nessun** task, nemmeno in piano (sezione sul
  pavimento di rumore, subito sotto). Va tolto dal confronto, non ricalcolato.

Perché C2 è il più colpito: la ricerca del terreno muove la zampa verso il
basso rapidamente, e con zampe mille volte più inerti quel movimento costa
coppie enormi. C1 non fa quel movimento.

**Limiti di queste prove:** una run per caso; deviazione e imbardata non sono
confrontabili (variano da sola a sola run, visto in T4D). Le colonne su cui si
ragiona sono coppie, energia, cot e assetto massimo.

**Stato e strumenti**

- `applica_inerzie.m` mette i valori corretti **in memoria**, come
  `applica_terreno`: il `.slx` resta quello del collega e le inerzie originali
  restano nel file.
- `controllo_inerzie.m` verifica, prima di rifare le campagne, che le tarature
  fatte con le inerzie vecchie reggano: chiusura dei sensori sul peso, firme
  della ricerca di terreno di C2 (soglia `t_threshold`), run sane.
- **[FATTO 23/9]** campagna rifatta per intero con le inerzie corrette: T2, T3,
  T4, T4 limite, T4D, T5, T6, con C1 e C2. `controllo_inerzie` era passato su
  tutte e tre le verifiche (sensori al 100.1% del peso, firme di C2 presenti,
  run sane), e la riga `v1.00x` di `script_T2` coincide cifra per cifra con
  quella di `controllo_inerzie`: la catena terreno → inerzie → metriche è
  riproducibile.
- **Dove è attiva:** `script_T2`, `script_T3`, `script_T4` (e quindi
  `script_T4_limite`), `script_T4D`, `script_T5`, `script_T6` chiamano
  `applica_inerzie` subito dopo `applica_terreno`. Se il collega correggerà il
  `.slx`, la chiamata è idempotente e si toglie senza conseguenze.
- I numeri della campagna con le inerzie dell'URDF sono conservati in
  `results/storico/inerzie_URDF/`, per poter mostrare il confronto.


Non sono voci di piano, sono debiti noti. Elencati perché non si perdano.

### [MISURATO 23/9] Il pavimento di rumore: quali colonne discriminano

Il simulatore è deterministico: rilanciare la stessa run dà gli stessi numeri
alla quindicesima cifra. Questo rende impossibile stimare una variabilità
ripetendo, e facile scambiare per risultato una differenza che non lo è.

**Il metodo.** Si tiene fermo tutto e si cambia una cosa irrilevante: `gait.z0`
di **±0.2 mm**, un quarto della penetrazione statica del contatto e un decimo di
`cfg.c2.tol`. Le tre run che ne escono danno l'escursione che quella metrica ha
**a parità di controllore e di condizioni**. Il criterio, scritto prima di
lanciare:

```
rapporto = |valore C1 − valore C2| / escursione fra le perturbazioni
rapporto >= 3  ->  la colonna discrimina
rapporto <  3  ->  non si conclude nulla da quella colonna su quel task
```

`rumore_metriche.m`; risultati in `results/diagnostica/rumore_metriche.csv`
(T4D, T5) e `rumore_T2.csv` (T2, C1 e C2).

**L'esito, per famiglia di metriche:**

| colonna | T2 (piano) | T4D (dosso) | T5 (ostacolo) |
|---|---|---|---|
| `cot`, `energia` | **7.6** | **3.7** | 2.5 |
| `z_media` | **7.6** | **5.8** | **5.3** |
| `vel_media`, `frazione_task` | **6.8 / 9.2** | 0.8 | 1.7 |
| `pitch_max` | **9.0** | **3.3** | 0.8 |
| `roll_max` | 1.7 | 2.4 | 0.5 |
| `tau_max` | 0.8 | 0.5 | 2.1 |
| `dev_lat_max` | 2.9 | 0.5 | 1.2 |
| `yaw_err_fin` | 2.8 | 0.5 | 0.7 |

**Tre letture, tutte da mettere in relazione:**

1. **Le medie sull'intera run reggono, i massimi no.** `cot`, `energia` e
   `z_media` discriminano ovunque siano state misurate; `tau_max` non discrimina
   **in nessun task**, nemmeno in piano dove urti non ce ne sono: il massimo di
   un segnale rumoroso è instabile per costruzione.
2. **Su terreno irregolare cadono anche deriva e imbardata.** 0.2 mm decidono
   quale piede tocca per primo uno spigolo; da lì in poi le traiettorie
   divergono. Un robot vero si comporta allo stesso modo: è il motivo per cui
   gli esperimenti si ripetono.
3. **Quello che sopravvive ovunque è una cosa sola:** `z_media`, cioè la quota
   del corpo. Ed è proprio la firma della ricerca del terreno di C2.

**Conseguenza operativa.** Le colonne sotto 3 non compaiono nelle conclusioni
dei rispettivi task — non sono cancellate dai CSV, sono dichiarate non
discriminanti. Per recuperarle servirebbero 3–5 ripetizioni per cella con
perturbazione, riportando mediana e intervallo: è la campagna per tre, e si fa
solo se avanza tempo dopo C3.

**Cosa resta da misurare:** il pavimento di rumore di T3, T4, T4-limite e T6.
Per T3 e T4 si può ragionevolmente usare quello di T2 (terreno piano, andatura
nominale); per T6, il più irregolare di tutti, no.

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
| ostacolo di T5 | ~~45 mm~~ **32.8–34.0 mm** |
| **franco minimo per T5/T6** | ~~**~55 mm**~~ **~43–47 mm** |

> **[RITIRATO 2/10]** Il 45 mm non ha una provenienza scritta, e con lui la
> tabella chiedeva ~55 mm di franco per poi adottare `H` = 50 mm. Il valore che
> vale è quello misurato da `script_T5`: colonna `alt_ost` di
> `results/T5_C*.csv`, 32.8 / 33.1 / 34.0 mm per C1 / C2 / C3. I gradini di T6
> salgono di ~35 mm l'uno (35 → 70 → 106 mm), quindi lo stesso franco vale anche
> lì. Vedi `docs/andatura.md` §6.

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

### [MISURATO 23/9] T3: il 22% dell'imbardata comandata

*Riscritta il 23/9 con le inerzie corrette. Il rapporto d'imbardata di C1 passa
dal 15% al **22%**: il numero va aggiornato ovunque compaia. La
caratterizzazione con `diagnosi_imbardata` (sotto) è del 21/9 e **non** è stata
rifatta: i suoi valori sono indicativi.*

Sessione pulita, forze dai sensori, `yaw_d = ±0.1 rad/s`, 15 s, una run per
verso e per controllore.

| | C1 +0.1 | C1 −0.1 | C2 +0.1 | C2 −0.1 |
|---|---|---|---|---|
| `yaw_rapporto` | 21.9% | 21.8% | **31.9%** | **21.2%** |
| `z_media` [m] | 0.1521 | 0.1521 | 0.1617 | 0.1645 |
| `cot` | 0.970 | 0.972 | 1.179 | 1.165 |
| `tau_max` [N·m] | 1.91 | 1.91 | 2.69 | 2.92 |
| `dev_lat_max` [m] | 0.329 | 0.332 | 0.408 | 0.310 |

**C1 è simmetrico allo 0.5%** (21.9 contro 21.8): impianto e andatura non hanno
preferenze di verso, e il raggio effettivo è v/ω ≈ 0.134/0.0219 ≈ **6.1 m**
contro gli 1.2 comandati.

**Le firme di C2 ricompaiono con gli stessi valori di T2:** corpo +9.6 e
+12.4 mm (9.0 in T2), `cot` +21% (+16% in T2). Due task diversi, stesso
meccanismo, stessa ampiezza: il difetto d'innesco della ricerca non è un caso
di una campagna. Il pavimento di rumore è misurato su T2 e non su T3, ma i due
task condividono terreno piano e andatura nominale: applicandolo, `cot` e
`z_media` stanno a 8–11 volte il rumore.

> **[APERTO 23/9] L'asimmetria di C2.** Con le inerzie corrette C2 realizza
> 31.9% di imbardata in un verso e 21.2% nell'altro: **40% di differenza**,
> mentre con le inerzie vecchie era simmetrico entro il 7%. Ipotesi, non
> misura: C2 rileva il contatto zampa per zampa, e i due tripodi non sono
> speculari rispetto all'asse di marcia (il baricentro del triangolo d'appoggio
> è spostato di ±1.7 cm). Con inerzie mille volte più grandi quella differenza
> veniva schiacciata. Il simulatore è deterministico, quindi ripetere la stessa
> run non verifica nulla: servirebbe perturbare la fase iniziale e vedere se il
> verso favorito resta lo stesso. Non fatto.

**Le due letture del limite d'imbardata** (caratterizzazione del 21/9, non
rifatta): sul posto il tripode comandato in posizione realizza il 56% del
comando — è il limite di aderenza del **metodo**; sull'arco scende al 15–24%
perché le sei zampe condividono una sola lunghezza di passo mentre in curva
l'interna e l'esterna ne vorrebbero due diverse. Questa seconda parte è della
**nostra implementazione** — due blocchi di traiettoria invece di sei — e va
dichiarata accanto al confronto con C3, altrimenti si attribuisce all'MPC un
vantaggio che è in parte nostro.

**Tre errori di metodo, tutti costati giorni, da non ripetere:**

1. **Stato residuo della sessione.** Tutte le misure d'imbardata prima del 21/9
   sono da scartare: erano prese in una sessione MATLAB con stato residuo nel
   workspace, dove il robot in rettilineo aveva 0.455 piedi a terra e 4° di
   beccheggio. **Ogni campagna parte da `clear all; bdclose all;
   startup_phantomx`.**
2. **Il segno giudicato sul rumore.** Con quella misura `applica_imbardata` è
   stato portato a `segno = −1`, e ci è rimasto tre giorni. Invertire il segno
   non cambiava niente — e andava letto come «la misura non vale», non come «il
   segno è giusto comunque».
3. **Il segno prima del modulo.** Il controllo in `script_T3` guardava `sign()`
   senza chiedere un modulo minimo, quindi scattava sul rumore. Ora il modulo
   viene prima (soglia 5%).
### [MISURATO 23/9] T4: in salita i due controllori si equivalgono

*Riscritta il 23/9 con le inerzie corrette. **Ritirate** due affermazioni della
versione precedente: la riduzione del 64% del rollio in transizione da parte di
C2 (il rollio non supera il pavimento di rumore) e i 13.7 N·m di `tau_max` di
C2 (`tau_max` non discrimina in nessun task). Il risultato a regime resta.*

Rampa di 8° in salita, non nota al controllore, velocità nominale, 20 s.
`script_T4`, una run per controllore. La rampa non finisce entro la run: si
confronta il **regime in salita** con il **regime in piano** della stessa run,
più la **transizione** fra i due (finestre in testa a `script_T4.m`).

| | C1 | C2 | |
|---|---|---|---|
| pendenza misurata dagli appoggi | 8.00° | 7.97° | verifica della posa: 8° |
| errore d'inclinazione rispetto alla rampa | **0.106°** | **0.032°** | entrambi paralleli |
| inclinazione picco-picco in salita | 0.99° | **7.40°** | |
| escursione di rollio in transizione | 2.16° | 1.33° | rumore non misurato |
| distanza corpo–superficie, salita − piano | −0.9 mm | **+18.2 mm** | |
| `z_media` [m] | 0.2553 | 0.2752 | corpo +20 mm |
| velocità in salita / in piano | 0.903 | 0.901 | |
| `cot` | 1.151 | 1.304 | +13% |
| `tau_max` [N·m] | 2.10 | 2.52 | non concludente |

**A regime la salita non discrimina.** Su una pendenza uniforme anche C1, in
anello aperto, si allinea alla rampa entro 0.11°: il tripode in appoggio sta
tutto sulla stessa superficie, e il corpo la segue perché la cinematica tiene
costante la distanza piedi–corpo. Nessuno dei due tiene il corpo orizzontale, e
non è un difetto: nessuno dei due conosce la pendenza. La tenuta orizzontale su
terreno *noto* è la IK estesa del paper (§2, punto 1), qui non implementata.

**C2 allinea meglio in media e peggio nel dettaglio.** L'errore medio è tre
volte più piccolo di quello di C1 (0.032° contro 0.106°), ma l'inclinazione
oscilla di **7.4°** picco-picco contro 0.99° — con le inerzie vecchie erano
0.17°. È la stessa ricerca che in piano alza il corpo: qui lo alza di 18 mm
sopra la rampa e lo fa ballare. Il rumore su queste due colonne non è misurato
in T4 (lo è in T4D, dove `pitch_max` discrimina e `roll_max` no): l'oscillazione
è un fattore 7 sopra quella di C1, quindi l'ordine di grandezza regge, il
valore preciso no.

**Quello che regge senza riserve** è la firma già vista in T2 e T3: corpo più
alto (+20 mm) e `cot` più alto (+13%). Quattro task, stesso meccanismo.

**Come si misura:** l'appoggio è cinematico come in T5; la quota del pavimento è
il 10° percentile degli appoggi, non la mediana (in T4 la maggior parte degli
appoggi è sulla rampa). I sensori chiudono al 17% del peso, perché vedono solo
il pavimento e il robot passa quasi tutta la run sulla rampa: colonne di
contatto a `NaN`.

**Limiti:** una run per controllore; pavimento di rumore non misurato su questo
task. La discesa è misurata a parte, in T4D.

**La posa della rampa.** `ispeziona_rampa` ha seguito la catena dal solido al
mondo: tutti i Rigid Transform del terreno hanno il **solido sulla porta B e il
mondo sulla F**, quindi l'offset scritto nel blocco è la posa del mondo vista
dal solido, e il solido sta nella posa inversa. È la causa della regola dei
«segni opposti» trovata a tentativi su pavimento e ostacoli. La posa del collega
era giusta: rampa davanti, in salita. È stata spostata di 0.3 m in avanti con lo
stesso criterio di T5 (la salita inizia a 0.66 m invece che a 0.36, dopo il
transitorio).
### [MISURATO 23/9] T4: angolo limite di salita 20° per entrambi

*Riscritta il 23/9 con le inerzie corrette. Il limite **non cambia**, ma i
margini diventano netti: prima i due casi al bordo si decidevano per pochi
millesimi, ora per decimi.*

`script_T4_limite`: griglia 10–30° e bisezione, criterio di «salita riuscita»
dichiarato in testa a `script_T4` prima della ricerca (tutte le zampe sulla
rampa, due cicli a regime, velocità ≥ metà di quella in piano, assetto entro 30°
rispetto alla rampa). Una run per angolo.

| gradi | C1 v salita/piano | C2 v salita/piano | esito |
|---|---|---|---|
| 10 | 0.874 | 0.816 | salgono |
| 15 | 0.800 | 0.811 | salgono |
| **20** | **0.654** ✓ | **0.576** ✓ | salgono |
| **21** | **0.437** ✗ | **0.280** ✗ | falliscono |
| 21.5 | zampa RR mai sulla rampa ✗ | zampa RR mai sulla rampa ✗ | |
| 22.5 | −0.03 ✗ | zampa RR mai sulla rampa ✗ | |
| 25 / 30 | zampe posteriori mai sulla rampa ✗ | idem ✗ | |

**Limite del banco: 20° per entrambi**, fra 20 e 21. Con le inerzie vecchie
questa conclusione stava su margini risibili — C1 falliva a 21° per 0.005 sul
criterio e C2 passava a 20° per 0.003; ora a 20° passano con 0.65 e 0.58 e a 21°
crollano a 0.44 e 0.28. **È una soglia, non una coincidenza.**

**La ricerca del terreno non sposta il limite.** Che sia uguale per i due
controllori indica una causa comune — aderenza, spazio di lavoro delle zampe o
geometria del tripode — non il controllore. Non è verificata: `mu_plant = 0.9`
darebbe un limite statico di ~42°, quindi non è la sola aderenza statica. Da non
scrivere come causa senza una prova.

**Come falliscono, oltre il limite.** Con le inerzie vecchie il robot «si
fermava»; adesso nella maggior parte dei casi la causa è *«non tutte le zampe
sulla rampa (RR)»* — la posteriore destra resta indietro e il robot si mette di
traverso. Cambia la descrizione del fallimento, non l'angolo.

**Il robot sale più lentamente a ogni angolo** rispetto alle inerzie vecchie
(0.87 contro 0.98 già a 10°): l'inerzia di marcia mascherava lo scivolamento.

**Limite realistico.** Già a 10° il picco di coppia supera il datasheet
dell'AX-12A (1.5 N·m) con entrambi. `tau_max` però non è una colonna
discriminante — il suo rumore è dell'ordine del valore stesso — quindi la
formulazione difendibile è: *sul robot vero con gli AX-12A queste rampe non si
salgono con questa andatura*, senza attribuire un angolo preciso al limite
realistico né una differenza fra i due controllori.
### [MISURATO 23/9] T4D: sul dosso resta la firma, non il vantaggio

*Riscritta il 23/9. La versione precedente titolava «in discesa C1 lascia le
zampe appese, C2 no» ed è **ritirata**: con le inerzie corrette C1 non lascia
più le zampe appese. Sono ritirate anche le righe su rollio, coppie di picco,
deviazione e imbardata, che non superano il pavimento di rumore misurato su
questo stesso task.*

Dosso di 14 cm non noto ai controllori: salita di 8° (come T4), cima piana di
0.6 m, discesa di 8°, poi di nuovo piano. Velocità nominale, run da 30 s tagliata
al bordo del pavimento. `script_T4D`, una run per controllore. La geometria
(`dosso_profilo`) è scritta da `applica_terreno('T4D')` come STL e caricata nel
solido della rampa, senza toccare il `.slx`.

**Quello che regge** (rapporto differenza/rumore ≥ 3, rumore misurato con
`rumore_metriche` su questo task):

| | C1 | C2 | diff. / rumore |
|---|---|---|---|
| `z_media` [m] | 0.2128 | 0.2286 | **5.8** — corpo +16 mm |
| `cot` | 1.117 | 1.329 | **3.7** — +19% |
| `pitch_max` [rad] | 0.151 | 0.215 | **3.3** |

**Quello che non regge**, con il rumore accanto:

| | C1 | C2 | diff. / rumore |
|---|---|---|---|
| `roll_max` | 0.037 | 0.070 | 2.4 |
| `tau_max` [N·m] | 2.58 | 4.27 | 0.5 |
| `dev_lat_max` [m] | 0.023 | 0.104 | 0.5 |
| `yaw_err_fin` [rad] | 0.005 | −0.129 | 0.5 |

Sul dosso una perturbazione di 0.2 mm sulla quota comandata del piede cambia
`tau_max` di 3.5 N·m e la deriva di 16 cm: decide quale piede tocca per primo lo
spigolo, e con l'ordine degli urti cambiano tutti i massimi. Non è un difetto del
simulatore — un robot vero fa lo stesso — ma significa che **su terreno
irregolare i massimi di una singola run non sono un dato**.

> **[RITIRATO 23/9] Le zampe appese di C1.** Con le inerzie vecchie C1 perdeva
> due terzi dei piedi fermi sullo spigolo convesso (2.28 → 0.80) mentre C2
> restava a 1.71, ed era il risultato più forte di tutta la campagna. Con le
> inerzie corrette C1 sta a **2.01** e C2 a **1.76**: il fallimento che la
> retroazione del paper esiste per evitare **non si verifica più**, perché
> nasceva dal corpo mille volte più difficile da ruotare che beccheggiava sullo
> spigolo sollevando le zampe. Il confronto C1–C2 su questa colonna resta senza
> stima di rumore (le colonne di finestra non sono nel test): non si conclude.

**Cosa resta di T4D**, ed è la stessa cosa di T2, T3 e T4: C2 tiene il corpo più
alto (+16 mm) e paga in cost of transport (+19%). Il beccheggio maggiore di C2
(rapporto 3.3) è l'unica differenza d'assetto che sopravvive, ed è **a sfavore**
di C2.

**Come si leggono i piedi fermi.** Anche in piano non sono 3: al cambio di
tripode per un istante nessun piede è fermo. Il confronto sarebbe sulla
variazione rispetto al proprio piano, non sul valore assoluto.

**Il bordo del pavimento.** Il pavimento è il cubo 8 × 8 (x fino a 4 m), e C2,
più veloce, ci arrivava negli ultimi 2 s e cadeva. La run intera è troncata al
primo piede oltre x = 3.95 m **prima** di `metriche` (colonna `t_bordo`).
`cfg.floor_dim = [4 4 0.05]` non descrive la mesh del pavimento: da correggere o
da commentare.
### [MISURATO 23/9] T5: sull'ostacolo resta solo la quota del corpo

*Riscritta il 23/9. La versione precedente titolava «C2 dimezza il rollio e
annulla la deriva» ed è **interamente ritirata**: con le inerzie corrette i
ruoli si invertono, e il pavimento di rumore misurato su questo task mostra che
nessuna di quelle colonne è in grado di distinguere i due controllori.*

Ostacolo 1, ~33 mm, spostato 0.5 m in avanti rispetto al `.slx` (vedi sotto),
velocità nominale, 20 s. `script_T5`, una run per controllore.

**Quello che regge:**

| | C1 | C2 | diff. / rumore |
|---|---|---|---|
| `z_media` [m] | 0.1536 | 0.1722 | **5.3** — corpo +19 mm |

**Quello che non regge:**

| | C1 | C2 | diff. / rumore |
|---|---|---|---|
| `cot` | 1.198 | 1.672 | 2.5 |
| `tau_max` [N·m] | 4.46 | 10.82 | 2.1 |
| `dev_lat_max` [m] | 0.018 | 0.135 | 1.2 |
| `pitch_max` [rad] | 0.132 | 0.164 | 0.8 |
| `roll_max` [rad] | 0.078 | 0.088 | 0.5 |

Il `cot` a 2.5 e le coppie a 2.1 sono **vicini** alla soglia di 3: con tre
ripetizioni per cella diventerebbero probabilmente conclusivi. Con una sola run
no.

> **[RITIRATO 23/9] Il vantaggio di C2 sull'ostacolo.** Con le inerzie vecchie
> C1 usciva dall'ostacolo girato di 10° e spostato di 25 cm mentre C2 usciva
> dritto, ed era il caso in cui «la ricerca del terreno fa quello per cui
> esiste». Con le inerzie corrette i numeri si scambiano quasi esattamente — C1
> 1.8 cm di deriva, C2 13.5 cm — ma **entrambe le versioni sono dentro il
> rumore**: 0.2 mm sulla quota comandata del piede spostano la deriva di 9 cm.
> Non è che avevamo il segno sbagliato: non avevamo una misura.

> **[RITIRATO 23/9] I 20 N·m di C2.** La versione precedente li dichiarava «13
> volte il datasheet» e ne faceva un argomento per l'MPC. Con le inerzie
> corrette sono 10.8, e `tau_max` non discrimina qui (2.1) né altrove.
> L'argomento per i limiti di coppia nel QP dell'MPC resta valido in linea di
> principio, ma **non è sostenuto da questa misura**.

**Come si misura** (dettagli in testa a `script_T5.m`): il passaggio si riconosce
da un piede **fermo nel mondo** e più alto del pavimento di 15 mm, non dai
sensori di forza — che vedono solo il pavimento (chiudono all'80% del peso in
T5), quindi le colonne di contatto sono a `NaN`. L'assetto è un'**escursione**
rispetto alla media in piano, presa fuori dalla finestra.

**Limiti:** una run per controllore; le finestre non coincidono (C1 e C2
riconosciuti sull'ostacolo a quote di tempo diverse).

**Lo spostamento dell'ostacolo.** Nel `.slx` l'ostacolo 1 stava a 16 cm dalla
partenza: il robot ci arrivava a 1.2 s, prima della fine del transitorio.
`cfg.terreno.ost_dx.T5` lo sposta di 0.5 m in avanti a runtime, senza toccare il
modello. Il segno è negativo perché i Rigid Transform degli ostacoli hanno x e z
opposte al mondo: la causa è il solido montato sulla porta B (vedi T4).

**Un difetto trovato per strada.** La riscrittura di `applica_terreno` cercava i
Rigid Transform per nome (`Rigid Transform_Ostacolo1`), ma nel modello si
chiamano `Rigid⏎Transform10` — con un a capo dentro il nome. `set_param` falliva
e un `catch` vuoto lo nascondeva. Ora il blocco si trova seguendo il
collegamento del solido, e ogni fallimento viene segnalato anche senza
`verbose`.
### [MISURATO 23/9] T6: C1 non si ribalta più, e i due non fanno lo stesso percorso

*Riscritta il 23/9 con le inerzie corrette. La versione precedente titolava «C1
supera la soglia d'assetto, C2 no» ed è **ritirata**: era l'effetto più vistoso
delle inerzie sbagliate. Il pavimento di rumore **non** è misurato su questo
task: le differenze qui sotto sono molto più grandi di quelle di T4D e T5, ma
restano da confermare.*

Pavimento + sette ostacoli, disposizione del modello, velocità nominale, 30 s.
`script_T6`, una run per controllore.

| | C1 | C2 | |
|---|---|---|---|
| beccheggio massimo | **14.7°** | **22.6°** | prima: 45.1° e 14.4° |
| rollio massimo | 7.0° | 9.4° | prima: 30.9° e 8.1° |
| esito | superato | superato | prima C1: `ribaltamento` |
| distanza in 30 s | 2.45 m | 3.44 m | |
| `frazione_task` | 0.68 | 0.94 | |
| imbardata finale | **−57°** | +19° | |
| altezza media degli ostacoli incontrati | 4.3 cm | 6.2 cm | |
| `energia` [J] | 108.8 | 104.5 | |
| `cot` | 2.855 | **1.954** | **−32%** |

**C1 non supera più la soglia dei 30°.** Con le inerzie dell'URDF arrivava a 45°
di beccheggio e `metriche` dichiarava `ribaltamento`; con quelle corrette sta a
14.7°. Era il risultato più forte a favore di C2 in tutta la campagna, ed era un
artefatto del modello.

> **[LIMITE 23/9] I due non fanno lo stesso task.** C1 percorre 2.45 m contro i
> 3.44 di C2 e finisce ruotato di 57°: incontra ostacoli diversi e mediamente
> più bassi (4.3 contro 6.2 cm). Confrontare i massimi d'assetto fra due
> percorsi diversi non è un confronto fra controllori. Va detto in relazione,
> non nascosto.

**L'unico task in cui C2 conviene.** Il cost of transport di C2 è **il 32% più
basso** di quello di C1 — in tutti gli altri task è dal 13 al 21% più alto. Su
terreno molto irregolare la ricerca del terreno serve davvero: C2 avanza di un
metro in più consumando meno energia in assoluto. È l'unico punto della campagna
in cui il controllore del paper ripaga il suo costo, ed è coerente con la sua
ragione d'essere.

**Limiti.** Una run per controllore; pavimento di rumore non misurato (sei run,
~20 minuti, se si vuole chiudere anche questo). I sensori di forza chiudono al
44% (C1) e 55% (C2) del peso — passano molto tempo sugli ostacoli, che i sensori
non vedono — quindi le colonne di contatto sono a `NaN`.

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
- ~~**Rampa (T4)**~~ — **chiuso**: −0.8 nell'offset del blocco è *davanti*
  nel mondo (solido sulla porta B, vedi T4). Resta solo estetica la metà del
  cubo sotto il pavimento.
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

---

## 10. [25/9] I video del 15 settembre: erano le inerzie, e C2 non è mai cambiato

**In due righe.** Il C2 che nei video del 15 settembre sembrava molto migliore
è il C2 con le inerzie dell'URDF, sbagliate di ~1000 volte: rilanciato oggi
con quelle inerzie, T6 riproduce la run archiviata **a sei cifre**. Il codice
di C2 è rimasto identico dal 15 settembre: stessa logica, stessi cinque
parametri. Le tabelle della campagna misurano il controllore del paper.

### Perché abbiamo speso due giorni su questo

Rivedendo i video del 15 settembre, C2 su T6 si comporta molto meglio di come
si comporta oggi. La domanda non era nostalgica: **se C2 oggi fosse scritto o
tarato male, una parte degli scarti che leggiamo nelle tabelle non misurerebbe
il controllore del paper, misurerebbe un difetto nostro** — e C3 verrebbe
progettato per correggere quel difetto, cioè contro un fantoccio.

Sistemare C2 prima non era un ripiego: era la condizione perché il confronto
fra i tre controllori voglia dire qualcosa.

### Il risultato: la logica di C2 è identica

Il codice del 15/9 vive dentro i due blocchi `MATLAB Function2` (SID 1134) e
`MATLAB Function3` (SID 1159), 71 righe ciascuno, con le costanti cablate. Il
`.slx` è binario, quindi git lo conserva ma non lo mostra: è stato estratto
con `estrai_funzioni.m` e confrontato riga per riga con `ricerca_terreno.m`.

**Stessa logica**, ramo per ramo: stessa condizione `z >= z_nom - tol`, stesso
`min(z_ext + v_search*dt, z_ext_max)`, stesso `min(z + z_ext, z_hold)` al
contatto, stessa guardia `dt <= 0 || dt > 0.1 -> 0.001`, stesso reset nel
transitorio.

**Stesse costanti**, tutte e cinque:

| | 15/9, cablate | oggi, da `c2_par` |
|---|---|---|
| `z_nominal` | 0.14 | `cfg.c2.z_nom = cfg.z0` = 0.14 |
| `v_search` | 0.06 | `cfg.c2.v_search` = 0.06 |
| `z_ext_max` | 0.03 | `cfg.c2.z_ext_max` = 0.03 |
| tolleranza | 0.002 | `cfg.c2.tol` = 0.002 |
| reset | `t < 0.5` | `cfg.c2.t_reset` = 0.5 |

**I due tripodi erano uguali fra loro.** Il diff fra `MATLAB Function2` e
`MATLAB Function3` del 15/9 mostra solo rinomine (`lf,lr,rm` contro
`rf,rr,lm`) e spazi. Il timore scritto in `come_funziona_ricerca_terreno.md`
— che due copie potessero divergere in silenzio — non si era materializzato.

L'unica differenza vera: il blocco del 15/9 **non ha l'interruttore C1/C2**
(`par(1)` non esiste, la ricerca è sempre attiva). Riguarda C1, non C2.

> **Conclusione.** C2 è algoritmicamente identico a quello dei video, con gli
> stessi cinque parametri. Le tabelle della campagna misurano il controllore
> del paper, non una nostra versione storpiata.

### La config è identica

`git show 1cb101d:common/phantomx_config.m`: `z0`, `z0_eff`, `T`, `S`, `H`,
`floor_dim`, `floor_off`, `floor_top` e la formula di `body_z0` sono gli
stessi di oggi. In particolare `cfg.H = 0.05`: il commit del 15/9 si intitola
«cambiato cfg.H» perché lo ha portato *a* quel valore, che è ancora quello di
oggi.

### Il percorso di T6, misurato dalle mesh

Non era documentato da nessuna parte. Letto dai bounding box degli `.stl`
(altezze riferite alla faccia superiore del pavimento):

| mesh | x [m] | y [m] | altezza |
|---|---|---|---|
| Ostacolo1 | 0.43 – 0.76 | tutta la larghezza | **35 mm** |
| Ostacolo5 | 0.73 – 1.06 | tutta la larghezza | **70 mm** |
| Ostacolo6 | 1.00 – 1.33 | tutta la larghezza | **106 mm** |
| Ostacolo7 | 1.23 – 1.56 | tutta la larghezza | **70 mm** |
| Ostacolo2 | 2.03 – 2.41 | 0.015 – 0.340 | 35 mm |
| Ostacolo3 | 2.32 – 3.13 | −0.396 – −0.070 | 35 mm |
| Ostacolo4 | 2.47 – 3.28 | −0.396 – −0.070 | 70 mm (sopra il 3) |

Due tratti:

- **x da 0.43 a 1.56: una scala** di quattro sbarre che attraversano tutta la
  pista, 35 → 70 → 106 → 70 mm, sovrapposte in x. **Non si aggira**: o si sale
  o ci si ferma;
- **x da 2.0 a 3.3**: ostacoli locali, laterali, col 4 impilato sul 3.

**Il terzo gradino è 106 mm contro `cfg.H = 50 mm` di alzata del piede in
volo.** Il robot non può scavalcarlo: può solo salirci passando dai gradini da
35 e 70. È il motivo per cui su T6 «distanza percorsa» è la metrica che conta
davvero — dice **fino a che gradino sono arrivati** — e per cui i due
controllori incontrano ostacoli diversi.

Questo spiega anche `alt_ost` (mediana dell'altezza degli ostacoli calpestati):
4.3 cm per C1, che si ferma presto sulla scala, contro 6.2 cm per C2, che
arriva più in alto. Non è un'incongruenza: è la scala.

### Le quattro differenze ipotizzate

| # | ipotesi | esito |
|---|---|---|
| 1 | quota degli ostacoli | **cade**: pavimento e ostacoli ricevono lo stesso offset in entrambe le versioni, e l'altezza relativa è nelle mesh, che sono le stesse. Gli ostacoli sporgono 35/70/106 mm in tutti e due i casi |
| 2 | inerzie URDF → corrette | **è questa** — vedi sotto |
| 3 | mappa dei contatti | **cade**: è la corrispondenza `File Solid k` → `Spatial Contact Force`, e `applica_terreno` **nasce** nel commit che la corregge. Il 15/9 non esisteva né la funzione né la mappa |
| 4 | soglia di C2 fuori dal `.slx` | **cade**: il valore `[0.04 0.1 0.1]` è quello che era cablato, verificato il 23/9 |

### [CHIUSO 25/9] La risposta: sono le inerzie

T6 con C2 rilanciato oggi, con `applica_inerzie` commentata — cioè con le
inerzie dell'URDF, le stesse che aveva il modello del 15/9 — contro la run
archiviata del 21/9:

| | pitch_max | roll_max | distanza | yaw fin | cot | energia | `alt_ost` |
|---|---|---|---|---|---|---|---|
| archivio 21/9, URDF | 14.390° | 8.085° | 3.680 m | −4.525° | 5.295 | 303.096 | 5.3 cm |
| run del 25/9, URDF | **14.390°** | **8.085°** | **3.680 m** | **−4.525°** | **5.295** | **303.096** | **5.3 cm** |

**Identiche a sei cifre.** Il C2 «migliore» dei video è il C2 con le inerzie
sbagliate, e si riproduce a comando.

Non è un risultato nuovo: è esattamente quello che la § T6 dice dal 23/9 —
*«era il risultato più forte a favore di C2 in tutta la campagna, ed era un
artefatto del modello»*. L'indagine lo conferma per una via indipendente,
partendo dai video invece che dalle tabelle.

> **Perché il primo tentativo era sembrato fallire.** Ricommentare
> `applica_inerzie` era stato provato subito, e giudicato guardando
> l'animazione. Su un percorso a gradini due run identiche possono sembrare
> diverse: la traiettoria è la stessa ma l'occhio non lo certifica. I numeri
> coincidono a sei cifre. **Un confronto fra run si fa sui CSV, non sul
> video** — ed è la stessa regola che il pavimento di rumore ci aveva già
> imposto sulle colonne.

La quota assoluta del terreno resta diversa fra le due versioni (il 15/9 tutti
gli otto solidi a `floor_off` = 0.025, oggi a 0.05) ma **non serve più
indagarla**: le inerzie da sole rendono conto di tutto lo scarto, sulle nove
colonne confrontate.

### Difetti veri trovati per strada

**`cfg.floor_dim = [4 4 0.05]` è metà della mesh.** Il cubo del pavimento è
8 × 8 × 0.1 m, misurato dal bounding box. Da `floor_dim` discendono
`floor_off` e `floor_top`.

Oggi `floor_top = 0` è comunque giusto, ma **per compensazione**: il blocco del
pavimento ha l'offset scritto a mano `[0,0,0.05]` invece di `floor_off`, e i
due errori si annullano.

> ⚠️ **Non correggere `floor_dim` da solo.** Raddoppiare `floor_dim(3)` senza
> toccare il resto sposta il pavimento di 25 mm e rompe la campagna. Vanno
> rifatti insieme: `floor_dim` vero, `floor_off` coerente, e l'offset del
> blocco che usa `floor_off` invece di un numero. Con una run di verifica.

**Le mesh del terreno non erano tracciate** fino al 16/9, e i blocchi
puntavano a `C:\Users\eros2\OneDrive\Desktop\…`: percorsi assoluti sul PC di
un solo membro del gruppo. Quel modello non era eseguibile da nessun altro, e
i video li ha girati chi aveva quei file.

**`script_T6.m` non era in git** quando sono state prodotte le run T6
archiviate del 21/9 sera: è entrato in `3e13f29`. Quel riferimento non è
ricostruibile esattamente dal repo. Il commit va fatto **prima** di lanciare
una campagna, non dopo.

### Conseguenze per le conclusioni già scritte

Nessuna sezione viene ritirata, e la riserva che temevamo — «forse stiamo
misurando un C2 rotto» — **è rimossa**: C2 è quello giusto, e lo scarto
rispetto ai video è interamente spiegato dalle inerzie.

Quindi la scelta di C3 può essere fatta sui dati della campagna, senza il
sospetto di star correggendo un difetto nostro invece di un limite del
controllore cinematico. Era la domanda da cui questa indagine è partita.

Resta aperta, come prima, la taratura di `cfg.c2.soglia_tau = [0.04 0.1 0.1]`,
mai validata contro i falsi positivi e negativi del flag di contatto. È una
cosa diversa dai cinque parametri della ricerca, ed è aperta dal 23/9.

### Errori di metodo di questa indagine, per non ripeterli

- **«gli ostacoli sono 25 mm più alti oggi»**: scritto come misura, era
  un'inferenza da due parametri. Falso: l'altezza relativa non è cambiata.
- **«3.5 cm contro i 4.3–6.2 riportati da T6»**: avevo misurato **un** ostacolo
  su sette e generalizzato. Gli ostacoli sono di tre altezze diverse.
- **«`alt_ost` usa `cfg.floor_top`, quindi è contaminata»**: falso, `alt_ost`
  è la mediana delle quote dei piedi fermi e non usa `floor_top`.

### Come rifare le prove

**Estrarre una versione vecchia senza toccare il repo:**

```bash
git archive --format=zip -o /c/fp15settembre.zip 1cb101d
```

Estrarre in `C:\fp15settembre`. Poi da MATLAB, ripuntando le mesh (i percorsi
originali sono su un altro PC):

```matlab
bdclose all;  restoredefaultpath
cd 'C:\fp15settembre';  clear all;  startup_phantomx
open_system('phantomx_sim_zero')

props = 'G:\Il mio Drive\FSR\Final_project\simscape\props';
b = find_system('phantomx_sim_zero','SearchDepth',1,'MaskType','File Solid');
for k = 1:numel(b)
    [~, n, e] = fileparts(get_param(b{k},'ExtGeomFileName'));
    set_param(b{k}, 'ExtGeomFileName', fullfile(props, [n e]));
end
```

poi Run. **Non salvare il modello.**

**Estrarre il codice che vive nel `.slx`:** `estrai_funzioni('<modello>','<etichetta>')`
scrive un file per blocco `MATLAB Function` in `estratti/<etichetta>/`, e da lì
due versioni si confrontano con `visdiff`. È l'unico modo di fare un diff su
logica che sta dentro un file binario.

> **Nota di metodo.** `git checkout` e `git worktree` su questo repo falliscono
> a metà: sta dentro Google Drive, che sfila i file mentre git li scrive, e si
> resta con l'albero di lavoro svuotato — recuperabile, ma spaventa, e serve
> `bdclose all` in MATLAB prima di ogni operazione. `git archive` scrive un solo
> zip fuori da Drive e non tocca il repo. Finché il progetto resta lì, è l'unico
> modo affidabile di guardare una versione vecchia.

---

## 11. [25/9] Dal 22 al 25 settembre C2 non ha cercato il terreno

> **Perché questa sezione esiste.** L'indagine della §10 aveva chiuso il *cosa*
> — i video del 15/9 erano migliori per le inerzie — ma non il *perché*. Questa
> sezione chiude il perché, e la risposta è che il difetto era **nostro**: una
> correzione applicata a metà. Le righe C2 di ogni tabella prodotta dopo il 22/9
> non descrivono C2.

### 11.1 La regola del flag, letta nel modello

Il repo la descriveva in due modi incompatibili, e non potevano essere entrambi
giusti:

| dove | cosa diceva |
|---|---|
| `phantomx_config` | `cont = OR( \|τ_mis − τ_att\| > soglia )` |
| `abilita_log`, `README` | `cont = OR( \|τ\| > soglia )` |

Risalendo nel `.slx` il collegamento del `Constant c2_soglia` (funzione
`vs_regola` in `valida_soglia.m`, sola lettura):

```
Inverse Dynamics → Reshape → Subtract [+ −] → Abs → Demux → Mux → Relational Operator
                                    ↑                                      ↑
                              tau_misurata                            c2_soglia
```

sei `Relational Operator`, uno per zampa. **Vince `phantomx_config`**: è la
differenza. `abilita_log` e `README` sono stati corretti.

`τ_attesa` esce da `phantomx_sim_zero/Inverse Dynamics`, un `MATLABSystem`
(`robotics.slmanip.internal.block.InverseDynamicsBlock`) la cui maschera ha il
parametro `RigidBodyTree = robotModel`. E `robotModel` nasce in
`Setup_robot_object.m` da `importrobot` **sull'URDF**, cioè con le inerzie
sbagliate di mille volte.

### 11.2 Il meccanismo

`applica_inerzie` corregge i 25 `Solid` **di Simscape** (`MomentsOfInertia`).
Il `rigidBodyTree` non lo tocca. Quindi dal 22/9:

| | inerzie |
|---|---|
| il robot che cammina | corrette |
| il modello che prevede la coppia | URDF, ~1000× |

`|τ_mis − τ_att|` diventa dominato dall'errore di modello, con o senza contatto,
e il flag resta acceso. Con il flag incollato a 1, in `ricerca_terreno` non si
entra mai nel ramo `c(i) == 0`, che è **l'unico** che fa crescere `z_ext` e
l'unico che azzera `z_ext` e `z_hold`: il comando degenera in
`min(z, z_hold(t_reset))`, una costante. Niente lo segnala — il robot cammina e
le metriche escono.

### 11.3 La misura

`valida_soglia.m`: una sola run T2 C2 da 10 s. Nel log ci sono insieme
`torque_sens`, `torque_estim` e `Fleg`, quindi il flag si ricalcola offline per
qualunque soglia e si confronta con la forza normale al piede, che è la verità.
Tre configurazioni, stessa misura, soglia `[0.04 0.1 0.1]`:

| | robot corretto, stimatore URDF **(campagna 22–25/9)** | entrambi URDF *(fino al 22/9)* | entrambi corretti *(dal 25/9)* |
|---|---|---|---|
| errore in appoggio | 7.0% | 0.9% | **0.6%** |
| errore di un flag **sempre acceso** | 7.0% | 8.7% | 3.2% |
| il flag è meglio della costante? | **no, identico** | sì | sì |
| flag acceso in volo | 95.1% | 8.3% | 41.9% |
| voli senza reset di `z_ext` | **20 / 60** | 1 / 60 | 2 / 60 |
| appoggi non rilevati | 0 / 79 | 0 / 86 | 0 / 67 |

La colonna di sinistra è la diagnosi: un flag binario che vale sempre 1 sbaglia
esattamente quanto la frazione di appoggio senza forza, e non porta
informazione.

> **Nota metodologica, costata due errori.** La prima versione di
> `valida_soglia` misurava l'errore **solo in fase di appoggio** e dichiarava la
> soglia «validata» al 97.6% di accordo. Quella maschera escludeva proprio i
> campioni in cui il flag sbaglia. E il criterio di separazione confrontava
> l'errore con zero invece che con quello di un predittore costante, che su una
> classe sbilanciata è la sola soglia di riferimento sensata. Entrambi corretti.

### 11.4 La correzione

`allinea_stimatore.m` riscrive massa e inerzia dei corpi di `robotModel` con gli
stessi `cfg.I_*` di `applica_inerzie` e forza la maschera a rivalutare, **in
memoria**, senza `save_system`. Fa tre verifiche in sequenza — scrittura
sull'oggetto, ripiego su `replaceBody`, e rilettura del *mask workspace* — e si
ferma con errore se una non passa: un fallimento silenzioso qui rimetterebbe in
piedi esattamente il problema che sta chiudendo.

Va chiamata **sempre subito dopo `applica_inerzie`**. È in
`script_T2/T3/T4/T4D/T5/T6/T7`, `spazzata_soglia` e `valida_soglia`.

### 11.5 La soglia, ritarata

Con lo stimatore allineato il flag funziona, ma con `[0.04 0.1 0.1]` si accende
sul 41.9% della fase di volo — e per il **39.7%** è colpa della sola **coxa**:
la sua distribuzione di `|Δτ|` ha p90 in volo `0.0421` contro p10 a terra
`0.0469`. Ha senso fisico: la coxa ruota attorno all'asse verticale e il
contatto è verticale, quindi le trasmette pochissima coppia, mentre in volo
porta tutta l'accelerazione laterale della zampa.

`soglia_ottima.m` (ottimo per giunto, poi discesa per coordinate sull'OR vero,
sui dati di una run già fatta) ha scelto `[0.385 0.136 0.105]`: sulla coxa
**sopra la sua stessa mediana in appoggio**, cioè spegnerla. Il contatto lo
rilevano femore e tibia. **Non è una ritaratura, è un cambio di regola**: il
flag diventa un OR su due giunti.

Decisa su T6, perché su terreno piano le due terne sono equivalenti
(`prova_soglia_T6.m`, metriche su finestra comune fino a x = 3.043 m):

| | `[0.04 0.1 0.1]` | `[0.385 0.136 0.105]` |
|---|---|---|
| esce dalla zona ostacoli | **mai** (t = 30 s = fine run) | **t = 21.55 s** |
| `roll_max` | 0.160 | **0.122** |
| `roll_rms` | 0.0580 | **0.0286** |
| `dev_lat_max` | 0.539 | **0.281** |
| `yaw_err_fin` | −0.546 | **+0.098** |
| `cot` | 2.085 | **1.330** |
| energia | 100.2 J | **63.2 J** |
| `tau_max` | **6.86 N·m** | 8.38 N·m |
| ritardo mediano del flag (T2) | **−220 ms** | **−10 ms** |

L'ultima riga è quella che spiega tutte le altre: con la terna del modello il
flag si alza 220 ms **prima** del contatto e congela la zampa in aria. Su un
pavimento non si vede, su sette ostacoli costa metà del rollio e un terzo
dell'energia.

Adottata `[0.385 0.136 0.105]`, a tre cifre perché l'ottimo viene da una run
sola. **Va dichiarato in relazione che questo parametro l'abbiamo tarato noi**,
con il criterio e la misura qui sopra: non è più il valore del modello di
partenza.

### 11.6 Un errore di definizione del task, trovato per strada

`script_T6` non troncava la run al bordo del pavimento, mentre `script_T4D` lo
fa da giorni. Il pavimento è il cubo 8 × 8, quindi `x ∈ [−4, 4]`; `t6_dur = 30 s`
a velocità nominale sono ~3.6 m e i piedi arrivano 15–20 cm davanti al corpo.
Nella run con la terna nuova:

```
esce dagli ostacoli   t = 21.55 s,  x = 2.709 m
piede oltre il bordo  t = 28.85 s,  x = 3.696 m
rollio > 30°          t = 29.64 s,  x = 3.907 m      ← 0.79 s DOPO
x massima             3.958 m                         ← il pavimento finisce a 3.95
```

`metriche` leggeva quella caduta come `causa_fallimento = "ribaltamento"` e
`superato` diventava `false`: **l'unica delle due run che aveva completato il
percorso risultava l'unica ad aver fallito**. Aggiunto `t6_taglia_bordo`, con la
colonna `t_bordo` nella riga.

Resta aperta una scelta di progetto del task: T6 è **al limite del pavimento per
costruzione**, e qualunque controllore più veloce ci finisce sopra. O si accorcia
`t6_dur`, o si allarga il pavimento. Va deciso prima di misurare C3, che se
funziona sarà più veloce.

### 11.7 Conseguenze

| | |
|---|---|
| righe **C1** | **valide.** Verificato: rifatta T2 C1 dopo l'allineamento, CSV identico byte per byte (`git diff` vuoto). In C1 `par(1) = 0`, il flag non è nel percorso del comando |
| righe **C2** | **da rifare tutte**, su ogni task |
| `results/diagnostica/spazzata_soglia.csv` (23/9) | prodotta con lo stimatore disallineato: da rifare o da marcare come non valida |
| righe **T7** | C1 valide, C2 da rifare |
| la §10 | resta valida: la causa prossima erano le inerzie. Questa sezione ne dà il meccanismo |

### 11.8 Cose aperte che questa indagine ha scoperto e non ha chiuso

1. ~~**Il filtro da 50 ms.**~~ **[RITIRATO 26/9]** Avevo scritto che
   l'`InitFcn` costruisce 18 filtri del primo ordine da 50 ms, che nessun
   documento del progetto lo menzionava, e che era il primo candidato a
   spiegare l'8% di flag acceso in volo rimasto dopo l'allineamento.
   **Sbagliato due volte.** Era documentato — `archivio/README.md`, indagine
   sul fallimento a 2× — e soprattutto quel filtro è **inerte**:
   `trova_filtri.m` ha censito 2085 blocchi, commentati e con coefficienti
   scritti a numero compresi, e nessuno legge `sys_filter`; `prova_filtri.m`
   ha spazzato cinque valori di `tau` ottenendo run identiche bit per bit.
   L'`InitFcn` lo costruisce, lo stampa, e il modello lo ignora.
   Conseguenza: **l'8% di flag acceso in volo resta senza spiegazione**, e il
   candidato va cercato altrove — verosimilmente nell'accuratezza residua
   dello stimatore durante il volo, dove la zampa accelera e il modello
   inverso sbaglia di più.

2. **`contact_sched` concorda col contatto reale solo al 67%** su T6 con la
   terna vecchia. Lo schema dell'andatura si scolla dal contatto su terreno
   accidentato: va tenuto presente per ogni metrica che lo usa come maschera.
3. **I sensori di forza chiudono al 41–65% del peso su T6**: vedono solo il
   pavimento, non i piedi sugli ostacoli. Tutte le colonne di contatto di T6
   sono `NaN` per costruzione. Preesistente.
4. **`taratura_T2.m` e `script_T4_limite.m`** non chiamano `applica_inerzie` né
   `allinea_stimatore`. Aggiungerle cambierebbe i risultati di quegli script,
   quindi la decisione è rimandata: non è una correzione neutra.
5. **`salva_grafico.m`** è stato modificato a mano durante l'indagine del 15/9 e
   scrive in `grafici/inerzie_og/`. Da rimettere a posto. **[CHIUSO, verificato
   il 2/10]** `salva_grafico` scrive in `grafici/`; `inerzie_og/` è in
   `archivio/grafici/`.
6. **La terna è stata decisa su T6 solo.** T5 e T4D non sono stati rifatti con
   quella nuova. Se una delle due peggiora molto, la scelta va ridiscussa.
