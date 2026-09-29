# Ricevimento di mercoledì — scaletta

*Progetto FSR, PhantomX in Simscape: confronto fra controllori.
Traccia per ~20 minuti. Scritta a due voci — **A** = Andrea, **E** = collega —
sul presupposto che ci siate entrambi; se ci vai solo tu, leggi i punti **E**
come «il collega ha fatto».*

---

## 0. L'apertura, in una frase (30 s) — A

> «Il confronto C1 / C2 è chiuso e misurato. Metà del lavoro però è stato
> scoprire che il banco di prova era sbagliato in due punti, e le tabelle di
> prima erano tutte da rifare. Vorremmo capire se questa parte conta, e come
> chiudere sulla saturazione.»

Non partire dai risultati: parti da lì. È la cosa che ti distingue.

---

## 1. Cos'è C1, cos'è C2 (1 min) — A

| | |
|---|---|
| **C1** | cinematico ad anello aperto, tripode, giunti in posizione |
| **C2** | Arrigoni et al. 2024 — C1 + **ricerca del terreno**: il piede scende finché la coppia non segnala il contatto |
| **C3** | retroazione sull'assetto (beccheggio). Fatto, vedi §5 |

Sette task: piano, curva, rampa, dosso, ostacolo singolo, percorso a ostacoli,
disturbo impulsivo.

---

## 2. I due difetti del banco — il pezzo forte (5 min) — A

Racconta i due in sequenza, perché il secondo è **causato** dalla correzione
del primo. È questa la storia.

### 2.1 Le inerzie dell'URDF sono sbagliate di ~1000×

Due prove, entrambe indipendenti dalla sorgente del file:

- **raggio di girazione** `r = √(I/m)`: il corpo avrebbe la massa a **1.79 m**
  dal centro ed è lungo 25 cm; ogni link a 0.46 m ed è lungo 2–12 cm
- **i 24 link hanno inerzia identica** allo stesso decimale — coxa, femore e
  tibia hanno forma diversa: è un segnaposto, non una misura

Le **masse sono giuste**: sbagliato solo il blocco `<inertia>`, e dividendo per
1000 rientra tutto. È la firma dell'export CAD con la massa in grammi.

**Perché conta:** con le inerzie dell'URDF il beccheggio di C1 su accidentato
era 45°, con quelle corrette 15°; il rapporto di costo C2/C1 passava da 1.35 a
0.68, cioè **cambiava segno**.

### 2.2 Correggendo il robot abbiamo rotto il rilevatore di contatto

Il flag di C2 non è una soglia sulla coppia, è

```
contatto = OR( |τ_misurata − τ_attesa| > soglia )   sui 3 giunti
```

e `τ_attesa` viene da un blocco *Inverse Dynamics* che porta dentro una copia
del robot importata **dallo stesso URDF**. Correggendo solo il robot simulato,
i due sono diventati robot diversi: `|Δτ|` grande **sempre**, flag acceso sul
95% della fase di volo. Con il flag incollato a 1 la ricerca del terreno
degenera in una costante.

> **Per tre giorni C2 non ha cercato il terreno**, e niente lo segnalava: il
> robot camminava, l'animazione era plausibile, le metriche uscivano.

**Come l'abbiamo misurato** (è la parte che al prof interessa): una sola
simulazione da 10 s in cui il log contiene insieme coppia misurata, coppia
attesa e **forza normale al piede**. Il flag si ricalcola offline per qualunque
soglia e si confronta con la forza, che è la verità.

| a soglia invariata | stimatore disallineato | allineato |
|---|---|---|
| errore del flag in appoggio | 7.0% | **0.6%** |
| errore di un flag **sempre acceso** | 7.0% | 3.2% |
| il flag porta informazione? | **no, come una costante** | sì |

La riga chiave è la seconda: un flag che vale sempre 1 sbagliava *esattamente
quanto il nostro*. È la diagnosi in un numero.

---

## 3. Il metodo, se te lo lascia dire (2 min) — A

Tre regole, adottate dopo aver sbagliato almeno una volta per ciascuna:

1. **Pavimento di rumore.** Ogni run ripetuta 3× spostando la quota d'appoggio
   di ±0.2 mm. Una differenza fra controllori **si dichiara solo se vale ≥ 3×**
   quell'escursione. Misurato su 4 task, entrambi i controllori.
2. **Il criterio si scrive prima di lanciare** — ogni script di diagnostica ha
   in testa domanda, soglia e conclusioni possibili.
3. **Le smentite restano scritte**, marcate `[RITIRATO <data>]` dov'erano.

Se ha poco tempo, questo è il punto da sacrificare — ma è il pezzo più
trasferibile ad altri progetti.

---

## 4. Cosa possiamo dichiarare (4 min) — A

Solo differenze che passano il ≥3×.

| | C1 | C2 | |
|---|---|---|---|
| **piano e curva** — beccheggio max | 0.0090 | 0.0354 | C2 **3.9× peggio** |
| **piano e curva** — energia, costo | — | — | nessuna differenza leggibile |
| **ostacolo (T5)** — beccheggio max | 0.132 | **0.112** | C2 meglio |
| **ostacolo (T5)** — costo di trasporto | 1.198 | **1.055** | C2 **−12%** |
| **dosso (T4D)** — rollio | — | **−28%** | C2 meglio |
| **dosso (T4D)** — beccheggio max | — | **+21%** | C2 **peggio**, solido |
| **percorso a ostacoli (T6)** | **2.44 m** | **3.71 m** | 1.3 m, contro 0.03–0.10 m di rumore |
| **disturbo impulsivo (T7)** | non recupera | non recupera | buco comune |

**La tesi, da dire così:**

> «La ricerca del terreno **costa assetto dove non c'è niente da cercare e lo
> ripaga sulle discontinuità**. È la tesi del paper, misurata, con in più il
> prezzo — che l'articolo non quantifica.»

Due cautele da dichiarare tu, prima che le chieda lui:

- su T6 **le metriche fini non vanno citate**: i due controllori seguono
  traiettorie diverse e non incontrano gli stessi ostacoli. Di T6 vale solo
  l'esito binario (arriva / non arriva). Il confronto fine lo danno T5 e T4D
- **una soglia l'abbiamo tarata noi**: `[0.04 0.1 0.1] → [0.385 0.136 0.105]`.
  La nuova **esclude di fatto la coxa**, perché quel giunto ruota attorno
  all'asse verticale e il contatto è verticale, quindi non lo vede. È un
  **cambio di regola**, non una ritaratura

---

## 5. Il pitch control (3 min) — E

- funziona bene **sulla rampa**
- provato con un **pacco poggiato sul robot** su T4 e su T6

Da dire in chiusura del punto, ed è una domanda per lui: *non ci sono ancora
righe di campagna per C3* — il pitch control non è stato misurato con lo stesso
protocollo e gli stessi task degli altri due.

---

## 5-bis. L'MPC: dirlo, ma dirlo nel modo giusto (2 min) — A

L'RF-MPC è stato **portato dal quadrupede all'esapode e fatto girare sul
simulatore ridotto** (`mpc_srb/`, modello a corpo singolo). Vale la pena
menzionarlo, ma **come si inquadra decide se aiuta o fa danno.**

| **non** dire | dire |
|---|---|
| «abbiamo fatto anche un MPC» | «l'MPC funziona sul simulatore ridotto, e **non l'abbiamo messo nel confronto per una ragione metodologica**» |

L'inquadramento che regge:

> «Il porting è stato fatto e l'MPC gira sul modello a corpo singolo. Non
> l'abbiamo portato nel confronto perché quel modello **è un robot diverso** da
> quello di Simscape: avremmo confrontato due impianti, non due controllori.
> Abbiamo preferito un terzo controllore sullo stesso impianto.»

Detta così, l'abbandono diventa una **decisione documentata** e non un pezzo
non finito — è la stessa disciplina della soglia di leggibilità e delle
smentite marcate. Detta nell'altro modo, la prima domanda che arriva è «e i
risultati?», e non ci sono.

Cosa puoi mostrare a supporto, se te lo chiede:

- il porting 4 → 6 zampe: cambia la schedulazione dei contatti e **la
  distribuzione delle forze diventa più ridondante** (6 piedi invece di 4 per
  lo stesso wrench a 6 componenti) — è un'osservazione tecnica vera, nata dal
  porting
- cosa servirebbe per chiudere l'anello su Simscape, e perché non è mezz'ora

**Domanda da fargli su questo punto:** un risultato parziale su simulatore
ridotto, con la ragione dichiarata per cui non entra nel confronto, **conta
come lavoro svolto** o è meglio non citarlo affatto?

---

## 6. La saturazione degli attuatori (2 min) — A

Il simulatore concede qualunque coppia; il servo reale dà **1.5 N·m**.
Misurato su tutte le celle nominali:

| | C1 | C2 |
|---|---|---|
| **coppia RMS del giunto peggiore** / limite | 41–46% | 41–44% |
| campioni oltre il limite | 0.4–1.0% | 0.4–0.7% |
| picco peggiore (T6) | 7.5 N·m = 5.0× | 8.5 N·m = 5.6× |

**La lettura in una frase:**

> «La marcia sta dentro con margine — il giunto più caricato viaggia sotto
> metà del limite. Quello che non sta dentro sono i **picchi d'urto**, su meno
> dell'1% dei campioni.»

Quindi la saturazione **non cambia il confronto**, ridistribuisce gli impatti.
È un risultato, non un problema aperto.

---

## 7. Le domande da fargli (3 min)

Portale scritte, e in quest'ordine.

1. **Il lavoro di diagnosi e correzione del banco** — due difetti trovati,
   misurati e documentati — **vale come contributo**, o il progetto si valuta
   solo sul confronto fra controllori?
2. **Saturazione**: basta la verifica di fattibilità qui sopra, dichiarando la
   campagna ad attuatore ideale, o vuole il confronto **rifatto** con il limite
   attivo su tutti e tre?
3. **C3 va misurato in formato campagna** su tutti e sette i task, o basta
   documentarlo dove agisce (rampa, dosso, carico)?
4. **T6 esaurisce il pavimento** (8 × 8 m, il percorso lo consuma tutto): un
   controllore più veloce esce dal mondo. Allarghiamo il pavimento o
   accorciamo la run? Va deciso **prima** di misurare C3.

---

## 8. Se ha tempo e chiede altro — riserva

| ti chiede | rispondi |
|---|---|
| *perché il flag resta acceso sull'8% del volo?* | non spiegato. Il candidato ovvio (un filtro da 50 ms nell'InitFcn) è stato **escluso**: esiste ma nessun blocco lo legge. Il sospetto è l'accuratezza del modello inverso durante il volo |
| *e i sensori di forza?* | su accidentato vedono solo il pavimento, non i piedi sugli ostacoli (chiudono al 41–65% del peso): le metriche di contatto di quel task sono **indisponibili per costruzione** |
| *perché C2 è più sensibile di C1?* | C1 è ad anello aperto: ripete la stessa traiettoria qualunque cosa faccia il terreno. La sua ripetibilità **non è una virtù del controllo, è l'assenza di controllo** |
| *l'MPC?* | abbandonato con ragione esplicita: girava su un modello a corpo singolo, diverso da quello Simscape. Si sarebbero confrontati due impianti, non due controllori |

---

## Da portare

- questo foglio
- `docs/stato_progetto.pdf` (4 pagine, per lasciarglielo)
- il grafico di fattibilità (due pannelli, RMS e picchi sul limite)
- il video/animazione di T6 C1 vs C2, se riesci: **1.3 m di differenza si
  vedono**, e vale più di qualunque tabella
