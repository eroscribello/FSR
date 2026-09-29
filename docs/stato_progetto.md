# Stato del progetto — confronto fra controllori sul PhantomX in Simscape

*Aggiornato al 29 settembre. Documento di sintesi: cosa è stato fatto, cosa si
può dichiarare, cosa resta aperto. Le misure e i dettagli stanno in
`docs/piano_confronto.md`.*

---

## 1. L'obiettivo

Confrontare su simulatore tre controllori per l'esapode PhantomX su sette task
(piano, curva, rampa, dosso, ostacolo singolo, percorso a ostacoli, disturbo
impulsivo):

| | |
|---|---|
| **C1** | cinematico ad anello aperto, andatura a tripode, giunti comandati in posizione |
| **C2** | Arrigoni et al. (*Robotics* 2024, 13, 142): C1 più la **ricerca del terreno** — il piede continua a scendere finché la coppia ai giunti non segnala il contatto |
| **C3** | retroazione sull'assetto del corpo (beccheggio), sullo stesso impianto degli altri due |

C3 era inizialmente un MPC (RF-MPC di Ding et al.). Abbandonato con ragione
esplicita: girava su un modello a corpo singolo, cioè un robot diverso da
quello di Simscape, e il confronto sarebbe stato fra due impianti invece che
fra due controllori.

---

## 2. Il risultato principale non era nel programma

La parte più consistente del lavoro è stata **trovare e correggere due difetti
del banco di prova**, entrambi nostri, entrambi capaci di invertire le
conclusioni. Le tabelle prodotte prima delle correzioni erano sbagliate e nulla
lo segnalava: il robot camminava, l'animazione era plausibile, i numeri
uscivano.

### 2.1 Le inerzie dell'URDF sono sbagliate di circa mille volte

Le **masse sono giuste**; le inerzie no. Due prove indipendenti dalla sorgente
del file:

- **raggio di girazione.** Da `I = m·r²`, il corpo avrebbe la massa distribuita
  a **1.79 m** dal centro — un pezzo da 25 cm. Ogni link a 0.46 m — pezzi da
  2–12 cm. Letti in kg·m², come impone lo standard URDF, quei numeri descrivono
  un oggetto che non esiste.
- **i 24 link hanno inerzia identica**, allo stesso decimale. Coxa, femore e
  tibia hanno forma e lunghezza diverse: è un valore segnaposto.

Dividendo per 1000 rientra tutto: è la firma dell'errore di esportazione CAD
con la massa in grammi.

**Effetto sulle conclusioni, non solo sui valori:** con le inerzie dell'URDF il
beccheggio di C1 su percorso accidentato arrivava a 45°, con quelle corrette a
15°; il rapporto di costo fra C2 e C1 passava da 1.35 a 0.68, cioè **cambiava
segno**.

### 2.2 Correggendo il robot abbiamo rotto il rilevatore di contatto di C2

Il flag di contatto di C2 non è una soglia sulla coppia misurata: il modello
calcola

```
contatto = OR( |τ_misurata − τ_attesa| > soglia )   sui tre giunti
```

e `τ_attesa` viene da un blocco *Inverse Dynamics* che porta dentro una copia
del robot importata **dallo stesso URDF**. Correggendo le inerzie del solo
robot simulato, il robot e il modello che ne prevede la coppia sono diventati
due robot diversi: la differenza `|τ_mis − τ_att|` è diventata grande
**sempre**, con o senza contatto, e il flag è rimasto acceso sul 95% della fase
di volo. Con il flag sempre acceso la ricerca del terreno degenera in una
costante — **per tre giorni C2 non ha cercato il terreno**.

Misurato con una sola simulazione da 10 s, in cui il log contiene insieme la
coppia misurata, quella attesa e la forza normale al piede: il flag si
ricalcola offline per qualunque soglia e si confronta con la forza, che è la
verità.

| a soglia invariata | stimatore disallineato | allineato |
|---|---|---|
| errore del flag in appoggio | 7.0% | **0.6%** |
| errore di un flag **sempre acceso** | 7.0% | 3.2% |
| il flag porta informazione? | **no, come una costante** | sì |
| voli senza reset della ricerca | 20 / 60 | 2 / 60 |

La seconda riga è la diagnosi in un numero: un flag binario che vale sempre 1
sbagliava esattamente quanto il nostro.

---

## 3. Il metodo, che è il contributo trasferibile

Tre regole adottate in corso d'opera, dopo aver sbagliato almeno una volta per
ciascuna.

**Un pavimento di rumore, e una soglia di leggibilità.** Ogni run si ripete tre
volte spostando la quota d'appoggio di ±0.2 mm — una grandezza che non riguarda
il controllore. L'escursione che ne risulta è il rumore. **Una differenza fra
due controllori si dichiara solo se vale almeno 3 volte quel rumore.** Misurato
su quattro task, con entrambi i controllori. Applicandolo, diverse differenze
che sembravano risultati sono sparite; applicandolo *male* — con il pavimento
misurato su un controllore diverso — per un giorno abbiamo creduto che non
fosse dichiarabile niente.

**Il criterio si scrive prima di lanciare.** Ogni script di diagnostica ha in
testa la domanda, la soglia e le conclusioni possibili, decise prima di vedere
i numeri.

**Le smentite restano scritte**, marcate `[RITIRATO <data>]` nel punto in cui
stavano. Su questo progetto un'affermazione sbagliata è sopravvissuta tre
giorni proprio perché era plausibile.

---

## 4. Cosa si può dichiarare

Solo differenze che passano il ≥3× sul rumore misurato.

### Su terreno liscio, C2 costa

| piano (T2) e curva (T3) | C1 | C2 | |
|---|---|---|---|
| beccheggio massimo | 0.0090 | **0.0354** | C2 **3.9×** peggio |
| beccheggio RMS | 0.0045 | **0.0221** | C2 **4.9×** peggio |
| energia, costo di trasporto | — | — | **nessuna differenza leggibile** |

La ricerca del terreno lavora anche dove non c'è niente da trovare, e si paga
in assetto senza comprare niente.

### Sulle discontinuità, C2 ripaga

| ostacolo singolo (T5) | C1 | C2 | rapporto sul rumore |
|---|---|---|---|
| beccheggio massimo | 0.132 | **0.112** | 10.0 |
| costo di trasporto | 1.198 | **1.055** | 6.1 |
| energia | 48.1 | **43.4** | 4.4 |
| velocità media, frazione di task completata | — | C2 meglio | 4.6–5.3 |
| rollio, deriva laterale, coppia di picco | — | — | **non leggibili** |

Sul dosso (T4D) lo stesso quadro — C2 meglio su rollio (4.5), energia (19.9),
velocità (10.4) — **con un'eccezione solida: il beccheggio massimo è il 21%
peggiore, rapporto 18.** Su una pendenza continua la ricerca estende i piedi a
valle e il corpo segue il pendio.

### Sul percorso a ostacoli, la differenza è grossolana

| T6 | C1 | C2 |
|---|---|---|
| distanza percorsa | **2.44 m** | 3.71 m |
| frazione del task | **81%** | 107% |
| arriva agli ostacoli finali | **no** | sì |

1.3 metri di differenza, contro un rumore di 0.03–0.10 m. *Le metriche fini di
T6* (rollio, beccheggio, costo) **non vanno citate**: i due controllori seguono
traiettorie diverse e quindi non incontrano gli stessi ostacoli. Il confronto
fine lo danno T5 e T4D.

### Sul disturbo impulsivo, nessuno dei due funziona

Impulso laterale crescente, dal 1% al 1600% della quantità di moto nominale:
**né C1 né C2 recuperano una frazione misurabile della deviazione**. A impulso
grande le due curve coincidono (394 e 395 mm di deviazione residua). Il
rilevamento del contatto non c'entra: è un buco di architettura comune.

### E una misura che è direttamente la tesi del paper

Il C2 riparato è da **5 a 25 volte meno sensibile** del C2 rotto a un errore di
quota del terreno di 0.2 mm — assorbire un errore di quota è precisamente il
mestiere della ricerca del terreno. Va però detto per intero: **C2 resta più
sensibile di C1**, fino a 40–166× sull'assetto in piano. C1 è ad anello aperto
e ripete la stessa traiettoria qualunque cosa faccia il terreno; la sua
ripetibilità non è una virtù del controllo, è l'assenza di controllo.

---

## 5. La tesi che ne esce

> **La ricerca del terreno costa assetto dove non c'è niente da cercare e lo
> ripaga sulle discontinuità.** Su terreno liscio C2 peggiora il beccheggio di
> 3–4 volte senza risparmiare energia; su spigolo e dosso migliora rollio,
> energia e completamento del task; su un percorso a ostacoli è la differenza
> fra arrivare in fondo e non arrivarci.

È la tesi di Arrigoni et al., misurata, **con il prezzo esplicito** — che
l'articolo non quantifica. È più difendibile di «il nostro C2 è migliore
ovunque», che non sarebbe vera.

---

## 6. Fattibilità con attuatori reali

La campagna gira ad attuatore ideale: il simulatore concede qualunque coppia.
Il limite del servo è `τ_max = 1.5 N·m`. Misura su tutte le celle nominali,
sul **giunto più caricato** di ciascuna run (la media sui 18 giunti diluisce:
la coxa porta quasi niente):

| | C1 | C2 |
|---|---|---|
| coppia RMS del giunto peggiore / limite | **41–46%** | **41–44%** |
| frazione di campioni oltre il limite | 0.4–1.0% | 0.4–0.7% |
| picco massimo (T6) | 7.51 N·m = 5.0× | 8.46 N·m = 5.6× |

**Lettura:** la marcia sta dentro con margine — il giunto più caricato viaggia
sotto metà del limite in ogni task, per entrambi i controllori. Quello che non
sta dentro sono i **picchi d'urto**, su meno dell'1% dei campioni. C2 ha picchi
un po' più alti di C1 ma frazione satura generalmente più bassa.

Quindi la saturazione **non ribalta il confronto**: ridistribuisce gli impatti.
La campagna resta ad attuatore ideale, dichiarandolo, con questa sezione come
verifica di fattibilità.

*(Regola su cui non transigere: se la saturazione si attiva, va messa su tutti
e tre i controllori o su nessuno. Metterla solo su C3 misurerebbe «C3 con i
motori veri contro gli altri con motori infiniti».)*

---

## 7. Cosa resta aperto

### 7.1 C3 non è misurato in formato campagna

Il pitch control esiste e funziona (bene sulla rampa; provato anche con un
carico poggiato sul robot su T4 e T6), ma **non ci sono righe di C3 in
`results/`**: non è stato passato per gli stessi sette task con lo stesso
protocollo. È il primo lavoro da fare.

### 7.2 Il pavimento di T6 è esaurito dal percorso

Il pavimento è un quadrato di 8 m e il percorso a ostacoli lo consuma tutto: un
controllore più veloce esce dal mondo. Gestito troncando le run al bordo, ma o
si allarga il pavimento o si accorcia la run — **va deciso prima di misurare
C3**, altrimenti le sue righe non sono confrontabili con le altre.

### 7.3 Un residuo non spiegato

Anche con lo stimatore allineato il flag di contatto resta acceso sull'**8%
della fase di volo**. Il candidato più ovvio — un filtro da 50 ms
nell'inizializzazione del modello — è stato **escluso**: quel filtro esiste ma
nessun blocco lo legge. Il sospetto residuo è l'accuratezza del modello inverso
durante il volo, dove la zampa accelera; non è stato misurato.

### 7.4 Limiti del banco, da dichiarare in relazione

- su percorso accidentato i sensori di forza vedono solo il pavimento e non i
  piedi sugli ostacoli (chiudono al 41–65% del peso): tutte le metriche di
  contatto di quel task sono indisponibili per costruzione
- **un parametro è stato tarato da noi**, la terna di soglie del rilevatore di
  contatto. Partiva da `[0.04 0.1 0.1]`, valore presente nel modello; misurando
  è risultato che sulla coxa il segnale non separa il contatto dal volo — il
  giunto ruota attorno all'asse verticale e il contatto è verticale, quindi gli
  trasmette pochissima coppia. La terna adottata `[0.385 0.136 0.105]` **esclude
  di fatto la coxa**: è un cambio di regola, non una ritaratura, e va dichiarato

### 7.5 Le domande da portare al professore

1. Il lavoro di diagnosi e correzione del banco — due difetti trovati, misurati
   e documentati — vale come contributo, o il progetto si valuta solo sul
   confronto fra controllori?
2. Saturazione: basta la verifica di fattibilità (§6), dichiarando la campagna
   ad attuatore ideale, o il confronto va rifatto con il limite attivo?
3. C3 va misurato su tutti e sette i task, o basta documentarlo dove agisce
   (rampa, dosso, carico)?

---

## 8. Stato operativo

| | |
|---|---|
| righe di campagna **C1** | complete, e verificate non toccate dalle correzioni (un task rifatto dà un file identico byte per byte) |
| righe di campagna **C2** | rifatte tutte dopo la correzione dello stimatore |
| righe di campagna **C3** | **assenti** — il controllore esiste, la campagna no |
| pavimento di rumore | misurato su quattro task, entrambi i controllori |
| fattibilità attuatori | misurata su tutte le celle nominali (§6) |
| documentazione | aggiornata, con le smentite marcate nel punto in cui stavano |
| repository | riordinato: 23 strumenti di indagine chiusa spostati in `archivio/` con il loro esito registrato |
