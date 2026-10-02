# Stato del progetto — confronto fra controllori sul PhantomX in Simscape

*Aggiornato al 1 ottobre. Documento di sintesi: cosa è stato fatto, cosa si
può dichiarare, cosa resta aperto. Le misure e i dettagli stanno in
`docs/piano_confronto.md`; la tabella completa C1–C2–C3, generata dai CSV, in
`results/tabella_confronti.md` (`tabella_confronti.m`).*

---

## 1. L'obiettivo

Confrontare su simulatore tre controllori per l'esapode PhantomX su sette task
(piano, curva, rampa, dosso, ostacolo singolo, percorso a ostacoli, disturbo
impulsivo):

| | |
|---|---|
| **C1** | cinematico ad anello aperto, andatura a tripode, giunti comandati in posizione |
| **C2** | Arrigoni et al. (*Robotics* 2024, 13, 142): C1 più la **ricerca del terreno** — il piede continua a scendere finché la coppia ai giunti non segnala il contatto |
| **C3** | C2 più una retroazione sull'assetto del corpo — beccheggio (PI, P = 10, I = 1) e, dall'1/10, rollio (PI, P = 5, I = 0.7) — in serie dopo la ricerca del terreno, sullo stesso impianto degli altri due |

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

### 4.0 La campagna corrente: tutti e tre con lo slew rate *(1/10 sera)*

**Cosa è cambiato nell'impianto.** Un Rate Limiter di **±0.4 m/s** sul comando
di quota di ogni piede, salvato in `phantomx_sim_zero` (C1, C2: prima non
c'era) e portato da ±0.3 a ±0.4 in `phantomx_sim_attitude` (C3). Deciso con il
collega dopo due prove su T6 (§4, T6). Con ±0.4 l'andatura nominale **non viene
toccata** (picco verticale del piede 0.375 m/s); il limitatore agisce solo sui
comandi rapidi: ricerca del terreno e correzioni d'assetto, e sulle celle di T2
da 1.2× in su. **Verificato: le righe e il pavimento di rumore di C1 sono
identici byte per byte a quelli senza limitatore** su T3, T4, T4D, T5, T7 e
sulla cella nominale di T2.

**Conseguenza sul metodo.** Ora C2 → C3 differisce per **un solo meccanismo**,
l'anello d'assetto (la saturazione di C3, da sola su C2, non interverrebbe).
I verdetti C2 → C3 qui sotto si attribuiscono all'anello.

Tutto rifatto: campagna 7 task × 3, rumore 4 task × 3 run × 3, fattibilità.
Tabella completa per task in `results/tabella_confronti.md`. Le righe di prima
sono in `results/storico/pre_slew_20261001/`.

| rapporto sul rumore | T2 piano | T3 curva | T4D dosso | T5 ostacolo |
|---|---|---|---|---|
| **C1 → C2, ricerca del terreno** | | | | |
| beccheggio max | **+279%** (16.3) | **+219%** (40.1) | **+21%** (19.8) | **−15%** (16.4) |
| rollio max | n.c. | **+57%** (4.7) | **−30%** (6.4) | n.c. |
| costo di trasporto | n.c. | **−2%** (3.7) | **−7%** (5.1) | **−10%** (5.2) |
| energia | n.c. | **−3%** (6.4) | **−7%** (3.5) | **−8%** (3.6) |
| velocità, frazione del task | **+1%** (5–7) | n.c. | **+2%** (3.6) | **+2/+3%** (5.2–5.3) |
| **C2 → C3, anello d'assetto** | | | | |
| beccheggio max | **−82%** (18.1) | **−79%** (46.0) | **−81%** (93.8) | **−73%** (19.8) |
| rollio max | n.c. | **−40%** (5.2) | **−59%** (7.6) | n.c. |
| costo di trasporto | **+28%** (27.7) | **+28%** (24.3) | **+40%** (16.8) | **+30%** (17.1) |
| energia | **+28%** (28.4) | **+28%** (22.5) | **+40%** (18.8) | **+30%** (17.2) |
| coppia RMS | **−8%** (8.0) | **−3%** (7.4) | **−4%** (8.4) | **−6%** (9.9) |
| deriva, imbardata, coppia di picco | n.c. | n.c. | n.c. | n.c. |

**Lettura.** Il quadro C1 → C2 di prima regge, con rapporti in genere più alti:
la ricerca del terreno **costa beccheggio** dove non c'è niente da cercare
(+220/+280% in piano e in curva) e **ripaga** su dosso e ostacolo (rollio,
energia, completamento). L'anello d'assetto restituisce quell'assetto — il
beccheggio scende di quasi cinque volte su **tutti** e quattro i task — e lo
paga con il **28–40% di energia**, ora dichiarabile anche in piano.

**La sensibilità anomala dell'energia di C3 non c'è più.** Nel terzo giro
(slew ±0.3) l'energia di C3 era 7–17× più sensibile di quella di C2; adesso il
rumore di C3 sul costo di trasporto in piano vale 0.007 contro 0.009 di C2.
Previsione scritta prima del quarto giro: *"se no, era l'interazione col
limite a 0.3"*. È così: §7.6 chiuso su questo punto.

**T6, con il limitatore** — solo esito binario e validità (soglia 0.183 m):

| T6 | C1 | C2 | C3 |
|---|---|---|---|
| deriva laterale max | 0.067 m | 0.104 m | 0.058 m |
| piede oltre il bordo | no | no | no |
| superato, valido | no | no | no |

La deriva di C2 (0.296 m senza limitatore) **sparisce**: veniva dalla
velocità con cui la ricerca del terreno muoveva i piedi in verticale. Sullo
stesso percorso e con lo stesso impianto **nessuno dei tre supera T6**: sul
percorso a ostacoli il confronto non distingue i controllori.

---

*[SUPERATO 1/10 sera] Da qui alla fine della §4: la campagna **senza** slew
rate su C1 e C2 (e con slew ±0.3 su C3). I numeri valevano per quell'impianto
e restano qui come storia, con i loro [RITIRATO]; righe in
`results/storico/pre_slew_20261001/`. Per i numeri correnti vale la §4.0.*

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

**[RITIRATO 1/10]** *La tabella e la conclusione qui sotto non reggono. C2
"arriva in fondo" perché deriva di lato e passa **di fianco** agli ostacoli
finali, non sopra: nell'animazione le zampe destre non toccano l'ultimo
ostacolo. Era già misurato il 26/9 (`asimmetria_T6.m`: "C2 trasla di 24 cm e
manca gli ostacoli locali") e la riga di C2 lo mostrava — deviazione laterale
massima 0.237 m contro 0.120 di C1 — ma l'esito binario è stato letto lo
stesso come un risultato. I numeri sono inoltre superati: il 1/10 gli
ostacoli 2 e 4 sono stati spostati di ~6.5 cm (commit "Gradini spostati") e
le righe di C1 e C2 rifatte.*

| T6 *(ritirato)* | C1 | C2 |
|---|---|---|
| distanza percorsa | **2.44 m** | 3.71 m |
| frazione del task | **81%** | 107% |
| arriva agli ostacoli finali | **no** | sì |

~~1.3 metri di differenza, contro un rumore di 0.03–0.10 m.~~ *Le metriche fini di
T6* (rollio, beccheggio, costo) **non vanno citate**: i due controllori seguono
traiettorie diverse e quindi non incontrano gli stessi ostacoli. Il confronto
fine lo danno T5 e T4D.

**[1/10] Come si legge T6 adesso.** Nemmeno l'esito binario basta da solo:
"superato" vale solo se la deriva laterale massima resta sotto **0.183 m**,
la deriva che basta a mancare per intero gli ostacoli 3–4 (metà destra della
pista). La soglia è ricavata dalla geometria — mesh e posa nominale dei piedi,
`soglia_deriva_T6.m` — e non dai valori dei controllori.

| T6, ostacoli del 1/10 | C1 | C2 | C3 |
|---|---|---|---|
| deriva laterale massima | 0.067 m | **0.296 m** | 0.068 m |
| piede oltre il bordo (`t_bordo`) | no | no | no |
| superato | no | sì | no |
| **superato, valido** | no | **non vale** (deriva) | no |

Nessuno dei tre supera il percorso passando sopra gli ostacoli. C2 deriva
**solo** dove il terreno è asimmetrico fra destra e sinistra (0.002 m in piano,
0.021 su T5, 0.084 su T4D, 0.296 su T6): l'ipotesi è che la ricerca del
terreno, allungando le zampe del lato basso, trasformi parte della spinta in
spinta laterale. **Non misurato** — vedi §7.6.

### Sul disturbo impulsivo, nessuno dei tre funziona

Impulso laterale crescente, dal 1% al 1600% della quantità di moto nominale:
**né C1 né C2 recuperano una frazione misurabile della deviazione**. A impulso
grande le due curve coincidono (394 e 395 mm di deviazione residua). Il
rilevamento del contatto non c'entra: è un buco di architettura comune.
*[1/10]* Lo stesso per **C3** (391 mm): l'anello d'assetto regola beccheggio e
rollio, non la posizione laterale, e non poteva cambiarlo. Su T7 il rumore non
è misurato, quindi nessuna differenza fine si dichiara.

### L'anello d'assetto (C2 → C3): compra assetto, lo paga in energia

*[1/10] Campagna C3 sui sette task e pavimento di rumore di C3 su quattro
(`rumore_metriche`, terzo giro, `results/diagnostica/rumore_C3.csv`). Il
verdetto usa il rumore peggiore fra C2 e C3, ciascuno misurato su sé stesso.*

**[1/10] Cosa misura davvero C2 → C3.** Il modello di C3 ha, oltre ai due PI
d'assetto, una `Saturation` (limite inferiore 0.07) e un `Rate Limiter`
(±0.3 m/s) su ogni piede, che nel modello di C1/C2 **non ci sono**. Esistono
per l'anello — la saturazione perché l'uscita dei PID non ha limiti, lo slew
rate per togliere il tremolio — ma agiscono anche da soli: il Rate Limiter
limita la velocità verticale di **ogni** comando di quota, compreso quello
della ricerca del terreno. Le differenze qui sotto sono quindi dell'**anello
d'assetto con i suoi limitatori**; quanta parte venga dai soli limitatori non
è misurato. La prova è C2 + Rate Limiter, senza anello. *La saturazione da sola
su C2 non interverrebbe mai: la quota di C2 sta fra 0.09 m (z0 − alzata del
passo) e 0.17 m (z0 + estensione massima della ricerca), sopra il limite di
0.07. Diventa attiva solo quando l'anello somma `d_z`.*

| C2 → C3 (rapporto sul rumore) | T2 piano | T3 curva | T4D dosso | T5 ostacolo |
|---|---|---|---|---|
| beccheggio massimo | **−80%** (15.8) | **−80%** (5.7) | **−80%** (84.7) | **−70%** (7.4) |
| rollio massimo | n.c. | **−41%** (7.0) | **−60%** (7.1) | n.c. |
| costo di trasporto | n.c. (+48%) | **+37%** (17.4) | **+54%** (9.1) | **+44%** (4.3) |
| energia | n.c. (+46%) | **+38%** (24.0) | **+52%** (10.2) | **+43%** (4.5) |
| deriva, imbardata, coppia di picco | n.c. | n.c. | n.c. | n.c. |

Su T4 (rampa) il rumore non è misurato, ma il comportamento è qualitativo e
netto: C3 tiene il corpo **orizzontale** (0.6° sulla rampa di 8°, contro 8.2°
di C1 e 9.6° di C2), che è quello per cui è costruito — riferimento 0.

**Una sensibilità non prevista.** Il criterio scritto prima del terzo giro
prevedeva C3 *meno* sensibile di C2 sull'assetto, e così è in 7 casi su 8
(eccezione: beccheggio su T5). Ma **l'energia di C3 è 7–17 volte più sensibile
di quella di C2** a una perturbazione di 0.2 mm: sul piano il rumore del costo
di trasporto vale il 16% del valore, ed è per questo che lì il +48% non si
può dichiarare. Non spiegato — vedi §7.6.

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
> energia e completamento del task; ~~su un percorso a ostacoli è la differenza
> fra arrivare in fondo e non arrivarci.~~ **[RITIRATO 1/10]** *su T6 C2 arriva
> in fondo passando di fianco agli ostacoli, non sopra: vedi §4.*

È la tesi di Arrigoni et al., misurata, **con il prezzo esplicito** — che
l'articolo non quantifica. È più difendibile di «il nostro C2 è migliore
ovunque», che non sarebbe vera.

*[1/10]* Con C3 la tesi si completa con un secondo scambio, simmetrico al primo.

**[RITIRATO 1/10, stessa sera]** *La prima stesura attribuiva tutto
all'«anello d'assetto». Ma C3 aggiunge a C2 anche saturazione e limite di
velocità dei piedi (§4), che C2 non ha: l'effetto è dell'anello **con** i suoi
limitatori, finché C2 + limitatori non è misurato da solo. La frase resta qui
sotto, con il soggetto corretto.*

> **~~L'anello d'assetto~~ C3 — anello d'assetto con i suoi limitatori —
> restituisce l'assetto che la ricerca del terreno toglie, e lo paga in
> energia.** Su piano, curva, dosso e ostacolo C3 riduce il
> beccheggio massimo del 70–80% rispetto a C2 (numericamente anche sotto C1:
> confronto non adiacente, non verificato sul rumore) e il rollio
> fino al 60%; dove il costo è leggibile, spende il 37–54% di energia in più.
> Non cambia né la deriva né la risposta a un urto laterale.

Nessuno dei tre controllori è migliore ovunque: ognuno compra qualcosa e lo
paga in un'altra moneta, e il lavoro è averle misurate tutte e due.

### La tesi con la campagna corrente *(1/10 sera, tutti con slew rate)*

Con il limitatore su tutti e tre, C2 → C3 torna a essere un meccanismo solo, e
la frase si può scrivere con il suo soggetto vero:

> **La ricerca del terreno costa assetto dove non c'è niente da cercare e lo
> ripaga sulle discontinuità; l'anello d'assetto restituisce quell'assetto e
> lo paga in energia.** C2 peggiora il beccheggio di 3–4 volte in piano e in
> curva, e su dosso e ostacolo migliora rollio, energia (−7/−8%) e
> completamento. C3 riduce il beccheggio di C2 del 73–82% su tutti e quattro i
> task misurati, e spende il 28–40% di energia in più. Nessuno dei due cambia
> la deriva, la risposta a un urto laterale, né l'esito sul percorso a
> ostacoli, che nessuno dei tre supera.

E una cosa che non era nel programma: **lo slew rate sui comandi di quota** è
quello che ha tolto la deriva laterale di C2 su T6 (da 0.296 a 0.104 m), e la
sensibilità anomala dell'energia di C3. Un dettaglio d'implementazione, non un
controllore, con effetti più grandi di alcune differenze fra controllori.

---

## 6. Fattibilità con attuatori reali

### 6.0 La fonte del limite, e il limite che conta *(1/10 sera)*

Il servo del PhantomX AX Mark II è il **Dynamixel AX-12A** (Arrigoni et al.,
pag. 4, rif. [40]). Il manuale ROBOTIS, ora in
`docs/ROBOTIS_AX-12A_emanual.pdf`, pag. 2–3:

| | |
|---|---|
| stall torque | **1.5 N·m** a 12 V, 1.5 A — è il `τ_max` usato finora. *Fino all'1/10 nel repo non c'era la fonte: il valore era stato dato a memoria. Ora è verificato* |
| nota del costruttore | *"Stall torque is the maximum instantaneous and static torque. Stable motions are possible with robots designed for loads with **1/5 or less** of the stall torque."* → **0.3 N·m** (`cfg.tau_lavoro`) |
| tensione raccomandata | 11.1 V (a 11.1 V lo stallo è ~1.4 N·m) |

Il carico **sostenuto** va confrontato con 0.3 N·m, non con lo stallo; lo
stallo resta il riferimento per i picchi. La soglia del 70% con cui
`fattibilita.m` diceva "la marcia sta dentro" era nostra e non giustificata.

**Campagna corrente, al punto nominale:**

| | C1 | C2 | C3 |
|---|---|---|---|
| giunto peggiore, RMS / stallo | 42–46% | 42–44% | 39–45% |
| giunto peggiore, RMS / **carico raccomandato** | **2.1–2.3×** | **2.1–2.2×** | **1.9–2.3×** |
| picco / stallo | 1.2–5.0× | 1.3–5.6× | 1.5–2.8× |
| campioni oltre lo stallo | 0.39–0.91% | 0.40–0.67% | 0.38–0.81% |

**Lettura:** la marcia resta **sotto lo stallo**, ma il giunto più caricato
lavora a **circa il doppio del carico che il costruttore indica per un moto
stabile**, per tutti e tre i controllori. Un servo vero non si fermerebbe, ma
lavorerebbe fuori dalla zona raccomandata e scalderebbe. È un limite del
robot simulato (massa, geometria, andatura), non di un controllore: nessuno
dei tre se ne discosta. Con lo slew rate C3 ha i picchi più bassi dei tre
(2.8× al massimo, contro 5.0 e 5.6).

*[SUPERATO 1/10 sera] La tabella e la lettura qui sotto sono della campagna
senza slew rate, e giudicavano col 70%.*

| | C1 | C2 | C3 *(1/10)* |
|---|---|---|---|
| coppia RMS del giunto peggiore / limite | **42–46%** | **41–44%** | **38–44%** |
| frazione di campioni oltre il limite | 0.4–0.9% | 0.4–0.7% | 0.6–0.8% |
| picco massimo (T6) | 7.51 N·m = 5.0× | 8.46 N·m = 5.6× | 6.01 N·m = 4.0× |

*[1/10] Colonna C1 aggiornata dopo il rifacimento di T6 (era 41–46%, 0.4–1.0%):
cambia solo T6.*

**Lettura:** ~~la marcia sta dentro con margine~~ **[RITIRATO 1/10 sera]**
*il "margine" era misurato contro lo stallo con una soglia nostra del 70%; il
costruttore indica 1/5 dello stallo per un moto stabile, e il giunto più
caricato sta a ~2 volte quel valore (§6.0).* — il giunto più caricato viaggia
sotto metà dello stallo in ogni task, per tutti e tre i controllori. Quello che
non sta dentro sono i **picchi d'urto**, su meno dell'1% dei campioni. C2 ha
picchi un po' più alti di C1 ma frazione satura generalmente più bassa.
*[1/10]* C3, malgrado il 37–54% di energia in più, ha il carico sostenuto **più
basso** dei tre e il picco più basso su T5 e T6. In compenso passa più tempo
sopra il limite di C2 in tutti e sette i task: picchi meno alti, ma più
frequenti. Previsione scritta in `fattibilita.m` prima di lanciare:
confermata. Tabella per task in `results/tabella_confronti.md`.

Quindi la saturazione **non ribalta il confronto**: ridistribuisce gli impatti.
La campagna resta ad attuatore ideale, dichiarandolo, con questa sezione come
verifica di fattibilità.

*(Regola su cui non transigere: se la saturazione si attiva, va messa su tutti
e tre i controllori o su nessuno. Metterla solo su C3 misurerebbe «C3 con i
motori veri contro gli altri con motori infiniti».)*

---

## 7. Cosa resta aperto

### 7.1 C3 non è misurato in formato campagna — **[CHIUSO 1/10]**

~~Il pitch control esiste e funziona, ma non ci sono righe di C3 in
`results/`.~~ Campagna C3 fatta sui sette task, con il carico tolto
(`commenta_carico`), e pavimento di rumore misurato su quattro (§4). Prima di
lanciarla sono stati corretti due difetti che avrebbero falsato C3 **senza
dare errori**:

- `script_T3` passava l'imbardata al modello di C1/C2 (default di
  `applica_imbardata`): il robot di C3 andava dritto;
- `rumore_metriche` avrebbe acceso la ricerca del terreno solo per `'C2'`,
  cioè misurato **C3 senza C2** — un altro controllore.

### 7.2 Il pavimento di T6 è esaurito dal percorso — **[CHIUSO 1/10]**

Il pavimento resta 8 × 8 m. Con gli ostacoli attuali e 25 s di run **nessuno
dei tre controllori raggiunge il bordo** (`t_bordo` = NaN per C1, C2 e C3). Il
taglio al bordo in `script_T6` è stato disattivato dal collega: il bordo viene
ancora registrato, e se un giorno un piede lo superasse la riga lo direbbe.

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
- *[1/10 sera]* **un limitatore aggiunto da noi a tutti e tre**: Rate Limiter
  ±0.4 m/s sul comando di quota dei piedi. Non c'è in Arrigoni et al.; non
  tocca l'andatura nominale ma limita la ricerca del terreno, e senza di lui
  C2 deriva su T6. Va dichiarato come parte dell'impianto comune
- *[1/10 sera]* **il robot simulato lavora a ~2 volte il carico raccomandato**
  dal costruttore del servo sul giunto più caricato (§6.0), per tutti e tre

### 7.5 Le domande da portare al professore

1. Il lavoro di diagnosi e correzione del banco — due difetti trovati, misurati
   e documentati — vale come contributo, o il progetto si valuta solo sul
   confronto fra controllori?
2. Saturazione: basta la verifica di fattibilità (§6), dichiarando la campagna
   ad attuatore ideale, o il confronto va rifatto con il limite attivo?
3. C3 va misurato su tutti e sette i task, o basta documentarlo dove agisce
   (rampa, dosso, carico)? *[1/10] Misurato su tutti e sette: la domanda
   resta solo per sapere dove concentrare la relazione.*
4. *[1/10]* Su T6 la soglia di validità sulla deriva (0.183 m, dalla
   geometria) è un criterio aggiunto da noi dopo aver visto l'animazione. Va
   bene dichiararlo così, o T6 va tolto dal confronto?

### 7.6 Aperti dal 1/10

- **Perché C2 deriva su T6.** — **[CHIUSO 1/10 sera]** Ipotesi in §4 (spinta
  laterale dalle zampe allungate sul lato basso): incompleta. Limitando la
  velocità verticale dei comandi di quota a ±0.4 m/s la deriva scende da 0.296
  a 0.104 m. Conta la **velocità** con cui la ricerca del terreno muove i
  piedi, non solo l'asimmetria del terreno. Lo script passo per passo non è
  più necessario per la decisione; resterebbe solo per descrivere il
  meccanismo.
- **Perché l'energia di C3 è così sensibile.** — **[CHIUSO 1/10 sera]** Non lo
  è più con il limitatore a ±0.4 (rumore del costo in piano 0.007 contro 0.009
  di C2): la sensibilità del terzo giro veniva dal limite a ±0.3, che tagliava
  anche il volo nominale.
- **Quanto dell'effetto C2 → C3 viene dai limitatori.** — **[CHIUSO 1/10
  sera]** Il Rate Limiter è ora in tutti e tre i modelli, uguale: C2 → C3
  misura il solo anello d'assetto (§4.0).
- *[1/10 sera]* **La cella v0.50x di T2 su C1 cambia di poco** con il
  limitatore (`frazione_task` 0.894 → 0.896), anche se a quella velocità il
  piede è più lento del limite. Candidato: l'alzata tarata a parte per quella
  cella (`cfg.t2_taratura`). Non misurato, effetto trascurabile.
- *[1/10 sera]* **`script_T4_limite` non è stato rifatto** con il
  limitatore: le sue righe sono in archivio, pre-slew.
- **Il beccheggio di C3 su T5** è l'unico caso in cui C3 è più sensibile di C2
  sull'assetto (rumore 5×). Il verdetto regge lo stesso (rapporto 7.4), ma è
  l'eccezione alla previsione.

---

## 8. Stato operativo

| | |
|---|---|
| impianto | **dal 1/10 sera, Rate Limiter ±0.4 m/s per piede in entrambi i modelli**, salvato nei `.slx` (verificato sull'XML: cambiano solo i limitatori, più le inerzie di `phantomx_sim_zero` salvate già corrette) |
| campagna | **7 task × 3 controllori**, tutta rifatta con il limitatore il 1/10 sera. Quella di prima in `results/storico/pre_slew_20261001/` |
| righe di **C1** | identiche byte per byte a quelle senza limitatore, salvo le celle veloci di T2 (e v0.50x, di pochissimo): il limitatore non tocca l'andatura nominale |
| pavimento di rumore | **quarto giro**, 36 run, tutti e tre i controllori: `results/diagnostica/rumore_slew.csv`. Quello di C1 è identico al precedente |
| non rifatto | `script_T4_limite` (righe pre-slew in archivio) |
| tabella dei confronti | generata dai CSV, `results/tabella_confronti.md`: si rigenera con `tabella_confronti`, non si modifica a mano |
| fattibilità attuatori | misurata su tutte le celle nominali, tutti e tre i controllori (§6) |
| documentazione | aggiornata, con le smentite marcate nel punto in cui stavano |
| repository | riordinato: 23 strumenti di indagine chiusa spostati in `archivio/` con il loro esito registrato |
