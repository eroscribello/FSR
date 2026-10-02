# L'andatura: come è generata e da dove vengono i parametri

*Scritto il 2/10, dopo il ricevimento: il professore ha chiesto come abbiamo
ricavato l'andatura e con quale generatore. Questo documento raccoglie la
risposta. La provenienza dei singoli numeri è scritta anche nei commenti di
`common/phantomx_config.m` (sezione "andatura a tripode"); quando i due
documenti divergono, fa fede il codice e va corretto questo documento.*

---

## 1. Cosa fa il paper, e cosa facciamo noi

| | Arrigoni et al. | noi |
|---|---|---|
| generatore | **gait engine di NUKE**, non descritto: *"The gait engine employed is not fundamental to describe the desired formulation and will not be discussed […] (the gait engine employed in this treatment is the one provided by NUKE)"* (pp. 6–7) | **tripode fisso**: `common/tripod_trajectory.m`, una traiettoria del piede in forma chiusa |
| ingresso | velocità x, velocità y, velocità di imbardata (p. 7) | parametri d'andatura `T, S, H, z0, duty`; la curva di T3 passa da `applica_imbardata` |
| andature | tutte quelle staticamente stabili; il tripode è *"the less-redundant one"* (p. 11) | solo il tripode |
| stabilità | **imposta a priori** dal generatore: *"the only gait types accepted by the algorithm are those that allow the hexapod robot to maintain static stability in all intermediate positions during movement […] the algorithm always ensures that the support triangle bounds the COG projection"* (p. 11) | **non imposta**: va verificata a posteriori, con lo ZMP (§5) |

Visto che il paper non dà la sua andatura, i parametri li abbiamo dovuti
scegliere noi: §3 dice da dove vengono. Il confronto fra controllori non ne
risente, perché la traiettoria dei piedi è **la stessa per C1, C2 e C3**. I
controllori intervengono tutti *dopo* il generatore (§2).

---

## 2. Dove sta il generatore nella catena del comando

Verificato sull'XML di entrambi i `.slx` (2/10): ognuno ha **due** MATLAB
Function che chiamano `tripod_trajectory(t_in, T, S, H, z0, duty)`, una per
tripode. Il secondo tripode riceve `t + T/2`.

```
tripod_trajectory  ->  ricerca_terreno  ->  (+ d_z  solo C3)  ->  Rate Limiter ±0.4 m/s  ->  inv_kyn  ->  giunti
  (x, y, z) comune       z per zampa         anello d'assetto       comune ai tre             per zampa
  alle 3 zampe           (C2, C3)
  del tripode
```

| tripode | zampe (ordine CAN `FL FR ML MR RL RR`) | fase |
|---|---|---|
| A | FL, MR, RL | 0 |
| B | FR, ML, RR | T/2 |

È `cfg.phase = [0; ½; ½; 0; 0; ½]`. Il generatore produce **una** posizione
del piede per tripode, nel frame corpo. Ogni zampa la porta nel proprio frame
dentro `inv_kyn` (con `alpha` e `side` della zampa). Un vettore di passo
diverso per ogni zampa non si può comandare. Per curvare in T3,
`applica_imbardata` ruota l'`alpha` usato da `inv_kyn` zampa per zampa, senza
toccare lo schema.

---

## 3. La traiettoria del piede

Coordinate nel frame corpo, `z` **positiva verso il basso** (convenzione di
`inv_kyn`). Con `n` = fase normalizzata in [0, 1]:

| fase | x | z |
|---|---|---|
| **appoggio** (`T_stance`) | `S/2 − S·n`, velocità costante `−S/T_stance` | `z0`, piede fermo |
| **volo** (`T_swing`) | `−S/2 + v0·n + (S − v0)·s(n)`, con `v0 = −S·T_swing/T_stance` | `z0 − H·s(u)`, con `u = 2n` in salita e `u = 2(1−n)` in discesa |

`s(u) = 10u³ − 15u⁴ + 6u⁵` è lo smoothstep quintico.

**Perché quintico.** Il profilo ha posizione, velocità e accelerazione
continue in tutto il ciclo, anche allo stacco e all'atterraggio. I giunti
sono comandati in posizione e Simulink deriva due volte il comando per
passarlo a Simscape: un salto di accelerazione diventerebbe un impulso di
coppia. Con `H·sin²(πn)` il salto di accelerazione allo stacco valeva
0.93 m/s²; con il quintico vale 0.03 m/s² (commento in `tripod_trajectory.m`).

**Raccordo orizzontale.** All'inizio e alla fine del volo la velocità in x
vale `v0/T_swing = −S/T_stance`, cioè quella dell'appoggio: il piede passa
dall'appoggio al volo senza scatto.

**Picco verticale del piede.** `ds/du` vale al massimo 30/16 = 1.875, quindi
`|ż|max = H · 1.875 · 2 / T_swing = 0.05 · 3.75 / 0.5 =` **0.375 m/s**. Il Rate
Limiter a ±0.4 m/s sta sopra questo valore, ed è per questo che **non tocca
l'andatura nominale** (CLAUDE.md, "C3, com'è fatto davvero"). Nelle celle
veloci di T2 invece interviene: `T_swing` si accorcia e il picco sale oltre
0.4 m/s.

---

## 4. I parametri e da dove vengono

| parametro | valore | tipo | da dove viene |
|---|---|---|---|
| `phase` | `[0 ½ ½ 0 0 ½]` | definizione | è il tripode: due triangoli alternati |
| `beta_stance` | 0.50 | definizione | è l'unico valore con appoggio continuo su **esattamente** tre zampe. Sotto 0.5 ci sono istanti con meno di tre piedi a terra (fuori dalla stabilità statica su cui si regge il paper); sopra compaiono finestre a sei piedi, cioè 18 vincoli di posizione per 6 gradi di libertà, e i due tripodi si contendono il moto attraverso la cedevolezza del contatto |
| `S` | 0.060 m | vincolo geometrico | a metà passo la gamba non deve superare l'**85% dell'estensione massima** `lf + lt` (assert `phantomx:config:reach`; *"a 89% il robot camminava all'indietro"*). Con S = 0.06 si arriva all'80.9%; l'assert scatta a S ≈ 0.095. Tabella completa in `phantomx_config.m` |
| `H` | 0.050 m | vincolo di franco | deve superare il **dislivello più alto che il piede incontra in un passo**: 32.8–34.0 mm su T5 (`alt_ost` misurato da `script_T5`, tutti e tre i controllori), ~35 mm per gradino su T6 (§6.2). `H` vale circa 1.5 volte quel dislivello. Su terreno piano l'ottimo di `H` è degenere: tende a zero, cioè a piedi che strisciano (`piano_confronto.md` §"H ha un ottimo degenere") |
| `T` | 1.00 s | dipendente | con S e β fissati, T fissa la velocità: `v_nom = S / (β·T) = 0.120 m/s` |
| `z0` | 0.140 m | tarato | profondità d'appoggio sotto l'anca, insieme a `r_offset = 0.14 m` (marcati `[TARATO]` in config) |

### Il solo grado di libertà è la velocità

```
T = S / (beta_stance · v_nom)
```

`beta_stance` è fissato dalla definizione, `S` dalla geometria e `H` dal
franco. L'unica scelta che resta è `v_nom`, e da lì discende `T`. È anche
il modo in cui T2 percorre la curva di velocità: varia `v`, e lo script
ricava `T` dalla formula (README, `piano_confronto.md` §T2).

`v_nom` è stata verificata **a posteriori** con `archivio/limite_velocita.m`.
Il criterio, dichiarato prima di lanciare, era un rimbalzo del corpo sotto
10 mm; l'esito è in `phantomx_config.m`:

| fattore | rimbalzo del corpo |
|---|---|
| 1.00 | 3.2 mm ← nominale |
| 1.20 | 3.4 mm ← ultima cella con moto stabile |
| 1.30 | 9.4 mm ← al bordo |
| 1.40 | 19.7 mm ← cede |

Il nominale sta all'83% del limite misurato. Il numero di Froude
`v²/(g·h) = 0.120² / (9.81 · 0.154) ≈ 0.0095` colloca la marcia in regime
quasi-statico: le forze d'inerzia valgono circa l'1% della gravità. Questa
ipotesi si controlla direttamente con lo ZMP (§5).

---

## 5. La garanzia di stabilità che il nostro generatore non dà

Il gait engine del paper scarta in anticipo le andature in cui il baricentro
uscirebbe dal triangolo d'appoggio. Il nostro tripode fisso non fa nessun
controllo: la stabilità dipende solo dal fatto che geometria e parametri
siano scelti bene. Ma anche la garanzia del paper vale solo sulla geometria
**nominale**: su rampa e ostacoli i piedi non stanno dove la cinematica li
mette.

Per questo la stabilità va misurata **a posteriori**, sullo stato effettivo
di ogni task (ZMP sul poligono d'appoggio). *[2/10] Lo ZMP per task lo sta
facendo il collega: questa sezione va completata con il suo lavoro.*

---

## 6. Punti aperti

### 6.1 L'ostacolo di T5: vale la misura dello script, non il 45 mm dei documenti

**Fonte: `script_T5`**, colonna `alt_ost` di `results/T5_C*.csv`. È la
mediana della quota dei piedi in appoggio sull'ostacolo, rispetto al piano:

| | C1 | C2 | C3 |
|---|---:|---:|---:|
| `alt_ost` [mm] | 32.8 | 33.1 | 34.0 |
| superato | sì | sì | sì |

Il **45 mm** di `piano_confronto.md` (tabella del franco), ripreso in
`commit_msg_misure.txt` e `archivio/taratura_T2.m`, non ha una provenienza
scritta. Con quel numero la tabella chiedeva un franco di ~55 mm e poi
adottava `H` = 50 mm, meno del requisito appena scritto. In
`piano_confronto.md` la riga è marcata `[RITIRATO 2/10]`. Con il valore
misurato il conto torna: 50 mm contro ~43 mm (32.8 + 4.3 di penetrazione +
~8.6 di oscillazione del corpo, le altre due righe di quella tabella).

### 6.2 T6: `H` basta, perché i gradini si salgono uno alla volta

~~`H` non basta per T6: il terzo gradino è di 106 mm contro 50 mm di alzata.~~
**[RITIRATO 2/10]** *Il confronto giusto non è con la quota assoluta del
gradino ma con il **dislivello da superare in un passo**. Il robot arriva al
terzo gradino stando già sul secondo (Andrea, 2/10).*

| gradino | quota | dislivello dal precedente |
|---|---:|---:|
| 1 | 35 mm | 35 mm |
| 2 | 70 mm | 35 mm |
| 3 | 106 mm | 36 mm |

Quote da `piano_confronto.md`, "Il percorso di T6, misurato dalle mesh".
Ogni dislivello è dell'ordine di T5, ~35 mm, quindi **lo stesso vincolo di
franco copre T5 e T6**, e `H` = 50 mm ne vale circa 1.5 volte. *Le quote dei
gradini stanno in un `.md` e non in uno script: se servono in relazione, vanno
rimisurate dalle mesh o dai piedi in appoggio su T6.*

### 6.3 La data nel commento della config — **[CHIUSO 2/10]**

Il commento in `phantomx_config.m` diceva *"scritto il 3/10"*, ma il file è
stato modificato il 2/10. Corretto.
