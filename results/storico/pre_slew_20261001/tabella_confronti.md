# Confronto C1 - C2 - C3

*Generata da `tabella_confronti.m` il 1/10/2026 19:09 dai CSV in `results/`. Non modificare a mano: si rigenera.*

| simbolo | significato |
|---|---|
| **meglio −40%** (7.4×) | differenza dichiarabile: rapporto sul rumore ≥ 3 |
| peggio +5% (n.c. 1.2×) | non concludente: la differenza sta nel rumore |
| +12% (n.d.) | rumore non misurato su questo task: non si dichiara |

Confronti solo fra controllori adiacenti: **C1→C2** misura la ricerca del terreno, **C2→C3** l'anello d'assetto. Rumore = escursione su tre run con z0 ±0.2 mm, il peggiore dei due controllori.

## T2 - piano, velocita' nominale
*cella `v1.00x`*

| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| beccheggio max [deg] | 0.514 | 2.03 | 0.398 | **peggio +295%** (14.6×) | **meglio −80%** (15.7×) |
| rollio max [deg] | 0.206 | 0.349 | 0.213 | peggio +69% (n.c. 2.8×) | meglio −39% (n.c. 2.6×) |
| costo di trasporto | 0.888 | 0.869 | 1.28 | meglio −2% (n.c. 1.5×) | peggio +48% (n.c. 2.0×) |
| energia [J] | 18.6 | 18.2 | 26.7 | meglio −2% (n.c. 1.4×) | peggio +46% (n.c. 2.1×) |
| coppia RMS [N m] | 0.4 | 0.405 | 0.376 | peggio +1% (n.c. 0.9×) | **meglio −7%** (5.2×) |
| coppia di picco [N m] | 1.87 | 2.05 | 2.55 | peggio +10% (n.c. 0.5×) | peggio +24% (n.c. 1.5×) |
| velocita' media [m/s] | 0.135 | 0.136 | 0.134 | **meglio +1%** (4.6×) | peggio −1% (n.c. 0.8×) |
| frazione del task | 1.12 | 1.12 | 1.11 | meglio +0% (n.c. 1.6×) | peggio −1% (n.c. 0.7×) |

## T3 - curva, +0.1 rad/s
*cella `yaw+0.100`*

| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| beccheggio max [deg] | 0.579 | 1.85 | 0.37 | **peggio +220%** (4.9×) | **meglio −80%** (5.7×) |
| rollio max [deg] | 0.224 | 0.365 | 0.216 | **peggio +63%** (6.6×) | **meglio −41%** (6.9×) |
| costo di trasporto | 0.97 | 0.957 | 1.31 | meglio −1% (n.c. 1.0×) | **peggio +37%** (17.3×) |
| energia [J] | 30.3 | 29.6 | 40.8 | meglio −2% (n.c. 2.2×) | **peggio +38%** (24.0×) |
| coppia RMS [N m] | 0.398 | 0.39 | 0.383 | **meglio −2%** (4.0×) | **meglio −2%** (3.7×) |
| coppia di picco [N m] | 1.91 | 2.59 | 3.53 | peggio +36% (n.c. 1.4×) | peggio +36% (n.c. 1.1×) |
| velocita' media [m/s] | 0.134 | 0.133 | 0.134 | peggio −1% (n.c. 2.6×) | meglio +1% (n.c. 1.1×) |
| frazione del task | 1.10 | 1.08 | 1.09 | peggio −2% (n.c. 2.9×) | meglio +1% (n.c. 2.0×) |
| imbardata mis./comandata | 0.219 | 0.262 | 0.231 | +0.0429 (n.d.) | −0.0312 (n.d.) |

## T4 - rampa di 8 gradi
*cella `rampa8`*

| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| corpo meno rampa [deg] | 0.106 | 0.322 | -7.53 | +0.216 (n.d.) | −7.85 (n.d.) |
| beccheggio p-p sulla rampa [deg] | 0.994 | 1.97 | 0.923 | peggio +98% (n.d.) | meglio −53% (n.d.) |
| rollio sulla rampa [deg] | 0.662 | 0.694 | 0.272 | peggio +5% (n.d.) | meglio −61% (n.d.) |
| costo di trasporto | 1.15 | 1.07 | 1.68 | meglio −7% (n.d.) | peggio +56% (n.d.) |
| energia [J] | 43.8 | 41.9 | 63.0 | meglio −4% (n.d.) | peggio +50% (n.d.) |
| velocita' in salita / in piano | 0.903 | 0.924 | 0.88 | meglio +2% (n.d.) | peggio −5% (n.d.) |
| salita riuscita | sì | sì | sì | uguale | uguale |

## T4D - dosso 8/8 gradi
*cella `dosso8-8`*

| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| beccheggio max [deg] | 8.65 | 10.4 | 2.04 | **peggio +21%** (18.0×) | **meglio −80%** (84.6×) |
| rollio max [deg] | 2.12 | 1.53 | 0.611 | **meglio −28%** (4.5×) | **meglio −60%** (7.0×) |
| costo di trasporto | 1.12 | 1.04 | 1.59 | **meglio −7%** (8.5×) | **peggio +54%** (9.1×) |
| energia [J] | 64.1 | 60.1 | 91.6 | **meglio −6%** (19.9×) | **peggio +52%** (10.2×) |
| coppia RMS [N m] | 0.391 | 0.391 | 0.379 | meglio −0% (n.c. 0.1×) | **meglio −3%** (3.9×) |
| coppia di picco [N m] | 2.58 | 2.91 | 3.14 | peggio +13% (n.c. 0.2×) | peggio +8% (n.c. 0.1×) |
| velocita' media [m/s] | 0.13 | 0.133 | 0.131 | **meglio +2%** (10.3×) | peggio −2% (n.c. 2.7×) |
| frazione del task | 1.09 | 1.11 | 1.09 | **meglio +2%** (11.9×) | **peggio −1%** (3.7×) |

## T5 - ostacolo singolo
*cella `ost1`*

| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| beccheggio max [deg] | 7.56 | 6.41 | 1.93 | **meglio −15%** (10.0×) | **meglio −70%** (7.4×) |
| rollio max [deg] | 4.49 | 2.41 | 1.13 | meglio −46% (n.c. 2.3×) | meglio −53% (n.c. 1.4×) |
| costo di trasporto | 1.20 | 1.06 | 1.52 | **meglio −12%** (6.1×) | **peggio +44%** (4.2×) |
| energia [J] | 48.1 | 43.4 | 61.9 | **meglio −10%** (4.4×) | **peggio +43%** (4.5×) |
| coppia RMS [N m] | 0.408 | 0.399 | 0.382 | **meglio −2%** (3.1×) | **meglio −4%** (5.8×) |
| coppia di picco [N m] | 4.46 | 5.17 | 4.14 | peggio +16% (n.c. 0.8×) | meglio −20% (n.c. 1.1×) |
| velocita' media [m/s] | 0.129 | 0.132 | 0.131 | **meglio +3%** (5.4×) | peggio −1% (n.c. 2.0×) |
| frazione del task | 1.08 | 1.10 | 1.09 | **meglio +2%** (5.2×) | peggio −1% (n.c. 1.9×) |
| ostacolo superato | sì | sì | sì | uguale | uguale |

## T6 - percorso a sette ostacoli
*cella `ost1-7`*

| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| distanza [m] | 2.91 | 3.19 | 2.91 | +0.28 (n.d.) | −0.278 (n.d.) |
| percorso superato | no | sì | no | cambia | cambia |
| deriva laterale max [m] | 0.0669 | 0.296 | 0.0678 | peggio +343% (n.d.) | meglio −77% (n.d.) |
| **superato, valido** | no | non vale (deriva) | no | | |

*"Superato" vale solo con deriva laterale massima < 0.183 m: oltre, il robot puo' aver mancato per intero l'ostacolo 3 invece di passarci sopra (soglia ricavata dalla geometria in `soglia_deriva_T6.m`, non dai controllori). Le altre metriche di T6 non vanno citate: i tre controllori non incontrano gli stessi ostacoli.*

## T7 - impulso laterale
| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| deviazione residua, impulso 1x [mm] | 1.34 | 2.27 | 3.43 | peggio +70% (n.d.) | peggio +51% (n.d.) |
| deviazione residua, impulso 16x [mm] | 393 | 395 | 391 | peggio +0% (n.d.) | meglio −1% (n.d.) |
| recupera a 16x | no | no | no | uguale | uguale |

## Fattibilità sui motori veri
*Da `fattibilita.m`, al punto di lavoro nominale. Limite del servo 1.5 N·m (datasheet). La campagna gira ad attuatore ideale e i giunti sono attuati in posizione: la coppia è una reazione, non un ingresso. Qui si misura quanto il controllore **chiede**, non come andrebbe con i motori veri. Rumore non misurato: nessun verdetto.*

RMS = coppia efficace del giunto più caricato / limite · picco = coppia massima / limite · sopra = campioni oltre il limite

| task | RMS C1 | RMS C2 | RMS C3 | picco C1 | picco C2 | picco C3 | sopra C1 | sopra C2 | sopra C3 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| T2 | 46% | 43% | 38% | 1.2× | 1.4× | 1.7× | 0.40% | 0.40% | 0.60% |
| T3 | 46% | 42% | 40% | 1.3× | 1.7× | 2.4× | 0.50% | 0.37% | 0.66% |
| T4 | 44% | 44% | 44% | 1.4× | 2.8× | 1.6× | 0.39% | 0.50% | 0.57% |
| T4D | 42% | 41% | 39% | 1.7× | 1.9× | 2.1× | 0.45% | 0.37% | 0.60% |
| T5 | 45% | 43% | 39% | 3.0× | 3.4× | 2.8× | 0.61% | 0.45% | 0.63% |
| T6 | 42% | 42% | 42% | 5.0× | 5.6× | 4.0× | 0.91% | 0.70% | 0.81% |
| T7 | 46% | 44% | 38% | 1.2× | 1.4× | 1.7× | 0.41% | 0.39% | 0.61% |

*Lettura: la **marcia sta dentro** per tutti e tre (giunto peggiore al massimo al 46%, sotto la soglia del 70% di `fattibilita.m`); i **picchi d'urto** escono dal limite, su al massimo il 0.91% dei campioni.*
