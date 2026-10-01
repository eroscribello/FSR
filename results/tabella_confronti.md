# Confronto C1 - C2 - C3

*Generata da `tabella_confronti.m` il 1/10/2026 21:50 dai CSV in `results/`. Non modificare a mano: si rigenera.*

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
| beccheggio max [deg] | 0.514 | 1.95 | 0.356 | **peggio +279%** (16.3×) | **meglio −82%** (18.1×) |
| rollio max [deg] | 0.206 | 0.28 | 0.205 | peggio +36% (n.c. 1.2×) | meglio −27% (n.c. 1.3×) |
| costo di trasporto | 0.888 | 0.865 | 1.11 | meglio −3% (n.c. 2.5×) | **peggio +28%** (27.7×) |
| energia [J] | 18.6 | 18.2 | 23.3 | meglio −2% (n.c. 2.2×) | **peggio +28%** (28.4×) |
| coppia RMS [N m] | 0.4 | 0.403 | 0.372 | peggio +1% (n.c. 0.5×) | **meglio −8%** (8.0×) |
| coppia di picco [N m] | 1.87 | 2.02 | 2.23 | peggio +8% (n.c. 2.0×) | peggio +11% (n.c. 1.8×) |
| velocita' media [m/s] | 0.135 | 0.136 | 0.135 | **meglio +1%** (6.9×) | peggio −0% (n.c. 2.3×) |
| frazione del task | 1.12 | 1.13 | 1.13 | **meglio +0%** (5.2×) | peggio −0% (n.c. 0.2×) |

## T3 - curva, +0.1 rad/s
*cella `yaw+0.100`*

| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| beccheggio max [deg] | 0.579 | 1.85 | 0.392 | **peggio +219%** (40.0×) | **meglio −79%** (45.9×) |
| rollio max [deg] | 0.224 | 0.351 | 0.21 | **peggio +57%** (4.6×) | **meglio −40%** (5.1×) |
| costo di trasporto | 0.97 | 0.951 | 1.22 | **meglio −2%** (3.6×) | **peggio +28%** (24.3×) |
| energia [J] | 30.3 | 29.5 | 37.8 | **meglio −3%** (6.4×) | **peggio +28%** (22.5×) |
| coppia RMS [N m] | 0.398 | 0.389 | 0.376 | **meglio −2%** (7.0×) | **meglio −3%** (7.3×) |
| coppia di picco [N m] | 1.91 | 2.11 | 2.43 | peggio +10% (n.c. 1.6×) | peggio +15% (n.c. 0.4×) |
| velocita' media [m/s] | 0.134 | 0.133 | 0.133 | peggio −0% (n.c. 1.3×) | peggio −0% (n.c. 2.4×) |
| frazione del task | 1.10 | 1.08 | 1.09 | peggio −1% (n.c. 2.8×) | meglio +0% (n.c. 0.5×) |
| imbardata mis./comandata | 0.219 | 0.259 | 0.244 | +0.0402 (n.d.) | −0.0157 (n.d.) |

## T4 - rampa di 8 gradi
*cella `rampa8`*

| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| corpo meno rampa [deg] | 0.106 | 0.259 | -7.54 | +0.153 (n.d.) | −7.8 (n.d.) |
| beccheggio p-p sulla rampa [deg] | 0.994 | 1.97 | 0.839 | peggio +98% (n.d.) | meglio −57% (n.d.) |
| rollio sulla rampa [deg] | 0.662 | 0.563 | 0.314 | meglio −15% (n.d.) | meglio −44% (n.d.) |
| costo di trasporto | 1.15 | 1.09 | 1.58 | meglio −5% (n.d.) | peggio +45% (n.d.) |
| energia [J] | 43.8 | 42.5 | 59.3 | meglio −3% (n.d.) | peggio +40% (n.d.) |
| velocita' in salita / in piano | 0.903 | 0.92 | 0.873 | meglio +2% (n.d.) | peggio −5% (n.d.) |
| salita riuscita | sì | sì | sì | uguale | uguale |

## T4D - dosso 8/8 gradi
*cella `dosso8-8`*

| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| beccheggio max [deg] | 8.65 | 10.4 | 1.98 | **peggio +21%** (19.8×) | **meglio −81%** (93.8×) |
| rollio max [deg] | 2.12 | 1.49 | 0.611 | **meglio −30%** (6.4×) | **meglio −59%** (7.6×) |
| costo di trasporto | 1.12 | 1.04 | 1.45 | **meglio −7%** (5.1×) | **peggio +40%** (16.8×) |
| energia [J] | 64.1 | 59.9 | 83.6 | **meglio −7%** (3.5×) | **peggio +40%** (18.8×) |
| coppia RMS [N m] | 0.391 | 0.393 | 0.376 | peggio +0% (n.c. 0.5×) | **meglio −4%** (8.4×) |
| coppia di picco [N m] | 2.58 | 3.98 | 3.12 | peggio +54% (n.c. 1.7×) | meglio −22% (n.c. 1.0×) |
| velocita' media [m/s] | 0.13 | 0.133 | 0.131 | **meglio +2%** (3.6×) | **peggio −1%** (3.0×) |
| frazione del task | 1.09 | 1.11 | 1.09 | **meglio +2%** (3.6×) | **peggio −1%** (3.0×) |

## T5 - ostacolo singolo
*cella `ost1`*

| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| beccheggio max [deg] | 7.56 | 6.41 | 1.73 | **meglio −15%** (16.4×) | **meglio −73%** (19.8×) |
| rollio max [deg] | 4.49 | 2.31 | 1.13 | meglio −49% (n.c. 2.0×) | meglio −51% (n.c. 1.1×) |
| costo di trasporto | 1.20 | 1.08 | 1.40 | **meglio −10%** (5.2×) | **peggio +30%** (17.1×) |
| energia [J] | 48.1 | 44.3 | 57.5 | **meglio −8%** (3.6×) | **peggio +30%** (17.2×) |
| coppia RMS [N m] | 0.408 | 0.403 | 0.38 | meglio −1% (n.c. 1.9×) | **meglio −6%** (9.9×) |
| coppia di picco [N m] | 4.46 | 8.33 | 4.01 | peggio +87% (n.c. 0.7×) | meglio −52% (n.c. 0.8×) |
| velocita' media [m/s] | 0.129 | 0.132 | 0.132 | **meglio +3%** (5.3×) | **peggio −0%** (3.6×) |
| frazione del task | 1.08 | 1.10 | 1.10 | **meglio +2%** (5.2×) | peggio −0% (n.c. 1.7×) |
| ostacolo superato | sì | sì | sì | uguale | uguale |

## T6 - percorso a sette ostacoli
*cella `ost1-7`*

| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| distanza [m] | 2.91 | 3.12 | 2.99 | +0.206 (n.d.) | −0.13 (n.d.) |
| percorso superato | no | no | no | uguale | uguale |
| deriva laterale max [m] | 0.0669 | 0.104 | 0.0582 | peggio +55% (n.d.) | meglio −44% (n.d.) |
| **superato, valido** | no | no | no | | |

*"Superato" vale solo con deriva laterale massima < 0.183 m: oltre, il robot puo' aver mancato per intero l'ostacolo 3 invece di passarci sopra (soglia ricavata dalla geometria in `soglia_deriva_T6.m`, non dai controllori). Le altre metriche di T6 non vanno citate: i tre controllori non incontrano gli stessi ostacoli.*

## T7 - impulso laterale
| | C1 | C2 | C3 | C1 → C2 | C2 → C3 |
|---|---:|---:|---:|---|---|
| deviazione residua, impulso 1x [mm] | 1.34 | 6.04 | 0.164 | peggio +351% (n.d.) | meglio −97% (n.d.) |
| deviazione residua, impulso 16x [mm] | 393 | 400 | 392 | peggio +2% (n.d.) | meglio −2% (n.d.) |
| recupera a 16x | no | no | no | uguale | uguale |

## Fattibilità sui motori veri
*Da `fattibilita.m`, al punto di lavoro nominale. Coppia di stallo dell'AX-12A 1.5 N·m (manuale ROBOTIS, `docs/ROBOTIS_AX-12A_emanual.pdf`); carico raccomandato per un moto stabile 1/5 dello stallo. La campagna gira ad attuatore ideale e i giunti sono attuati in posizione: la coppia è una reazione, non un ingresso. Qui si misura quanto il controllore **chiede**, non come andrebbe con i motori veri. Rumore non misurato: nessun verdetto.*

RMS = coppia efficace del giunto più caricato / limite · picco = coppia massima / limite · sopra = campioni oltre il limite

| task | RMS C1 | RMS C2 | RMS C3 | picco C1 | picco C2 | picco C3 | sopra C1 | sopra C2 | sopra C3 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| T2 | 46% | 44% | 39% | 1.2× | 1.3× | 1.5× | 0.40% | 0.45% | 0.38% |
| T3 | 46% | 42% | 39% | 1.3× | 1.4× | 1.6× | 0.50% | 0.40% | 0.43% |
| T4 | 44% | 44% | 45% | 1.4× | 2.3× | 1.8× | 0.39% | 0.49% | 0.48% |
| T4D | 42% | 42% | 39% | 1.7× | 2.7× | 2.1× | 0.45% | 0.41% | 0.49% |
| T5 | 45% | 44% | 40% | 3.0× | 5.6× | 2.7× | 0.61% | 0.50% | 0.51% |
| T6 | 42% | 43% | 42% | 5.0× | 4.9× | 2.8× | 0.91% | 0.67% | 0.81% |
| T7 | 46% | 44% | 39% | 1.2× | 1.3× | 1.5× | 0.41% | 0.45% | 0.38% |

*Lettura: la **marcia supera il carico raccomandato** dal costruttore: giunto peggiore fino al 46% dello stallo, cioè 2.3 volte il 20% indicato da ROBOTIS per un moto stabile. Resta sotto lo stallo; i **picchi d'urto** escono dallo stallo, su al massimo il 0.91% dei campioni.*
