# `results/` — che cosa c'è dentro

Aggiornato il 2/10/2026. *La versione del 23/9 elencava file che non c'erano
più (`T4_limite_C*.csv`, `diagnostica/prova_inerzie*.csv`) e non conosceva C3,
il pavimento di rumore e la fattibilità: riscritta, non corretta riga per riga.*

Regola: **un CSV per task e controllore**, una riga di metriche per run (più
righe dove il task ha più celle: T2 per velocità, T3 per verso della curva, T7
per ampiezza dell'impulso). Le run grezze (`.mat`) non sono tracciate da git, i
CSV sì: sono il risultato, non un artefatto.

## Campagna corrente — 7 task × 3 controllori, con il Rate Limiter ±0.4 m/s

Rifatta tutta il 1/10 sera. Qui stanno i numeri **validi**, quelli che finiscono
in relazione.

| file | cosa | scritto da |
|---|---|---|
| `T2_C*.csv` | piano, spazzata di velocità 0.5×–2.0× | `script_T2` |
| `T3_C*.csv` | curva a ±0.1 rad/s | `script_T3` |
| `T4_C*.csv` | rampa 8° | `script_T4` |
| `T4D_C*.csv`, `T4D_C*_finestre.csv` | dosso: una riga per run, e una per finestra (salita, cima, discesa) | `script_T4D` |
| `T5_C*.csv` | ostacolo singolo | `script_T5` |
| `T6_C*.csv` | sette ostacoli. **Vale solo l'esito**, e solo con deriva sotto `soglia_deriva_T6` | `script_T6` |
| `T7_C*.csv` | impulso laterale, sette ampiezze | `script_T7` |
| `tabella_confronti.md` | C1–C2–C3 affiancati con il verdetto sul rumore. **Non si modifica a mano** | `tabella_confronti` |

## `diagnostica/`

| file | cosa | scritto da |
|---|---|---|
| `rumore_slew.csv` | pavimento di rumore, quarto giro, 36 run, tutti e tre i controllori. È l'unico che vale | `rumore_metriche` |
| `fattibilita.csv` | coppia richiesta contro il limite del servo AX-12A | `fattibilita` |

## `storico/`

Roba superata, tenuta perché dietro c'è una conclusione scritta. **Non si
confronta con la campagna corrente.**

| cartella / file | cosa | perché è superata |
|---|---|---|
| `pre_slew_20261001/` | campagna del 1/10 prima del Rate Limiter, con il suo `LEGGIMI.md`. Contiene anche `T4_limite/` (vedi sotto) | senza limitatore C2 → C3 mescolava anello e limitatore |
| `pre_slew_20261001/T4_limite/` | una riga per angolo provato da `script_T4_limite`, C1 e C2. Spostata qui il 2/10 da `results/T4_limite/` | post-inerzie ma **pre-slew**: `script_T4_limite` non è stato rifatto col limitatore |
| `stimatore_disallineato/` | campagna del 22–25/9 | lo stimatore aveva le inerzie dell'URDF: C2 non cercava il terreno |
| `inerzie_URDF/` | campagna T2–T6 con le inerzie dell'URDF | inerzie ~1000× troppo grandi (`docs/piano_confronto.md` §9) |
| `taratura_T2_20260917/` | tre giri di taratura del 17/9 | forze di contatto ricostruite, non dai sensori; da qui «H è un vincolo, non un parametro libero» |
| `C3_srb_20260907/` | il vecchio "C3" sul simulatore a corpo rigido, 7/9 | altro impianto; l'MPC non si cita (vedi `archivio/README.md`) |
| `controllo_inerzie.csv` | le tre verifiche prima della campagna del 23/9. Spostato qui il 2/10 | lo scrive `archivio/controllo_inerzie.m` |
| `prova_inerzie*.csv`, `prova_soglia_T6*.csv`, `spazzata_soglia.csv`, `valida_soglia.csv`, `tau_statico_misurato.csv`, `rumore_T2.csv`, `rumore_metriche.csv`, `T6_C2_soglia*.csv` | prove delle indagini chiuse (inerzie, soglia di contatto, primo rumore) | gli script che li scrivono sono in `archivio/`, con l'esito |

## Cosa non finisce qui

- `.mat` delle run grezze: non tracciati. Una run non salvata non si recupera.
- Le figure stanno in `grafici/`, scritte da `salva_grafico`.
- CSV con **fonti di contatto diverse** non si confrontano sulle colonne di
  contatto: la colonna `note` di ogni riga dice da dove vengono le forze.
