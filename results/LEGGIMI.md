# `results/` — che cosa c'è dentro

Aggiornato il 23/9/2026. Riordinato da `sistema_results.m` (lanciabile di
nuovo: sposta solo, e toglie un doppione solo dopo averlo confrontato byte per
byte con la copia buona).

Regola: **un CSV per task e controllore**, una riga di metriche per run. Le run
grezze (`.mat`) e le figure di lavoro non sono tracciate da git, i CSV sì —
sono il risultato, non un artefatto.

## Campagna corrente

Qui stanno i numeri **validi**, quelli che finiscono in relazione.

| file | cosa | scritto da |
|---|---|---|
| `T2_C1.csv`, `T2_C2.csv` | piano, spazzata di velocità 0.5×–1.4× | `script_T2` |
| `T3_C1.csv`, `T3_C2.csv` | curva, tasso d'imbardata | `script_T3` |
| `T4_C1.csv`, `T4_C2.csv` | rampa 8° | `script_T4` |
| `T4_limite_C1.csv`, `T4_limite_C2.csv` | angolo limite di salita, tabella riassuntiva | `script_T4_limite` |
| `T4_limite/T4_<ctrl>_<g>deg.csv` | una riga per angolo provato | `script_T4` via `script_T4_limite` |
| `T4D_C1.csv`, `T4D_C2.csv` | dosso: salita, cima, discesa | `script_T4D` |
| `T4D_<ctrl>_finestre.csv` | una riga per finestra (salita/cima/discesa), tutte le metriche | `script_T4D` |
| `T5_C1.csv`, `T5_C2.csv` | ostacolo singolo | `script_T5` |
| `T6_C1.csv`, `T6_C2.csv` | sette ostacoli | `script_T6` |
| `controllo_inerzie.csv` | le tre verifiche prima della campagna | `controllo_inerzie` |

> **[23/9] Stato:** i CSV `T2…T6` qui sopra sono ancora quelli con le **inerzie
> vecchie** e verranno sovrascritti dalla campagna in corso. La copia di
> sicurezza è in `storico/inerzie_URDF/`. `T4_limite/` è stato svuotato apposta:
> la nuova bisezione può fermarsi su angoli diversi, e vecchi e nuovi mescolati
> nella stessa cartella non si distinguerebbero.

## `diagnostica/`

Prove che servono a decidere, non a misurare i controllori.

| file | cosa |
|---|---|
| `prova_inerzie.csv` | A/B inerzie del file contro inerzie corrette, T2 1× |
| `prova_inerzie_T6.csv` | lo stesso A/B su T6, C1 e C2 — è la prova che le inerzie cambiano le conclusioni |

## `storico/`

Roba superata, tenuta perché dietro c'è una conclusione scritta nel piano. **Non
si confronta con la campagna corrente.**

| cartella | cosa | perché è superata |
|---|---|---|
| `inerzie_URDF/` | campagna completa T2–T6 + `T4_limite/`, con le inerzie prese dall'URDF | inerzie ~1000× troppo grandi (`docs/piano_confronto.md` §9) |
| `taratura_T2_20260917/` | tre giri di taratura del 17/9, `_griglia` (tutte le celle provate) e `_scelte` (le celle ammissibili) | forze di contatto *ricostruite*, non dai sensori; dietro c'è la conclusione «H è un vincolo, non un parametro libero» |
| `C3_srb_20260907/` | misure di C3 sul simulatore a corpo rigido (task `T2a`), 7/9 | altro impianto di simulazione: si confrontano con C1/C2 solo dichiarando che il simulatore è diverso (`piano_confronto.md` §4) |

## Cosa non finisce qui

- `.mat` delle run grezze e figure: non tracciati.
- CSV con **fonti di contatto diverse** non si confrontano sulle colonne di
  contatto: la colonna `note` di ogni riga dice da dove vengono le forze
  (sensori `Fleg` oppure ricostruite dalla penetrazione).
