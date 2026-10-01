# Campagna del 1/10/2026, PRIMA dello slew rate su tutti i modelli

Righe di `results/` com'erano la sera del 1/10, prima che il Rate Limiter
(±0.4 m/s, uno per piede) venisse salvato in `phantomx_sim_zero` e portato da
±0.3 a ±0.4 in `phantomx_sim_attitude`. Spostate qui il 1/10 per rifare la
campagna su tutti e tre i controllori. I grafici non sono conservati: sono
stati sovrascritti dalla campagna nuova.

| | |
|---|---|
| C1, C2 | **senza** Rate Limiter |
| C3 | Rate Limiter **±0.3 m/s**: taglia il 32% del volo nominale (picco 0.375 m/s) |
| `T6_C1.csv` | **ATTENZIONE**: è la run di prova di C1 **con** slew rate ±0.3 (19:29), non la riga di campagna. Identica a `diagnostica/T6_C1_slew.csv`. La riga di campagna di C1 su T6 di quella sera non è stata conservata |
| `T6_C3.csv` | la riga di campagna a ±0.3 (copia presa prima della prova di C3 a ±0.4) |
| `diagnostica/T6_C2_slew.csv`, `T6_C1_slew.csv` | le prove con slew rate ±0.3 da cui è nata la decisione |
| `diagnostica/rumore_piano.csv`, `rumore_ostacoli.csv` | pavimento di rumore di C1 e C2 (25-26/9) |
| `diagnostica/rumore_C3.csv` | pavimento di rumore di C3, slew ±0.3 (1/10) |
| `diagnostica/fattibilita.csv`, `tabella_confronti.md` | come generati la sera del 1/10 |

Valgono come documento della decisione, non come confronto: qui C2→C3
mescola anello d'assetto e limitatore. Le conclusioni di allora stanno in
`docs/stato_progetto.md`, con i [RITIRATO] del caso.
