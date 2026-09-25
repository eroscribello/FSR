# I due blocchi `MATLAB Function2` e `MATLAB Function3`

*Nota per chi ha scritto la versione originale: la **logica è la tua**, non è
stata cambiata. Quello che è cambiato è dove sta scritta e da dove prende i
numeri. Questo documento spiega cosa fa, cosa è stato spostato e perché.*

---

## 1. Cosa fanno questi due blocchi

Sono la **ricerca del terreno** di Arrigoni et al. §5 — il contributo che
distingue il controllore C2 dal C1.

L'idea: durante l'abbassamento del piede, se la zampa non risulta ancora a
terra, il comando di profondità viene **esteso verso il basso** a velocità
costante finché il contatto non viene rilevato. Quando il contatto arriva, la
zampa si **blocca** alla quota raggiunta. Così una zampa che trova il terreno
più in basso del previsto — un gradino, un avvallamento — ci arriva comunque,
invece di restare per aria.

Nell'articolo il contatto si stima con un modello dinamico, perché sul robot
vero non c'è un sensore. Qui no: in Simscape la coppia ai giunti si **misura**,
quindi la stima non serve e la logica è la stessa in forma più semplice.

**Sono nel percorso del comando, non nel logging.** Questo va detto subito
perché è il malinteso più facile: `z1` e i flag di contatto entrano, `z_lf`…
`z_rr` escono e vanno ai sei blocchi gamba. Se i blocchi si commentano, il
robot cambia comportamento.

---

## 2. Dove stanno nel flusso

```
   generatore di traiettoria                flag di contatto
        │  z (profondità comandata,              │  c_lf, c_lr, c_rm
        │     comune al tripode)                 │  (0 = zampa non a terra)
        ▼                                        ▼
   ┌──────────────────────────────────────────────────┐
   │   MATLAB Function2     tripode  FL · RL · MR     │
   │   MATLAB Function3     tripode  FR · RR · ML     │
   └──────────────────────────────────────────────────┘
        │  z_lf, z_lr, z_rm  (una profondità PER ZAMPA)
        ▼
   sei blocchi leg_ik  →  giunti
```

Un blocco per tripode. In ingresso **una** profondità comune alle tre zampe;
in uscita **tre** profondità distinte, una per zampa. È tutta qui la
differenza fra C1 e C2: in C1 le tre uscite sono uguali all'ingresso, in C2 si
separano.

### La convenzione di segno, che confonde sempre

`z` è una **profondità**: positiva verso il **basso**. Quindi «estendere la
zampa più in giù» vuol dire **aumentare** `z`. Nel codice:

```matlab
z_ext(i) = min(z_ext(i) + v_search*dt, z_ext_max);   % scende
```

---

## 3. Cosa è cambiato, e perché

La logica è identica. Sono cambiate due cose.

### a) La logica è uscita dal `.slx` ed è finita in `ricerca_terreno.m`

Prima stava scritta **dentro i due blocchi**, cioè due volte. Tre conseguenze,
tutte reali:

| Problema | Perché |
|---|---|
| invisibile a git | il `.slx` è un file binario: nessun diff, nessuna storia, nessuna revisione possibile su quella logica |
| due copie che possono divergere | nulla segnala se un giorno correggi un blocco e ti dimentichi l'altro |
| `z_nominal = 0.14` duplicava `cfg.z0` | ritarare `z0` per la campagna T2 avrebbe sfasato la soglia di ricerca **in silenzio** |

Adesso i due blocchi sono una riga sola che chiama `ricerca_terreno`, esattamente
come già facevano `tripod_gait` e `leg_ik`. La funzione è sul path, sotto git,
e si può leggere e correggere come qualunque altro file.

### b) Le costanti cablate sono diventate parametri

Erano cinque numeri scritti a mano dentro il blocco: `0.14`, `0.06`, `0.03`,
`0.5`, `0.002`. Ora arrivano da `phantomx_config` attraverso `init_gait`, in un
vettore `1 x 6` chiamato `c2_par`, collegato alla **sesta porta** del blocco
tramite un `Constant`.

```
c2_par = [attiva, z_nom, v_search, z_ext_max, t_reset, tol]
```

Perché un `Constant` e non lasciare che il blocco legga la variabile: un blocco
`MATLAB Function` **non risolve da sé** un identificatore contro il base
workspace. Va dichiarato come `Parameter` nel pannello Symbols, oppure passato
da una porta. È stata scelta la porta perché è il pattern che il modello già
usa per `gait.T`, `gait.S`, `gait.H`, `gait.z0`, `gait.duty`, e perché non
dipende dalla risoluzione implicita dei parametri, che cambia da release a
release.

---

## 4. La logica, caso per caso

Il cuore è un ciclo sulle tre zampe del tripode, con tre rami:

```matlab
for i = 1:3
    if c(i) == 0                          % la zampa NON trasmette forza
        if z >= (z_nom - tol)             %   e il comando la vuole a terra
            z_ext(i)  = min(z_ext(i) + v_search*dt, z_ext_max);
            z_hold(i) = z + z_ext(i);
            z_out(i)  = z_hold(i);        %   → CERCA: scende
        else                              %   ma il comando la vuole in volo
            z_ext(i)  = 0;
            z_hold(i) = z;
            z_out(i)  = z;                %   → niente ricerca, stato azzerato
        end
    else                                  % contatto rilevato
        z_out(i) = min(z + z_ext(i), z_hold(i));   % → SI FERMA
    end
end
```

| Situazione | `c(i)` | `z` vs soglia | Cosa fa |
|---|---|---|---|
| fase di appoggio, terreno non ancora trovato | 0 | `z ≥ z_nom − tol` | estende verso il basso a `v_search`, fino a `z_ext_max` |
| fase di volo | 0 | `z < z_nom − tol` | nessuna ricerca, azzera lo stato per il prossimo appoggio |
| contatto rilevato | 1 | — | si blocca: non scende oltre `z_hold` |

Tre dettagli che non sono ovvi leggendo il codice:

- **Il ramo di volo azzera lo stato.** Serve: senza quell'azzeramento
  l'estensione accumulata in un appoggio verrebbe portata dentro il successivo.
- **`min(z + z_ext(i), z_hold(i))`** nel ramo di contatto non è una ridondanza:
  il comando `z` continua a variare dopo il contatto, e il `min` impedisce alla
  zampa di scendere oltre la quota dove il terreno è stato trovato.
- **Il `dt` è protetto.** `if dt <= 0 || dt > 0.1, dt = 0.001; end`: al primo
  passo e dopo un passo anomalo del solver, un `dt` sbagliato produrrebbe
  un'estensione sbagliata. Con passo variabile succede.

E un reset all'avvio:

```matlab
if t_prev < 0 || t < t_reset
```

Per i primi `t_reset = 0.5 s` non si cerca niente. Prima che il robot si sia
assestato una zampa può risultare non a terra per il transitorio, e una ricerca
partita per quel motivo è rumore, non terreno.

---

## 5. I parametri

Tutti in `phantomx_config.m`, sezione `cfg.c2`.

| Campo | Valore | Cosa fa |
|---|---|---|
| `cfg.c2.attiva` | `true` | **l'interruttore C1/C2**, vedi sotto |
| `cfg.c2.z_nom` | `= cfg.z0` | profondità oltre cui si considera «fase di abbassamento» |
| `cfg.c2.v_search` | `0.06 m/s` | velocità di discesa in ricerca |
| `cfg.c2.z_ext_max` | `0.03 m` | estensione massima sotto la nominale |
| `cfg.c2.t_reset` | `0.5 s` | nessuna ricerca nel transitorio iniziale |
| `cfg.c2.tol` | `0.002 m` | tolleranza sulla soglia di abbassamento |
| `cfg.c2.soglia_tau` | `[0.385, 0.136, 0.105] N·m` | soglia **per giunto** del flag di contatto |

> **[CORRETTO 25/9]** Questa riga diceva `0.5 N·m`, uno scalare. È sbagliata due
> volte. La soglia è un **vettore di tre**, una per giunto (coxa, femore, tibia), e
> il valore `0.5` non è mai arrivato al modello: fino al 23/9 nessun blocco leggeva
> `c2_soglia`. Il valore attuale è stato **tarato da noi** il 25/9 partendo da
> `[0.04, 0.1, 0.1]`, che era quello del modello. Sulla coxa la soglia è
> volutamente altissima: la sua distribuzione di `|Δτ|` in appoggio e in volo si
> sovrappone, quindi il giunto è **escluso di fatto** e il contatto lo rilevano
> femore e tibia. Motivazione e misure in `common/phantomx_config.m` e
> `docs/piano_confronto.md` §11.

`z_nom` è **legato** a `cfg.z0`, non riscritto: se si ritara `z0`, la soglia
segue. Prima erano due `0.14` indipendenti, e ritarare l'uno sfasava l'altro
senza dirlo.

---

## 5-bis. [25/9] Da dove viene il flag di contatto, e cosa succede se resta acceso

I sei flag `c_lf`…`c_rr` che entrano in questi blocchi **non** sono una soglia sulla
coppia misurata. La regola, letta nel `.slx` risalendo il collegamento del
`Constant c2_soglia`, è

```
Inverse Dynamics → Reshape → Subtract [+ −] → Abs → Demux → Mux → Relational Operator
                                    ↑                                      ↑
                              tau_misurata                            c2_soglia
```

cioè `cont = OR( |τ_misurata − τ_attesa| > soglia )` sui tre giunti, con `τ_attesa`
prodotta da un blocco `Inverse Dynamics` che porta dentro un `rigidBodyTree`
importato dall'URDF. È la forma del paper.

**Conseguenza da tenere a mente:** il flag dipende da quanto il modello dello
stimatore somiglia al robot simulato. Se i due divergono, `|Δτ|` è grande sempre e
il flag resta acceso — che è quello che è successo dal 22 al 25 settembre.

### Cosa fa `ricerca_terreno` con il flag incollato a 1

Guardando i rami: `z_ext` cresce **solo** dentro `if c(i) == 0`, e `z_ext`/`z_hold`
si azzerano **solo** lì. Con `c ≡ 1` non si entra mai in quel ramo, quindi

- `z_ext` resta a `0` per sempre → **nessuna ricerca del terreno**;
- `z_hold` resta il valore preso all'ultimo reset, cioè a `t_reset`;
- il comando diventa `z_out = min(z, z_hold(t_reset))`, una **costante**.

Non è una ricerca degradata: è un tosatore di profondità. E non lo segnala niente —
l'animazione resta plausibile, il robot cammina, le metriche escono. Per questo la
diagnosi ha richiesto di misurare il flag contro la forza vera (`valida_soglia.m`).

Un caso intermedio conta quasi quanto: il flag che si alza **prima** del contatto.
Lì si entra nel ramo `else`, `z_out = min(z + z_ext, z_hold)`, e la zampa **si
congela alla quota che aveva in quel momento**. Su terreno piano non si vede; su un
dislivello la zampa resta per aria. Con la terna `[0.04, 0.1, 0.1]` il flag si
alzava in mediana **220 ms prima** del contatto.

---

## 6. L'interruttore C1 / C2 è `par(1)`, **non** `c2_soglia`

Questo è il punto più importante del documento, perché la documentazione
precedente diceva il contrario e i dati raccolti in base a quella indicazione
erano sbagliati.

**Era scritto** che per avere l'anello aperto bastasse portare la soglia di
coppia a infinito. **È falso, e fa il contrario di quello che serve.**

Con `c2_soglia = inf` il flag di contatto è sempre falso. Ma guarda il primo
ramo: con `c(i) == 0` e `z ≥ z_nom − tol` la ricerca **entra nel ramo di
discesa**. E la condizione è sempre verificata, perché `cfg.z0 = 0.140` contro
una soglia di `0.140 − 0.002 = 0.138`. Risultato: a ogni appoggio la zampa
scende di 3 cm a 0,06 m/s. Non è anello aperto: è la ricerca al massimo della
sua autorità, sempre.

L'anello aperto vero è `par(1) = 0`, che fa uscire la funzione subito con il
comando invariato:

```matlab
if attiva == 0
    z_hold = [z, z, z];
    z_ext  = [0, 0, 0];
    t_prev = t;
    return
end
```

(lo stato viene comunque azzerato, così riaccendendo C2 a metà sessione non si
eredita una ricerca rimasta a metà).

In pratica, per scegliere il controllore senza toccare la configurazione:

```matlab
OVERRIDE_C2 = false;  init_gait     % C1, anello aperto vero
OVERRIDE_C2 = true;   init_gait     % C2
clear OVERRIDE_C2                   % torna a cfg.c2.attiva
```

---

## 7. Perché lo stato sta nel blocco e non nella funzione

`ricerca_terreno.m` **non ha variabili `persistent`**. Lo stato — `z_hold`,
`z_ext`, `t_prev` — vive nelle `persistent` del blocco, che lo passa alla
funzione e se lo riprende indietro:

```matlab
persistent z_hold z_ext t_prev
if isempty(t_prev)
    z_hold = [0 0 0];  z_ext = [0 0 0];  t_prev = -1;
end

[z_out, z_hold, z_ext, t_prev] = ricerca_terreno( ...
        z, [c_lf, c_lr, c_rm], t, z_hold, z_ext, t_prev, par);
```

Il motivo: **i due blocchi chiamano la stessa funzione**, e i due tripodi
devono avere stati separati. Se le `persistent` stessero dentro
`ricerca_terreno`, sarebbero condivise e i due tripodi si sovrascriverebbero a
vicenda — un bug silenzioso, che si manifesterebbe come un'andatura
leggermente sbagliata e non come un errore.

---

## 8. Cosa non rompere

**I nomi e l'ordine degli argomenti.** Le porte di un blocco `MATLAB Function`
prendono il nome dalle variabili della firma: cambiare un nome rinomina la
porta e stacca la linea. In particolare:

```matlab
% MATLAB Function2  (SID 1134)
function [z_lf, z_lr, z_rm] = fcn(z, c_lf, c_lr, c_rm, t, par)

% MATLAB Function3  (SID 1159)
function [z_rf, z_rr, z_lm] = fcn(z, c_rf, c_rr, c_lm, t, par)
```

`MATLAB Function3` ha l'ordine **`rf, rr, lm`**. È quello vero, letto dal
modello: non va riordinato in alfabetico.

**`c2_par` deve esistere prima del primo `sim`.** Lo crea `init_gait`. Se il
`Constant` viene valutato prima, l'errore è

```
Undefined function or variable c2_par.
```

e sembra che la modifica non abbia funzionato, mentre manca solo un
`init_gait`.

**Il `Constant` deve valere `c2_par`, non `c2_par(1)`.** Servono tutti e sei
gli elementi.

---

## 9. Come verificare che funzioni

```matlab
init_gait
applica_terreno('T1')

OVERRIDE_C2 = false;  init_gait
out1 = sim('phantomx_sim_zero','StopTime','10');

OVERRIDE_C2 = true;   init_gait
out2 = sim('phantomx_sim_zero','StopTime','10');

clear OVERRIDE_C2
```

Il controllo che conta: **in C1 le sei profondità comandate devono essere
identiche fra loro**, perché nessuna zampa cerca il terreno. In C2 devono
differire.

```matlab
zz = @(o) [o.z_lf.Data(end) o.z_lr.Data(end) o.z_rm.Data(end) ...
           o.z_rf.Data(end) o.z_rr.Data(end) o.z_lm.Data(end)];
fprintf('C1  escursione fra zampe %.4f mm   (attesa: ~0)\n', 1e3*(max(zz(out1))-min(zz(out1))));
fprintf('C2  escursione fra zampe %.4f mm   (attesa: > 0)\n', 1e3*(max(zz(out2))-min(zz(out2))));
```

Se in C1 l'escursione non è nulla, `par(1)` non sta arrivando al blocco:
controlla che il `Constant` sia collegato alla porta 6 e che il suo Value sia
`c2_par`.

---

## 10. Conseguenza sui dati vecchi

I numeri di T2 già in tabella — 86% / 116% / 18% — sono stati misurati quando
si credeva che `c2_soglia = inf` desse l'anello aperto. Non lo dava: quelle run
giravano con la ricerca del terreno attiva al massimo. **Non sono C1**, e sono
state rifatte.

Per lo stesso motivo `adatta_simscape` ora **legge** l'etichetta del
controllore da `c2_par(1)` invece di scriverla a mano: era cablata a `'C1'`,
quindi una run fatta senza override girava in C2 e finiva in tabella marcata
C1. Adesso l'etichetta non può più mentire.

---

## Riferimenti

- `ricerca_terreno.m` — la logica, commentata
- `docs/modifica_slx_ricerca_terreno.md` — come è stata applicata la modifica
  al modello, passo per passo
- `common/phantomx_config.m`, sezione `cfg.c2` — i parametri
- `simscape/init_gait.m` — dove `c2_par` viene assemblato e dove agisce
  `OVERRIDE_C2`
- Arrigoni et al., *Robotics* 2024, 13, 142, §5 — l'articolo di riferimento
