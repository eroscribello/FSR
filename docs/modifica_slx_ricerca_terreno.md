# La modifica al .slx — due script di blocco da sostituire

**Durata: 2–4 minuti. Nessuna linea esistente da ridisegnare.**

Prima di tutto, la copia di sicurezza:

```matlab
bdclose all
copyfile('simscape/phantomx_sim_zero.slx', ...
         sprintf('simscape/phantomx_sim_zero_backup_%s.slx', ...
                 char(datetime('now','Format','yyyyMMdd_HHmmss'))));
```

---

## Il problema da risolvere: come arriva `c2_par` dentro il blocco

`c2_par` è un vettore `1 x 6` che `init_gait` mette nel base workspace:

```
[attiva, z_nom, v_search, z_ext_max, t_reset, tol]
```

Un blocco `MATLAB Function` **non** risolve da sé un identificatore sconosciuto
contro il workspace: bisogna dirglielo. L'errore che si ottiene altrimenti è

```
Undefined function or variable c2_par.
```

Due modi. **B è quello consigliato**: è il pattern che questo modello già usa
per `gait.T`, `gait.S`, `gait.H`, `gait.z0`, `gait.duty` — tutti passati da un
`Constant` a una porta d'ingresso. Non dipende dalla risoluzione implicita dei
parametri, che cambia da release a release.

---

## A — via pannello Symbols (nessun cablaggio)

1. Doppio clic sul blocco → si apre l'editor della funzione.
2. Scheda **Function** → **Edit Data** (nelle release recenti: pannello
   laterale **Symbols**, oppure **View → Symbols**).
3. `c2_par` compare già nell'elenco, senza Scope. Impostalo a:

   | campo | valore |
   |---|---|
   | Name | `c2_par` |
   | Scope | **Parameter** |
   | Size | `-1` (eredita) |
   | Type | `Inherit: Same as Simulink` |
   | Complexity | Off |

4. Ripeti sull'altro blocco. Salva il modello.

Gli script dei due blocchi sono quelli della sezione **Script, versione A** qui
sotto — firma a 5 argomenti, identica a oggi.

---

## B — via Constant e porta d'ingresso (consigliato)

Per ciascuno dei due blocchi:

1. Incolla lo script della sezione **Script, versione B**: ha un argomento in
   più, `par`. Appena salvi, Simulink aggiunge da solo la **sesta porta
   d'ingresso** al blocco. Le cinque esistenti non si spostano e le loro linee
   restano attaccate.
2. Copia uno dei `Constant` già presenti (per esempio `Constant2`, quello di
   `gait.T`), incollalo accanto al blocco e metti come **Value**:

   ```
   c2_par
   ```

3. Collega quel `Constant` alla nuova porta 6.
4. Salva il modello.

Totale: due blocchi nuovi, due linee nuove, nessuna linea toccata.

---

## Script, versione A — firma a 5 argomenti

### `MATLAB Function2` (SID 1134) → zampe FL, RL, MR

```matlab
function [z_lf, z_lr, z_rm] = fcn(z, c_lf, c_lr, c_rm, t)
%#codegen
% La logica sta in ricerca_terreno.m, sul path: il .slx e' binario e quello
% che ci sta dentro e' invisibile a git.
% L'interruttore C1/C2 e' c2_par(1), definito da init_gait.
persistent z_hold z_ext t_prev
if isempty(t_prev)
    z_hold = [0 0 0];
    z_ext  = [0 0 0];
    t_prev = -1;
end

[z_out, z_hold, z_ext, t_prev] = ricerca_terreno( ...
        z, [c_lf, c_lr, c_rm], t, z_hold, z_ext, t_prev, c2_par);

z_lf = z_out(1);
z_lr = z_out(2);
z_rm = z_out(3);
end
```

### `MATLAB Function3` (SID 1159) → zampe FR, RR, ML

```matlab
function [z_rf, z_rr, z_lm] = fcn(z, c_rf, c_rr, c_lm, t)
%#codegen
persistent z_hold z_ext t_prev
if isempty(t_prev)
    z_hold = [0 0 0];
    z_ext  = [0 0 0];
    t_prev = -1;
end

[z_out, z_hold, z_ext, t_prev] = ricerca_terreno( ...
        z, [c_rf, c_rr, c_lm], t, z_hold, z_ext, t_prev, c2_par);

z_rf = z_out(1);
z_rr = z_out(2);
z_lm = z_out(3);
end
```

---

## Script, versione B — firma a 6 argomenti *(consigliata)*

### `MATLAB Function2` (SID 1134) → zampe FL, RL, MR

```matlab
function [z_lf, z_lr, z_rm] = fcn(z, c_lf, c_lr, c_rm, t, par)
%#codegen
% La logica sta in ricerca_terreno.m, sul path: il .slx e' binario e quello
% che ci sta dentro e' invisibile a git.
% par = c2_par, dal Constant collegato alla porta 6. L'interruttore C1/C2
% e' par(1), lo assembla init_gait.
persistent z_hold z_ext t_prev
if isempty(t_prev)
    z_hold = [0 0 0];
    z_ext  = [0 0 0];
    t_prev = -1;
end

[z_out, z_hold, z_ext, t_prev] = ricerca_terreno( ...
        z, [c_lf, c_lr, c_rm], t, z_hold, z_ext, t_prev, par);

z_lf = z_out(1);
z_lr = z_out(2);
z_rm = z_out(3);
end
```

### `MATLAB Function3` (SID 1159) → zampe FR, RR, ML

```matlab
function [z_rf, z_rr, z_lm] = fcn(z, c_rf, c_rr, c_lm, t, par)
%#codegen
persistent z_hold z_ext t_prev
if isempty(t_prev)
    z_hold = [0 0 0];
    z_ext  = [0 0 0];
    t_prev = -1;
end

[z_out, z_hold, z_ext, t_prev] = ricerca_terreno( ...
        z, [c_rf, c_rr, c_lm], t, z_hold, z_ext, t_prev, par);

z_rf = z_out(1);
z_rr = z_out(2);
z_lm = z_out(3);
end
```

---

## Due cose da non sbagliare

**I nomi degli argomenti restano quelli di prima.** Le porte dei blocchi
`MATLAB Function` prendono il nome dalle variabili della firma: cambiandoli si
rinominano le porte. I nomi e l'ordine qui sopra sono quelli letti dal
modello. In particolare **`MATLAB Function3` ha l'ordine `rf, rr, lm`**: è
quello vero, non riordinarlo in alfabetico.

**`c2_par` deve esistere nel workspace prima del primo `sim`.** Lo definisce
`init_gait`, quindi lancia `init_gait` una volta a mano dopo aver salvato il
modello: se il `Constant` viene valutato prima che quella variabile esista,
l'errore è lo stesso di prima e sembra che la modifica non abbia funzionato.

---

## Verifica, subito dopo

```matlab
init_gait
applica_terreno('T1')

% --- C1, anello aperto vero ---
OVERRIDE_C2 = false;  init_gait
out1 = sim('phantomx_sim_zero','StopTime','10');

% --- C2, il comportamento di oggi ---
OVERRIDE_C2 = true;   init_gait
out2 = sim('phantomx_sim_zero','StopTime','10');

clear OVERRIDE_C2
```

Il controllo che conta: in C1 le sei profondità comandate devono essere
**identiche fra loro**, perché nessuna zampa cerca il terreno. In C2 devono
differire.

```matlab
zz = @(o) [o.z_lf.Data(end) o.z_lr.Data(end) o.z_rm.Data(end) ...
           o.z_rf.Data(end) o.z_rr.Data(end) o.z_lm.Data(end)];
fprintf('C1  escursione fra zampe %.4f mm   (attesa: ~0)\n', 1e3*(max(zz(out1))-min(zz(out1))));
fprintf('C2  escursione fra zampe %.4f mm   (attesa: > 0)\n', 1e3*(max(zz(out2))-min(zz(out2))));
```

Se in C1 l'escursione non è nulla, `par(1)` non sta arrivando al blocco: con
la versione B controlla che il `Constant` sia collegato alla porta giusta e che
il suo Value sia `c2_par` e non `c2_par(1)`.

---

## Cosa era sbagliato prima, e va corretto nei documenti

`c2_soglia = inf` **non** dava l'anello aperto. Rendeva il flag di contatto
sempre falso, e con il flag falso la ricerca entrava nel ramo di discesa a
**ogni** fase di appoggio: `cfg.z0 = 0.14` contro la soglia `0.138` del blocco
(`z_nominal - 0.002`), quindi la condizione era sempre verificata e la zampa
scendeva di 3 cm a 0,06 m/s.

Correzioni già applicate in `phantomx_config.m`, `init_gait.m`, `README.md` e
`docs/piano_confronto.md`.

**Conseguenza sui dati già raccolti:** i numeri di T2 in tabella (86% / 116% /
18%) sono stati misurati prima che la retroazione entrasse nel modello. Sono
C1 di una versione che non esiste più, e vanno rifatti con `OVERRIDE_C2 =
false`. È il motivo per cui la campagna riparte da `taratura_T2`.
