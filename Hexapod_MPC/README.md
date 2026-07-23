# Hexapod Predictive Control (MPC) Framework

This repository contains an advanced **Model Predictive Control (MPC)** framework written in MATLAB for dynamic locomotion. Originally designed for a quadruped robot, the framework has been fully refactored and upgraded to support an **hexapod (6-legged) robot** architecture with 18 independent actuation inputs.

The controller optimizes ground reaction forces (GRF) using a linearized rigid body dynamics formulation combined with a quadratic programming (QP) solver, taking into account friction cones and unilateral contact constraints.

---

## 🚀 Key Upgrades: Quad to Hexapod

Transitioning from a 4-legged system to a 6-legged system required structural modifications across the entire state-space formulation and constraint matching loops. Below is an overview of why and where these modifications were made:

### 1. State-Space Alignment ($X_t \in \mathbb{R}^{36}$)
* **The Issue:** The robot's full state vector includes the Center of Mass (CoM) kinematics (18 states) and the spatial positions ($x, y, z$) of each foot. In the quadruped version, this vector was of size 30 ($18 + 4 \times 3$).
* **The Fix:** Expanded the state vector `Xt` to **36 elements** ($18 + 6 \times 3$) to continuously track the positions of the 2 additional feet (Foot 5 and Foot 6) in the workspace.

### 2. Input Vector Expansion ($U_t \in \mathbb{R}^{18}$)
* **The Issue:** The core control variables are the ground reaction forces ($F_x, F_y, F_z$) for each leg. 4 legs generated 12 control inputs.
* **The Fix:** Modified the controller configuration to operate with **18 control inputs** ($6 \text{ legs} \times 3 \text{ forces}$). The input weight matrix `R` inside `get_params.m` was expanded from a $12 \times 12$ matrix to a **$18 \times 18$** diagonal matrix using a `repmat(..., [6,1])` structure.

### 3. QP Formulator Overhaul (`fcn_get_QP_form_eta.m`)
* **Dynamic Dimensioning:** Hardcoded dimension boundaries (`nU = 12`) were upgraded to `nU = 18` to map the expanded $18 \times 18$ input penalization trajectories over the predictive horizon.
* **Constraint Matrices Matched:** The inequality constraints matrix (`Aineq`) was redesigned to evaluate friction cones and maximum vertical loads for all 6 limbs instead of 4. The constraints stack size was expanded from $4 \cdot n_{\text{friction}}$ to **$6 \cdot n_{\text{friction}}$** per horizon step.
* **Fz Extraction:** Corrected the indexing scheme extracting desired vertical forces (`Ud([3 6 9 12 15 18], :)`) to safely extract the normal forces from the updated 18-element reference trajectory buffer.

---

## 📊 Core Architecture & Script Breakdown

The repository is organized into distinct algorithmic units:

*   **`MAIN.m`**: The main execution engine. Initializes the workspace, executes the gait planner, steps through the predictive horizon loop, and manages data logging.
*   **`get_params.m`**: System parameter configuration file. Contains friction coefficients ($\mu$), horizon lengths, cost functions (`Q`, `Qf`, `R`), and hexapod physical properties.
*   **`fcn_get_QP_form_eta.m`**: Formulates the high-speed Quadratic Programming problem ($\min \frac{1}{2} x^T H x + g^T x$) satisfying matching dynamics and boundary parameters.
*   **`fcn_get_ABD_eta.m`**: Discretizes and linearizes the continuous-time system dynamics into the predictive state matrices ($A, B, d$).

---

## 📉 Diagnostics & Telemetry Plotting

The plotting sequence provides complete telemetry specific to hexapod walking behaviors. Running the evaluation outputs 5 specialized structural figures:

1. **CoM Overview & Global Forces:** Tracking validation of the Center of Mass 3D position, linear velocity, and angular velocities alongside a global 6-channel overlay of all vertical reaction forces ($F_z$).
2. **Individual Vertical Forces (Fz Grid):** A structured $3 \times 2$ grid comparison mapping the actual optimized $F_z$ profile against the planner's reference inputs for every leg.
3. **Foot Swing Trajectories (Z-tracking):** Monitors foot clearance and spatial trajectory tracking to analyze the step profile during swing phases.
4. **Friction Cone Analysis (Per-Leg Local Slipping):** Evaluates local traction safety margins by solving $\theta_{\text{foot}} = \arctan(\frac{\sqrt{F_x^2 + F_y^2}}{F_z})$ for each foot and comparing it to the physical slip boundary ($\theta_{\text{lim}} = \arctan(\mu)$).
5. **Total Horizontal Traction:** Tracks structural acceleration forces pushing along the X and Y axes.

---

## 🛠️ Usage

To execute the hexapod simulation framework:

1. Open MATLAB and navigate to the project directory.
2. Ensure all helper functions are added to your path.
3. Open `get_params.m` and verify that your desired hexapod gait (e.g., Tripod gait) is selected.
4. Run `MAIN.m` from the command window or editor:
   ```matlab
   run('MAIN.m')


# UPDATE ANDREA RUSSO

# RF-MPC su esapode PhantomX — stato del porting

Aggiornamento del lavoro svolto: conversione del codice RF-MPC dell'hw4
(quadrupede, Ding et al.) a **esapode PhantomX AX Metal Mark II**.

Stato: **l'MPC cammina in tripode sul simulatore ridotto a corpo rigido.**
Verificato su piano, velocita' 0.1 m/s, 4.5 s di simulazione.

---

## 1. Convenzioni condivise (da rispettare in TUTTI i file)

**Ordine canonico delle zampe** — se un file usa un ordine diverso, il robot
cammina storto senza dare errori. E' la convenzione piu' importante:

| idx | zampa | anca (x, y) da URDF |
|----|-------|---------------------|
| 1 | LF (ant-sx)   | ( 0.1248,  0.0616) |
| 2 | RF (ant-dx)   | ( 0.1248, -0.0616) |
| 3 | LM (med-sx)   | ( 0.0000,  0.1034) |
| 4 | RM (med-dx)   | ( 0.0000, -0.1034) |
| 5 | LH (post-sx)  | (-0.1248,  0.0616) |
| 6 | RH (post-dx)  | (-0.1248, -0.0616) |

Il layout e' **esagonale**, non rettangolare: le zampe centrali sporgono in y
piu' di anteriori/posteriori. Non ricostruire le anche da `L`/`W`: usare `p.p_hip`.

**Dimensioni** (da 4 a 6 zampe):

| | quadrupede | esapode |
|---|---|---|
| stato `Xt` | 30 | **36** (18 + 3x6) |
| controllo `Ut` | 12 | **18** (3 forze x 6 piedi) |
| stato ridotto QP `nX` | 12 | **12** (invariato: e' il corpo) |
| `p.R` | 12x12 | **18x18** |
| `p.Q`, `p.Qf` | 12x12 | **12x12** (invariati) |

Regola mnemonica: scala tutto cio' che riguarda i **piedi**; non scala nulla di
cio' che riguarda il **corpo**.

**Naming**: la stance nominale si chiama `p.pf36` (prima `p.pf34`, fuorviante).
Non si scrive a mano: si deriva da `p.p_hip`.

---

## 2. Modifiche file per file

### `get_params.m` — riscritto
- `p.nLeg = 6`, `p.p_hip` (3x6, da URDF).
- **Parametri fisici PhantomX** (prima erano ancora del quadrupede):
  `mass` 1.56 kg, `z0` 0.08 m, `l1` 0.066, `l2` 0.075, `d` 0.054, `mu` 0.6.
- `p.J = diag([0.008, 0.010, 0.016])` — **stima**, da confermare (vedi punto 5).
- Andature parametriche: `p.T`, `p.beta`, `p.phase`; da questi
  `p.Tst = beta*T` e `p.Tsw = (1-beta)*T`.
- `p.pf36` **derivata** da `p_hip` spingendo i piedi radialmente in fuori di
  `p.spread = 0.07` m.
- Costanti di scala andatura: `stepLenRef` 0.05, `stepClamp` 0.05,
  `swingHeight` 0.025 (erano 0.2 / 0.15 / 0.1 del quadrupede).
- Rimossi i rami legacy quadrupede (bound, pacing, gallop, trot run).

### `dynamics_SRB.m`
Indici piedi `19:36`, tutti i `[3,4]` -> `[3,nLeg]`, loop momenti `1:nLeg`.
Le equazioni di Newton-Eulero sono **invariate**: cambia solo su quanti piedi si somma.

### `fcn_gen_XdUd.m`
`Xd` 36, `Ud` 18, `p.pf36`, ripartizione peso `Ud(3:3:3*nLeg,ii)`.

### `fcn_FSM.m`
- Esteso a 6 zampe.
- **Schedulazione generica per vettore di fasi**: al posto della catena di
  `if gait ==` c'e' una riga sola, `Ta(i) = t + p.phase(i)*(Tst+Tsw)`.
  Aggiungere un'andatura = aggiungere un `case` in `get_params`.
- `p_hip_b = p.p_hip` (era ricostruito come rettangolo -> sbagliato).
- **Capture point centrato su `p_nom_R = R*p.pf36`**, non sulle anche (vedi bug 3.1).
- Costanti di scala parametrizzate.

### `fcn_get_QP_form_eta.m`
`nU = 18`; `Fzd = Ud([3 6 9 12 15 18],:)`; `Aineq` e `idx_A` con `6*nAineq_unit`;
`Fi = zeros(6*nAineq_unit, 18)`; loop coni d'attrito `1:6`.
Attenzione: `nAineq_unit` vale 6 (vincoli per zampa) e le zampe sono 6 — sono due "6" diversi.

### `fcn_get_ABD_eta.m`
`pf` 3x6; `Cv_u` e `sum_fop` con `repmat(eye(3),[1,6])`; aggiunti `r5`, `r6` in
`Mop` e `Cu`; in `B` i `zeros(3,12)` -> `zeros(3,18)`.

### `MAIN.m`
`Ut = Ut + zval(1:18)`; init con `true(p.nLeg,1)`; **rimossi i due rami `if gait == 1`**
che chiamavano il codice del bound quadrupede; grafici estesi a 6 piedi.

### Visualizzazione — `fig_plot_robot.m`, `fig_plot_robot_d.m`
Riscritti: corpo disegnato come **prisma esagonale** dalle vere posizioni delle
anche; gamba disegnata con la **cinematica PhantomX** (coxa in yaw + femore +
tibia) invece dell'ABAD del quadrupede; 6 frecce GRF.
Non usano piu' `fcn_invKin3` (era tarato su layout rettangolare).

### File rimossi
`fcn_bound_ref_traj.m`, `fcn_FSM_bound.m` — specifici del bound quadrupede,
non piu' chiamati.

---

## 3. Bug trovati (utili per la relazione)

Il porting **non e' stato solo un cambio di indici**. I bug veri erano di scala e
di geometria, e nessuno dava errore a runtime.

### 3.1 Piedi che atterravano sotto le anche
Il capture point era centrato su `p_hip_R`: il robot partiva con la stance
allargata ma al primo swing ogni piede tornava sotto la propria anca e ci restava.
Poligono d'appoggio ristretto, gambe verticali, andatura innaturale — **con il
baricentro perfettamente inseguito**, perche' il modello SRB non sa nulla delle gambe.
Fix: capture point su `p_nom_R = R*p.pf36`.

### 3.2 Costanti di scala del quadrupede
`swingHeight` era 0.1 m: il piede si sarebbe sollevato piu' in alto del corpo
(che sta a 0.08 m). Idem `stepClamp` 0.15 m su un robot lungo 0.25 m.

### 3.3 Parametri fisici misti
`p.p_hip` era gia' PhantomX ma `mass`, `J`, `z0`, `l1`, `l2`, `mu` erano ancora
del quadrupede. In particolare `z0 = 0.2` e' **irraggiungibile**: femore + tibia
fanno 0.141 m. E `mu = 1.4` e' un attrito irrealistico che allarga il cono e
maschera i problemi invece di risolverli.

### 3.4 Layout esagonale vs rettangolare
`p_hip_b` ricostruito da `L`/`W` dava alle zampe anteriori la stessa y delle
centrali. Influenza il foot placement: non e' un problema cosmetico.

---

## 4. Verifiche di correttezza

Da rifare dopo ogni modifica sostanziale.

1. **Dimensionale**: un giro del loop MPC senza errori di dimensione (36/18 ovunque).
2. **Posizione finale dei piedi**:
   `Xout(end,19:36)` meno la x del CoM deve dare le colonne di `p.pf36`.
   Se da' `p.p_hip`, il capture point e' sbagliato.
3. **Pattern del tripode** (Figura 2, Fz per piede): piedi 1, 4, 5 caricano
   insieme e in **antifase** rispetto a 2, 3, 6.
4. **Ripartizione del peso**: all'avvio, con tutte e 6 a terra, ogni Fz vale
   `1.56*9.81/6 = 2.55 N`; in tripode `/3 = 5.1 N`.
5. **Distribuzione del carico**: le Fz anteriori salgono durante l'appoggio, le
   posteriori scendono, le centrali sono piatte. E' il corpo che avanza sopra il
   poligono d'appoggio.
6. **Altezza corpo**: z del CoM piatta a 0.08 m, nessun affondamento.

---

## 5. Cosa resta aperto

**Da confermare (numeri provvisori)**
- `p.J`: attualmente una stima a parallelepipedo. Le inerzie nell'URDF sono
  **inutilizzabili** (stesso blocco copiato su tutti i link, valori ~1000x troppo
  grandi). Prendere l'inerzia aggregata dal model report di Simscape.
- `p.l2` (tibia, 0.075 m): stima, confermare dalla mesh.

**Da testare**
- Andature ripple e wave (implementate, non ancora provate).
  Nota: il wave e' molto piu' lento — abbassare `p.vel_d` di conseguenza.
- Velocita' piu' alte: ora si gira a `vel_d = [0.1;0]`.

**Prossimi blocchi del progetto**
- Simscape: mancano contatto piede-terreno, base flottante, sensing di coppia;
  va rimosso il blocco RL Agent legacy.
- Baseline cinematico del paper (Arrigoni) per il confronto.
- Robustezza: massa del controllore sottostimata del 10% + stimatore
  (e' la richiesta del Progetto 3 della traccia).

---

## 6. Nota di metodo per la relazione

Vale la pena scriverlo esplicitamente: **estendere un MPC da 4 a 6 zampe non e'
un lavoro di indici.** Il modello SRB e la linearizzazione su SO(3) restano
identici — cambia solo su quante forze si somma. Il lavoro vero e' stato:

1. riscalare tutte le costanti di andatura alla taglia del robot;
2. adattare la geometria (layout esagonale, stance raggiungibile);
3. generalizzare la schedulazione dell'andatura per gestire il tripode e le
   altre andature tipiche degli esapodi, che nel quadrupede non esistono.
