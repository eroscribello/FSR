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
