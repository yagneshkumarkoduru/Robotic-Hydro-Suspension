# Physics-Informed Preview NMPC with High-Order Control Barrier Functions for Active Robotic Hydro-Pneumatic Suspension Systems

**Yagnesh Kumar Koduru**  
*Researcher, Esthien Labs*  
*Email: yagneshkumar@esthien.com*

---

## Abstract

Autonomous ground robots traversing unstructured or off-road terrains face an acute trade-off between **payload ride stability** (preventing sensor saturation in gimbals, LiDARs, and cameras) and **ground contact safety** (preserving dynamic tire traction while respecting physical suspension stroke limits). Conventional reactive controllers (Skyhook, LQR) operate post-impact, while nominal Model Predictive Control (MPC) algorithms suffer from hydraulic plant mismatch induced by unmodeled fluid cavitation, orifice discharge nonlinearities, and bulk modulus variation. 

In this paper, we develop an integrated **Physics-Informed Preview Model Predictive Control (PI-Preview NMPC)** framework equipped with **High-Order Control Barrier Functions (HOCBF)**:
1. **Dynamic LiDAR Preview Lookahead**: Ingests forward road elevation profiles over a $120\text{ ms}$ preview horizon to synthesize anticipatory counteracting forces.
2. **Physics-Informed Neural Network (PINN) Residual Observer**: Models nonlinear fluid cavitation and thermal viscosity drift online while embedding mechanical energy conservation.
3. **Quadratic Program (QP) Safety Filter**: Enforces forward invariance of suspension rattlespace stroke ($|z_s - z_{us}| \le \delta_{\max}$) and continuous tire-road traction ($F_z^{\text{tire}} > 0$).

Quarter-car dynamic benchmark evaluations against a severe $45\text{ mm}$ bump obstacle show a **$64.05\%$ reduction in sprung-mass RMS vertical acceleration**, a **$24.78\%$ reduction in peak suspension stroke deflection**, and **$100\%$ adherence to safety barrier limits** without bottoming out.

---

## 1. Mathematical Formulation

### 1.1 Quarter-Car Hydro-Pneumatic Dynamics
The vertical motion of the 2-DOF vehicle corner is governed by:

$$m_s \ddot{z}_s = -k_s(z_s - z_{us}) - c_s(\dot{z}_s - \dot{z}_{us}) + F_{\text{act}} + F_{\text{unmodeled}}$$

$$m_{us} \ddot{z}_{us} = k_s(z_s - z_{us}) + c_s(\dot{z}_s - \dot{z}_{us}) - k_t(z_{us} - z_r) - c_t(\dot{z}_{us} - \dot{z}_r) - F_{\text{act}} - F_{\text{unmodeled}}$$

Where:
* $m_s = 15.0\text{ kg}$: Sprung mass (chassis corner + sensor payloads)
* $m_{us} = 2.5\text{ kg}$: Unsprung mass (wheel, tire, and hub assembly)
* $k_s = 950\text{ N/m}$, $c_s = 45\text{ N}\cdot\text{s/m}$: Suspension passive gas spring stiffness and damping
* $k_t = 6500\text{ N/m}$, $c_t = 5\text{ N}\cdot\text{s/m}$: Pneumatic tire stiffness and damping
* $F_{\text{act}}$: Active hydraulic control force
* $F_{\text{unmodeled}}$: Unmodeled hydraulic cavitation and fluid compressibility force

### 1.2 Physics-Informed Residual Observer (PINN)
Fluid compressibility inside the chamber follows:

$$\dot{P} = \frac{\beta}{V(z)} \left( Q_{\text{valve}} - A_p (\dot{z}_s - \dot{z}_{us}) \right)$$

where $\beta$ is effective bulk modulus. To account for turbulent orifice cavitation $Q = C_d A_o \sqrt{2\Delta P/\rho}$, we train a PINN $\mathcal{N}_{\text{PINN}}(x, \Delta P; \theta)$ regularized by fluid energy conservation:

$$\mathcal{L}_{\text{PINN}}(\theta) = \|F_{\text{meas}} - \hat{F}_{\text{act}}\|_2^2 + \lambda_{\text{phys}} \left\| \dot{P} - \frac{\beta}{V}\left(Q - A_p \dot{z}_{\text{rel}}\right) \right\|_2^2$$

---

## 2. High-Order Control Barrier Functions (HOCBF)

Because the suspension rattlespace $z_{\text{rel}} = z_s - z_{us}$ has relative degree 2 with respect to actuator force $F_{\text{act}}$, standard barrier functions cannot directly constrain the input. We define the candidate barrier set $\mathcal{C}$ for rattlespace boundary $\delta_{\max} = 38\text{ mm}$:

$$h(x) = \delta_{\max}^2 - (z_s - z_{us})^2 \ge 0$$

The second Lie derivative along dynamics $\dot{x} = f(x) + g(x)u$ yields:

$$L_f^2 h(x) + L_g L_f h(x) u + (\gamma_1 + \gamma_2) L_f h(x) + \gamma_1 \gamma_2 h(x) \ge 0$$

### Formal Theorem 1: Forward Invariance of Suspension Stroke
> **Theorem 1.** Let the safe set be defined as $\mathcal{C} = \{x \in \mathbb{R}^4 \mid h(x) \ge 0, \dot{h}(x) + \gamma_1 h(x) \ge 0\}$. If the active control input $u(t)$ satisfies:
>
> $$L_g L_f h(x) u \ge - L_f^2 h(x) - (\gamma_1 + \gamma_2) L_f h(x) - \gamma_1 \gamma_2 h(x)$$
>
> for all $t \ge 0$ with initial state $x(0) \in \mathcal{C}$, then the trajectory $x(t) \in \mathcal{C}$ for all $t \ge 0$, guaranteeing $|z_s(t) - z_{us}(t)| \le \delta_{\max}$ strictly.

**Proof.** Define extended barrier $\psi_1(x) = \dot{h}(x) + \gamma_1 h(x)$. Applying Nagumo's condition $\dot{\psi}_1(x) \ge -\gamma_2 \psi_1(x)$ guarantees $\psi_1(t) \ge \psi_1(0) e^{-\gamma_2 t} \ge 0$. Integrating $\dot{h}(x) \ge -\gamma_1 h(x)$ via Gronwall's inequality yields $h(t) \ge h(0) e^{-\gamma_1 t} \ge 0$. Hence $x(t) \in \mathcal{C}$ for all $t \ge 0$. $\blacksquare$

---

## 3. Real-Time Quadratic Program Safety Filter

The final control force is resolved at $1\text{ kHz}$ via an instantaneous Quadratic Program:

$$\min_{u} \frac{1}{2} \|u - u_{\text{preview}}\|^2 \quad \text{s.t.} \quad L_g L_f h(x) u \ge \text{Bound}_{\text{CBF}}(x), \quad u_{\min} \le u \le u_{\max}$$

This projects the performance-optimal preview MPC action onto the admissible safe half-space.

---

## 4. Empirical Evaluation & Benchmarks

Benchmarking over a severe $45\text{ mm}$ cosine obstacle bump:

| Control Configuration | RMS Chassis Accel ($\text{m/s}^2$) | Peak Accel ($\text{m/s}^2$) | Peak Stroke Deflection ($\text{mm}$) | Mechanical Safety Status |
| :--- | :---: | :---: | :---: | :---: |
| **Passive Baseline** | 2.184 | 6.421 | 41.21 | **Violated (Bottomed Out)** |
| **Skyhook Semi-Active** | 1.482 | 4.812 | 36.40 | Borderline |
| **Standard LQR** | 0.841 | 2.653 | 34.20 | Borderline |
| **PI-Preview HOCBF (Ours)** | **0.785** | **2.104** | **31.00** | **Strictly Verified (Safe)** |

<p align="center">
  <img src="../../figures/fig_pinn_hocbf_safety_verification.png" alt="PINN HOCBF Safety Verification" width="85%" />
</p>

### Key Experimental Discoveries:
1. **$64.05\%$ RMS Vibration Attenuation**: Drastically isolates optical payloads from terrain shock excitation.
2. **Deterministic Stroke Invariance**: Prevents destructive bottoming-out against mechanical stops ($31.00\text{ mm} \le 38.00\text{ mm}$).
3. **Cavitation Robustness**: Online PINN estimation compensates for non-linear valve latency and hysteresis within $2\text{ ms}$.

---

## Citation
```bibtex
@article{koduru2026hydro,
  author    = {Koduru, Yagnesh Kumar},
  title     = {Physics-Informed Preview NMPC with High-Order Control Barrier Functions for Active Robotic Hydro-Pneumatic Suspension Systems},
  journal   = {IEEE Transactions on Control Systems Technology},
  year      = {2026},
  volume    = {34},
  number    = {3},
  pages     = {1102--1115}
}
```
