# Fluid Rheology, Electromagnetics & Thermodynamics

This document provides the first-principles mathematical and physical derivations underlying the **Active Magnetorheological Hydro-Pneumatic (MRHP)** suspension system.

---

## 1. Magnetorheological Fluid Rheology (LORD MRF-132DG)

The active fluid medium is formulated using **LORD MRF-132DG**, a hydrocarbon-based synthetic carrier oil containing suspended spherical carbonyl iron particles ($82\text{ wt}\%$, $32\text{ vol}\%$, particle diameter $1\text{--}5\,\mu\text{m}$).

### 1.1 Field-Dependent Yield Stress
In the absence of an external magnetic field ($H = 0$), the fluid behaves as a Newtonian liquid with base plastic viscosity $\eta$. Upon applying a magnetic field $B$, the magnetic dipoles of the iron particles align into columnar chain fibril structures parallel to the magnetic flux lines within $< 1.0\text{ ms}$, generating a yield shear stress $\tau_y(B)$:

$$\tau_y(B) = \alpha \cdot B^\beta$$

Where:
* $\alpha = 48{,}000\text{ Pa/T}^\beta$: Magnetorheological yield coefficient
* $\beta = 1.65$: Magnetic saturation power law exponent
* $B$: Magnetic flux density in the annular fluid gap (Tesla)

### 1.2 Continuous Bingham-Papanastasiou Non-Newtonian Model
Standard ideal Bingham plastic formulations introduce a discontinuous step function $\operatorname{sgn}(\dot{\gamma})$ at zero shear rate, which causes numerical instability and high-frequency force chattering in dynamic solvers. We employ the **Bingham-Papanastasiou regularization model**:

$$\tau(\dot{\gamma}, B) = \tau_y(B) \cdot \operatorname{sgn}(\dot{\gamma}) \left[ 1 - \exp\left(-m |\dot{\gamma}|\right) \right] + \eta(T) \cdot \dot{\gamma}$$

Where:
* $\dot{\gamma} = \frac{v_{\text{rel}}}{h_{\text{gap}}}$: Shear rate in the annular damper orifice ($s^{-1}$)
* $h_{\text{gap}} = 1.0\text{ mm}$: Damper annular flow gap clearance
* $m = 120.0$: Regularization parameter providing smooth, differentiable zero-velocity crossover
* $\eta(T)$: Temperature-dependent carrier fluid viscosity

### 1.3 Temperature-Dependent Arrhenius Viscosity
Fluid temperature shifts during aggressive off-road excitation alter base viscosity according to the Arrhenius relationship:

$$\eta(T) = \eta_0 \cdot \exp\left( \frac{E_a}{R \cdot T} \right)$$

Where:
* $\eta_0 = 0.045\text{ Pa}\cdot\text{s}$: Asymptotic infinite-temperature viscosity
* $E_a = 18{,}500\text{ J/mol}$: Activation energy for viscous flow
* $R = 8.314\text{ J/(mol}\cdot\text{K)}$: Universal gas constant
* $T$: Absolute fluid temperature in Kelvin ($T \in [253\text{ K}, 373\text{ K}]$)

---

## 2. Electromagnetics & Coil Dynamics

The magnetic flux density $B$ inside the damper piston core is governed by coil current $I(t)$ and core magnetic saturation:

$$B(I) = B_{\text{sat}} \cdot \tanh\left( k_{\text{mag}} \cdot I(t) \right)$$

Where:
* $B_{\text{sat}} = 1.45\text{ T}$: Piston pole core saturation flux density
* $k_{\text{mag}} = 1.25\text{ A}^{-1}$: Coil magnetic coupling constant
* $I(t) \in [0.0, 2.5]\text{ A}$: Controlled electromagnetic coil current

The electrical driver operates at $10\text{ kHz}$ closed-loop current regulation over the coil RL circuit:

$$V_{\text{bus}}(t) = L \frac{dI}{dt} + R \cdot I(t) + \mathcal{E}_{\text{back-EMF}}$$

with $L = 12.5\text{ mH}$, $R = 3.2\,\Omega$, and $V_{\text{bus}} = 24.0\text{ V}$.

---

## 3. High-Pressure Nitrogen ($N_2$) Gas Hydropneumatic Accumulator

The mechanical steel spring is replaced by a high-pressure **Nitrogen ($N_2$) Hydropneumatic Gas Accumulator** ($P_0 = 3.2\text{ MPa}$, $V_0 = 0.8\text{ L}$).

Under adiabatic/polytropic compression ($P \cdot V^\gamma = \text{constant}$, $\gamma = 1.4$), the dynamic restoring force $F_{\text{gas}}(z_{\text{rel}})$ around static equilibrium is:

$$F_{\text{gas}}(z_{\text{rel}}) = k_{\text{nominal}} \cdot z_{\text{rel}} \cdot \left( 1 - \frac{A_p \cdot z_{\text{rel}}}{V_0} \right)^{-\gamma}$$

Where:
* $z_{\text{rel}} = z_s - z_{us}$: Suspension relative stroke displacement ($m$)
* $A_p = 1.25 \times 10^{-3}\text{ m}^2$: Actuator hydraulic piston area
* $k_{\text{nominal}} = 950.0\text{ N/m}$: Baseline gas elasticity around curb weight equilibrium

Dynamic stiffness increases non-linearly during compression ($z > 0$), naturally resisting rattlespace bottoming out against chassis bump stops.

---

## 4. Annular Duct Damping Force Synthesis

The total controllable damping force synthesized by the MR damper piston traversing the fluid chamber is:

$$F_{\text{damper}} = 3 \left( \frac{L_{\text{duct}}}{h_{\text{gap}}} \right) A_p \cdot \tau_y(B) \cdot \tanh\left(\frac{v_{\text{rel}}}{v_{\text{reg}}}\right) + C_{\text{viscous}} \cdot v_{\text{rel}}$$

Where:
* $L_{\text{duct}} = 25.0\text{ mm}$: Active magnetic pole length
* $h_{\text{gap}} = 1.0\text{ mm}$: Annular clearance gap
* $A_p = 1.02 \times 10^{-3}\text{ m}^2$: Effective piston cross-sectional area
* $C_{\text{viscous}} = 45.0\text{ N}\cdot\text{s/m}$: Newtonian viscous drag
