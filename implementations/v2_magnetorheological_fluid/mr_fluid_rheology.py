#!/usr/bin/env python3
"""
Magnetorheological (MR) Fluid Rheology and Nitrogen Gas Accumulator Model.
LORD MRF-132DG hydrocarbon-based fluid dynamics coupled with high-pressure N2 gas springs.

Physics Formulations:
1. Bingham-Papanastasiou non-Newtonian shear stress:
     tau(gamma_dot, B) = tau_y(B) * (1 - exp(-m * |gamma_dot|)) + eta * gamma_dot
2. Magnetostatic coil flux density:
     B(I) = B_sat * tanh(k_mag * I)
3. Nitrogen gas accumulator progressive restoring force around static equilibrium:
     F_gas(z) = k_gas * z * (1 - (Ap*z)/V0)^(-gamma)
"""

import numpy as np

class MagnetorheologicalFluid:
    """
    Rheological properties of LORD MRF-132DG Magnetorheological Fluid.
    Carrier fluid: Synthetic hydrocarbon oil.
    Magnetic particles: Carbonyl iron (82 wt%, 32 vol%, 1-5 um diameter).
    """
    def __init__(self, temp_c=25.0):
        self.density = 3090.0        # kg/m^3
        self.temp_k = temp_c + 273.15
        
        # Zero-field base plastic viscosity via Arrhenius law
        # eta(T) = eta_0 * exp(E_a / (R * T))
        self.eta_0 = 0.045           # Pa*s at reference
        self.E_a = 18500.0           # Activation energy J/mol
        self.R_gas = 8.314           # J/(mol*K)
        self.eta = self.eta_0 * np.exp(self.E_a / (self.R_gas * self.temp_k)) # ~0.092 Pa*s at 25C
        
        # Magnetic saturation parameters
        self.B_sat = 1.45            # Tesla (saturation flux density)
        self.k_mag = 1.25            # A^-1 (coil coupling constant)
        
        # Yield stress parameters: tau_y(B) = alpha * B^beta
        self.alpha_mr = 48000.0      # Pa/T^beta
        self.beta_mr = 1.65          # Power law index
        self.papanastasiou_m = 120.0 # Regularization parameter for smooth zero-shear transition

    def compute_yield_stress(self, current_amps):
        """Calculates field-dependent yield shear stress tau_y(B) in Pascals."""
        I = np.clip(current_amps, 0.0, 3.0)
        B = self.B_sat * np.tanh(self.k_mag * I)
        tau_y = self.alpha_mr * (B ** self.beta_mr)
        return float(tau_y), float(B)

    def compute_shear_stress(self, shear_rate, current_amps):
        """
        Computes total shear stress tau using the Bingham-Papanastasiou continuous model.
        Prevents non-physical force discontinuities at zero velocity cross-overs.
        """
        tau_y, B = self.compute_yield_stress(current_amps)
        gamma_dot = float(shear_rate)
        # Papanastasiou exponential regularization
        reg_factor = 1.0 - np.exp(-self.papanastasiou_m * abs(gamma_dot))
        tau = tau_y * np.sign(gamma_dot) * reg_factor + self.eta * gamma_dot
        return float(tau), B


class NitrogenGasAccumulator:
    """
    High-pressure Nitrogen (N2) Gas Hydropneumatic Accumulator.
    Replaces mechanical steel coil springs with progressive gas elasticity.
    Operates dynamically around vehicle weight static equilibrium.
    """
    def __init__(self, k_nominal=950.0, v0_liters=0.8, piston_area_m2=1.25e-3, gamma=1.4):
        self.k_nominal = k_nominal   # Nominal stiffness around equilibrium (N/m)
        self.V0 = v0_liters * 1e-3   # 0.8 L internal volume
        self.Ap = piston_area_m2     # Piston cross-sectional area
        self.gamma = gamma           # Polytropic gas constant

    def compute_restoring_force(self, stroke_displacement_m):
        """
        Calculates progressive restoring force F_gas(z) around equilibrium (N).
        Compression (z > 0) stiffens exponentially; rebound (z < 0) softens progressively.
        """
        z = np.clip(stroke_displacement_m, -0.040, 0.040)
        vol_ratio = 1.0 - (self.Ap * z) / self.V0
        vol_ratio = max(vol_ratio, 0.25)
        force_gas = self.k_nominal * z * (vol_ratio ** (-self.gamma))
        return float(force_gas)
