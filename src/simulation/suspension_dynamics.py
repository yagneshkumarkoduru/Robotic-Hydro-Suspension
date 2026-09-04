"""
Active Hydro-Pneumatic Suspension Dynamics & Optimal Control Simulation
Author: Yagnesh Kumar Koduru
Domain: Mechatronic Systems, Active Damping, Physical Intelligence

This module models a 2-DOF quarter-car hydro-pneumatic suspension system,
comparing passive damping against Skyhook semi-active control and full-state LQR
active force actuation under deterministic bump and stochastic ISO road profiles.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.signal import welch
from scipy.linalg import solve_continuous_are

# Set publication style
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 11
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['xtick.labelsize'] = 9
plt.rcParams['ytick.labelsize'] = 9
plt.rcParams['legend.fontsize'] = 9
plt.rcParams['lines.linewidth'] = 2.0
plt.rcParams['axes.grid'] = True
plt.rcParams['grid.alpha'] = 0.35


class SuspensionSystem:
    def __init__(self, ms=15.0, mus=2.5, ks=950.0, cs=45.0, kt=6500.0, ct=5.0):
        """
        Quarter-car suspension parameters (Robotic platform scale):
        - ms: Sprung mass (chassis quarter, kg)
        - mus: Unsprung mass (wheel/linkage assembly, kg)
        - ks: Suspension spring stiffness (N/m)
        - cs: Base hydraulic passive damping (N*s/m)
        - kt: Tire vertical stiffness (N/m)
        - ct: Tire damping coefficient (N*s/m)
        """
        self.ms = ms
        self.mus = mus
        self.ks = ks
        self.cs = cs
        self.kt = kt
        self.ct = ct
        
        # Skyhook damping coefficient
        self.c_sky = 120.0  # N*s/m
        
        # Compute LQR gain matrix K
        self.K_lqr = self._compute_lqr_gain()

    def _compute_lqr_gain(self):
        """
        State space representation for 2-DOF Quarter Car:
        States x = [z_s - z_us,  dot{z}_s,  z_us - z_r,  dot{z}_us]^T
        Input u = F_act (active hydraulic force)
        """
        # Linearized state-space matrices
        A = np.array([
            [0,                  1,              0,                 -1],
            [-self.ks/self.ms,  -self.cs/self.ms, 0,          self.cs/self.ms],
            [0,                  0,              0,                  1],
            [self.ks/self.mus,   self.cs/self.mus, -self.kt/self.mus, -(self.cs + self.ct)/self.mus]
        ])
        
        B = np.array([
            [0],
            [1.0 / self.ms],
            [0],
            [-1.0 / self.mus]
        ])
        
        # Performance weights: emphasize body velocity damping (comfort) while managing rattlespace
        Q = np.diag([10.0, 1000.0, 10.0, 1.0])
        R = np.array([[0.001]])
        
        # Solve continuous Algebraic Riccati Equation
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ (B.T @ P)
        return K

    def road_bump(self, t, height=0.04, duration=0.15, start_time=0.2):
        """Single cosine bump road profile."""
        if start_time <= t <= start_time + duration:
            phase = 2.0 * np.pi * (t - start_time) / duration
            z_r = 0.5 * height * (1.0 - np.cos(phase))
            dz_r = 0.5 * height * (2.0 * np.pi / duration) * np.sin(phase)
        else:
            z_r = 0.0
            dz_r = 0.0
        return z_r, dz_r

    def dynamics(self, t, state, control_type='passive', road_func=None):
        """
        System state vector:
        x = [z_s,  v_s,  z_us,  v_us]
        """
        z_s, v_s, z_us, v_us = state
        
        if road_func is None:
            road_func = self.road_bump
        z_r, dz_r = road_func(t)
        
        # Relative kinematics
        susp_deflection = z_s - z_us
        susp_velocity = v_s - v_us
        tire_deflection = z_us - z_r
        tire_velocity = v_us - dz_r
        
        # Passive forces
        F_spring = self.ks * susp_deflection
        F_damper = self.cs * susp_velocity
        F_tire = self.kt * tire_deflection + self.ct * tire_velocity
        
        # Active actuator force computation
        if control_type == 'passive':
            F_act = 0.0
        elif control_type == 'skyhook':
            # Semi-active Skyhook damping logic: opposes sprung mass absolute velocity
            if v_s * susp_velocity >= 0.0:
                F_act = -self.c_sky * v_s
            else:
                F_act = 0.0
        elif control_type == 'lqr':
            x_lqr = np.array([susp_deflection, v_s, tire_deflection, v_us])
            F_act = -float(np.squeeze(self.K_lqr @ x_lqr))
            F_act = np.clip(F_act, -120.0, 120.0)
        else:
            raise ValueError(f"Unknown control type: {control_type}")
            
        a_s = (-F_spring - F_damper + F_act) / self.ms
        a_us = (F_spring + F_damper - F_tire - F_act) / self.mus
        
        return [v_s, a_s, v_us, a_us]


def run_simulation():
    print("=" * 75)
    print("ACTIVE HYDRO-PNEUMATIC SUSPENSION CONTROL BENCHMARK")
    print("Author: Yagnesh Kumar Koduru")
    print("=" * 75)

    system = SuspensionSystem()
    t_span = (0.0, 1.5)
    t_eval = np.linspace(t_span[0], t_span[1], 1500)
    initial_state = [0.0, 0.0, 0.0, 0.0]

    controllers = ['passive', 'skyhook', 'lqr']
    results = {}

    for ctl in controllers:
        print(f"Simulating [{ctl.upper()}] configuration...")
        sol = solve_ivp(
            fun=lambda t, y: system.dynamics(t, y, control_type=ctl),
            t_span=t_span,
            y0=initial_state,
            t_eval=t_eval,
            method='RK45',
            rtol=1e-6,
            atol=1e-8
        )
        
        z_s, v_s, z_us, v_us = sol.y
        a_s = np.zeros_like(t_eval)
        
        for idx, t_val in enumerate(t_eval):
            derivs = system.dynamics(t_val, [z_s[idx], v_s[idx], z_us[idx], v_us[idx]], control_type=ctl)
            a_s[idx] = derivs[1]
            
        z_r_vec = np.array([system.road_bump(t)[0] for t in t_eval])
        susp_defl = z_s - z_us
        tire_defl = z_us - z_r_vec
        
        rms_accel = float(np.sqrt(np.mean(a_s**2)))
        peak_accel = float(np.max(np.abs(a_s)))
        rms_defl = float(np.sqrt(np.mean(susp_defl**2)))
        peak_defl = float(np.max(np.abs(susp_defl)))
        
        results[ctl] = {
            't': t_eval,
            'z_s': z_s,
            'a_s': a_s,
            'z_r': z_r_vec,
            'susp_defl': susp_defl,
            'tire_defl': tire_defl,
            'rms_accel': rms_accel,
            'peak_accel': peak_accel,
            'rms_defl': rms_defl,
            'peak_defl': peak_defl,
        }

    print("\n" + "-" * 75)
    print(f"{'Control Strategy':<16} | {'RMS Accel (m/s²)':<18} | {'Peak Accel (m/s²)':<18} | {'Peak Defl (mm)':<15}")
    print("-" * 75)
    for ctl in controllers:
        res = results[ctl]
        print(f"{ctl.capitalize():<16} | {res['rms_accel']:<18.4f} | {res['peak_accel']:<18.4f} | {res['peak_defl']*1000:<15.2f}")
    print("-" * 75)

    p_rms = results['passive']['rms_accel']
    s_rms = results['skyhook']['rms_accel']
    l_rms = results['lqr']['rms_accel']
    print(f"Skyhook Ride Comfort Gain (RMS Accel Reduction): {(1.0 - s_rms/p_rms)*100:.2f}%")
    print(f"LQR Active Ride Comfort Gain (RMS Accel Reduction): {(1.0 - l_rms/p_rms)*100:.2f}%")

    output_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'figures')
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Plot 1: Transient Response & Body Acceleration
    fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(8.5, 6.0), sharex=True)
    ax1.plot(t_eval, results['passive']['z_r'] * 1000, 'k--', label='Road Profile (Bump)', alpha=0.6)
    ax1.plot(t_eval, results['passive']['z_s'] * 1000, '#C0392B', label='Passive Suspension', alpha=0.85)
    ax1.plot(t_eval, results['skyhook']['z_s'] * 1000, '#2980B9', label='Skyhook Semi-Active', alpha=0.85)
    ax1.plot(t_eval, results['lqr']['z_s'] * 1000, '#27AE60', label='LQR Active Hydro-Pneumatic', linewidth=2.4)
    ax1.set_ylabel('Chassis Height $z_s$ (mm)', fontweight='bold')
    ax1.set_title('Transient Response: 40mm Road Bump Ingestion', fontweight='bold', pad=12)
    ax1.legend(loc='upper right', framealpha=0.95)

    ax2.plot(t_eval, results['passive']['a_s'], '#C0392B', label='Passive', alpha=0.8)
    ax2.plot(t_eval, results['skyhook']['a_s'], '#2980B9', label='Skyhook', alpha=0.8)
    ax2.plot(t_eval, results['lqr']['a_s'], '#27AE60', label='LQR Active', linewidth=2.4)
    ax2.set_xlabel('Time (s)', fontweight='bold')
    ax2.set_ylabel('Body Accel $\\ddot{z}_s$ (m/s²)', fontweight='bold')
    ax2.set_title('Sprung Mass Vertical Acceleration (Ride Comfort Metric)', fontweight='bold', pad=10)
    ax2.legend(loc='upper right', framealpha=0.95)
    plt.tight_layout()
    fig1.savefig(os.path.join(output_dir, 'fig1_bump_response_comparison.png'), dpi=300)
    plt.close(fig1)

    # Plot 2: Suspension Deflection (Rattlespace) & Tire Grip
    fig2, (ax3, ax4) = plt.subplots(2, 1, figsize=(8.5, 6.0), sharex=True)
    ax3.plot(t_eval, results['passive']['susp_defl'] * 1000, '#C0392B', label='Passive')
    ax3.plot(t_eval, results['skyhook']['susp_defl'] * 1000, '#2980B9', label='Skyhook')
    ax3.plot(t_eval, results['lqr']['susp_defl'] * 1000, '#27AE60', label='LQR Active', linewidth=2.4)
    ax3.set_ylabel('Suspension Deflection (mm)', fontweight='bold')
    ax3.set_title('Rattlespace Utilization: $(z_s - z_{us})$', fontweight='bold', pad=10)
    ax3.legend(loc='upper right', framealpha=0.95)

    ax4.plot(t_eval, results['passive']['tire_defl'] * 1000, '#C0392B', label='Passive')
    ax4.plot(t_eval, results['skyhook']['tire_defl'] * 1000, '#2980B9', label='Skyhook')
    ax4.plot(t_eval, results['lqr']['tire_defl'] * 1000, '#27AE60', label='LQR Active', linewidth=2.4)
    ax4.set_xlabel('Time (s)', fontweight='bold')
    ax4.set_ylabel('Tire Deflection (mm)', fontweight='bold')
    ax4.set_title('Dynamic Tire Deflection (Road Holding & Grip): $(z_{us} - z_r)$', fontweight='bold', pad=10)
    ax4.legend(loc='upper right', framealpha=0.95)
    plt.tight_layout()
    fig2.savefig(os.path.join(output_dir, 'fig2_suspension_deflection_tradeoff.png'), dpi=300)
    plt.close(fig2)

    print(f"Generated publication figures saved to: {os.path.abspath(output_dir)}")


if __name__ == '__main__':
    run_simulation()
