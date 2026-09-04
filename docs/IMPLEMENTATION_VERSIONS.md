# Implementation Versions Guide

The **Active Magnetorheological Hydro-Pneumatic (MRHP) Suspension Architecture** provides three tiered implementation targets, structured from embedded microcontroller hardware up to autonomous edge computing.

---

## 1. Version Matrix Overview

| Tier | Target Hardware | Primary Focus | Control Law | Fluid Medium | Cycle Time |
| :--- | :--- | :--- | :--- | :--- | :---: |
| **V1: Embedded Microcontroller** | Raspberry Pi 4 + STM32 FreeRTOS | Distributed Telemetry & Basic Actuation | Semi-Active Skyhook & Classical LQR | Hydro-Pneumatic Fluid | $1.0\text{ ms}$ ($1\text{ kHz}$) |
| **V2: Magnetorheological Damper** | Industrial PWM H-Bridge Driver | Non-Newtonian Rheology & Gas Springs | Continuous Skyhook-MR + Current PI Loop | LORD MRF-132DG + $N_2$ Gas Accumulator | $0.1\text{ ms}$ ($10\text{ kHz}$) |
| **V3: Autonomous Edge Compute** | FPGA / SoC (Zynq, Kria, Jetson) + LiDAR | Cavitation Compensation & Certified Safety | PINN Residual Observer + HOCBF Preview NMPC | Multi-Modal MRHP with Cavitation Modeling | $1.0\text{ ms}$ (Lookahead: $120\text{ ms}$) |

---

## 2. Version 1: Embedded Microcontroller Target (`implementations/v1_embedded_microcontroller/`)

### Architecture
Designed for resource-constrained ground robotics running low-power compute:
* **Host Processor**: Raspberry Pi 4 Model B (Broadcom BCM2711, Quad-core Cortex-A72 @ 1.5 GHz).
* **Hard Real-Time Node**: STM32F401 ARM Cortex-M4 running FreeRTOS.
* **Bus Interconnect**: ISO 11898 CAN 2.0B Bus at $500\text{ kbps}$ with cyclic redundancy checks (CRC-15).
* **Firmware**: [`firmware_stm32_freertos.c`](../implementations/v1_embedded_microcontroller/firmware_stm32_freertos.c) executes a deterministic $1\text{ kHz}$ task monitoring linear potentiometers and modulating PWM output.
* **HAL Driver**: [`can_transceiver_hal.py`](../implementations/v1_embedded_microcontroller/can_transceiver_hal.py) packs and unpacks structured telemetry frames.

### Execution
```bash
python implementations/v1_embedded_microcontroller/main_embedded_runner.py
```

---

## 3. Version 2: Magnetorheological Fluid & Thermodynamics (`implementations/v2_magnetorheological_fluid/`)

### Architecture
High-fidelity industrial mechatronic simulation modeling true non-Newtonian Bingham-Papanastasiou shear stress and nitrogen gas compression:
* **Fluid Physics**: [`mr_fluid_rheology.py`](../implementations/v2_magnetorheological_fluid/mr_fluid_rheology.py) models LORD MRF-132DG yield stress ($\tau_y = \alpha B^\beta$) and Arrhenius temperature viscosity.
* **Accumulator**: Polytropic gas compression ($P V^{1.4} = \text{const}$) replacing mechanical steel springs.
* **Driver**: [`mr_damper_hardware_driver.py`](../implementations/v2_magnetorheological_fluid/mr_damper_hardware_driver.py) executes a $10\text{ kHz}$ closed-loop current PI regulator over the coil RL circuit ($12.5\text{ mH}, 3.2\,\Omega$).

### Execution
```bash
python implementations/v2_magnetorheological_fluid/mr_suspension_benchmark.py
```

---

## 4. Version 3: Autonomous Edge PINN-HOCBF Preview NMPC (`implementations/v3_edge_pinn_preview_nmpc/`)

### Architecture
Next-generation autonomous vehicle terrain navigation coupling deep learning observers with certified control barrier functions:
* **Forward LiDAR Preview**: [`lidar_road_surface_profiler.py`](../implementations/v3_edge_pinn_preview_nmpc/lidar_road_surface_profiler.py) extracts forward road height profiles over a $120\text{ ms}$ lookahead horizon.
* **PINN Fluid Observer**: [`pinn_fluid_observer.py`](../implementations/v3_edge_pinn_preview_nmpc/pinn_fluid_observer.py) predicts unmodeled fluid cavitation and bulk modulus variation while enforcing Navier-Stokes continuity.
* **HOCBF Safety Filter**: [`differentiable_hocbf_nmpc.py`](../implementations/v3_edge_pinn_preview_nmpc/differentiable_hocbf_nmpc.py) solves an instantaneous Quadratic Program guaranteeing forward invariance of suspension rattlespace stroke ($|z_s - z_{us}| \le 38.0\text{ mm}$) and continuous tire-ground grip.

### Execution
```bash
python implementations/v3_edge_pinn_preview_nmpc/differentiable_hocbf_nmpc.py
```
