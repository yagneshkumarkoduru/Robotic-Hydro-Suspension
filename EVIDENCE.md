# Robotic-Hydro-Suspension Evidence Ledger

Version: 2026-11-13
Status: living ledger. Every number in README, docs, or the manuscripts
must map to a record here. A number without a record does not exist.

Classes: **SIMULATION** (digital model of the quarter-car plant and its
controllers, no physical hardware), **MODEL** (analytical or rheological
model evaluation), **SOFTWARE MEASURED** (test-suite outcomes),
**NOT ESTABLISHED** (claimed numbers with no reproducible artifact).

---

## E01: Test suite (SOFTWARE MEASURED)

Command: `.venv\Scripts\python.exe -m pytest tests/ -q`
Result (2026-11-13): `44 passed`.
Coverage: sensor mocks (IMU, ToF, CAN), actuator physics (MR damper,
proportional valve, servo motor), suspension dynamics integration, and
vehicle control safety cutoffs.

## E02: Quarter-car benchmark harness (SIMULATION)

Command: `python suspension_benchmark.py`
Artifact: `results/suspension_benchmark.json` (committed).
Plant: 2-DOF quarter-car, ms = 15.0 kg, mus = 2.5 kg, ks = 950 N/m,
cs = 45 N s/m, kt = 6500 N/m, ct = 5 N s/m, stroke limit +/- 38 mm,
actuator force limit +/- 400 N. Tuning: grid search per controller class
on the 45 mm bump scenario, objective = RMS body acceleration + 5 x
stroke-excess (mm); gains cached in `results/tuned_gains.json`.

### 45 mm cosine bump (t_end 2.5 s, dt 0.2 ms)

| Controller | RMS accel (m/s2) | Peak accel (m/s2) | Peak stroke (mm) | Violations | RMS force (N) |
|---|---|---|---|---|---|
| Passive | 1.049 | 4.093 | 44.78 | 258 | 0.0 |
| Skyhook | 0.718 | 3.342 | 35.37 | 0 | 8.5 |
| LQR | 0.268 | 1.321 | 48.60 | 387 | 10.7 |
| Preview LQR | 1.038 | 4.885 | 34.26 | 0 | 24.4 |
| HOCBF-LQR | 0.586 | 5.535 | 37.95 | 0 | 12.8 |

Derived reductions vs passive (SIMULATION): LQR 74.4 %, HOCBF-LQR 44.1 %,
Skyhook 31.6 %, Preview LQR 1.0 %.

### ISO 8608 class-C random road (5 m/s, seed 20261011)

| Controller | RMS accel (m/s2) | Peak stroke (mm) | Violations |
|---|---|---|---|
| Passive | 0.449 | 13.51 | 0 |
| Skyhook | 0.482 | 12.48 | 0 |
| LQR | 0.270 | 21.85 | 0 |
| Preview LQR | 0.630 | 12.21 | 0 |
| HOCBF-LQR | 0.270 | 21.85 | 0 |

On class C the barrier filter never activates (LQR = HOCBF-LQR).

### Monte Carlo (20 samples, +/-15 % on ms, ks, cs, kt; bump)

| Controller | RMS accel mean +/- std (m/s2) | Peak stroke mean (mm) |
|---|---|---|
| Passive | 1.064 +/- 0.107 | 44.81 |
| Skyhook | 0.714 +/- 0.056 | 35.38 |
| LQR | 0.266 +/- 0.012 | 48.22 |
| Preview LQR | 1.038 +/- 0.016 | 33.71 |
| HOCBF-LQR | 0.580 +/- 0.031 | 37.95 |

## E03: MR damper benchmark (SIMULATION + MODEL)

Command: `python -m implementations.v2_magnetorheological_fluid.mr_suspension_benchmark`
Artifact: `results/mr_suspension_benchmark.json` (committed).
MR damper: Bingham-Papanastasiou yield stress model (MODEL class) with
LORD MRF-132DG parameters (alpha = 48 kPa/T^1.65, B_sat = 1.45 T);
skyhook-equivalent current law via inverse yield-stress map; effective
flow-amplification area 6.0e-3 m2 sized for ~300 N peak yield force.

45 mm bump results (SIMULATION): RMS accel 0.801 m/s2 (23.6 % reduction
vs passive 1.049 m/s2), peak stroke 33.82 mm, 0 violations, peak yield
stress 7.3 kPa, peak yield force 44 N.
Coil driver 10-90 % rise time: 1.20 ms measured from the RL + PI current
loop model (L = 12.5 mH, R = 3.2 ohm, V_bus = 24 V) stepped at 0.2 ms
(SIMULATION of the driver model, not bench measurement).

Note: the 1 kHz current loop in the driver model is the validated
configuration; the "10 kHz PWM" wording in older documentation refers to
the STM32 hardware timer capability and is not what the model steps.

## E04: Extended benchmark matrix (SIMULATION)

Command: `python extended_benchmark.py`
Artifact: `results/extended_benchmark.json` (committed).
All six controllers over six deterministic road profiles with identical
plant, gains, and metric definitions: the 45 mm cosine bump plus ISO 8608
classes A (smooth), B (good rural), C (urban), D (poor urban) and E
(off-road); PSD band RMS of body acceleration per ISO 2631-relevant bands
(0.5-2, 2-8, 8-80 Hz).

### Peak stroke (mm) and violation counts

| Scenario | Passive | Skyhook | LQR | Preview LQR | HOCBF-LQR | MR skyhook |
|---|---|---|---|---|---|---|
| Bump 45 mm | 44.78 / 258 | 35.37 / 0 | 48.60 / 387 | 34.26 / 0 | 37.95 / 0 | 33.82 / 0 |
| ISO A smooth, 5 m/s | 3.38 / 0 | 3.12 / 0 | 5.46 / 0 | 3.05 / 0 | 5.46 / 0 | 2.70 / 0 |
| ISO B good-rural, 5 m/s | 6.75 / 0 | 6.24 / 0 | 10.93 / 0 | 6.10 / 0 | 10.93 / 0 | 5.91 / 0 |
| ISO C urban, 5 m/s | 13.51 / 0 | 12.48 / 0 | 21.85 / 0 | 12.21 / 0 | 21.85 / 0 | 11.49 / 0 |
| ISO D poor-urban, 10 m/s | 52.53 / 442 | 42.07 / 332 | 89.70 / 2458 | 44.41 / 249 | 38.05 / 274 | 37.96 / 0 |
| ISO E off-road, 10 m/s | 105.05 / 3164 | 84.13 / 3314 | 179.39 / 6065 | 91.62 / 2269 | 38.06 / 292 | 79.75 / 3052 |

### RMS acceleration (m/s2)

| Scenario | Passive | Skyhook | LQR | Preview LQR | HOCBF-LQR | MR skyhook |
|---|---|---|---|---|---|---|
| Bump 45 mm | 1.049 | 0.718 | 0.268 | 1.038 | 0.586 | 0.801 |
| ISO A smooth | 0.112 | 0.120 | 0.068 | 0.157 | 0.068 | 0.192 |
| ISO B good-rural | 0.224 | 0.241 | 0.135 | 0.315 | 0.135 | 0.345 |
| ISO C urban | 0.449 | 0.482 | 0.270 | 0.630 | 0.270 | 0.606 |
| ISO D poor-urban | 1.607 | 1.554 | 0.863 | 2.094 | 2.251 | 1.675 |
| ISO E off-road | 3.214 | 3.108 | 1.725 | 4.907 | 6.230 | 3.229 |

### Acceleration PSD band RMS, 2-8 Hz (m/s2)

| Scenario | Passive | Skyhook | LQR | Preview LQR | HOCBF-LQR | MR skyhook |
|---|---|---|---|---|---|---|
| Bump 45 mm | 0.323 | 0.355 | 0.133 | 0.557 | 0.297 | 0.401 |
| ISO A smooth | 0.050 | 0.062 | 0.031 | 0.070 | 0.031 | 0.100 |
| ISO B good-rural | 0.101 | 0.125 | 0.061 | 0.139 | 0.061 | 0.180 |
| ISO C urban | 0.201 | 0.249 | 0.122 | 0.279 | 0.122 | 0.313 |
| ISO D poor-urban | 0.544 | 0.707 | 0.330 | 0.764 | 1.118 | 0.771 |
| ISO E off-road | 1.088 | 1.414 | 0.660 | 2.143 | 3.135 | 1.479 |

Measured findings (SIMULATION class):
1. LQR is comfort-optimal on every profile but violates the stroke limit
   whenever the road excites the rattlespace; it is not deployable as-is.
2. On the mild profiles (ISO A-C) the barrier filter never activates
   (LQR = HOCBF-LQR) and all controllers keep stroke well inside the
   38 mm limit.
3. HOCBF-LQR caps peak stroke at the 38 mm boundary on the severe
   profiles (37.95-38.06 mm). On ISO D/E the filter is active for long
   intervals, which raises RMS acceleration above passive; the trade is
   stroke safety, not comfort, on severe roads.
4. The MR skyhook damper is the only configuration with zero violations
   on ISO D while staying near passive comfort (1.675 vs 1.607 m/s2).
5. Residual violation counts on HOCBF-LQR under ISO D/E (274 / 292
   samples) are single-step boundary overshoots of the discrete-time
   filter at dt = 0.2 ms; peak stroke stays within 0.2 % of the limit.

## E05: PINN residual observer step test (MODEL)

Artifact: `implementations/v3_edge_pinn_preview_nmpc/pinn_fluid_observer.py`
(`self_check()` in module, deterministic at seed 42). Unit-step response of
the residual observer about the operating point (z_rel = 0, v_rel = 0,
delta_p = 0, T = 25 C), force residual in N:

| Step input | Residual (N) |
|---|---|
| baseline | 0.00 |
| v_rel = -1 m/s | -13.92 |
| v_rel = +1 m/s | +13.92 |
| delta_p = +1 MPa | +23.98 |
| delta_p = -1 MPa | -23.98 |
| T step +50 C | +8.62 |

Note: figures of -0.95 N / +37.85 N attributed to a "PINN step test" in
earlier session briefings have no artifact in this repository and do not
reproduce from the module; they are superseded by the measured values
above. The observer is an untrained random-weight network (weights drawn
at construction, no training data committed); its outputs are a
deterministic model evaluation, not learned physics.

## E06: Numbers not established (NOT ESTABLISHED)

The following figures circulate in analysis notes but have no
reproducible artifact in this repository and must not be cited:
- "LQR 35.16 % reduction", "Skyhook 1.46 %", "MR 56.9 %" (provenance
  unknown; inconsistent with the committed harness outputs above).
- "64.05 %" (does not exist anywhere on disk).
- "61.49 % at 45 mm obstacle" appears only in the LaTeX manuscript
  (`docs/paper/Robotic_Hydro_Suspension_TCST.tex`, abstract) and has no
  committed artifact; it describes a Tier-3 PINN-preview configuration
  that is not implemented in the benchmark harness. The manuscript number
  is a placeholder target, not a measured result.

---

Boundary statement: every number in this ledger is digital simulation or
model evaluation. No physical quarter-car rig, damper, or controller
hardware has been instrumented for this project.
