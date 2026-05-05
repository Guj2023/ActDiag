# ActDiag

ActDiag is a simple tool for testing and tuning actuators in simulation. 
It uses **MuJoCo** to simulate how your actuator behaves and helps you find the best controller settings.

## 🚀 What can it do?

1.  **Run Simulations (`run`):** Test your actuator with a specific controller and see how it follows a target (step, sine, or frequency sweep).
2.  **Fit Parameters (`fit`):** Automatically find the best controller gains (`kp`, `kd`) to match a real-world reference trajectory.
3.  **Sweep Parameters (`sweep`):** Systematically scan 1D or 2D controller and actuator settings and compare tracking and stability metrics.

---

## 🛠️ Quick Start

### 1. Install
```bash
pip install -e .
```

### 2. Run a simulation
See how your actuator handles a simple step move:
```bash
actdiag run --system examples/system_pd.yaml --scenario examples/scenario_step.yaml
```
Output lands in `runs/<timestamp>/` automatically.

### 3. Fit to a reference
Find the best gains to match a recording (`reference.csv`):
```bash
actdiag fit \
  --system examples/system_pd.yaml \
  --scenario examples/scenario_step.yaml \
  --reference reference.csv \
  --search search.yaml
```
Output lands in `fits/<timestamp>/` automatically. Pass `--output-dir <path>` to override.

### 4. Run a parameter sweep
Explore controller sensitivity or actuator/controller trade-offs:
```bash
actdiag sweep \
  --system examples/system_dynamic_torque.yaml \
  --scenario examples/scenario_chirp.yaml \
  --sweep examples/sweep_kp_tc.yaml
```
Output lands in `sweeps/<timestamp>/` automatically. Use `--workers N` for parallel execution.

---

## 📖 Key Concepts

### System (`system.yaml`)
Defines your **Controller** (like PD or PID) and your **Actuator** (torque limits).

#### Actuator Types

- **`ideal_torque`** (alias: `limited_torque`): Simple clipped torque — no dynamics.
- **`dynamic_torque`**: Control-oriented model with bandwidth and rate limits.

```yaml
controller:
  type: pd_position
  kp: 100.0
  kd: 2.0
actuator:
  type: dynamic_torque
  torque_limit: 40.0
  time_constant: 0.01
  torque_rate_limit: 400.0   # Optional
  deadzone: 0.2              # Optional
```

### Scenario (`scenario.yaml`)
Defines the **Scene** (physics properties like inertia) and the **Test** (what movement to perform).

```yaml
scene:
  type: single_joint
  inertia: 0.05
test:
  type: step
  target: 0.5
  start_time: 0.2
simulation:
  duration: 2.0
  dt: 0.001
```

### Search (`search.yaml`)
Defines the range for parameters you want to optimize with `fit`.

```yaml
fit:
  parameters:
    controller.kp: {min: 10.0, max: 200.0}
    controller.kd: {min: 0.0, max: 10.0}
  samples: 500
```

### Sweep (`sweep.yaml`)
Defines a full Cartesian grid of parameter values and which summary metrics to report.

Instead of listing every value explicitly, you can specify a range — all three forms resolve to the same flat list internally:

```yaml
sweep:
  parameters:

    # Explicit list (original style, still supported)
    controller.kp:
      values: [20, 40, 60, 80, 100]

    # Step-based range (arange-style)
    controller.kp:
      min: 10
      max: 100
      step: 10          # → [10, 20, 30 … 100]

    # N evenly-spaced points (linspace)
    controller.kp:
      min: 1
      max: 100
      num: 20           # → 20 linear points

    # N log-spaced points — ideal for time constants and small gains
    actuator.time_constant:
      min: 0.001
      max: 0.5
      num: 15
      scale: log        # → 15 points on log scale

  metrics:
    - tracking_rmse
    - jitter_metric
    - stable
```

### Reference Data (`reference.csv`)
For the `fit` command, you can provide a CSV file with your recorded data. **This argument is optional.**

- **If provided:** It must contain at least a `time` column. Missing columns for `q`, `dq`, or `tau_applied` default to `0.0`.
- **If NOT provided:** `actdiag` uses the **desired trajectory** (from your scenario) as the reference for fitting.

**Column names used for cost calculation:**
- `time`: Time in seconds (required).
- `q`: Position in radians.
- `dq`: Velocity in radians per second.
- `tau_applied`: Torque applied in Newton-meters.

> **Tip:** Every `run` command generates a `data/timeseries.csv` that matches this structure perfectly.

---

## 📊 Outputs

### `run`
Creates `runs/<timestamp>/` (or `--output-dir`) containing:
- **`data/timeseries.csv`**: Full record of the simulation (`time`, `q`, `dq`, `q_des`, `tau_applied`, …).
- **`figures/`**: Position, velocity, and torque plots.
- **`summary/`**: Calculated metrics (rise time, overshoot, settling time).
- **`video/`**: MP4 of the simulation (if `--save-video` is used).

### `fit`
Creates `fits/<timestamp>/` (or `--output-dir`) containing:
- **`best_system.yaml`**: Drop-in replacement for your system YAML with the fitted parameters applied — pass directly to `actdiag run --system`.
- **`best_fit.yaml`**: Raw best-fit parameter values for reference.
- **`top_candidates.csv`**: Top 50 candidates ranked by total cost.
- **`objective_breakdown.json`**: Cost breakdown for the best result.
- **`plots/`**: Sim vs. reference overlays for position and velocity.

### `sweep`
Creates `sweeps/<timestamp>/` (or `--output-dir`) containing:
- **`summary.csv`**: One row per parameter combination with all metrics.
- **`plots/`**: `heatmap_<metric>.png` for 2D sweeps, `line_<metric>.png` for 1D sweeps.
- **`cases/<parameter-slug>/`**: Full per-case run artifacts.

Supported sweep metrics:
- **`tracking_rmse`**: RMSE of position tracking error.
- **`max_abs_error`**: Maximum absolute position tracking error.
- **`stable`**: Whether the simulated response remained bounded.
- **`jitter_metric`**: High-frequency component of the tracking error.

---

## 🧪 Supported Tests

- **Step:** Jump to a target position.
- **Sine:** Follow a smooth wave.
- **Chirp:** Frequency sweep over time.
- **Frequency Response:** Sweep through many frequencies to see the bandwidth (Bode plots).

## 🔁 Parameter Sweeps

Use sweeps when you want to:
- understand controller sensitivity
- identify stable regions
- inspect trade-offs such as `kp` versus actuator bandwidth
- debug sim-to-real mismatch

Notes:
- Only 1D and 2D sweeps are supported.
- Sweep axes must target `controller.*` or `actuator.*` parameters.
- All parameter combinations are evaluated (Cartesian product).
- Output directory must not already exist (use a new path or let it auto-generate).
- Failed cases are recorded as errors and do not stop the sweep.
- Use `--workers N` to run cases in parallel (`0` = all CPUs). The live progress bar updates as each case completes, showing elapsed time, ETA, and the running best metric value.

---

## ⚙️ Requirements
- Python 3.10+
- MuJoCo, NumPy, Pandas, Matplotlib, Pydantic, PyYAML, Rich

# ActDiag Roadmap

---

## 🧭 Roadmap

ActDiag is evolving from a simple simulation tool into a diagnostic framework for actuator behavior.  
The development focuses on adding complexity only when it improves interpretability and fitting capability.

---

### v0.x — Current (Foundation)

- Single-joint simulation with MuJoCo
- Actuator models:
  - `ideal_torque` / `limited_torque` (alias)
  - `dynamic_torque`
- Standard test protocols:
  - `step`, `sine`, `chirp`, `frequency_response`
- Parameter fitting (`fit`) with automatic output and `best_system.yaml`
- Parameter sweeps with range spec, parallel execution, and live TUI progress

Goal:
- Establish a reproducible testing and fitting pipeline

---

### v0.x++ — Asymmetric / Nonlinear Effects

Extend actuator model to capture real-world asymmetries:

- Different behavior for positive vs negative torque
- Asymmetric rate limits
- Asymmetric deadzones

Example:

```yaml
torque_rate_limit_pos: ...
torque_rate_limit_neg: ...
deadzone_pos: ...
deadzone_neg: ...
```

Goal:
- Capture actuator bias and directional differences
- Improve realism without introducing excessive complexity

---

### v0.x+++ — Linear Actuator Abstraction

Introduce a force-based actuator:

#### `dynamic_force`

- Output: force instead of torque
- Same non-ideal effects as `dynamic_torque`

Goal:
- Support linear systems (e.g., sliders, cylinders)
- Move toward more general actuator abstraction

---

### Future — Hydraulic-like Actuator (Conceptual)

Instead of full first-principles modeling, introduce a behavioral model inspired by hydraulic systems:

Possible features:
- Hysteresis
- Strong asymmetry (extend vs retract)
- Load-dependent response
- Slow pressure buildup (modeled as lag)

Goal:
- Capture key hydraulic behaviors without full fluid simulation
- Enable diagnosis of systems with strong nonlinearities

---

### Long-Term Direction

- Improve identifiability in `fit`:
  - Detect ambiguity
  - Report model mismatch
- Design better excitation signals (e.g., chirp, multi-sine)
- Expand from "simulate and fit" to:
  - diagnose why systems fail
  - compare actuator/controller robustness across scenarios

---

## 🧠 Design Philosophy

- Start simple, add complexity only when necessary
- Prefer interpretable parameters over physically complete models
- Focus on:
  - reproducibility
  - diagnosability
  - fitting robustness

If a simpler model can explain the behavior, prefer it over a more complex one.
