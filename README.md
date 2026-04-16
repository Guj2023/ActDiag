# ActDiag

ActDiag is a simple tool for testing and tuning actuators in simulation. 
It uses **MuJoCo** to simulate how your actuator behaves and helps you find the best controller settings.

## 🚀 What can it do?

1.  **Run Simulations (`run`):** Test your actuator with a specific controller and see how it follows a target (step, sine, or frequency sweep).
2.  **Fit Parameters (`fit`):** Automatically find the best controller gains (`kp`, `kd`) to match a real-world reference trajectory.

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

### 3. Fit to a reference
Find the best gains to match a recording (`reference.csv`):
```bash
actdiag fit \
  --system examples/system_pd.yaml \
  --scenario examples/scenario_step.yaml \
  --reference reference.csv \
  --search search.yaml \
  --output-dir my_fit_results
```

---

## 📖 Key Concepts

### System (`system.yaml`)
Defines your **Controller** (like PD or PID) and your **Actuator** (torque limits).

#### Actuator Types

- **`ideal_torque`**: Simple clipped torque.
- **`limited_torque`**: Equivalent to `ideal_torque`.
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

### Search (`search.yaml`) - *New in v0.5.0!*
Defines the range for parameters you want to optimize.

```yaml
fit:
  parameters:
    controller.kp: {min: 10.0, max: 200.0}
    controller.kd: {min: 0.0, max: 10.0}
  samples: 500
```

### Reference Data (`reference.csv`)
For the `fit` command, you can provide a CSV file with your recorded data. **This argument is optional.**

- **If provided:** It must contain at least a `time` column. Missing columns for `q`, `dq`, or `tau_applied` will default to `0.0`.
- **If NOT provided:** `actdiag` will use the **Desired trajectory** (the target signals defined in your scenario) as the reference for fitting.

**Column names used for cost calculation:**
- `time`: Time in seconds (Required).
- `q`: Position in radians.
- `dq`: Velocity in radians per second.
- `tau_applied`: Torque applied in Newton-meters.

> **Tip:** Every `run` command generates a `data/timeseries.csv` file that matches this structure perfectly, making it easy to use simulation results as fitting references.

---

## 📊 Outputs

Every time you run `actdiag`, it creates a folder with:
- **`data/timeseries.csv`**: A full record of the simulation. It includes `time`, `q`, `dq`, `q_des`, `dq_des`, `tau_applied`, and more.
- **`figures/`**: Visual plots of position, velocity, and torque.
- **`summary/`**: Calculated metrics like rise time, overshoot, and settling time.
- **`video/`**: An MP4 of the simulation (if `--save-video` is used).

---

## 🧪 Supported Tests

- **Step:** Jump to a target position.
- **Sine:** Follow a smooth wave.
- **Frequency Response:** Sweep through many frequencies to see the bandwidth (Bode plots).

---

## ⚙️ Requirements
- Python 3.10+
- MuJoCo, NumPy, Pandas, Matplotlib, Pydantic, PyYAML

# ActDiag Roadmap

---

## 🧭 Roadmap

ActDiag is evolving from a simple simulation tool into a diagnostic framework for actuator behavior.  
The development focuses on adding complexity only when it improves interpretability and fitting capability.

---

### v0.x — Current (Foundation)

- Single-joint simulation with MuJoCo
- Basic actuator models:
  - `ideal_torque`
  - `limited_torque`
  - `dynamic_torque`
- Standard test protocols:
  - `step`
  - `sine`
  - `frequency_response`
- Parameter fitting (`fit`) for controller gains (`kp`, `kd`)

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
- Expand from “simulate and fit” to:
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
