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

```yaml
controller:
  type: pd_position
  kp: 100.0
  kd: 2.0
actuator:
  type: ideal_torque
  torque_limit: 40.0
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
For the `fit` command, you need a CSV file with your recorded data. 

**Required columns:**
- `time`: Time in seconds.
- `q`: Position in radians.

**Optional (used for cost calculation if present):**
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
