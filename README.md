# ActDiag

ActDiag is a minimal CLI project for actuator diagnosis in simulation. The target is intentionally narrow: run a simple, reproducible actuator test in MuJoCo and save the logs and plots needed to understand what happened.

## Status

`v0.1` is implemented as a working CLI. The current tool supports the MVP flow described below: loading four YAML profiles, running a single-joint MuJoCo simulation, saving CSV logs and plots, and optionally exporting video.

## Why ActDiag

Actuator issues are often hidden inside a full robot stack, where it is hard to tell whether bad behavior comes from the actuator model, controller gains, mechanical load, reference signal, or simulator details. ActDiag strips that down to the smallest useful experiment:

- one actuator
- one joint
- one scene
- one test profile
- one output folder with reproducible artifacts

The core question is simple: how does a given actuator model behave in a controlled test?

## MVP Scope

ActDiag combines four YAML profiles into a single run:

- `actuator`: how commanded torque is applied
- `controller`: how torque commands are generated
- `scene`: mechanical setup
- `test`: command signal and timing

Planned `v0.1` support:

- MuJoCo only
- single-joint scene only
- `ideal_actuator`
- controller modes: `pd`, `inverse_dynamics`, and `none`
- position tests: `step` and `sine`
- torque tests: `torque_step` and `torque_sine`
- CSV time-series logging
- five diagnostic plots
- optional video export
- preservation of original and resolved configs

Explicit non-goals for the MVP:

- multi-DOF systems
- contact-rich tasks
- disturbance injection
- friction or delay models
- parameter sweeps
- automatic diagnosis labels
- GUI or web frontend
- system identification

## CLI

Primary command:

```bash
actdiag run \
  --actuator examples/actuator_ideal.yaml \
  --controller examples/controller_pd.yaml \
  --scene examples/scene_single_joint.yaml \
  --test examples/test_step.yaml
```

Optional video export:

```bash
actdiag run \
  --actuator examples/actuator_ideal.yaml \
  --controller examples/controller_pd.yaml \
  --scene examples/scene_single_joint.yaml \
  --test examples/test_sine.yaml \
  --save-video
```

Inverse-dynamics controller example:

```bash
actdiag run \
  --actuator examples/actuator_ideal.yaml \
  --controller examples/controller_inverse_dynamics.yaml \
  --scene examples/scene_single_joint.yaml \
  --test examples/test_step.yaml
```

Direct torque injection example:

```bash
actdiag run \
  --actuator examples/actuator_ideal.yaml \
  --controller examples/controller_none.yaml \
  --scene examples/scene_single_joint.yaml \
  --test examples/test_torque_step.yaml
```

Current command:

- `actdiag run`

Commands like `compare`, `sweep`, or `report` can come later.

## Configuration

### Actuator profile

```yaml
type: ideal_actuator
torque_limit: 40.0
```

### Controller profiles

PD controller:

```yaml
type: pd
kp: 100.0
kd: 2.0
```

Inverse-dynamics controller:

```yaml
type: inverse_dynamics
kp: 25.0
kd: 6.0
```

Direct torque mode:

```yaml
type: none
```

### Scene profile

```yaml
scene_type: single_joint
joint:
  inertia: 0.05
  damping: 0.1
  gravity: false
  q0: 0.0
  dq0: 0.0
```

### Test profiles

Step input:

```yaml
test_type: step
target: 0.5
start_time: 0.2
duration: 2.0
dt: 0.001
```

Sine input:

```yaml
test_type: sine
amplitude: 0.2
frequency: 1.0
offset: 0.0
duration: 5.0
dt: 0.001
```

Torque step input:

```yaml
test_type: torque_step
target_torque: 1.5
start_time: 0.2
duration: 2.0
dt: 0.001
```

Torque sine input:

```yaml
test_type: torque_sine
amplitude: 1.2
frequency: 1.0
offset: 0.0
duration: 5.0
dt: 0.001
```

Recommended validation rules:

- actuator: supported `type`, `torque_limit > 0`
- controller: supported `type`, `kp >= 0`, `kd >= 0` when applicable
- scene: supported `scene_type`, `inertia > 0`, `damping >= 0`, finite initial state
- test: supported `test_type`, `duration > 0`, `dt > 0`
- pairing: `pd` and `inverse_dynamics` require position tests, `none` requires torque tests

## Outputs

Each run produces a self-contained directory:

```text
runs/
  2026-04-06_191530/
    config/
      actuator.yaml
      controller.yaml
      scene.yaml
      test.yaml
      resolved.yaml
    data/
      timeseries.csv
    figures/
      position.png
      velocity.png
      torque.png
      error.png
      phase.png
    video/
      sim.mp4
```

Why `resolved.yaml` matters: it captures the final effective configuration after defaults and merges, which is essential for reproducibility.

Expected CSV columns:

```text
time,q,dq,q_des,dq_des,tau_des,position_error,velocity_error,tau_cmd,tau_applied
```

Planned figures:

- position vs time
- velocity vs time
- torque vs time
- position error vs time
- phase plot (`q` vs `dq`)

Raw logs are the primary artifact. Plots and video are derived outputs.

## Installation

The project is packaged as a standard Python CLI:

- Python 3.10+
- MuJoCo
- NumPy
- Pandas
- Matplotlib
- PyYAML
- Pydantic

Suggested setup:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e .
```

## Layout

Current project structure:

```text
actdiag/
  __init__.py
  cli.py
  config.py
  controller.py
  signals.py
  actuator.py
  scene.py
  simulate.py
  logging_io.py
  plotting.py

examples/
  actuator_ideal.yaml
  controller_pd.yaml
  controller_inverse_dynamics.yaml
  controller_none.yaml
  scene_single_joint.yaml
  test_step.yaml
  test_sine.yaml
  test_torque_step.yaml
  test_torque_sine.yaml

runs/
README.md
```

## Implemented Features

1. YAML loading and schema validation with Pydantic.
2. Single-joint MuJoCo scene generation.
3. `ideal_actuator` plus separate `pd`, `inverse_dynamics`, and `none` controller modes.
4. `step`, `sine`, `torque_step`, and `torque_sine` reference generation.
5. CSV logging, resolved config export, and plot generation.
6. Optional MP4 video export.

## Future Extensions

Likely next steps after the MVP:

- actuator non-idealities such as saturation, deadzone, delay, or friction
- more test signals such as chirp, ramp, or trajectory replay
- summary metrics such as RMS error, overshoot, and settling time
- comparison workflows across actuator profiles or gain settings
- more scenes, including loaded pendulums or coupled systems

The intended order is simple: make the minimal pipeline clean, make the logs reliable, then add more analysis and complexity.

## License

TBD. If the project is published later, MIT would be a reasonable default.
