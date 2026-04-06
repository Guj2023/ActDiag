# ActDiag

ActDiag is a minimal CLI tool for actuator diagnosis in simulation with MuJoCo. The scope is narrow on purpose: run one controlled experiment, log the result, and save a few plots that make the actuator behavior easy to inspect.

## Status

`v0.3.0` is still an MVP. The codebase keeps controller, actuator, scene, and test logic split internally, but the user-facing configuration is now reduced to two YAML files:

- `system.yaml`
- `scenario.yaml`

That keeps the tool small while making the experiment structure easier to read.

## Core Idea

The conceptual model is:

```text
scenario.test
-> system.controller
-> system.actuator
-> scenario.scene
-> logs / plots / outputs
```

In this model:

- the `system` profile is the thing being diagnosed
- the `scenario` profile is the diagnostic condition

Controller and actuator are separate concepts. They are not treated as one combined "PD actuator". The earlier simple behavior is better understood as:

```text
reference -> PD controller -> torque actuator -> plant
```

For the current MVP, that means:

- controller type: `pd_position`
- actuator type: `ideal_torque`

## MVP Scope

ActDiag uses two user-facing profiles per run.

`system.yaml` contains:

- `controller`
- `actuator`

`scenario.yaml` contains:

- `scene`
- `test`
- `simulation` settings
- `logging` settings
- `plots` settings
- `output` settings

Supported MVP features:

- controller types: `pd_position`
- actuator types: `ideal_torque`
- scene types: `single_joint`
- test types: `step`, `sine`
- CSV time-series logging
- diagnostic plots
- optional video export

Explicit non-goals for the current MVP:

- multi-DOF systems
- contact-rich tasks
- plugin systems
- large framework abstractions
- parameter sweeps
- automatic diagnosis labels
- GUI or web frontend

## CLI

Primary command:

```bash
actdiag run --system system.yaml --scenario scenario.yaml
```

With video:

```bash
actdiag run --system system.yaml --scenario scenario.yaml --save-video
```

The CLI is intentionally small. `actdiag run` is the only command that matters for the MVP.

## Configuration

### `system.yaml`

```yaml
controller:
  type: pd_position
  kp: 100.0
  kd: 2.0

actuator:
  type: ideal_torque
  torque_limit: 40.0
```

### `scenario.yaml`

Step test example:

```yaml
scene:
  type: single_joint
  inertia: 0.05
  damping: 0.1
  gravity: false
  q0: 0.0
  dq0: 0.0

test:
  type: step
  target: 0.5
  start_time: 0.2

simulation:
  duration: 2.0
  dt: 0.001

logging:
  save_csv: true

plots:
  position: true
  velocity: true
  torque: true
  error: true
  phase: true

output:
  save_video: false
```

Sine test example:

```yaml
scene:
  type: single_joint
  inertia: 0.05
  damping: 0.1
  gravity: false
  q0: 0.0
  dq0: 0.0

test:
  type: sine
  amplitude: 0.2
  frequency: 1.0
  offset: 0.0

simulation:
  duration: 2.0
  dt: 0.001

logging:
  save_csv: true

plots:
  position: true
  velocity: true
  torque: true
  error: true
  phase: true

output:
  save_video: false
```

## Validation

Recommended validation rules:

- `controller.type` must be supported
- `actuator.type` must be supported
- `kp >= 0`
- `kd >= 0`
- `torque_limit > 0`
- `scene.type` must be supported
- `inertia > 0`
- `damping >= 0`
- `simulation.duration > 0`
- `simulation.dt > 0`
- `step.start_time >= 0`
- `sine.frequency > 0`

## Outputs

Each run produces a compact directory:

```text
runs/
  2026-04-06_191530/
    config/
      system.yaml
      scenario.yaml
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

`resolved.yaml` matters because it records the effective configuration that actually produced the run.

Expected CSV columns:

```text
time,q,dq,q_des,dq_des,tau_des,position_error,velocity_error,tau_cmd,tau_applied
```

The primary artifacts are still the raw logs. Plots and video are derived outputs.

## Installation

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

## Package Layout

A reasonable MVP package layout is:

```text
actdiag/
  __init__.py
  cli.py
  config.py
  signals.py
  controller.py
  actuator.py
  scene.py
  simulate.py
  logging_io.py
  plotting.py
```

Example config files can stay simple:

```text
examples/
  system.yaml
  scenario_step.yaml
  scenario_sine.yaml
```

## Development Checklist

- keep two user-facing profiles: `system.yaml` and `scenario.yaml`
- keep controller and actuator conceptually separate inside `system.yaml`
- keep scene and test separate inside `scenario.yaml`
- keep the simulation path explicit: test -> controller -> actuator -> scene
- keep schemas strict and validation simple
- keep outputs reproducible with copied inputs and a resolved config

## Roadmap

Near-term work that still fits the MVP:

- tighten schema validation and error messages
- add a few compact summary metrics on top of the existing logs
- improve plot readability without expanding the architecture
- add one or two more actuator or scene variants only if the current pipeline stays clean

Longer-term vision, still grounded:

- make actuator diagnosis runs easier to reproduce
- keep the experiment definition explicit and small
- add capability only when it improves diagnosis clarity, not because the project should become a framework

## License

TBD. MIT would be a reasonable default if the project is published later.
