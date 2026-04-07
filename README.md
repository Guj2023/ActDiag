# ActDiag

ActDiag is a minimal CLI tool for actuator diagnosis in simulation with MuJoCo. The scope is intentionally small: run one diagnostic scenario, log the response, and save a few plots that are easy to inspect.

## Status

`v0.4.0` is still an MVP. The user-facing configuration stays deliberately narrow:

- `system.yaml`
- `scenario.yaml`

Conceptually, a run is:

```text
scenario.test
-> system.controller
-> system.actuator
-> scenario.scene
-> logs / plots / outputs
```

For the current MVP, the intended defaults are:

- controller type: `pd_position`
- actuator type: `limited_torque`
- scene type: `single_joint`
- simulation engine: MuJoCo

## MVP Scope

`system.yaml` contains:

- `controller`
- `actuator`

`scenario.yaml` contains:

- `scene`
- `test`
- `simulation`
- `logging`
- `plots`
- `output`

Supported test modes:

- `step`
- `sine`
- `frequency_response`

Torque trajectory tests from earlier iterations are still accepted internally for compatibility, but the main MVP path is still position-reference diagnosis.

## CLI

Run a scenario:

```bash
actdiag run --system system.yaml --scenario scenario.yaml
```

Export video for single-run tests:

```bash
actdiag run --system system.yaml --scenario scenario.yaml --save-video
```

`frequency_response` does not export video in this MVP.

## Configuration

### `system.yaml`

```yaml
controller:
  type: pd_position
  kp: 100.0
  kd: 2.0

actuator:
  type: limited_torque
  torque_limit: 40.0
```

Supported controller types:

- `pd_position`
- `pid_position`

Supported actuator types:

- `ideal_torque`
- `limited_torque`

Controller meanings:

- `pd_position`: `tau_cmd = kp * (q_des - q) + kd * (dq_des - dq)`
- `pid_position`: `tau_cmd = kp * e + ki * integral_e + kd * de`

Actuator meanings:

- `limited_torque` clips `tau_cmd` into `[-torque_limit, +torque_limit]`
- `ideal_torque` keeps the project's existing backward-compatible behavior and also clips with `torque_limit`

In the logs, `tau_cmd` is the controller output and `tau_applied` is the actuator output after actuator constraints.

### Step test

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

Behavior:

- before `start_time`, `q_des` stays at `0.0`
- at `start_time`, `q_des` jumps to `target`
- `dq_des` is logged as `0.0`

Step runs also save `summary/step_metrics.json` with:

- `steady_state_value`
- `steady_state_error`
- `peak_value`
- `percent_overshoot`
- `rise_time`
- `settling_time`

The definitions are intentionally simple and robust:

- steady-state value is the mean of the final small response window
- overshoot uses a basic relative measure against the target
- rise time uses first 10% and 90% crossings
- settling time uses a 5% band around the target

### Sine test

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
```

### Frequency response test

```yaml
scene:
  type: single_joint
  inertia: 0.05
  damping: 0.1
  gravity: false
  q0: 0.0
  dq0: 0.0

test:
  type: frequency_response
  amplitude: 0.05
  frequencies: [0.5, 1.0, 2.0, 4.0, 8.0]
  cycles_per_frequency: 8
  settle_cycles: 3
  offset: 0.0

simulation:
  dt: 0.001

logging:
  save_csv: true

plots:
  position: true
  velocity: true
  torque: true
  error: true
  phase: true
  frequency_response: true

output:
  save_video: false
```

Behavior:

- ActDiag runs one sinusoidal reference per frequency
- each frequency is simulated separately from the same initial state
- the first `settle_cycles` are discarded for estimation
- the remaining cycles are fit against `sin(omega t)` and `cos(omega t)`
- negative phase values indicate output lag, which matches the usual Bode-style convention

Only `simulation.dt` is required for `frequency_response`. Total duration is derived automatically as:

```text
sum(cycles_per_frequency / frequency_hz)
```

Internally, each frequency still runs as its own simulation segment with a fresh reset. The derived duration is the total sweep duration recorded in the resolved config.

The sweep summary is saved to `summary/frequency_response.csv` with:

- `frequency_hz`
- `input_amplitude`
- `output_amplitude`
- `gain`
- `phase_rad`
- `phase_deg`

## Validation

Current validation includes:

- `controller.type` must be supported
- `actuator.type` must be supported
- `kp >= 0`
- `ki >= 0` for `pid_position`
- `kd >= 0`
- `torque_limit > 0`
- `scene.type` must be `single_joint`
- `inertia > 0`
- `damping >= 0`
- `simulation.dt > 0`
- step: `target` finite, `start_time >= 0`
- sine: `frequency > 0`
- frequency response:
  - `amplitude > 0`
  - `frequencies` must be a non-empty list of positive finite numbers
  - `cycles_per_frequency >= 1`
  - `settle_cycles >= 0`
  - `settle_cycles < cycles_per_frequency`

For single-run tests such as `step` and `sine`, `simulation.duration` is required.

## Outputs

Each run produces a compact directory:

```text
runs/
  2026-04-07_201500/
    config/
      system.yaml
      scenario.yaml
      resolved.yaml
    data/
      timeseries.csv
      frequency_response/
        0_500_hz/
          timeseries.csv
        1_000_hz/
          timeseries.csv
    figures/
      position.png
      velocity.png
      torque.png
      error.png
      phase.png
      frequency_response_gain.png
      frequency_response_phase.png
      frequency_response/
        0_500_hz/
          position.png
          velocity.png
          torque.png
          error.png
          phase.png
    summary/
      step_metrics.json
      frequency_response.csv
    video/
      sim.mp4
```

Single-run CSV columns:

```text
time,q,dq,q_des,dq_des,tau_des,position_error,velocity_error,tau_cmd,tau_applied,integral_error,is_saturated
```

Standard plots:

- position vs time
- velocity vs time
- torque vs time
- error vs time
- phase plot

Frequency response plots:

- gain vs frequency
- phase vs frequency

## Examples

Example files are in `examples/`:

- `examples/system_pd.yaml`
- `examples/system_pd_limited_torque.yaml`
- `examples/system_pid_limited_torque.yaml`
- `examples/scenario_step.yaml`
- `examples/scenario_sine.yaml`
- `examples/scenario_frequency_response.yaml`

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
