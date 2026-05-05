# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
# Install in editable mode (required before first use)
python -m venv .venv && source .venv/bin/activate
pip install -e .

# Run a diagnostic scenario
actdiag run --system examples/system_pd.yaml --scenario examples/scenario_step.yaml

# With video export
actdiag run --system examples/system_pd.yaml --scenario examples/scenario_step.yaml --save-video

# Custom output directory
actdiag run --system examples/system_pd.yaml --scenario examples/scenario_sine.yaml --output-dir /tmp/my_run
```

There is no test suite.

## Architecture

ActDiag runs one diagnostic scenario per invocation. The data flow is:

```
system.yaml + scenario.yaml
  → load_run_config()     (config.py)   → RunConfig
  → build_signal_series() (signals.py)  → SignalSeries
  → _simulate_signal_series() (simulate.py)
      → build_single_joint_model() (scene.py)    → mujoco.MjModel (XML built at runtime)
      → build_controller() (controller.py)       → PDController | InverseDynamicsController | NoController
      → build_actuator()   (actuator.py)         → IdealActuator
  → SimulationArtifacts / FrequencyResponseArtifacts
  → logging_io.py + plotting.py                  → runs/<timestamp>/
```

### Two-profile config model

`system.yaml` holds hardware-like parameters (`controller`, `actuator`). `scenario.yaml` holds experiment parameters (`scene`, `test`, `simulation`, `logging`, `plots`, `output`). `load_run_config()` merges both into a single `RunConfig`.

### config.py — the type system

All config types are Pydantic `StrictModel` (extra fields forbidden). YAML type aliases are normalized during parsing before Pydantic sees them:
- `ideal_torque` → `ideal_actuator`
- `pd_position` → `pd`

Test type strings in YAML use `type:` but internal Pydantic models use `test_type:` as the discriminator. The `_parse_user_test_profile()` function handles the rename.

A `RunConfig` model validator enforces controller–test pairing: PD and inverse-dynamics controllers require position tests (`step`, `sine`, `frequency_response`); `none` controller requires torque tests (`torque_step`, `torque_sine`).

For `frequency_response`, `simulation.duration` is automatically derived and must not be specified in the YAML. All other test types require an explicit `simulation.duration`.

### signals.py — SignalSeries convention

`SignalSeries` is a frozen dataclass of pre-computed NumPy arrays (`time`, `q_des`, `dq_des`, `qdd_des`, `tau_des`). For position tests, `tau_des` is filled with `NaN`. For torque tests, `q_des`/`dq_des`/`qdd_des` are `NaN`. The simulation loop reads both, so controllers must handle the NaN for their unused channel.

`frequency_response` tests skip `build_signal_series()` and use `build_frequency_response_signal()` directly, called once per frequency inside `run_frequency_response_simulation()`.

### scene.py — MuJoCo model generation

`build_single_joint_model()` constructs a MuJoCo XML string at runtime from `SingleJointSceneProfile` parameters and returns `mujoco.MjModel.from_xml_string(...)`. The inertia geometry (mass, CoM offset, diagonals) is derived from the scalar `inertia` parameter.

### Output layout

Each run writes to `runs/<timestamp>/` (or `--output-dir`):
- `config/` — copies of `system.yaml`, `scenario.yaml`, and `resolved.yaml`
- `data/` — `timeseries.csv` or `frequency_response/<freq_hz>/timeseries.csv`
- `figures/` — PNG plots; frequency response gets a `frequency_response/` subdirectory
- `summary/` — `step_metrics.json` (step test) or `frequency_response.csv`
- `video/` — `sim.mp4` (only created when `--save-video` is used)
