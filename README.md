# ActDiag

ActDiag is a diagnostic and tuning tool for actuators in simulation. It simulates a single revolute joint with pluggable physics backends — **MuJoCo** (default), **NVIDIA PhysX** (via pyphysx), and **OpenModelica** (FMI 2.0 Co-Simulation, no MATLAB required) — and helps you find the best controller settings, validate that different simulators agree, and quantify how your actuator actually behaves.

## 🚀 What can it do?

1. **Run Simulations (`run`):** Test your actuator with any supported physics backend against a step, sine, chirp, or frequency-sweep reference.
2. **Compare Backends:** Run the same scenario under MuJoCo, PhysX, and OpenModelica and overlay the results to catch sim-to-sim discrepancies before they become sim-to-real problems.
3. **Fit Parameters (`fit`):** Automatically find the best controller gains (`kp`, `kd`) to match a real-world reference trajectory.
4. **Sweep Parameters (`sweep`):** Systematically scan 1D or 2D controller and actuator settings and compare tracking and stability metrics across the full grid.

---

## 🛠️ Quick Start

### 1. Install
```bash
pip install -e .
```

### 2. Run a simulation
See how your actuator handles a simple step move (MuJoCo default):
```bash
actdiag run --system examples/system_pd.yaml --scenario examples/scenario_step.yaml
```
Output lands in `runs/<timestamp>/` automatically.

Switch physics backend without changing any config file:
```bash
# PhysX backend (requires arm64 pyphysx build — see Requirements)
actdiag run --system examples/system_pd.yaml --scenario examples/scenario_step.yaml \
    -- simulation.backend physx

# OpenModelica FMI backend (requires omc + fmpy — see Requirements)
actdiag run --system examples/system_pd.yaml --scenario examples/scenario_step.yaml \
    -- simulation.backend openmodelica
```

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
  backend: mujoco   # "mujoco" (default), "physx", or "openmodelica"
```

#### Physics backend: `mujoco`, `physx`, `openmodelica`

`simulation.backend` controls which physics engine runs the simulation. All three model an identical single revolute joint with the same equation of motion:

```
I · q̈  =  τ_cmd  −  b · q̇  [+  m · g · com_x · cos(q)]
```

| Backend | Default | Notes |
|---|---|---|
| `mujoco` | ✓ | Default. No extra install. RK4 integrator. |
| `physx` | — | NVIDIA PhysX 4 via pyphysx — requires a manual arm64 build (see below). |
| `openmodelica` | — | FMI 2.0 Co-Simulation FMU compiled by OpenModelica — free, no MATLAB needed. |

---

##### How the backends are kept physically equivalent

Both engines model the same 1-DOF revolute joint, but their internal conventions differ in two important ways. The PhysX backend corrects for both so the equations of motion are identical to MuJoCo.

**1. Gravity torque — analytical injection**

MuJoCo automatically computes gravitational torque from the link's mass distribution: the COM sits `com_x` metres along the arm from the pivot, so gravity produces a joint torque

```
τ_grav = m · g · com_x · cos(q)
```

PhysX applies gravity as a body force at the rigid-body COM, which in the D6-joint model is placed at the pivot (origin). A force applied at zero moment arm produces **zero rotational torque** — the arm behaves as if gravity is absent regardless of the `gravity: true` setting.

Fix: PhysX body gravity is always disabled (`link.disable_gravity()`). The gravity torque is injected analytically each step using the same `com_x` formula as MuJoCo's `scene.py`, so both backends see an identical load.

**2. Angular damping — analytical injection**

MuJoCo's `joint.damping` (`b`) adds a viscous torque at each integration stage:
```
τ_damp = −b · dq          units: N·m·s/rad
```
This gives exponential velocity decay with time constant **τ = I / b**.

PhysX's `set_angular_damping(d)` does something physically equivalent but with a different parameterisation. Empirical measurement (zero external torque, free decay from ω₀ = 1 rad/s) confirms it applies a **direct velocity scaling** each step:
```
ω_new = ω_old · (1 − d · dt)
```
This is a valid first-order discretisation of `dω/dt = −d·ω`, giving exponential decay with time constant **τ = 1 / d**. The physics is real — but `d` has units of **1/s** (inverse time constant), not N·m·s/rad like MuJoCo's `b`.

The consequence: for the same numerical value the two parameters encode very different damping strengths:

| Parameter | Damping torque | Decay time constant |
|---|---|---|
| MuJoCo `joint.damping = b` | −b · ω | I / b |
| PhysX `angular_damping = d` | −(d · I) · ω | 1 / d |

With `b = 0.1 N·m·s/rad`, `d = 0.1 1/s`, `I = 0.05 kg·m²`:

| | MuJoCo | PhysX (d=0.1) |
|---|---|---|
| Effective damping torque | −0.1 · ω | −0.005 · ω |
| Velocity decay τ | **0.5 s** | **10 s** |

PhysX damps the joint 20× more slowly, which visibly changes the oscillation envelope and damping ratio.

Fix: PhysX angular damping is set to `0.0`. The joint damping torque `−b·dq` is injected analytically alongside the gravity torque each step, using exactly the same formula and units as MuJoCo.

---

##### Residual difference after the fixes

After both corrections the equations of motion are identical. The only remaining difference is the **integrator**:

| Property | MuJoCo | PhysX |
|---|---|---|
| Integration scheme | RK4 (4th-order) | Semi-implicit Euler (1st-order) |
| Per-step error | O(dt⁵) | O(dt²) |
| Energy behaviour | Near-exact | Slightly dissipative |

For typical settings (`dt = 0.002 s`, `ωn ≈ 8–11 rad/s`, `ωn·dt ≈ 0.02`), the integrator difference is very small — expect closely matching trajectories with only a slight phase drift over long windows. Qualitative behaviour (oscillation vs. overshoot, settling time, steady-state) should be indistinguishable.

---

##### Known limitations of the PhysX backend

**Single-joint scope only**

The PhysX backend is implemented using a `RigidDynamic` body constrained by a `D6Joint` — not PhysX's dedicated reduced-coordinate articulation solver. This is a deliberate choice given the tool's single-joint scope, but it has consequences worth understanding.

**Why not articulations?**

Tools like Isaac Lab and Isaac Sim model robots as **PhysX articulations** (`PxArticulationReducedCoordinate`). Articulations have two important advantages over the D6-joint approach:

1. **Built-in joint drives with correct units.** The articulation drive `damping` parameter operates in Force mode (N·m·s/rad) by default, directly matching MuJoCo's `joint.damping` with no scaling required.
2. **Reduced-coordinate solver.** For multi-body chains (robot arms, legged robots), the reduced-coordinate formulation avoids constraint drift that accumulates with sequential D6 joints.

pyphysx 0.2.5 — the Python binding used here — does not expose the articulation API at all. The complete set of available classes (`RigidDynamic`, `RigidStatic`, `D6Joint`, `Scene`, `Material`, `Shape`) contains no articulation or articulation-joint type. Switching would require writing new C++ pybind11 bindings against the arm64-patched PhysX source, which is a significant undertaking for a tool that targets single-joint diagnostics.

For the single revolute joint case the D6 approach is numerically equivalent: there is no constraint chain to drift, and the analytical injection of gravity and damping (described above) produces the same equations of motion as a properly configured articulation drive.

**The broader sim-to-sim damping problem**

The damping unit mismatch we found in pyphysx is a specific instance of a wider issue that has caused real problems in robotics research. Three conventions coexist across the major simulators:

| Simulator / API | Damping parameter meaning | Units | Effective torque |
|---|---|---|---|
| MuJoCo `joint.damping` | Viscous torque coefficient | N·m·s/rad | `−b · dq` |
| Isaac Lab `ImplicitActuator` (Force mode, default) | Same as MuJoCo | N·m·s/rad | `−b · dq` |
| Isaac Gym (deprecated, Acceleration mode default) | Inertia-normalised gain | 1/s | `−(d · I) · dq` |
| pyphysx `RigidDynamic.set_angular_damping(d)` | Inverse time constant | 1/s | `−(d · I) · dq` |

Isaac Gym used PhysX articulation drives in **Acceleration mode** (`isAcceleration=True`) by default, where the engine internally multiplies the gain by the link's inertia. Isaac Lab switched this to **Force mode** (`isAcceleration=False`), making its `damping` directly comparable to MuJoCo. This breaking change is documented in Isaac Lab's migration guide, but the inertia-scaling factor is rarely explained explicitly, and RL policies ported from Isaac Gym environments to Isaac Lab (or MuJoCo) produced incorrect dynamics unless the damping was rescaled by `I`.

pyphysx's body-level `set_angular_damping` is a third, separate API — not the articulation drive at all — and happens to share the same `1/s` unit problem as Isaac Gym's acceleration mode, even though the underlying mechanism differs.

Our fix (setting `angular_damping=0` and injecting `−b·dq` analytically) sidesteps all three conventions and ensures ActDiag's PhysX backend always uses the same physical definition as MuJoCo regardless of which pyphysx version or PhysX drive mode would be in play.

**When to consider a different approach**

If ActDiag is extended to multi-joint systems (robot arm, leg, full kinematic chain), the D6-joint approach should be replaced. At that point, the right path is to use Isaac Lab's `omni.isaac.core` articulation API directly, which exposes reduced-coordinate articulations with Force-mode drives, proper contact handling, and GPU-parallel simulation — rather than extending the pyphysx binding further.

---

#### Physics backend: `openmodelica`

See the **[🏭 OpenModelica Backend — FMI / FMU](#-openmodelica-backend--fmi--fmu)** section for the full explanation: what FMI/FMU is, how the Modelica model is generated and cached, integrator comparison, usage, installation, and limitations.

```yaml
simulation:
  backend: openmodelica
```

---

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

## 🏭 OpenModelica Backend — FMI / FMU

ActDiag supports a third physics backend driven by the open **FMI 2.0 Co-Simulation** standard (Functional Mock-up Interface). You do not need MATLAB, Simulink, or any commercial license — only the free [OpenModelica](https://openmodelica.org/) compiler (`omc`) and two pip packages.

### What is FMI / FMU?

**FMI (Functional Mock-up Interface)** is an open IEC standard for exchanging simulation models between tools. A **Functional Mock-up Unit (FMU)** is a portable zip archive containing:

- `modelDescription.xml` — variable declarations (inputs, outputs, parameters, states)
- `binaries/<platform>/` — a compiled shared library implementing the model
- `sources/` — optional C source code for cross-compilation

The **Co-Simulation** variant (used here) lets the FMU advance its own internal states by one communication step (`doStep`). The Python master — ActDiag — just sets inputs (`tau_cmd`) before each step and reads outputs (`q`, `dq`) after. No knowledge of the FMU's internal solver is needed.

Other tools that can export Co-Simulation FMUs include Dymola, Modelon Impact, Ansys Twin Builder, and Simulink (via the FMU exporter). An FMU compiled by any of these tools can — in principle — be dropped into ActDiag's cache directory and used without any code changes.

### How ActDiag uses it

```
scene.yaml physics params
    │
    ▼
_modelica_source()           ← generates a .mo file at runtime
    │
    ▼  (first run only, requires omc on PATH)
omc buildModelFMU()          ← compiles to a Co-Simulation FMU
    │
    ▼
~/.actdiag/fmu_cache/<hash>/ ← cached; subsequent runs skip compilation
    │
    ▼
fmpy FMU2Slave               ← drives the FMU step-by-step
    setReal([tau_cmd], [τ])
    doStep(t, dt)
    getReal([q]), getReal([dq])
```

The cache key is the SHA-256 of the physics parameters (`inertia`, `damping`, `gravity`, `com_x`, `q0`, `dq0`, …). Identical parameters → same FMU, loaded instantly. Different parameters → one new compilation.

### The generated Modelica model

ActDiag generates this Modelica model at runtime (values filled in from `scene.yaml`):

```modelica
model ActDiagSingleJoint
  "Single-revolute-joint ODE (ActDiag auto-generated). I*ddq = tau_cmd - b*dq + m*g*com_x*cos(q)."
  output Real q(start = 0.0, fixed = true)
    "Joint angle (rad)";
  output Real dq(start = 0.0, fixed = true)
    "Joint angular velocity (rad/s)";
  input Real tau_cmd(start = 0.0)
    "Applied torque (N.m)";
equation
  der(q) = dq;
  0.05 * der(dq) = tau_cmd - 0.1 * dq + 1.0 * 9.81 * 0.1414 * cos(q);
end ActDiagSingleJoint;
```

The equation of motion is **identical** to the MuJoCo and PhysX backends:

```
I · q̈  =  τ_cmd  −  b · q̇  +  m · g · com_x · cos(q)
```

The `com_x` offset is derived from `inertia` using the same formula used in `scene.py`:

```
com_x = min(0.15, max(0.03, sqrt(inertia × 0.4 / mass)))
```

### Integrator comparison

| Backend | Integrator | Order | Notes |
|---|---|---|---|
| `mujoco` | RK4 | 4th | Fixed step, symplectic-like energy behaviour |
| `physx` | Semi-implicit Euler | 1st | Slightly dissipative; analytical damping+gravity injected |
| `openmodelica` | DASSL | variable | Implicit multi-step; adaptive internal step; most accurate |

The OpenModelica FMU uses DASSL by default. With `dt = 0.002 s` you will get very close agreement with MuJoCo — any residual difference is numerical (integrator order), not a difference in the physics model.

### Usage

Enable in `scenario.yaml`:

```yaml
simulation:
  duration: 2.5
  dt: 0.002
  backend: openmodelica   # ← only this line changes vs mujoco
```

Or override from the command line without editing any file:

```bash
actdiag run --system examples/system_pd.yaml \
            --scenario examples/scenario_step.yaml \
            -- simulation.backend openmodelica
```

**First run** (cache miss): compiles the Modelica model via `omc` — takes 10–30 s.  
**Subsequent runs** (cache hit): loads the precompiled FMU via `fmpy` — starts in < 1 s.

### Installation

```bash
# 1. Install OpenModelica (provides the omc compiler)
#    macOS:
brew install --cask openmodelica
#    Linux: https://openmodelica.org/download/download-linux/

# Verify omc is on PATH and is arm64 (Apple Silicon):
which omc && file $(which omc)

# 2. Install the Python packages
pip install fmpy OMPython

# 3. (Optional) install as an ActDiag extra
pip install "actdiag[openmodelica]"
```

### Limitations

| Limitation | Detail |
|---|---|
| `omc` required on first run | Only needed to compile the FMU. Cached FMUs run with `fmpy` alone — no OpenModelica needed. |
| arm64 / Apple Silicon | OpenModelica must be a native arm64 or universal-binary build. An x86_64 FMU `.dylib` cannot be loaded by a native arm64 Python process. Verify: `file $(which omc)`. If x86_64-only, compile on an x86_64 machine and copy `~/.actdiag/fmu_cache/` to the arm64 host. |
| FMI 2.0 Co-Simulation | The master cannot control internal solver sub-steps. `dt` is the coarsest integration granularity. |
| Single-joint only | Like all ActDiag backends, this models a single revolute joint. FMI itself is not the limiting factor — the constraint is ActDiag's current single-joint scope. |

---

## 🧪 Tests

```bash
# Install pytest (one-time)
pip install pytest

# Run the full suite (184 tests, ~1 s)
python -m pytest

# With coverage
pip install pytest-cov
python -m pytest --cov=actdiag --cov-report=term-missing
```

The suite covers config parsing and validation, signal generation, actuator and controller logic, MuJoCo scene construction, MuJoCo backend physics (energy conservation, damping, gravity), OpenModelica backend error handling, and sweep metric computation. No external services or optional backends are required — OpenModelica and PhysX tests run in environments without those engines installed and exercise only the error-handling paths.

---

## ⚙️ Requirements
- Python 3.10+
- MuJoCo, NumPy, Pandas, Matplotlib, Pydantic, PyYAML, Rich

### Optional: PhysX backend (Apple Silicon / arm64 macOS)

The NVIDIA PhysX 4 Conan package does not ship an arm64 binary, so pyphysx must be built from source with the arm64 patches applied.

```bash
# 1. Prerequisites
brew install cmake ninja eigen

# 2. Download and patch PhysX 4.1.2 source
curl -L "https://github.com/NVIDIAGameWorks/PhysX/archive/a2c0428acab643e60618c681b501e86f7fd558cc.zip" \
     -o /tmp/physx-src.zip
unzip /tmp/physx-src.zip -d /tmp/
```

Apply the following patches (from NVIDIAGameWorks/PhysX commit `9fb98ab`):
- `pxshared/include/foundation/PxPreprocessor.h` — treat arm64 Mac as macOS, not iOS
- `physx/source/compiler/cmake/mac/CMakeLists.txt` — set `-arch arm64`, remove `-msse2` / `-Werror`
- `physx/source/foundation/src/unix/PsUnixFPU.cpp` — disable x86 SIMD FPU paths on macOS
- `physx/source/geomutils/include/GuSIMDHelpers.h` — use scalar store path on macOS
- `physx/source/physxextensions/src/serialization/SnSerialUtils.cpp` — add macOS ARM platform tag
- `physx/buildtools/presets/public/mac64.xml` — switch to shared libs

Also add `cmake --install` targets to `physx/source/compiler/cmake/mac/CMakeLists.txt`
(see the Conan Center patch `0005-CMake-macos-ios-android-install-targets.patch`).

```bash
# 3. Build PhysX for arm64
PHYSX_SRC="/tmp/PhysX-a2c0428acab643e60618c681b501e86f7fd558cc"
cmake -S "${PHYSX_SRC}/physx/compiler/public" -B /tmp/physx-arm64-build \
      -DCMAKE_BUILD_TYPE=Release \
      -DTARGET_BUILD_PLATFORM=mac \
      -DPX_BUILDSNIPPETS=OFF -DPX_BUILDPUBLICSAMPLES=OFF \
      "-DCMAKEMODULES_PATH=${PHYSX_SRC}/externals/cmakemodules" \
      "-DPHYSX_ROOT_DIR=${PHYSX_SRC}/physx" \
      -DPX_GENERATE_STATIC_LIBRARIES=ON \
      "-DPXSHARED_PATH=${PHYSX_SRC}/pxshared" \
      -DNV_APPEND_CONFIG_NAME=OFF -DNV_USE_GAMEWORKS_OUTPUT_DIRS=OFF \
      -DNV_FORCE_64BIT_SUFFIX=OFF -DNV_FORCE_32BIT_SUFFIX=OFF \
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build /tmp/physx-arm64-build -j$(sysctl -n hw.logicalcpu)

# 4. Clone pyphysx and build against the local arm64 PhysX
git clone https://github.com/petrikvladimir/pyphysx.git /tmp/pyphysx-arm64
```

Edit `/tmp/pyphysx-arm64/CMakeLists.txt` to:
- Remove the `physx/4.1.1` Conan requirement
- Add `include_directories` for PhysX and pxshared headers
- Link against the static libs in `/tmp/physx-arm64-build/sdk_source_bin/`
- Add `tinyobjloader` as a source file in the module (copy `tiny_obj_loader.h/cc` to `include/` and `src/`)
- Add `/opt/homebrew/opt/eigen/include/eigen3` for Eigen

Also add `set_mass_space_inertia_tensor` / `get_mass_space_inertia_tensor` bindings to
`include/RigidDynamic.h` and `src/pyphysx.cpp`.

```bash
# 5. Install the modified pyphysx (no extra Python deps needed)
pip install /tmp/pyphysx-arm64 --no-deps
pip install numpy-quaternion
```

```bash
# 6. Verify
python -c "from pyphysx._pyphysx import Scene, RigidDynamic; s = Scene(); print('pyphysx ok')"
```

### Optional: OpenModelica backend (FMI Co-Simulation)

See the **[🏭 OpenModelica Backend — FMI / FMU](#-openmodelica-backend--fmi--fmu)** section above for full details. Quick install:

```bash
brew install --cask openmodelica   # macOS; for Linux: openmodelica.org/download
pip install fmpy OMPython
# or: pip install "actdiag[openmodelica]"
```

Verify:

```bash
which omc && file $(which omc)   # confirm omc is on PATH and is arm64/universal
python -c "import fmpy; from OMPython import OMCSessionZMQ; print('OK')"
```

# ActDiag Roadmap

---

## 🧭 Roadmap

ActDiag is evolving from a simple simulation tool into a diagnostic framework for actuator behavior.  
The development focuses on adding complexity only when it improves interpretability and fitting capability.

---

### v0.x — Current (Foundation)

- Single-joint simulation with MuJoCo
- Pluggable physics backends: `mujoco` (default), `physx` (NVIDIA PhysX 4 via pyphysx), and `openmodelica` (FMI 2.0 Co-Simulation via OpenModelica / fmpy)
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

- **Start simple, add complexity only when necessary.** One joint, one scenario, one run. Scale up only when the simpler model fails to explain the data.
- **Prefer interpretable parameters over physically complete models.** Every parameter in the config maps to something you can measure or tune, not a numerical coefficient in a solver.
- **Pluggable backends, identical physics.** All three backends (MuJoCo, PhysX, OpenModelica) implement the same equation of motion. Differences you observe between backends are real — integrator order, constraint drift, damping convention — not artifacts of different model definitions. The backend layer exists to expose and quantify those differences, not to hide them.
- **Sim-to-sim before sim-to-real.** If two simulators disagree on your actuator's step response, the discrepancy will show up as a policy transfer gap or an unexplained fitting residual. ActDiag makes those disagreements visible before they cost you hardware experiments.
- **Focus on reproducibility, diagnosability, and fitting robustness.** Every run writes a full config snapshot, so results are always reproducible. Every metric is traceable back to a time series.
