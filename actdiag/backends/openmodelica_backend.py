"""OpenModelica / FMI Co-Simulation physics backend.

OpenModelica compiles the single-joint equation-of-motion as a Modelica model
to a Co-Simulation FMU (FMI 2.0); ``fmpy`` runs it step-by-step in Python
without any MATLAB license.

Physics model (identical to the MuJoCo and PhysX backends)
-----------------------------------------------------------
    I · q̈  =  τ_cmd  −  b · q̇  [+  m · g · com_x · cos(q)]

Installation
------------
1. Install OpenModelica (provides the ``omc`` compiler):

       macOS:   brew install --cask openmodelica
       Linux:   https://openmodelica.org/download/download-linux/

   After installation ensure ``omc`` is on your ``PATH``.

2. Install the Python packages::

       pip install fmpy OMPython

FMU caching
-----------
The first run for a given set of physics parameters compiles a Modelica model
to a Co-Simulation FMU and stores it in ``~/.actdiag/fmu_cache/<hash>/``.
Subsequent runs with identical parameters load the cached FMU directly without
invoking OpenModelica again (fmpy is still required at runtime).

arm64 / Apple Silicon note
--------------------------
If the OpenModelica installer you used is x86_64-only (running under Rosetta),
the generated FMU shared library will be x86_64 and will fail to load in a
native arm64 Python process.  Confirm your build with::

    file $(which omc)

Use the arm64 or universal-binary build of OpenModelica to avoid this.
"""
from __future__ import annotations

import hashlib
import json
import math
import shutil
import tempfile
from pathlib import Path

from actdiag.config import SingleJointSceneProfile

# ---------------------------------------------------------------------------
# Optional dependency guards
# ---------------------------------------------------------------------------

try:
    from fmpy import extract as _fmpy_extract
    from fmpy import read_model_description as _fmpy_rmd
    from fmpy.fmi2 import FMU2Slave

    _FMPY_AVAILABLE = True
except ImportError:
    _FMPY_AVAILABLE = False


def _require_fmpy() -> None:
    if not _FMPY_AVAILABLE:
        raise ImportError(
            "The openmodelica backend requires fmpy.\n"
            "Install it with:  pip install fmpy OMPython\n"
            "Then install OpenModelica: https://openmodelica.org/download/"
        )


def _omc_on_path() -> bool:
    import shutil as _sh

    return _sh.which("omc") is not None


def _require_omc() -> None:
    if not _omc_on_path():
        raise RuntimeError(
            "OpenModelica compiler (omc) not found on PATH.\n"
            "Install OpenModelica first:\n"
            "  macOS:   brew install --cask openmodelica\n"
            "  Linux:   https://openmodelica.org/download/download-linux/\n"
            "After installation, ensure 'omc' is on your PATH and retry.\n"
            "If omc is already installed but not in PATH, add its directory to\n"
            "  your shell profile (e.g. export PATH=\"/opt/openmodelica/bin:$PATH\")."
        )


# ---------------------------------------------------------------------------
# FMU building and caching
# ---------------------------------------------------------------------------

_CACHE_ROOT = Path.home() / ".actdiag" / "fmu_cache"
_MODEL_NAME = "ActDiagSingleJoint"


def _cache_key(params: dict) -> str:
    canonical = json.dumps(params, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode()).hexdigest()[:16]


def _modelica_source(
    inertia: float,
    damping: float,
    mass: float,
    g: float,
    com_x: float,
    gravity: bool,
    q0: float,
    dq0: float,
) -> str:
    """Return the Modelica source for the single-joint ODE."""
    grav_term = (
        f"+ {mass:.17g} * {g:.17g} * {com_x:.17g} * cos(q)"
        if gravity
        else ""
    )
    phys_formula = "I*ddq = tau_cmd - b*dq" + (
        " + m*g*com_x*cos(q)" if gravity else ""
    )
    lines = [
        f"model {_MODEL_NAME}",
        f'  "Single-revolute-joint ODE (ActDiag auto-generated). {phys_formula}."',
        f"  output Real q(start = {q0:.17g}, fixed = true)",
        f'    "Joint angle (rad)";',
        f"  output Real dq(start = {dq0:.17g}, fixed = true)",
        f'    "Joint angular velocity (rad/s)";',
        f"  input Real tau_cmd(start = 0.0)",
        f'    "Applied torque (N.m)";',
        "equation",
        "  der(q) = dq;",
        (
            f"  {inertia:.17g} * der(dq) = "
            f"tau_cmd - {damping:.17g} * dq {grav_term};"
        ),
        f"end {_MODEL_NAME};",
        "",
    ]
    return "\n".join(lines)


def _compile_fmu(model_src: str, dest_fmu: Path) -> None:
    """Compile a Modelica source string to a Co-Simulation FMU (FMI 2.0)."""
    try:
        from OMPython import OMCSessionZMQ  # type: ignore[import]
    except ImportError:
        raise ImportError(
            "OMPython is required to compile Modelica FMUs.\n"
            "Install it with:  pip install OMPython"
        )

    _require_omc()
    dest_fmu.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="actdiag_om_") as tmpdir:
        tmp = Path(tmpdir)
        mo_file = tmp / f"{_MODEL_NAME}.mo"
        mo_file.write_text(model_src, encoding="utf-8")

        omc = OMCSessionZMQ()
        try:
            omc.sendExpression(f'cd("{tmpdir}")')

            ok = omc.sendExpression(f'loadFile("{mo_file!s}")')
            if not ok:
                err = omc.sendExpression("getErrorString()")
                raise RuntimeError(
                    f"OpenModelica could not load the generated model:\n{err}"
                )

            result: str = omc.sendExpression(
                f'buildModelFMU({_MODEL_NAME}, version="2.0", fmuType="cs")'
            )
            err_str: str = omc.sendExpression("getErrorString()")

            if not result or result in ('""', ""):
                raise RuntimeError(
                    f"OpenModelica buildModelFMU failed.\n{err_str}"
                )

            # OMC returns the FMU path as a quoted string: '"/tmp/.../Foo.fmu"'
            fmu_candidate = Path(result.strip('"'))
            if not fmu_candidate.exists():
                # Some OMC versions return a relative path → resolve vs tmpdir
                fmu_candidate = tmp / f"{_MODEL_NAME}.fmu"
            if not fmu_candidate.exists():
                raise RuntimeError(
                    f"buildModelFMU returned {result!r} but "
                    f"{_MODEL_NAME}.fmu was not found.\n{err_str}"
                )

            shutil.copy2(str(fmu_candidate), str(dest_fmu))
        finally:
            try:
                omc.__del__()
            except Exception:
                pass


def _get_or_build_fmu(
    inertia: float,
    damping: float,
    mass: float,
    g: float,
    com_x: float,
    gravity: bool,
    q0: float,
    dq0: float,
) -> Path:
    """Return the path to a cached FMU, building it if necessary."""
    params: dict = {
        "com_x": com_x,
        "damping": damping,
        "g": g,
        "gravity": int(gravity),
        "inertia": inertia,
        "mass": mass,
        "q0": q0,
        "dq0": dq0,
    }
    key = _cache_key(params)
    fmu_path = _CACHE_ROOT / key / f"{_MODEL_NAME}.fmu"

    if fmu_path.exists():
        return fmu_path

    model_src = _modelica_source(inertia, damping, mass, g, com_x, gravity, q0, dq0)
    _compile_fmu(model_src, fmu_path)
    return fmu_path


# ---------------------------------------------------------------------------
# Variable-reference helper
# ---------------------------------------------------------------------------


def _resolve_vr(vrs: dict[str, int], *candidates: str) -> int:
    """Return the FMI value reference for the first matching variable name."""
    for name in candidates:
        if name in vrs:
            return vrs[name]
    available = sorted(vrs)
    raise KeyError(
        f"FMI variable not found (tried {candidates!r}).  "
        f"Available variables: {available}"
    )


# ---------------------------------------------------------------------------
# Backend class
# ---------------------------------------------------------------------------


class OpenModelicaBackend:
    """FMI Co-Simulation backend driven by an OpenModelica-compiled FMU.

    Implements the same single-revolute-joint physics as the MuJoCo and PhysX
    backends:

        I · q̈  =  τ_cmd  −  b · q̇  [+  m · g · com_x · cos(q)]

    The Modelica model is compiled once and cached in
    ``~/.actdiag/fmu_cache/``.  Subsequent runs with identical physics
    parameters skip compilation and load the cached FMU directly.
    """

    name = "openmodelica"

    def __init__(self, scene_profile: SingleJointSceneProfile, dt: float) -> None:
        _require_fmpy()

        joint = scene_profile.joint
        inertia = float(joint.inertia)
        damping = float(joint.damping)
        gravity = bool(joint.gravity)
        q0 = float(joint.q0)
        dq0 = float(joint.dq0)

        _mass = 1.0
        _g = 9.81
        com_x = min(0.15, max(0.03, math.sqrt(inertia * 0.4 / _mass)))

        fmu_path = _get_or_build_fmu(
            inertia=inertia,
            damping=damping,
            mass=_mass,
            g=_g,
            com_x=com_x,
            gravity=gravity,
            q0=q0,
            dq0=dq0,
        )

        self.dt = dt
        self._t: float = 0.0
        self._q: float = q0
        self._dq: float = dq0

        # Initialize these so __del__ is always safe even if init fails
        self._fmu: FMU2Slave | None = None  # type: ignore[name-defined]
        self._unzip_dir: Path | None = None

        # --- Read model description to get variable value references -----
        model_desc = _fmpy_rmd(str(fmu_path))
        vrs: dict[str, int] = {
            v.name: v.valueReference for v in model_desc.modelVariables
        }

        self._vr_q = _resolve_vr(vrs, "q", f"{_MODEL_NAME}.q")
        self._vr_dq = _resolve_vr(vrs, "dq", f"{_MODEL_NAME}.dq")
        self._vr_tau = _resolve_vr(vrs, "tau_cmd", f"{_MODEL_NAME}.tau_cmd")

        # --- Extract FMU (stable cached extraction) ----------------------
        unzip_dir = fmu_path.parent / "extracted"
        if not (unzip_dir / "modelDescription.xml").exists():
            _fmpy_extract(str(fmu_path), unzipdir=str(unzip_dir))
        self._unzip_dir = unzip_dir

        # --- Instantiate FMU slave ---------------------------------------
        self._fmu = FMU2Slave(
            guid=model_desc.guid,
            unzipDirectory=str(unzip_dir),
            modelIdentifier=model_desc.coSimulation.modelIdentifier,
            instanceName="actdiag",
        )
        self._fmu.instantiate()
        self._fmu.setupExperiment(startTime=0.0)
        self._fmu.enterInitializationMode()
        self._fmu.exitInitializationMode()

    # ------------------------------------------------------------------
    # PhysicsBackend interface
    # ------------------------------------------------------------------

    def get_state(self) -> tuple[float, float]:
        return self._q, self._dq

    def apply_torque_and_step(self, torque: float) -> None:
        assert self._fmu is not None
        self._fmu.setReal([self._vr_tau], [float(torque)])
        self._fmu.doStep(
            currentCommunicationPoint=self._t,
            communicationStepSize=self.dt,
        )
        self._t += self.dt
        self._q = float(self._fmu.getReal([self._vr_q])[0])
        self._dq = float(self._fmu.getReal([self._vr_dq])[0])

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def __del__(self) -> None:
        fmu = getattr(self, "_fmu", None)
        if fmu is not None:
            try:
                fmu.terminate()
            except Exception:
                pass
            try:
                fmu.freeInstance()
            except Exception:
                pass
