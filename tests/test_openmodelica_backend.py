"""Tests for actdiag.backends.openmodelica_backend — unit tests that do not require omc."""
from __future__ import annotations

import hashlib
import json

import pytest

from actdiag.backends.openmodelica_backend import (
    _MODEL_NAME,
    _cache_key,
    _modelica_source,
    _resolve_vr,
)


# ---------------------------------------------------------------------------
# _cache_key
# ---------------------------------------------------------------------------

class TestCacheKey:
    def test_deterministic(self):
        params = {"inertia": 0.05, "damping": 0.1, "gravity": 0}
        assert _cache_key(params) == _cache_key(params)

    def test_key_length_16(self):
        params = {"inertia": 0.05}
        assert len(_cache_key(params)) == 16

    def test_different_params_different_key(self):
        k1 = _cache_key({"inertia": 0.05, "damping": 0.1})
        k2 = _cache_key({"inertia": 0.10, "damping": 0.1})
        assert k1 != k2

    def test_key_order_independent(self):
        """Keys should be the same regardless of dict insertion order."""
        k1 = _cache_key({"a": 1, "b": 2})
        k2 = _cache_key({"b": 2, "a": 1})
        assert k1 == k2

    def test_gravity_bool_vs_int(self):
        """Boolean and integer representation must produce the same key."""
        k1 = _cache_key({"gravity": int(True)})
        k2 = _cache_key({"gravity": 1})
        assert k1 == k2


# ---------------------------------------------------------------------------
# _modelica_source
# ---------------------------------------------------------------------------

class TestModelicaSource:
    def _src(self, **kwargs):
        defaults = dict(
            inertia=0.05, damping=0.1, mass=1.0, g=9.81,
            com_x=0.1, gravity=False, q0=0.0, dq0=0.0,
        )
        defaults.update(kwargs)
        return _modelica_source(**defaults)

    def test_model_name_present(self):
        src = self._src()
        assert _MODEL_NAME in src

    def test_contains_equation_keyword(self):
        src = self._src()
        assert "equation" in src

    def test_inertia_in_source(self):
        src = self._src(inertia=0.07)
        assert "0.07" in src

    def test_damping_in_source(self):
        src = self._src(damping=0.25)
        assert "0.25" in src

    def test_initial_angle_in_source(self):
        src = self._src(q0=1.234)
        assert "1.234" in src

    def test_gravity_term_absent_when_disabled(self):
        src = self._src(gravity=False)
        assert "cos(q)" not in src

    def test_gravity_term_present_when_enabled(self):
        src = self._src(gravity=True, com_x=0.1, mass=1.0, g=9.81)
        assert "cos(q)" in src

    def test_tau_cmd_declared_as_input(self):
        src = self._src()
        assert "input Real tau_cmd" in src

    def test_q_declared_as_output(self):
        src = self._src()
        assert "output Real q" in src

    def test_dq_declared_as_output(self):
        src = self._src()
        assert "output Real dq" in src

    def test_der_q_equation_present(self):
        src = self._src()
        assert "der(q)" in src

    def test_der_dq_equation_present(self):
        src = self._src()
        assert "der(dq)" in src

    def test_ends_with_end_statement(self):
        src = self._src()
        assert f"end {_MODEL_NAME};" in src

    def test_valid_modelica_structure(self):
        """Source should open with model and close with end."""
        src = self._src()
        lines = [l.strip() for l in src.strip().splitlines() if l.strip()]
        assert lines[0].startswith(f"model {_MODEL_NAME}")
        assert lines[-1] == f"end {_MODEL_NAME};"


# ---------------------------------------------------------------------------
# _resolve_vr
# ---------------------------------------------------------------------------

class TestResolveVr:
    def test_finds_first_candidate(self):
        vrs = {"q": 1, "dq": 2, "tau_cmd": 3}
        assert _resolve_vr(vrs, "q") == 1

    def test_falls_back_to_second_candidate(self):
        vrs = {"ActDiagSingleJoint.q": 99}
        assert _resolve_vr(vrs, "q", "ActDiagSingleJoint.q") == 99

    def test_raises_key_error_when_not_found(self):
        vrs = {"other": 5}
        with pytest.raises(KeyError, match="not found"):
            _resolve_vr(vrs, "q", "ActDiagSingleJoint.q")

    def test_error_message_lists_available(self):
        vrs = {"alpha": 1, "beta": 2}
        with pytest.raises(KeyError) as exc_info:
            _resolve_vr(vrs, "gamma")
        assert "alpha" in str(exc_info.value) or "beta" in str(exc_info.value)


# ---------------------------------------------------------------------------
# OpenModelicaBackend instantiation — error when omc not available
# ---------------------------------------------------------------------------

class TestOpenModelicaBackendInit:
    def test_missing_omc_raises_runtime_error(self):
        from actdiag.backends.openmodelica_backend import OpenModelicaBackend
        from actdiag.config import SingleJointConfig, SingleJointSceneProfile

        profile = SingleJointSceneProfile(
            scene_type="single_joint",
            joint=SingleJointConfig(inertia=0.05, damping=0.1),
        )
        with pytest.raises(RuntimeError, match="omc"):
            OpenModelicaBackend(profile, dt=0.002)

    def test_error_contains_install_instructions(self):
        from actdiag.backends.openmodelica_backend import OpenModelicaBackend
        from actdiag.config import SingleJointConfig, SingleJointSceneProfile

        profile = SingleJointSceneProfile(
            scene_type="single_joint",
            joint=SingleJointConfig(inertia=0.05, damping=0.1),
        )
        try:
            OpenModelicaBackend(profile, dt=0.002)
        except RuntimeError as exc:
            msg = str(exc)
            assert "openmodelica.org" in msg or "brew" in msg
