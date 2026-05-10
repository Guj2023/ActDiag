"""Tests for actdiag.sweep — parameter overrides and metrics computation."""
from __future__ import annotations

import math

import numpy as np
import pandas as pd
import pytest

from actdiag.sweep import (
    _apply_parameter_override,
    _case_slug,
    _compute_jitter_metric,
    _compute_metrics,
    _default_metric_values,
    _is_stable,
    _path_slug,
    _value_slug,
)


# ---------------------------------------------------------------------------
# _apply_parameter_override
# ---------------------------------------------------------------------------

class TestApplyParameterOverride:
    def test_simple_key(self):
        data = {"controller": {"kp": 5.0}}
        _apply_parameter_override(data, "controller.kp", 10.0)
        assert data["controller"]["kp"] == 10.0

    def test_nested_path(self):
        data = {"a": {"b": {"c": 1.0}}}
        _apply_parameter_override(data, "a.b.c", 99.0)
        assert data["a"]["b"]["c"] == 99.0

    def test_unknown_root_raises(self):
        data = {"controller": {"kp": 5.0}}
        with pytest.raises(KeyError):
            _apply_parameter_override(data, "nonexistent.kp", 1.0)

    def test_unknown_leaf_raises(self):
        data = {"controller": {"kp": 5.0}}
        with pytest.raises(KeyError):
            _apply_parameter_override(data, "controller.nosuchfield", 1.0)


# ---------------------------------------------------------------------------
# Slug helpers
# ---------------------------------------------------------------------------

class TestSlugs:
    def test_path_slug_replaces_dots(self):
        assert _path_slug("controller.kp") == "controller_kp"

    def test_value_slug_integer(self):
        assert _value_slug(5.0) == "5"

    def test_value_slug_negative(self):
        assert _value_slug(-1.5) == "neg_1_5"

    def test_value_slug_decimal(self):
        s = _value_slug(0.1)
        assert s == "0_1"

    def test_case_slug_single_param(self):
        slug = _case_slug({"controller.kp": 10.0})
        assert "controller_kp" in slug
        assert "10" in slug

    def test_case_slug_two_params(self):
        slug = _case_slug({"controller.kp": 5.0, "controller.kd": 0.5})
        assert "controller_kp" in slug
        assert "controller_kd" in slug


# ---------------------------------------------------------------------------
# _default_metric_values
# ---------------------------------------------------------------------------

class TestDefaultMetricValues:
    def test_float_metrics_are_nan(self):
        vals = _default_metric_values(["tracking_rmse", "max_abs_error"])
        assert math.isnan(vals["tracking_rmse"])
        assert math.isnan(vals["max_abs_error"])

    def test_stable_is_false(self):
        vals = _default_metric_values(["stable"])
        assert vals["stable"] is False


# ---------------------------------------------------------------------------
# _is_stable
# ---------------------------------------------------------------------------

class TestIsStable:
    def _df(self, q=None, dq=None, q_des=None, dq_des=None, n=100):
        t = np.linspace(0, 1, n)
        return pd.DataFrame({
            "q":     np.ones(n) * 1.0 if q is None else np.array(q),
            "dq":    np.zeros(n) if dq is None else np.array(dq),
            "q_des": np.ones(n) * 1.0 if q_des is None else np.array(q_des),
            "dq_des": np.zeros(n) if dq_des is None else np.array(dq_des),
        })

    def test_stable_trajectory(self):
        assert _is_stable(self._df()) is True

    def test_empty_df_not_stable(self):
        assert _is_stable(pd.DataFrame()) is False

    def test_nan_values_not_stable(self):
        df = self._df()
        df.loc[50, "q"] = float("nan")
        assert _is_stable(df) is False

    def test_huge_position_not_stable(self):
        df = self._df(q=np.ones(100) * 1000.0)
        assert _is_stable(df) is False

    def test_huge_velocity_not_stable(self):
        df = self._df(dq=np.ones(100) * 500.0)
        assert _is_stable(df) is False

    def test_missing_columns_not_stable(self):
        df = pd.DataFrame({"q": [1.0], "dq": [0.0]})  # missing q_des, dq_des
        assert _is_stable(df) is False


# ---------------------------------------------------------------------------
# _compute_jitter_metric
# ---------------------------------------------------------------------------

class TestComputeJitterMetric:
    def test_constant_signal_is_small(self):
        # A constant signal has zero true jitter; the only non-zero residual comes
        # from np.convolve(mode='same') edge effects at the two boundary samples.
        # For 100 samples and window=3 the RMSE is sqrt(2*(1/3)^2/100) ≈ 0.047.
        err = np.ones(100)
        assert _compute_jitter_metric(err, dt=0.01) < 0.1

    def test_linear_signal_low_jitter(self):
        err = np.linspace(0, 1, 200)
        j = _compute_jitter_metric(err, dt=0.01)
        assert j < 0.05

    def test_noisy_signal_higher_jitter(self):
        rng = np.random.default_rng(42)
        smooth = np.linspace(0, 1, 200)
        noisy = smooth + rng.normal(0, 0.1, 200)
        j_smooth = _compute_jitter_metric(smooth, dt=0.01)
        j_noisy = _compute_jitter_metric(noisy, dt=0.01)
        assert j_noisy > j_smooth

    def test_empty_array_is_nan(self):
        assert math.isnan(_compute_jitter_metric(np.array([]), dt=0.01))

    def test_tiny_array_is_zero(self):
        assert _compute_jitter_metric(np.array([1.0, 2.0, 3.0]), dt=0.01) == 0.0


# ---------------------------------------------------------------------------
# _compute_metrics
# ---------------------------------------------------------------------------

class TestComputeMetrics:
    def _df(self, n=200, error_amp=0.1):
        t = np.linspace(0, 2, n)
        q_des = np.ones(n)
        q = q_des + error_amp * np.sin(2 * np.pi * t)
        return pd.DataFrame({
            "q": q, "dq": np.zeros(n),
            "q_des": q_des, "dq_des": np.zeros(n),
            "position_error": q_des - q,
        })

    def test_tracking_rmse(self):
        df = self._df(error_amp=0.1)
        metrics = _compute_metrics(df, ["tracking_rmse"], dt=0.01)
        assert metrics["tracking_rmse"] == pytest.approx(0.1 / math.sqrt(2), rel=0.05)

    def test_max_abs_error(self):
        df = self._df(error_amp=0.5)
        metrics = _compute_metrics(df, ["max_abs_error"], dt=0.01)
        assert metrics["max_abs_error"] == pytest.approx(0.5, abs=0.05)

    def test_stable_true(self):
        df = self._df()
        metrics = _compute_metrics(df, ["stable"], dt=0.01)
        assert metrics["stable"] is True

    def test_stable_false_for_diverging(self):
        n = 200
        df = pd.DataFrame({
            "q": np.linspace(0, 1000, n),
            "dq": np.zeros(n),
            "q_des": np.ones(n),
            "dq_des": np.zeros(n),
            "position_error": np.ones(n) - np.linspace(0, 1000, n),
        })
        metrics = _compute_metrics(df, ["stable"], dt=0.01)
        assert metrics["stable"] is False

    def test_jitter_metric_returned(self):
        df = self._df()
        metrics = _compute_metrics(df, ["jitter_metric"], dt=0.01)
        assert "jitter_metric" in metrics
        assert math.isfinite(metrics["jitter_metric"])
