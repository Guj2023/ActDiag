from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import pandas as pd

from actdiag.config import load_run_config, load_yaml_mapping, StepTestProfile
from actdiag.fit.config import load_search_config
from actdiag.logging_io import make_output_dir
from actdiag.fit.evaluator import (
    evaluate_sample,
    interpolate_reference,
    load_reference,
)
from actdiag.fit.sampler import sample_parameters
from actdiag.fit.reporter import save_fit_results
from actdiag.signals import build_signal_series


def run_fit(
    system_path: Path,
    scenario_path: Path,
    reference_path: Path | None,
    search_path: Path,
    output_dir: Path | None = None,
) -> int:
    output_dir = make_output_dir(Path.cwd(), "fits", output_dir)

    # 1. Load configs
    run_config = load_run_config(system_path, scenario_path)
    system_data = load_yaml_mapping(system_path)
    fit_config = load_search_config(search_path)
    
    # Generate signals to get the simulation time grid and desired trajectory
    signals = build_signal_series(run_config.test, run_config.simulation)
    
    # 2. Load and prepare reference
    if reference_path is not None:
        print(f"Loading reference from {reference_path}...")
        reference_raw = load_reference(reference_path)
        reference_interpolated = interpolate_reference(reference_raw, signals.time)
    else:
        print("No reference provided. Using desired trajectory from scenario as reference.")
        # Create a reference from desired signals
        reference_interpolated = pd.DataFrame({
            "time": signals.time,
            "q": signals.q_des,
            "dq": signals.dq_des,
            "tau_applied": signals.tau_des,
        })
    
    # Ensure q_des and other signals are available for metrics/plotting
    reference_interpolated["q_des"] = signals.q_des
    reference_interpolated["dq_des"] = signals.dq_des
    reference_interpolated["tau_des"] = signals.tau_des

    # 3. Sample parameters
    samples = sample_parameters(fit_config)
    
    # Pre-compute reference metrics if it's a step test
    reference_metrics = None
    if isinstance(run_config.test, StepTestProfile):
        from actdiag.simulate import compute_step_response_metrics
        reference_metrics = compute_step_response_metrics(reference_interpolated, run_config.test)

    # 4. Evaluation loop
    results = []
    best_result = None
    
    print(f"Fitting {len(samples)} samples...")
    for i, sample in enumerate(samples):
        if (i + 1) % 20 == 0:
            print(f"  Processed {i + 1}/{len(samples)} samples...")
            
        result = evaluate_sample(
            sample, 
            run_config, 
            reference_interpolated, 
            fit_config.objective,
            reference_metrics=reference_metrics
        )
        results.append(result)
        
        if best_result is None or result.total_cost < best_result.total_cost:
            best_result = result

    if best_result is None or best_result.total_cost >= 1e9:
        print("Error: All simulations failed.")
        return 1

    # 5. Build best-fit system config by patching the original system YAML
    best_system_data = _apply_params_to_dict(system_data, best_result.parameters)

    # 6. Save results
    save_fit_results(
        output_dir,
        results,
        best_result,
        run_config.plots,
        reference_interpolated,
        best_system_data,
    )

    print(f"Fit complete. Best cost: {best_result.total_cost:.6f}")
    print(f"Results saved to: {output_dir}")
    print(f"Best parameters: {best_result.parameters}")

    return 0


def _apply_params_to_dict(
    system_data: dict[str, Any], parameters: dict[str, float]
) -> dict[str, Any]:
    """Return a deep copy of system_data with each parameter path applied."""
    data = copy.deepcopy(system_data)
    for path, value in parameters.items():
        parts = path.split(".")
        node: Any = data
        for part in parts[:-1]:
            node = node[part]
        node[parts[-1]] = value
    return data
