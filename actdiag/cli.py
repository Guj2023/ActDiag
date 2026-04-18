from __future__ import annotations

import argparse
from pathlib import Path
import sys

from actdiag import __version__
from actdiag.config import (
    FrequencyResponseTestProfile,
    load_run_config,
    run_config_to_dict,
)
from actdiag.logging_io import (
    create_run_paths,
    frequency_slug,
    save_frequency_response_summary,
    save_frequency_response_timeseries,
    save_input_configs,
    save_resolved_config,
    save_step_metrics,
    save_timeseries,
    save_video,
)
from actdiag.plotting import save_frequency_response_plots, save_plots
from actdiag.simulate import run_frequency_response_simulation, run_simulation


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="actdiag",
        description="Actuator diagnosis experiments in a minimal MuJoCo scene.",
    )
    parser.add_argument(
        "--version", action="version", version=f"%(prog)s {__version__}"
    )

    subparsers = parser.add_subparsers(dest="command", required=True)
    
    # Run command
    run_parser = subparsers.add_parser("run", help="Run a single actuator experiment.")
    run_parser.add_argument(
        "--system", required=True, type=Path, help="System YAML profile."
    )
    run_parser.add_argument(
        "--scenario", required=True, type=Path, help="Scenario YAML profile."
    )
    run_parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional output directory. Defaults to runs/<timestamp>.",
    )
    run_parser.add_argument(
        "--save-video",
        action="store_true",
        default=None,
        help="Render an MP4 video into the run directory.",
    )
    run_parser.add_argument(
        "--video-fps",
        type=int,
        default=30,
        help="Video frame rate for exported MP4 files.",
    )
    run_parser.set_defaults(handler=handle_run)

    # Fit command
    fit_parser = subparsers.add_parser("fit", help="Fit system parameters to a reference trajectory.")
    fit_parser.add_argument(
        "--system", required=True, type=Path, help="System YAML profile."
    )
    fit_parser.add_argument(
        "--scenario", required=True, type=Path, help="Scenario YAML profile."
    )
    fit_parser.add_argument(
        "--reference", type=Path, help="Optional reference CSV file. If not provided, uses desired trajectory."
    )
    fit_parser.add_argument(
        "--search", required=True, type=Path, help="Search space YAML file."
    )
    fit_parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="Output directory for fit results.",
    )
    fit_parser.set_defaults(handler=handle_fit)

    sweep_parser = subparsers.add_parser(
        "sweep", help="Run a 1D or 2D parameter sweep."
    )
    sweep_parser.add_argument(
        "--system", required=True, type=Path, help="System YAML profile."
    )
    sweep_parser.add_argument(
        "--scenario", required=True, type=Path, help="Scenario YAML profile."
    )
    sweep_parser.add_argument(
        "--sweep", required=True, type=Path, help="Sweep YAML profile."
    )
    sweep_parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="Output directory for sweep results.",
    )
    sweep_parser.set_defaults(handler=handle_sweep)
    
    return parser


def handle_run(args: argparse.Namespace) -> int:
    if args.video_fps <= 0:
        raise ValueError("--video-fps must be positive")

    system_path = args.system.resolve()
    scenario_path = args.scenario.resolve()
    output_dir = args.output_dir.resolve() if args.output_dir is not None else None

    run_config = load_run_config(system_path, scenario_path)
    run_paths = create_run_paths(Path.cwd(), output_dir)
    save_input_configs(run_paths, system_path, scenario_path)
    save_resolved_config(run_paths, run_config_to_dict(run_config))

    should_save_video = bool(args.save_video or run_config.output.save_video)

    if isinstance(run_config.test, FrequencyResponseTestProfile):
        if should_save_video:
            raise ValueError(
                "video export is not supported for frequency_response tests"
            )

        artifacts = run_frequency_response_simulation(run_config)
        summary_path = save_frequency_response_summary(run_paths, artifacts.summary)

        if run_config.logging.save_csv:
            for frequency_hz, timeseries in artifacts.per_frequency_timeseries.items():
                save_frequency_response_timeseries(run_paths, frequency_hz, timeseries)

        for frequency_hz, timeseries in artifacts.per_frequency_timeseries.items():
            save_plots(
                timeseries,
                run_paths.figures_dir
                / "frequency_response"
                / frequency_slug(frequency_hz),
                run_config.plots,
            )
        save_frequency_response_plots(
            artifacts.summary,
            run_paths.figures_dir,
            run_config.plots,
        )

        print(f"Run complete: {run_paths.run_dir}")
        print(f"Frequency response summary: {summary_path}")
        return 0

    artifacts = run_simulation(
        run_config, save_video=should_save_video, video_fps=args.video_fps
    )
    csv_path = None
    if run_config.logging.save_csv:
        csv_path = save_timeseries(run_paths, artifacts.timeseries)
    save_plots(artifacts.timeseries, run_paths.figures_dir, run_config.plots)

    step_metrics_path = None
    if artifacts.summary_metrics is not None:
        step_metrics_path = save_step_metrics(run_paths, artifacts.summary_metrics)

    if should_save_video:
        if artifacts.video_frames is None or artifacts.video_fps is None:
            raise RuntimeError("video export requested but no frames were produced")
        save_video(run_paths, artifacts.video_frames, artifacts.video_fps)

    print(f"Run complete: {run_paths.run_dir}")
    if csv_path is not None:
        print(f"Timeseries: {csv_path}")
    if step_metrics_path is not None:
        print(f"Step metrics: {step_metrics_path}")
    return 0


def handle_fit(args: argparse.Namespace) -> int:
    from actdiag.fit import run_fit

    reference_path = args.reference.resolve() if args.reference is not None else None

    return run_fit(
        system_path=args.system.resolve(),
        scenario_path=args.scenario.resolve(),
        reference_path=reference_path,
        search_path=args.search.resolve(),
        output_dir=args.output_dir.resolve(),
    )


def handle_sweep(args: argparse.Namespace) -> int:
    from actdiag.sweep import run_sweep

    return run_sweep(
        system_path=args.system.resolve(),
        scenario_path=args.scenario.resolve(),
        sweep_path=args.sweep.resolve(),
        output_dir=args.output_dir.resolve(),
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        return args.handler(args)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
