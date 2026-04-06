from __future__ import annotations

import argparse
from pathlib import Path
import sys

from actdiag import __version__
from actdiag.config import load_run_config, run_config_to_dict
from actdiag.logging_io import (
    create_run_paths,
    save_input_configs,
    save_resolved_config,
    save_timeseries,
    save_video,
)
from actdiag.plotting import save_plots
from actdiag.simulate import run_simulation


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="actdiag",
        description="Actuator diagnosis experiments in a minimal MuJoCo scene.",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")

    subparsers = parser.add_subparsers(dest="command", required=True)
    run_parser = subparsers.add_parser("run", help="Run a single actuator experiment.")
    run_parser.add_argument("--actuator", required=True, type=Path, help="Actuator YAML profile.")
    run_parser.add_argument("--scene", required=True, type=Path, help="Scene YAML profile.")
    run_parser.add_argument("--test", required=True, type=Path, help="Test YAML profile.")
    run_parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional output directory. Defaults to runs/<timestamp>.",
    )
    run_parser.add_argument(
        "--save-video",
        action="store_true",
        help="Render an MP4 video into the run directory.",
    )
    run_parser.add_argument(
        "--video-fps",
        type=int,
        default=30,
        help="Video frame rate for exported MP4 files.",
    )
    run_parser.set_defaults(handler=handle_run)
    return parser


def handle_run(args: argparse.Namespace) -> int:
    if args.video_fps <= 0:
        raise ValueError("--video-fps must be positive")

    actuator_path = args.actuator.resolve()
    scene_path = args.scene.resolve()
    test_path = args.test.resolve()
    output_dir = args.output_dir.resolve() if args.output_dir is not None else None

    run_config = load_run_config(actuator_path, scene_path, test_path)
    run_paths = create_run_paths(Path.cwd(), output_dir)
    save_input_configs(run_paths, actuator_path, scene_path, test_path)
    save_resolved_config(run_paths, run_config_to_dict(run_config))

    artifacts = run_simulation(run_config, save_video=args.save_video, video_fps=args.video_fps)
    csv_path = save_timeseries(run_paths, artifacts.timeseries)
    save_plots(artifacts.timeseries, run_paths.figures_dir)

    if args.save_video:
        if artifacts.video_frames is None or artifacts.video_fps is None:
            raise RuntimeError("video export requested but no frames were produced")
        save_video(run_paths, artifacts.video_frames, artifacts.video_fps)

    print(f"Run complete: {run_paths.run_dir}")
    print(f"Timeseries: {csv_path}")
    return 0


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

