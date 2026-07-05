#!/usr/bin/env python3
"""Generate the standard plot set for one simulation instance directory."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate trajectory, time-series, and diagnostic PNGs for one instance directory."
    )
    parser.add_argument("--instance-dir", required=True, help="Directory containing trajectory.csv.")
    parser.add_argument("--output-dir", required=True, help="Directory for generated PNG files.")
    parser.add_argument("--title-prefix", default="", help="Optional title prefix.")
    return parser.parse_args()


def run_tool(script: Path, args: list[str]) -> None:
    command = [sys.executable, str(script)] + args
    subprocess.run(command, check=True)


def main() -> int:
    args = parse_args()
    script_dir = Path(__file__).resolve().parent
    instance_dir = Path(args.instance_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    trajectory = instance_dir / "trajectory.csv"
    diagnostics = instance_dir / "trajectory_diagnostics.csv"
    if not trajectory.exists():
        raise SystemExit(f"missing {trajectory}")
    if not diagnostics.exists():
        raise SystemExit(f"missing {diagnostics}")

    prefix = f"{args.title_prefix} " if args.title_prefix else ""
    run_tool(
        script_dir / "plot_trajectory.py",
        [
            "--trajectory",
            str(trajectory),
            "--output",
            str(output_dir / "trajectory.png"),
            "--title",
            f"{prefix}trajectory",
        ],
    )
    run_tool(
        script_dir / "plot_timeseries.py",
        [
            "--trajectory",
            str(trajectory),
            "--output",
            str(output_dir / "timeseries.png"),
            "--title",
            f"{prefix}flight time series",
        ],
    )
    run_tool(
        script_dir / "plot_diagnostics.py",
        [
            "--diagnostics",
            str(diagnostics),
            "--output",
            str(output_dir / "diagnostics.png"),
            "--title",
            f"{prefix}diagnostics",
        ],
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
