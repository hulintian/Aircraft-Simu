#!/usr/bin/env python3
"""Shared helpers for missile simulation plotting tools."""

from __future__ import annotations

import os
from pathlib import Path


os.environ.setdefault("MPLCONFIGDIR", "/tmp/missile-matplotlib")

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd


STYLE = {
    "missile": "#d62728",
    "target": "#1f77b4",
    "range": "#2ca02c",
    "altitude": "#9467bd",
    "speed": "#ff7f0e",
    "diag": "#17becf",
    "grid": "#d9d9d9",
}


def load_csv(path: str | Path, required_columns: list[str]) -> pd.DataFrame:
    """Load a CSV file and validate required columns."""
    csv_path = Path(path)
    if not csv_path.exists():
        raise SystemExit(f"input file does not exist: {csv_path}")
    data = pd.read_csv(csv_path)
    missing = [name for name in required_columns if name not in data.columns]
    if missing:
        raise SystemExit(f"{csv_path} missing required columns: {', '.join(missing)}")
    if data.empty:
        raise SystemExit(f"{csv_path} has no rows")
    return data


def ensure_output(path: str | Path) -> Path:
    """Create the output directory and return the normalized output path."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    return output


def finish_figure(fig: plt.Figure, output: str | Path, dpi: int = 160) -> None:
    """Apply common layout and write a figure."""
    out = ensure_output(output)
    fig.tight_layout()
    fig.savefig(out, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(out)


def set_axis_style(ax, xlabel: str | None = None, ylabel: str | None = None) -> None:
    """Apply a restrained engineering-report axis style."""
    ax.grid(True, color=STYLE["grid"], linewidth=0.7, alpha=0.8)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)


def maybe_series(data: pd.DataFrame, column: str):
    """Return a series when the column exists, otherwise None."""
    if column in data.columns:
        return data[column]
    return None
