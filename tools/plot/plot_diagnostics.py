#!/usr/bin/env python3
"""Plot numerical diagnostics from trajectory_diagnostics.csv."""

from __future__ import annotations

import argparse

from plot_common import STYLE, finish_figure, load_csv, set_axis_style

import matplotlib.pyplot as plt


REQUIRED_COLUMNS = [
    "time_s",
    "quat_norm_error",
    "dcm_orthogonality_error",
    "mass_kg",
    "propellant_mass_kg",
    "inertia_min_diag_kgm2",
    "aero_model_flags",
    "model_degradation_flags",
    "aero_uncertainty_scale",
    "force_norm_n",
    "moment_norm_nm",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create numerical diagnostic plots from trajectory_diagnostics.csv."
    )
    parser.add_argument("--diagnostics", required=True, help="Input trajectory_diagnostics.csv path.")
    parser.add_argument("--output", required=True, help="Output PNG path.")
    parser.add_argument("--title", default="Numerical diagnostics", help="Figure title.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    data = load_csv(args.diagnostics, REQUIRED_COLUMNS)
    t = data["time_s"]

    fig, axes = plt.subplots(3, 2, figsize=(15, 11), sharex=True)
    fig.suptitle(args.title, fontsize=15, fontweight="bold")

    axes[0, 0].semilogy(t, data["quat_norm_error"].clip(lower=1e-18), color=STYLE["diag"], linewidth=2.0)
    axes[0, 0].set_title("Quaternion norm error")
    set_axis_style(axes[0, 0], ylabel="abs error")

    axes[0, 1].semilogy(
        t,
        data["dcm_orthogonality_error"].clip(lower=1e-18),
        color="#bcbd22",
        linewidth=2.0,
    )
    axes[0, 1].set_title("DCM orthogonality error")
    set_axis_style(axes[0, 1], ylabel="matrix error")

    axes[1, 0].plot(t, data["mass_kg"], color="#4c78a8", linewidth=2.0, label="mass")
    axes[1, 0].plot(t, data["propellant_mass_kg"], color="#72b7b2", linewidth=1.7, label="propellant")
    axes[1, 0].set_title("Mass state")
    set_axis_style(axes[1, 0], ylabel="kg")
    axes[1, 0].legend(loc="best")

    axes[1, 1].plot(t, data["inertia_min_diag_kgm2"], color="#9467bd", linewidth=2.0)
    axes[1, 1].set_title("Minimum inertia diagonal")
    set_axis_style(axes[1, 1], ylabel="kg*m^2")

    axes[2, 0].plot(t, data["force_norm_n"], color="#f58518", linewidth=2.0, label="force")
    axes[2, 0].plot(t, data["moment_norm_nm"], color="#e45756", linewidth=1.7, label="moment")
    axes[2, 0].set_title("Force and moment norms")
    set_axis_style(axes[2, 0], "Time (s)", "N / N*m")
    axes[2, 0].legend(loc="best")

    axes[2, 1].step(t, data["model_degradation_flags"], where="post", color="#8c564b", label="model flags")
    axes[2, 1].step(t, data["aero_model_flags"], where="post", color="#17becf", label="aero flags")
    axes[2, 1].plot(t, data["aero_uncertainty_scale"], color="#7f7f7f", linewidth=1.5, label="aero uncertainty")
    axes[2, 1].set_title("Model flags and aero uncertainty")
    set_axis_style(axes[2, 1], "Time (s)", "flag / scale")
    axes[2, 1].legend(loc="best")

    finish_figure(fig, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
