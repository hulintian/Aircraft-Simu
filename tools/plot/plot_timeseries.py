#!/usr/bin/env python3
"""Plot flight state time-series from trajectory.csv."""

from __future__ import annotations

import argparse

from plot_common import STYLE, finish_figure, load_csv, maybe_series, set_axis_style

import matplotlib.pyplot as plt
import numpy as np


REQUIRED_COLUMNS = [
    "time_s",
    "missile_vx_ecef_mps",
    "missile_vy_ecef_mps",
    "missile_vz_ecef_mps",
    "missile_ax_ecef_mps2",
    "missile_ay_ecef_mps2",
    "missile_az_ecef_mps2",
    "missile_height_m",
    "missile_agl_m",
    "missile_mass_kg",
    "force_b_x_n",
    "force_b_y_n",
    "force_b_z_n",
    "range_m",
    "closing_velocity_mps",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create flight state time-series plots from trajectory.csv."
    )
    parser.add_argument("--trajectory", required=True, help="Input trajectory.csv path.")
    parser.add_argument("--output", required=True, help="Output PNG path.")
    parser.add_argument("--title", default="Flight state time series", help="Figure title.")
    return parser.parse_args()


def norm3(data, x_name: str, y_name: str, z_name: str):
    return np.sqrt(data[x_name] ** 2 + data[y_name] ** 2 + data[z_name] ** 2)


def main() -> int:
    args = parse_args()
    data = load_csv(args.trajectory, REQUIRED_COLUMNS)
    t = data["time_s"]

    speed = norm3(data, "missile_vx_ecef_mps", "missile_vy_ecef_mps", "missile_vz_ecef_mps")
    accel = norm3(data, "missile_ax_ecef_mps2", "missile_ay_ecef_mps2", "missile_az_ecef_mps2")
    force = norm3(data, "force_b_x_n", "force_b_y_n", "force_b_z_n")

    fig, axes = plt.subplots(3, 2, figsize=(15, 11), sharex=True)
    fig.suptitle(args.title, fontsize=15, fontweight="bold")

    axes[0, 0].plot(t, data["range_m"], color=STYLE["range"], linewidth=2.0)
    axes[0, 0].set_title("Range")
    set_axis_style(axes[0, 0], ylabel="Range (m)")

    axes[0, 1].plot(t, data["closing_velocity_mps"], color=STYLE["speed"], linewidth=2.0)
    axes[0, 1].set_title("Closing velocity")
    set_axis_style(axes[0, 1], ylabel="Closing velocity (m/s)")

    axes[1, 0].plot(t, speed, color=STYLE["speed"], linewidth=2.0)
    axes[1, 0].set_title("Missile speed norm")
    set_axis_style(axes[1, 0], ylabel="Speed (m/s)")

    axes[1, 1].plot(t, accel, color=STYLE["diag"], linewidth=2.0)
    axes[1, 1].set_title("Missile acceleration norm")
    set_axis_style(axes[1, 1], ylabel="Acceleration (m/s^2)")

    axes[2, 0].plot(t, data["missile_height_m"], color=STYLE["altitude"], linewidth=2.0, label="height")
    agl = maybe_series(data, "missile_agl_m")
    if agl is not None:
        axes[2, 0].plot(t, agl, color="#8c564b", linewidth=1.6, label="AGL")
        axes[2, 0].legend(loc="best")
    axes[2, 0].set_title("Altitude and AGL")
    set_axis_style(axes[2, 0], "Time (s)", "Height (m)")

    mass_axis = axes[2, 1]
    mass_axis.plot(t, data["missile_mass_kg"], color="#4c78a8", linewidth=2.0, label="mass")
    prop = maybe_series(data, "missile_propellant_mass_kg")
    if prop is not None:
        mass_axis.plot(t, prop, color="#72b7b2", linewidth=1.6, label="propellant")
    force_axis = mass_axis.twinx()
    force_axis.plot(t, force, color="#f58518", linewidth=1.6, alpha=0.85, label="force norm")
    mass_axis.set_title("Mass and body force")
    set_axis_style(mass_axis, "Time (s)", "Mass (kg)")
    force_axis.set_ylabel("Force norm (N)")
    mass_lines, mass_labels = mass_axis.get_legend_handles_labels()
    force_lines, force_labels = force_axis.get_legend_handles_labels()
    mass_axis.legend(mass_lines + force_lines, mass_labels + force_labels, loc="best")

    finish_figure(fig, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
