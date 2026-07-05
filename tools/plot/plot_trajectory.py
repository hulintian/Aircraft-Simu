#!/usr/bin/env python3
"""Plot ECEF and ground-track trajectory views from trajectory.csv."""

from __future__ import annotations

import argparse

from plot_common import STYLE, finish_figure, load_csv, set_axis_style

import matplotlib.pyplot as plt


REQUIRED_COLUMNS = [
    "time_s",
    "missile_x_ecef_m",
    "missile_y_ecef_m",
    "missile_z_ecef_m",
    "missile_lat_deg",
    "missile_lon_deg",
    "missile_height_m",
    "target_x_ecef_m",
    "target_y_ecef_m",
    "target_z_ecef_m",
    "target_lat_deg",
    "target_lon_deg",
    "target_height_m",
    "range_m",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create professional trajectory plots from trajectory.csv."
    )
    parser.add_argument("--trajectory", required=True, help="Input trajectory.csv path.")
    parser.add_argument("--output", required=True, help="Output PNG path.")
    parser.add_argument(
        "--title",
        default="Missile closed-loop trajectory",
        help="Figure title.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    data = load_csv(args.trajectory, REQUIRED_COLUMNS)

    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(args.title, fontsize=15, fontweight="bold")

    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax3d.plot(
        data["missile_x_ecef_m"],
        data["missile_y_ecef_m"],
        data["missile_z_ecef_m"],
        color=STYLE["missile"],
        linewidth=2.0,
        label="missile",
    )
    ax3d.plot(
        data["target_x_ecef_m"],
        data["target_y_ecef_m"],
        data["target_z_ecef_m"],
        color=STYLE["target"],
        linewidth=2.0,
        label="target",
    )
    ax3d.scatter(
        data["missile_x_ecef_m"].iloc[-1],
        data["missile_y_ecef_m"].iloc[-1],
        data["missile_z_ecef_m"].iloc[-1],
        color=STYLE["missile"],
        s=35,
    )
    ax3d.scatter(
        data["target_x_ecef_m"].iloc[-1],
        data["target_y_ecef_m"].iloc[-1],
        data["target_z_ecef_m"].iloc[-1],
        color=STYLE["target"],
        s=35,
    )
    ax3d.set_title("ECEF 3D trajectory")
    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.legend(loc="best")

    ax_ground = fig.add_subplot(2, 2, 2)
    ax_ground.plot(
        data["missile_lon_deg"],
        data["missile_lat_deg"],
        color=STYLE["missile"],
        linewidth=2.0,
        label="missile",
    )
    ax_ground.plot(
        data["target_lon_deg"],
        data["target_lat_deg"],
        color=STYLE["target"],
        linewidth=2.0,
        label="target",
    )
    ax_ground.set_title("Ground track")
    set_axis_style(ax_ground, "Longitude (deg)", "Latitude (deg)")
    ax_ground.legend(loc="best")
    ax_ground.set_aspect("equal", adjustable="datalim")

    ax_alt = fig.add_subplot(2, 2, 3)
    ax_alt.plot(
        data["time_s"],
        data["missile_height_m"],
        color=STYLE["missile"],
        linewidth=2.0,
        label="missile height",
    )
    ax_alt.plot(
        data["time_s"],
        data["target_height_m"],
        color=STYLE["target"],
        linewidth=2.0,
        label="target height",
    )
    ax_alt.set_title("Altitude profile")
    set_axis_style(ax_alt, "Time (s)", "Height (m)")
    ax_alt.legend(loc="best")

    ax_range = fig.add_subplot(2, 2, 4)
    ax_range.plot(
        data["time_s"],
        data["range_m"],
        color=STYLE["range"],
        linewidth=2.2,
        label="range",
    )
    min_index = data["range_m"].idxmin()
    ax_range.scatter(
        data["time_s"].loc[min_index],
        data["range_m"].loc[min_index],
        color="black",
        s=35,
        zorder=3,
        label=f"min {data['range_m'].loc[min_index]:.3f} m",
    )
    ax_range.set_title("Miss distance over time")
    set_axis_style(ax_range, "Time (s)", "Range (m)")
    ax_range.legend(loc="best")

    finish_figure(fig, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
