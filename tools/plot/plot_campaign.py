#!/usr/bin/env python3
"""Plot campaign-level statistics from campaign_summary.json."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from plot_common import STYLE, finish_figure, set_axis_style

import matplotlib.pyplot as plt
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create campaign statistics plots from campaign_summary.json."
    )
    parser.add_argument("--campaign", required=True, help="Input campaign_summary.json path.")
    parser.add_argument("--output", required=True, help="Output PNG path.")
    parser.add_argument("--title", default="Campaign summary", help="Figure title.")
    return parser.parse_args()


def load_campaign(path: str | Path) -> tuple[dict, pd.DataFrame]:
    campaign_path = Path(path)
    if not campaign_path.exists():
        raise SystemExit(f"input file does not exist: {campaign_path}")
    with campaign_path.open("r", encoding="utf-8") as handle:
        root = json.load(handle)
    instances = root.get("instances", [])
    if not isinstance(instances, list) or not instances:
        raise SystemExit(f"{campaign_path} does not contain non-empty instances[]")
    data = pd.DataFrame(instances)
    return root, data


def main() -> int:
    args = parse_args()
    root, data = load_campaign(args.campaign)

    required = ["instance_id", "miss_distance", "wall_time_s", "hit_flag", "env_status", "fc_status"]
    missing = [name for name in required if name not in data.columns]
    if missing:
        raise SystemExit(f"campaign instances missing required fields: {', '.join(missing)}")

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(args.title, fontsize=15, fontweight="bold")

    axes[0, 0].scatter(
        data["instance_id"],
        data["miss_distance"],
        c=data["hit_flag"].map({True: STYLE["range"], False: "#d62728"}),
        s=28,
        alpha=0.9,
    )
    axes[0, 0].set_title("Miss distance by instance")
    set_axis_style(axes[0, 0], "Instance ID", "Miss distance (m)")

    axes[0, 1].hist(data["miss_distance"].dropna(), bins=24, color=STYLE["range"], alpha=0.85)
    axes[0, 1].axvline(data["miss_distance"].min(), color="black", linestyle="--", linewidth=1.5, label="min")
    axes[0, 1].axvline(data["miss_distance"].mean(), color="#ff7f0e", linestyle="-", linewidth=1.5, label="mean")
    axes[0, 1].set_title("Miss distance distribution")
    set_axis_style(axes[0, 1], "Miss distance (m)", "Count")
    axes[0, 1].legend(loc="best")

    axes[1, 0].bar(data["instance_id"], data["wall_time_s"], color="#4c78a8", alpha=0.85)
    axes[1, 0].set_title("Wall time by instance")
    set_axis_style(axes[1, 0], "Instance ID", "Wall time (s)")

    completed = int(root.get("completed_count", 0))
    failed = int(root.get("failed_count", 0))
    hit_count = int(root.get("hit_count", int(data["hit_flag"].sum())))
    miss_count = max(int(len(data)) - hit_count, 0)
    axes[1, 1].bar(["completed", "failed", "hit", "no_hit"], [completed, failed, hit_count, miss_count],
                   color=["#2ca02c", "#d62728", "#1f77b4", "#ff7f0e"])
    axes[1, 1].set_title("Outcome counts")
    set_axis_style(axes[1, 1], ylabel="Count")

    summary = (
        f"instances={root.get('instance_count', len(data))}\n"
        f"completed={completed}, failed={failed}\n"
        f"hit={hit_count}, min miss={root.get('min_miss_distance', data['miss_distance'].min()):.3f} m\n"
        f"campaign wall={root.get('campaign_wall_time_s', 0.0):.3f} s"
    )
    axes[1, 1].text(
        0.98,
        0.95,
        summary,
        transform=axes[1, 1].transAxes,
        ha="right",
        va="top",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "#bbbbbb"},
    )

    finish_figure(fig, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
