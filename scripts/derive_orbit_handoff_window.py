#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import statistics
import time
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover
    plt = None


ROOT_DIR = Path(__file__).resolve().parent.parent
RESULTS_DIR = ROOT_DIR / "results"
ACTIVE_PHASES = {"APPROACH", "TRACKING", "DOCKING", "COMPLETED"}
CLUSTER_FEATURES = (
    "relative_distance",
    "rel_x",
    "rel_y",
    "rel_vy",
    "closing_rate",
    "projected_abs_y",
)
CLUSTER_SPREAD_FLOORS = {
    "relative_distance": 1.0,
    "rel_x": 1.0,
    "rel_y": 1.0,
    "rel_vy": 0.25,
    "closing_rate": 0.20,
    "projected_abs_y": 1.0,
}
BRIDGE_SCORE_WEIGHTS = {
    "relative_distance": 2.0,
    "rel_x": 1.1,
    "rel_y": 0.35,
    "rel_vy": 1.5,
    "closing_rate": 1.5,
    "projected_abs_y": 2.4,
}


def read_kv(path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    if not path.exists():
        return data
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key.strip()] = value.strip()
    return data


def quantile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, q)) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    alpha = position - lower
    return ordered[lower] * (1.0 - alpha) + ordered[upper] * alpha


def read_rows(path: Path) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    with path.open("r", encoding="utf-8", errors="ignore") as file:
        for raw in csv.DictReader(file):
            row: dict[str, float | str] = {}
            for key, value in raw.items():
                if key == "phase":
                    row[key] = value
                    continue
                try:
                    row[key] = float(value)
                except (TypeError, ValueError):
                    row[key] = math.nan
            rows.append(row)
    return rows


def relative_speed(row: dict[str, float | str]) -> float:
    return math.sqrt(
        float(row["rel_vx"]) ** 2 +
        float(row["rel_vy"]) ** 2 +
        float(row["rel_vz"]) ** 2
    )


def closing_rate(row: dict[str, float | str]) -> float:
    distance = float(row["relative_distance"])
    if distance <= 1e-6:
        return 0.0
    return (
        float(row["rel_x"]) * float(row["rel_vx"]) +
        float(row["rel_y"]) * float(row["rel_vy"]) +
        float(row["rel_z"]) * float(row["rel_vz"])
    ) / distance


def intercept_time(row: dict[str, float | str], *, horizon_max: float = 2.5) -> float:
    rel_x = float(row["rel_x"])
    rel_vx = float(row["rel_vx"])
    distance = float(row["relative_distance"])
    speed = max(relative_speed(row), 1e-3)
    if rel_x < 0.0 and rel_vx > 0.2:
        return min(max(-rel_x / rel_vx, 0.0), horizon_max)
    return min(max(distance / speed, 0.0), horizon_max)


def projected_abs_y(row: dict[str, float | str]) -> float:
    horizon = intercept_time(row)
    return abs(float(row["rel_y"]) + float(row["rel_vy"]) * horizon)


def orbit_tracking_candidate_score(row: dict[str, float | str]) -> float:
    return (
        0.95 * projected_abs_y(row) +
        0.18 * float(row["relative_distance"]) +
        0.10 * abs(float(row["rel_vy"])) +
        0.04 * abs(float(row["rel_vz"]))
    )


def active_rows(rows: list[dict[str, float | str]]) -> list[dict[str, float | str]]:
    filtered = [row for row in rows if str(row.get("phase", "")).upper() in ACTIVE_PHASES]
    return filtered or rows


def select_closest_row(rows: list[dict[str, float | str]]) -> dict[str, float | str]:
    filtered = active_rows(rows)
    return min(filtered, key=lambda row: float(row["relative_distance"]))


def select_handoff_candidate(
    rows: list[dict[str, float | str]],
    *,
    distance_margin_m: float,
    preclosest_window_sec: float,
) -> dict[str, float | str]:
    filtered = active_rows(rows)
    closest = select_closest_row(filtered)
    closest_t = float(closest["t"])
    closest_distance = float(closest["relative_distance"])

    pool = [
        row for row in filtered
        if float(row["t"]) <= closest_t and
        (closest_t - float(row["t"])) <= preclosest_window_sec
    ]
    if not pool:
        pool = [row for row in filtered if float(row["t"]) <= closest_t]
    if not pool:
        pool = filtered

    pool = [
        row for row in pool
        if float(row["relative_distance"]) <= closest_distance + distance_margin_m and
        closing_rate(row) < 0.0
    ] or pool

    return min(pool, key=orbit_tracking_candidate_score)


def has_glide_release(result_dir: Path) -> bool:
    launch_log = result_dir / "launch.log"
    if not launch_log.exists():
        return False
    text = launch_log.read_text(encoding="utf-8", errors="ignore")
    return "fixed-wing: glide window accepted" in text


def write_csv(path: Path, rows: list[dict[str, float | str]]) -> None:
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _feature_center(rows: list[dict[str, float | str]]) -> dict[str, float]:
    return {
        key: statistics.fmean(float(row[key]) for row in rows)
        for key in CLUSTER_FEATURES
    }


def _feature_scale(rows: list[dict[str, float | str]]) -> dict[str, float]:
    scales: dict[str, float] = {}
    for key in CLUSTER_FEATURES:
        values = [float(row[key]) for row in rows]
        scale = statistics.pstdev(values) if len(values) > 1 else 0.0
        scales[key] = max(scale, CLUSTER_SPREAD_FLOORS[key])
    return scales


def _normalized_sq_distance(
    row: dict[str, float | str],
    center: dict[str, float],
    scales: dict[str, float],
) -> float:
    total = 0.0
    for key in CLUSTER_FEATURES:
        error = (float(row[key]) - center[key]) / max(scales[key], 1e-6)
        total += error * error
    return total


def _bridge_score(
    row: dict[str, float | str],
    center: dict[str, float],
    spreads: dict[str, float],
) -> float:
    total = 0.0
    for key in CLUSTER_FEATURES:
        total += (
            BRIDGE_SCORE_WEIGHTS[key] *
            abs(float(row[key]) - center[key]) /
            max(spreads[key], 1e-6)
        )
    return total


def cluster_handoff_candidates(
    rows: list[dict[str, float | str]],
    *,
    cluster_count: int = 2,
    max_iterations: int = 16,
) -> tuple[list[dict[str, float | str]], list[dict[str, float | str]]]:
    if not rows:
        return [], []
    if len(rows) <= 1 or cluster_count <= 1:
        single_center = _feature_center(rows)
        single_spreads = _feature_scale(rows)
        clustered_rows = []
        for row in rows:
            enriched = dict(row)
            enriched["cluster"] = "A"
            enriched["cluster_score"] = 0.0
            clustered_rows.append(enriched)
        return clustered_rows, [{
            "cluster": "A",
            "size": len(rows),
            **{f"{key}_center": single_center[key] for key in CLUSTER_FEATURES},
            **{f"{key}_spread": single_spreads[key] for key in CLUSTER_FEATURES},
        }]

    normalization = _feature_scale(rows)
    seed_low = min(rows, key=lambda row: float(row["projected_abs_y"]))
    seed_high = max(rows, key=lambda row: float(row["projected_abs_y"]))
    centers = [
        {key: float(seed_low[key]) for key in CLUSTER_FEATURES},
        {key: float(seed_high[key]) for key in CLUSTER_FEATURES},
    ]

    assignments: list[int] = [-1] * len(rows)
    for _ in range(max_iterations):
        updated_assignments: list[int] = []
        for row in rows:
            best_index = min(
                range(cluster_count),
                key=lambda idx: _normalized_sq_distance(row, centers[idx], normalization),
            )
            updated_assignments.append(best_index)

        if updated_assignments == assignments:
            break
        assignments = updated_assignments

        new_centers: list[dict[str, float]] = []
        for idx in range(cluster_count):
            members = [row for row, assignment in zip(rows, assignments) if assignment == idx]
            if members:
                new_centers.append(_feature_center(members))
            else:
                new_centers.append(centers[idx])
        centers = new_centers

    cluster_members: list[list[dict[str, float | str]]] = []
    for idx in range(cluster_count):
        members = [row for row, assignment in zip(rows, assignments) if assignment == idx]
        if not members:
            members = [rows[idx % len(rows)]]
        cluster_members.append(members)

    ordered_indices = sorted(
        range(cluster_count),
        key=lambda idx: statistics.fmean(float(row["projected_abs_y"]) for row in cluster_members[idx]),
    )
    label_map = {
        original_idx: chr(ord("A") + ordered_position)
        for ordered_position, original_idx in enumerate(ordered_indices)
    }

    cluster_stats_by_label: dict[str, dict[str, float | str]] = {}
    for original_idx in ordered_indices:
        label = label_map[original_idx]
        members = cluster_members[original_idx]
        center = _feature_center(members)
        spreads = _feature_scale(members)
        cluster_stats_by_label[label] = {
            "cluster": label,
            "size": len(members),
            **{f"{key}_center": center[key] for key in CLUSTER_FEATURES},
            **{f"{key}_spread": spreads[key] for key in CLUSTER_FEATURES},
        }

    clustered_rows: list[dict[str, float | str]] = []
    for row, assignment in zip(rows, assignments):
        label = label_map[assignment]
        stats = cluster_stats_by_label[label]
        center = {key: float(stats[f"{key}_center"]) for key in CLUSTER_FEATURES}
        spreads = {key: float(stats[f"{key}_spread"]) for key in CLUSTER_FEATURES}
        enriched = dict(row)
        enriched["cluster"] = label
        enriched["cluster_score"] = _bridge_score(row, center, spreads)
        clustered_rows.append(enriched)

    cluster_stats = [cluster_stats_by_label[label] for label in sorted(cluster_stats_by_label.keys())]
    return clustered_rows, cluster_stats


def save_plots(output_dir: Path, candidates: list[dict[str, float | str]], closest: list[dict[str, float | str]]) -> None:
    if plt is None or not candidates:
        return

    fig, ax = plt.subplots(figsize=(6.8, 5.8), dpi=160)
    ax.scatter(
        [float(row["rel_x"]) for row in closest],
        [float(row["rel_y"]) for row in closest],
        color="#d95f02",
        s=42,
        label="Closest orbit-only samples",
    )
    ax.scatter(
        [float(row["rel_x"]) for row in candidates],
        [float(row["rel_y"]) for row in candidates],
        color="#1b9e77",
        s=46,
        label="Pre-handoff candidates",
    )
    ax.axhline(0.0, color="#666666", lw=0.8, alpha=0.5)
    ax.axvline(0.0, color="#666666", lw=0.8, alpha=0.5)
    ax.set_xlabel("rel_x [m]")
    ax.set_ylabel("rel_y [m]")
    ax.set_title("Orbit-Only Candidate Geometry")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_dir / "orbit_tracking_candidate_scatter.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.5), dpi=160)
    ax.hist(
        [float(row["relative_distance"]) for row in closest],
        bins=min(10, max(3, len(closest))),
        color="#7570b3",
        alpha=0.85,
    )
    ax.set_xlabel("Closest distance [m]")
    ax.set_ylabel("Count")
    ax.set_title("Orbit-Only Closest-Distance Distribution")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_dir / "orbit_tracking_closest_distance_hist.png")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Derive a handoff window from orbit-only tracking experiment logs."
    )
    parser.add_argument("result_dirs", nargs="*", help="Result directories to analyze.")
    parser.add_argument("--manifest", help="Manifest CSV produced by validate_px4_change_batch.py.")
    parser.add_argument("--distance-margin-m", type=float, default=6.0)
    parser.add_argument("--preclosest-window-sec", type=float, default=6.0)
    parser.add_argument("--output-dir", help="Optional output directory.")
    args = parser.parse_args()

    selected_dirs: list[Path] = []
    if args.manifest:
        manifest_path = Path(args.manifest)
        with manifest_path.open("r", encoding="utf-8", errors="ignore") as file:
            for row in csv.DictReader(file):
                result_dir = Path(row["result_dir"])
                if result_dir.exists():
                    selected_dirs.append(result_dir)

    for item in args.result_dirs:
        result_dir = Path(item)
        if result_dir.exists():
            selected_dirs.append(result_dir)

    selected_dirs = list(dict.fromkeys(selected_dirs))
    if not selected_dirs:
        raise SystemExit("no result directories provided")

    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        output_dir = RESULTS_DIR / f"{time.strftime('%Y%m%d_%H%M%S')}_orbit_tracking_window"
    output_dir.mkdir(parents=True, exist_ok=True)

    analyzed_rows: list[dict[str, float | str]] = []
    candidate_rows: list[dict[str, float | str]] = []
    closest_rows: list[dict[str, float | str]] = []

    for result_dir in selected_dirs:
        csv_path = result_dir / "docking_log.csv"
        if not csv_path.exists():
            continue
        rows = read_rows(csv_path)
        if not rows:
            continue

        closest = select_closest_row(rows)
        candidate = select_handoff_candidate(
            rows,
            distance_margin_m=args.distance_margin_m,
            preclosest_window_sec=args.preclosest_window_sec,
        )
        summary = read_kv(result_dir / "summary.txt")

        closest_record = {
            "result_dir": str(result_dir),
            "t": float(closest["t"]),
            "phase": str(closest["phase"]),
            "relative_distance": float(closest["relative_distance"]),
            "rel_x": float(closest["rel_x"]),
            "rel_y": float(closest["rel_y"]),
            "rel_z": float(closest["rel_z"]),
            "rel_vx": float(closest["rel_vx"]),
            "rel_vy": float(closest["rel_vy"]),
            "rel_vz": float(closest["rel_vz"]),
            "relative_speed": relative_speed(closest),
            "closing_rate": closing_rate(closest),
            "projected_abs_y": projected_abs_y(closest),
            "glide_activated": int(has_glide_release(result_dir)),
            "best_window_phase": summary.get("best_window_phase", "IDLE"),
        }
        candidate_record = {
            "result_dir": str(result_dir),
            "t": float(candidate["t"]),
            "phase": str(candidate["phase"]),
            "relative_distance": float(candidate["relative_distance"]),
            "rel_x": float(candidate["rel_x"]),
            "rel_y": float(candidate["rel_y"]),
            "rel_z": float(candidate["rel_z"]),
            "rel_vx": float(candidate["rel_vx"]),
            "rel_vy": float(candidate["rel_vy"]),
            "rel_vz": float(candidate["rel_vz"]),
            "relative_speed": relative_speed(candidate),
            "closing_rate": closing_rate(candidate),
            "projected_abs_y": projected_abs_y(candidate),
            "score": orbit_tracking_candidate_score(candidate),
            "glide_activated": int(has_glide_release(result_dir)),
        }

        analyzed_rows.append({
            "result_dir": str(result_dir),
            "closest_distance_m": closest_record["relative_distance"],
            "closest_projected_abs_y_m": closest_record["projected_abs_y"],
            "candidate_distance_m": candidate_record["relative_distance"],
            "candidate_projected_abs_y_m": candidate_record["projected_abs_y"],
            "candidate_rel_x_m": candidate_record["rel_x"],
            "candidate_rel_y_m": candidate_record["rel_y"],
            "candidate_rel_vx_mps": candidate_record["rel_vx"],
            "candidate_rel_vy_mps": candidate_record["rel_vy"],
            "candidate_closing_rate_mps": candidate_record["closing_rate"],
            "glide_activated": candidate_record["glide_activated"],
        })
        closest_rows.append(closest_record)
        candidate_rows.append(candidate_record)

    write_csv(output_dir / "orbit_tracking_run_table.csv", analyzed_rows)
    write_csv(output_dir / "orbit_tracking_closest_rows.csv", closest_rows)
    write_csv(output_dir / "orbit_tracking_handoff_candidates.csv", candidate_rows)
    clustered_candidates, cluster_stats = cluster_handoff_candidates(candidate_rows)
    write_csv(output_dir / "orbit_tracking_handoff_clustered.csv", clustered_candidates)
    write_csv(output_dir / "orbit_tracking_cluster_stats.csv", cluster_stats)
    save_plots(output_dir, candidate_rows, closest_rows)

    glide_runs = sum(int(row["glide_activated"]) for row in candidate_rows)
    closest_distances = [float(row["relative_distance"]) for row in closest_rows]
    candidate_distances = [float(row["relative_distance"]) for row in candidate_rows]
    candidate_abs_y = [abs(float(row["rel_y"])) for row in candidate_rows]
    candidate_projected_abs_y = [float(row["projected_abs_y"]) for row in candidate_rows]
    candidate_rel_x = [float(row["rel_x"]) for row in candidate_rows]
    candidate_rel_vx = [float(row["rel_vx"]) for row in candidate_rows]
    candidate_abs_vy = [abs(float(row["rel_vy"])) for row in candidate_rows]
    candidate_closing = [float(row["closing_rate"]) for row in candidate_rows]

    with (output_dir / "summary.txt").open("w", encoding="utf-8") as file:
        file.write(f"runs={len(candidate_rows)}\n")
        file.write(f"glide_activated_runs={glide_runs}\n")
        file.write(f"closest_distance_p10_m={quantile(closest_distances, 0.10):.3f}\n")
        file.write(f"closest_distance_p50_m={quantile(closest_distances, 0.50):.3f}\n")
        file.write(f"closest_distance_p90_m={quantile(closest_distances, 0.90):.3f}\n")
        file.write(f"candidate_distance_p10_m={quantile(candidate_distances, 0.10):.3f}\n")
        file.write(f"candidate_distance_p50_m={quantile(candidate_distances, 0.50):.3f}\n")
        file.write(f"candidate_distance_p90_m={quantile(candidate_distances, 0.90):.3f}\n")
        file.write(f"candidate_abs_y_p75_m={quantile(candidate_abs_y, 0.75):.3f}\n")
        file.write(f"candidate_projected_abs_y_p75_m={quantile(candidate_projected_abs_y, 0.75):.3f}\n")
        file.write(f"candidate_rel_x_p10_m={quantile(candidate_rel_x, 0.10):.3f}\n")
        file.write(f"candidate_rel_x_p90_m={quantile(candidate_rel_x, 0.90):.3f}\n")
        file.write(f"candidate_rel_vx_p10_mps={quantile(candidate_rel_vx, 0.10):.3f}\n")
        file.write(f"candidate_abs_vy_p75_mps={quantile(candidate_abs_vy, 0.75):.3f}\n")
        file.write(f"candidate_closing_rate_p50_mps={quantile(candidate_closing, 0.50):.3f}\n")
        file.write(
            f"recommended_handoff_distance_m={quantile(candidate_distances, 0.50):.3f}\n"
        )
        file.write(
            f"recommended_handoff_max_abs_y_m={quantile(candidate_abs_y, 0.75):.3f}\n"
        )
        file.write(
            f"recommended_handoff_max_projected_abs_y_m={quantile(candidate_projected_abs_y, 0.75):.3f}\n"
        )
        file.write(
            f"recommended_handoff_min_rel_x_m={quantile(candidate_rel_x, 0.10):.3f}\n"
        )
        file.write(
            f"recommended_handoff_max_rel_x_m={quantile(candidate_rel_x, 0.90):.3f}\n"
        )
        file.write(
            f"recommended_handoff_min_rel_vx_mps={quantile(candidate_rel_vx, 0.10):.3f}\n"
        )
        file.write(
            f"recommended_handoff_max_abs_vy_mps={quantile(candidate_abs_vy, 0.75):.3f}\n"
        )

    if cluster_stats:
        with (output_dir / "cluster_summary.txt").open("w", encoding="utf-8") as file:
            file.write(f"runs={len(clustered_candidates)}\n")
            file.write(f"clusters={len(cluster_stats)}\n")
            for stats in cluster_stats:
                label = str(stats["cluster"])
                members = [row for row in clustered_candidates if row["cluster"] == label]
                rel_x_negative = sum(float(row["rel_x"]) < 0.0 for row in members)
                rel_y_positive = sum(float(row["rel_y"]) > 0.0 for row in members)
                rel_vy_negative = sum(float(row["rel_vy"]) < 0.0 for row in members)
                closing_negative = sum(float(row["closing_rate"]) < 0.0 for row in members)
                file.write(f"cluster_{label}_size={int(stats['size'])}\n")
                file.write(f"cluster_{label}_rel_x_negative_rows={rel_x_negative}\n")
                file.write(f"cluster_{label}_rel_y_positive_rows={rel_y_positive}\n")
                file.write(f"cluster_{label}_rel_vy_negative_rows={rel_vy_negative}\n")
                file.write(f"cluster_{label}_closing_negative_rows={closing_negative}\n")
                for key in CLUSTER_FEATURES:
                    file.write(f"cluster_{label}_{key}_center={float(stats[f'{key}_center']):.3f}\n")
                    file.write(f"cluster_{label}_{key}_spread={float(stats[f'{key}_spread']):.3f}\n")
            file.write(
                "bridge_score_weights="
                + ",".join(f"{BRIDGE_SCORE_WEIGHTS[key]:.3f}" for key in CLUSTER_FEATURES)
                + "\n"
            )

    print(output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
