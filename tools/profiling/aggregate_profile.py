#!/usr/bin/env python3
"""Aggregate the profiling CSVs (detection_node, semantic_mapping_node, room_segmentation)
into mean / median / p95 per timing column. Usage: aggregate_profile.py <profile_dir>"""
import csv
import statistics
import sys
from pathlib import Path


def pctl(vals, p):
    s = sorted(vals)
    return s[min(len(s) - 1, int(round(p / 100 * (len(s) - 1))))]


def summarize(name, rows, cols):
    if not rows:
        print(f"\n{name}: no data")
        return
    print(f"\n=== {name} ({len(rows)} samples) ===")
    print(f"{'stage':>14s} {'mean':>8s} {'median':>8s} {'p95':>8s} {'max':>8s}")
    for c in cols:
        vals = [float(r[c]) for r in rows]
        print(f"{c:>14s} {statistics.mean(vals):8.1f} {statistics.median(vals):8.1f} "
              f"{pctl(vals, 95):8.1f} {max(vals):8.1f}")
    if 'n_boxes' in rows[0]:
        nb = [float(r['n_boxes']) for r in rows]
        print(f"{'n_boxes':>14s} {statistics.mean(nb):8.1f} {statistics.median(nb):8.1f} "
              f"{pctl(nb, 95):8.1f} {max(nb):8.1f}")


def main():
    d = Path(sys.argv[1])
    for csv_name, cols in [
        ("detection_node.csv", ["infer_ms", "xfer_ms", "annotate_ms", "publish_ms", "total_ms"]),
        ("semantic_mapping_node.csv",
         ["sam2_ms", "annotate_ms", "map_update_ms", "publish_ms", "total_ms"]),
    ]:
        p = d / csv_name
        rows = list(csv.DictReader(p.open())) if p.exists() else []
        summarize(csv_name.removesuffix(".csv"), rows, cols)

    p = d / "room_segmentation.csv"
    if p.exists():
        vals = [float(line.split(",")[1]) for line in p.read_text().splitlines() if "," in line]
        if vals:
            print(f"\n=== room_segmentation ({len(vals)} cycles) ===")
            print(f"{'stage':>14s} {'mean':>8s} {'median':>8s} {'p95':>8s} {'max':>8s}")
            print(f"{'segment_ms':>14s} {statistics.mean(vals):8.1f} {statistics.median(vals):8.1f} "
                  f"{pctl(vals, 95):8.1f} {max(vals):8.1f}")
    else:
        print("\nroom_segmentation: no data")


if __name__ == "__main__":
    main()
