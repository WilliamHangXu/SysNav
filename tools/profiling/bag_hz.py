#!/usr/bin/env python3
"""Per-topic achieved rates from rosbag2 recordings, from message receive timestamps.

For each topic: count, active window (first->last message), mean Hz over the active
window, and median inter-arrival Hz (robust to stalls/bursts). Writes one CSV row per
(bag, topic) and prints a per-topic cross-bag summary at the end.
"""
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

import rosbag2_py


def scan(bag: str) -> dict[str, list[int]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"),
    )
    stamps: dict[str, list[int]] = defaultdict(list)
    while reader.has_next():
        topic, _data, t = reader.read_next()
        stamps[topic].append(t)
    return stamps


def rates(ts: list[int]):
    if len(ts) < 2:
        return None
    ts = sorted(ts)
    window = (ts[-1] - ts[0]) / 1e9
    if window <= 0:
        return None
    mean_hz = (len(ts) - 1) / window
    gaps = [(b - a) / 1e9 for a, b in zip(ts, ts[1:]) if b > a]
    med_hz = 1.0 / statistics.median(gaps) if gaps else float("nan")
    return len(ts), window, mean_hz, med_hz


def main():
    out_csv = Path(sys.argv[1])
    bags = sys.argv[2:]
    per_topic: dict[str, list[float]] = defaultdict(list)
    per_topic_med: dict[str, list[float]] = defaultdict(list)
    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bag", "topic", "count", "active_window_s", "mean_hz", "median_hz"])
        for bag in bags:
            name = Path(bag).name
            print(f"scanning {name} ...", flush=True)
            for topic, ts in sorted(scan(bag).items()):
                r = rates(ts)
                if r is None:
                    continue
                n, window, mean_hz, med_hz = r
                w.writerow([name, topic, n, f"{window:.1f}", f"{mean_hz:.3f}", f"{med_hz:.3f}"])
                per_topic[topic].append(mean_hz)
                per_topic_med[topic].append(med_hz)

    print("\n=== cross-bag summary (mean Hz over active window) ===")
    print(f"{'topic':40s} {'bags':>4s} {'mean':>7s} {'min':>7s} {'max':>7s} {'med-gap':>8s}")
    for topic in sorted(per_topic):
        vals = per_topic[topic]
        meds = per_topic_med[topic]
        print(f"{topic:40s} {len(vals):4d} {statistics.mean(vals):7.2f} "
              f"{min(vals):7.2f} {max(vals):7.2f} {statistics.mean(meds):8.2f}")


if __name__ == "__main__":
    main()
