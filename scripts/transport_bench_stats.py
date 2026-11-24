#!/usr/bin/env python3
"""Recompute the transport benchmark figures from the committed CSVs (deploy/measurements/transport-benchmark/*.csv) with ONE stated method.

Rows used: every row with a numeric rtt_ms (607 per run). The CSV also holds 1 trailing row without a reply (sent in
the last seconds, still in flight when the run ended); it is excluded from the percentiles and from the windows.
Percentiles: linear interpolation between order statistics (numpy default, `np.percentile(x, p)`).
Windows: sliding windows of 6 consecutive replies (3 s at 2 Hz, 009a-R10 stable window); a window is bad if any reply
exceeds the threshold (default 500 ms = close_threshold_s).
Usage: python3 -I scripts/transport_bench_stats.py [csv_dir] [threshold_ms]     (prints a markdown table)
"""
import csv
import glob
import os
import sys


def pct(sorted_x, p):
    k = (len(sorted_x) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(sorted_x) - 1)
    return sorted_x[lo] + (sorted_x[hi] - sorted_x[lo]) * (k - lo)


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), '..', 'deploy', 'measurements', 'transport-benchmark')
    thr = float(sys.argv[2]) if len(sys.argv) > 2 else 500.0
    print(f'| run | rows | numeric | p50 | p95 | p99 | max | mean | >{thr:g} ms | bad 6-windows |')
    print('|---|---|---|---|---|---|---|---|---|---|')
    for path in sorted(glob.glob(os.path.join(d, '*.csv'))):
        rows = list(csv.DictReader(open(path)))
        seq = [float(r['rtt_ms']) if r['rtt_ms'] else None for r in rows]
        x = sorted(v for v in seq if v is not None)
        while seq and seq[-1] is None:
            seq.pop()                                   # drop the in-flight tail
        wins = [seq[i:i + 6] for i in range(len(seq) - 5)]
        bad = sum(1 for w in wins if any(v is None or v > thr for v in w))
        over = sum(1 for v in x if v > thr)
        name = os.path.basename(path)[len('lyrical-'):-len('.csv')]
        print(f'| {name} | {len(rows)} | {len(x)} | {pct(x, 50):.2f} | {pct(x, 95):.2f} | {pct(x, 99):.2f} | {x[-1]:.2f} | {sum(x) / len(x):.2f} '
              f'| {over} ({100 * over / len(x):.1f} %) | {bad}/{len(wins)} ({100 * bad / len(wins):.1f} %) |')


if __name__ == '__main__':
    main()
