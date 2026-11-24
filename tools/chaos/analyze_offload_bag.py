#!/usr/bin/env python3
"""Reads the rosbag2 recorded by kill_detector.sh and prints the 009a-R15/R16 report (thin layer over offload_analysis).

Needs rosbag2_py and the vacuum_interfaces overlay: run it inside the detector image (kill_detector.sh does).
Usage: analyze_offload_bag.py <bag_dir> <kills.csv> [--open-threshold-s 1.5]
kills.csv: header `index,kill_epoch,ready_epoch`, epoch seconds on the host clock (the bag receive time uses the same clock).
Writes <bag_dir>/../report.md and report.json next to the bag; nothing is committed (tools/chaos/local-logs/ is gitignored).
"""
import argparse
import csv
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import offload_analysis as oa  # noqa: E402

STATUS, DETECTIONS = "/myRobot/perception/offload_status", "/myRobot/perception/detections"
NOTIFY, MODE = "/myRobot/notifications", "/myRobot/behaviour/mode"


def read_bag(bag_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    statuses, notes, modes, dets = [], [], [], []
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in types:
            continue
        t = t_ns / 1e9
        msg = deserialize_message(raw, get_message(types[topic]))
        if topic == STATUS:
            statuses.append((t, msg.state, msg.forwarded, msg.dropped_inactive, msg.dropped_stale,
                             msg.dropped_watermark, msg.transitions))
        elif topic == NOTIFY:
            notes.append((t, msg.code))
        elif topic == MODE:
            modes.append((t, msg.data))
        elif topic == DETECTIONS:
            dets.append((t, msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9))
    return statuses, notes, modes, dets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir")
    ap.add_argument("kills_csv")
    ap.add_argument("--open-threshold-s", type=float, default=1.5)
    a = ap.parse_args()
    with open(a.kills_csv) as f:
        kills = [(float(r["kill_epoch"]), float(r["ready_epoch"]) if r["ready_epoch"] else None) for r in csv.DictReader(f)]
    statuses, notes, modes, dets = read_bag(a.bag_dir)
    if not statuses:
        sys.exit(f"no OffloadStatus on {STATUS} in {a.bag_dir}: is the detection_selector running?")
    rep = oa.analyze(statuses, notes, modes, dets, kills, a.open_threshold_s)
    md = oa.to_markdown(rep)
    print(md)
    out = pathlib.Path(a.bag_dir).resolve().parent
    (out / "report.md").write_text(md + "\n")
    (out / "report.json").write_text(json.dumps(
        {"bound_s": rep.bound_s, "detections": rep.detections, "monotonic_violations": rep.monotonic_violations,
         "kills": [k.__dict__ for k in rep.kills]}, indent=2))
    ok = rep.monotonic_violations == 0 and all(k.open_within_bound and not k.mode_changed and k.time_to_closed_s is not None
                                               for k in rep.kills)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
