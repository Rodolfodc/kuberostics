# T902: robot <-> K3s transport, `rmw_zenoh` vs `zenoh-bridge-ros2dds` (009a-R03, input to Q-06)

Measured 2026-11-13 on ROS 2 **Lyrical** only (Jazzy not measured). Raw data: `t902/*.csv` (one row per image sent),
`t902/*.log` (the SUMMARY line), `t902/summary.txt`. Reproduce with `scripts/transport-bench.sh`.

## Method

- Cloud side: Pod in namespace `t902` of the local k3d cluster `kuberostics` (k3s v1.35.5), reached through a NodePort.
  Robot side: Docker container on the `k3d-kuberostics` network (same host, so these are lower bounds, see caveats).
- The robot publishes a 640x480 JPEG (80 187 bytes, synthetic scene with noise) as `sensor_msgs/CompressedImage` at 2 Hz
  on `/myRobot/perception/rgb_throttled/compressed` (QoS `sensor`: best_effort, depth 5), stamped with the send time.
  The Pod stands in for the detector and answers a `vision_msgs/Detection2DArray` (3 detections) with the same header on
  `/myRobot/perception/detections/cloud` (QoS `default`: reliable, depth 10). RTT = robot clock at reception minus
  `header.stamp`: the timestamp-echo age of 009a-R05, one clock, no skew.
- 600 samples (300 s) per cell, first reply to last reply minus a 4 s tail. One run per cell.
- **rmw_zenoh** (`ros-lyrical-rmw-zenoh-cpp` 0.10.6): `rmw_zenohd` router in the Pod (NodePort 30447), a local
  `rmw_zenohd` on the robot connecting to it, nodes with `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
- **zenoh-bridge-ros2dds** (`eclipse/zenoh-bridge-ros2dds:1.10.2` (digest-pinned)): nodes on CycloneDDS with localhost discovery, one bridge
  on each side (robot bridge connects to the Pod bridge, NodePort 30448), allow-lists limited to the two bench topics.
- Scenarios: `lan` = no impairment; `wan` = `tc netem delay 20ms 5ms loss 1%` on the robot egress only
  (robot -> cloud direction, i.e. the image path; the reply path is clean).

## Results (RTT in ms)

| transport | scenario | sent | received | loss | rate (Hz) | min | p50 | p95 | p99 | max | mean |
|---|---|---|---|---|---|---|---|---|---|---|---|
| rmw_zenoh | lan | 600 | 600 | 0 % | 2.000 | 2.64 | 4.05 | 4.48 | 4.67 | 5.69 | 4.00 |
| zenoh-bridge-ros2dds | lan | 600 | 600 | 0 % | 2.000 | 3.11 | 5.15 | 5.46 | 5.89 | 6.91 | 5.05 |
| rmw_zenoh | wan | 600 | 600 | 0 % | 2.000 | 63.16 | 149.33 | 309.69 | 407.74 | 430.25 | 165.34 |
| zenoh-bridge-ros2dds | wan | 600 | 600 | 0 % | 2.000 | 145.62 | 393.16 | 511.35 | 579.88 | 624.26 | 386.16 |

No reordering in any run. Both transports sustain 2 Hz with no loss; the images were always delivered (TCP).
A first bridge run (without `ROS_DISTRO` set in the bridge containers, a WARN in the bridge log) gave the same numbers
(lan p50 5.16, wan p50 395.19, p99 544.08): `summary.txt` holds the second run.

> **WARNING: the bridge result on the impaired link conflicts with the 009a thresholds.** Computed from
> `t902/lyrical-bridge-wan.csv` (607 replies; percentile convention: sorted RTTs, index `int(p/100*n)`, as in
> `bench.py`):
> - 42 of 607 replies (6.9 %) took more than 500 ms, and p95 is 511 ms. For rmw_zenoh the maximum is 430 ms, so no reply
>   exceeds 500 ms.
> - 009a-R10 needs every response in the 3 s stable window (6 replies at 2 Hz) to have `age <= close_threshold_s`
>   (0.5 s). Counting sliding windows of 6 consecutive replies in the bridge wan CSV, 190 of 603 (31.5 %) contain at least
>   one reply above 500 ms (non-overlapping windows: 32 of 101, 31.7 %); the independence estimate
>   `1 - (1 - 0.069)^6` gives 35 %. Roughly a third of the recovery windows would fail and send the selector back to OPEN.
>   rmw_zenoh has 0 such windows in the wan run.
> - RTT here excludes detector inference. With yolo11x in K3s, the bridge p99 of 0.58 s plus inference probably crosses
>   `max_age_s` (about 0.67 s), so R05 would drop those frames as stale. Not measured.
> - The architect must either re-tune `close_threshold_s`, `stable_window_s` and `max_age_s` for the bridge, or treat this
>   as a risk against the bridge recommendation.

Against the other 009a thresholds (`open_threshold_s` 1.5 s): both transports are far below it in every run. On the clean
link both are far below all thresholds. The reason for the bridge's larger penalty was not investigated. One plausible
factor, not tested: with 1 % loss, an 80 KB image is about 55 packets, so roughly 40 % of images hit at least one lost
packet, and the penalty would then be TCP retransmission and slow start rather than anything transport-specific; this
does not explain why the bridge suffers more than rmw_zenoh.

## Caveats and limits

- Same host, containers on one Docker bridge: the lan numbers are a **lower bound**, not Wi-Fi. The wan scenario is a
  synthetic netem model, uplink only, one TCP transport.
- One run per cell (a second bridge run, with `ROS_DISTRO` set, gave the same figures; both summaries are committed:
  `t902/first-run-summary.txt` and `t902/summary.txt`). Run order and host load were not controlled. p95/p99 over 600
  samples are indicative only.
- The bridge was measured with CycloneDDS and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, unlike the real robot graph.
- Image payload is synthetic, not from the House scene. CPU load of each transport was not measured.
- Lyrical only. rmw_zenoh is 0.10.x on Lyrical and 0.2.x on Jazzy (apt candidates seen: 0.10.6 and 0.2.11); the bridge on
  Jazzy is untested, and so is rmw_zenoh on Jazzy (Art. X).
- The R03 allow-list is configured and the logs show the allowed topics being created, but it was **not negatively
  tested** (no run published `/myRobot/cmd_vel` or a depth topic and checked that the cloud never receives it).
- Supply chain: the bridge image is pinned by digest, but apt package versions and `ros:lyrical-ros-base` are floating
  tags. `rmw_zenohd` and the bridge listen unauthenticated on NodePorts 30447/30448 of the local k3d node: fine for a local
  bench, not for anything else (TLS and authentication are out of scope, spec 009a).

## Recommendation for Q-06 (for the architect to record)

Latency alone does not separate them on a good link (4 vs 5 ms), and rmw_zenoh was better in this single run on the
impaired link (see the warning above). Non-latency criteria point the other way:

| criterion | rmw_zenoh | zenoh-bridge-ros2dds |
|---|---|---|
| 009a-R03 "only compressed RGB leaves the robot" | whole graph on Zenoh unless ACLs are configured | configured allow-list (not negatively tested) |
| Blast radius on the motion loop (Art. VI) | changes the RMW of every node, including `twist_mux`/`stop_guard` | only the perception nodes' traffic crosses; the robot graph stays on DDS |
| Art. X, two distros | 0.2.x (Jazzy) vs 0.10.x (Lyrical): different versions to qualify | same bridge binary (1.10.2) for both |
| Cloud Pod | needs `rmw_zenohd` + the cloud node on rmw_zenoh | bridge sidecar + ordinary DDS node |
| Measured on impaired link | p50 149 ms, p99 408 ms | p50 393 ms, p99 580 ms |

The R03 cell reads "configured allow-list, not negatively tested". Added risks of the bridge: it is a robot-side process
outside `lifecycle_manager_core`, so the robot must keep cleaning if it dies, and its allow-list must never include a
`twist_mux` input or `/myRobot/cmd_vel`.

Recommendation, conditional: **`zenoh-bridge-ros2dds` if the architect accepts re-tuning
`close_threshold_s`, `stable_window_s` and `max_age_s` for it, or if the real link is cleaner than this netem scenario**;
otherwise the measured impaired-link behaviour is a risk against it and `rmw_zenoh` (with ACLs for R03 and the Art. VI
isolation cost) is the alternative. Re-run `scripts/transport-bench.sh` on the real link before deciding, and add Jazzy
once an image for it exists (the manifest pins `t902-bench:lyrical`; the script rewrites the tag).
