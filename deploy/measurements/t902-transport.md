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
- **zenoh-bridge-ros2dds** (`eclipse/zenoh-bridge-ros2dds:1.10.2`): nodes on CycloneDDS with localhost discovery, one bridge
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

Against 009a thresholds (`max_age_s` about 0.67 s, `open_threshold_s` 1.5 s, `close_threshold_s` 0.5 s):
on a clean link both are far below. With 20 ms +/- 5 ms and 1 % loss on the uplink, rmw_zenoh keeps p99 at 0.41 s,
within `close_threshold_s`; the bridge has p50 0.39 s and p99 0.58 s, above `close_threshold_s` and close to `max_age_s`
(plus the detector inference time, not included here), so on a lossy link the bridge would make the HALF_OPEN -> CLOSED
transition harder. The cause of the bridge's larger penalty was not investigated (the best case is 146 ms against 63 ms for
rmw_zenoh with the same 20 ms added, which suggests extra round trips in its path); do not generalise beyond this setup.

## Caveats

- Same host, containers on one Docker bridge: the lan numbers are a **lower bound**, not Wi-Fi. The wan scenario is a
  synthetic netem model, uplink only.
- Single run per cell; p95/p99 over 600 samples are indicative only.
- Image payload is synthetic, not from the House scene. CPU load of each transport was not measured.
- Lyrical only. rmw_zenoh is 0.10.x on Lyrical and 0.2.x on Jazzy (apt candidates seen: 0.10.6 and 0.2.11), so
  Jazzy behaviour may differ and was not tested (Art. X).
- ROS_DISTRO was not set in the first bridge run (see above); no effect on the numbers.

## Recommendation for Q-06 (for the architect to record)

Latency alone does not separate them on a good link (4 vs 5 ms), and rmw_zenoh was clearly better on the impaired link.
Non-latency criteria point the other way:

| criterion | rmw_zenoh | zenoh-bridge-ros2dds |
|---|---|---|
| 009a-R03 "only compressed RGB leaves the robot" | whole graph on Zenoh unless ACLs are configured | enforced structurally by the bridge allow-list (tested) |
| Blast radius on the motion loop (Art. VI) | changes the RMW of every node, including `twist_mux`/`stop_guard` | only the perception nodes' traffic crosses; the robot graph stays on DDS |
| Art. X, two distros | 0.2.x (Jazzy) vs 0.10.x (Lyrical): different versions to qualify | same bridge binary (1.10.2) for both |
| Cloud Pod | needs `rmw_zenohd` + the cloud node on rmw_zenoh | bridge sidecar + ordinary DDS node |
| Measured on impaired link | p50 149 ms, p99 408 ms | p50 393 ms, p99 580 ms |

Recommendation: **`zenoh-bridge-ros2dds`**, because R03 and the isolation of the motion loop are structural
requirements while the latency gap only appears on a lossy link and stays below `max_age_s` at p50; keep the circuit
breaker thresholds as specified. Reconsider `rmw_zenoh` if the measured Wi-Fi link is lossy, or if both distros'
rmw_zenoh versions are qualified; re-run `scripts/transport-bench.sh` on the real link (and with `--distro jazzy` once
the image exists for it; the manifest currently pins `t902-bench:lyrical` and the script rewrites the tag).
