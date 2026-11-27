# Local offload run and detector kill test (009a-R15, R16)

Measured 2026-11-27 on ROS 2 **Jazzy** only, entirely on one machine: local k3d cluster `kuberostics`, no remote cloud.
The raw rosbag, `kills.csv` and the logs stay in `tools/chaos/local-logs/` (gitignored); this file holds the derived numbers.

## Setup

- Cloud: Pod `detector` (`detector_cloud` + `zenoh-bridge-ros2dds` sidecar) rendered from `deploy/k8s/detector.yaml.tpl`
  (`scripts/render-detector-local.sh`), image `vacuum-detector:jazzy`, **yolo11n** weights, CPU only, NodePort 30448.
- Robot: one container (`tools/chaos/local_robot.sh`, 2 CPUs) with a synthetic camera (one JPEG, `rgb8`, 2 Hz), `image_throttle`,
  `detector_local` (yolo11n, hot standby) and `detection_selector`; the bridge sidecar shares its network namespace.
  The projector was **not** in the run: its `activate` transition hangs (see below).
- Transport: `zenoh-bridge-ros2dds` (assumption for Q-06, not confirmed by the owner). A local run is LAN-like, so the impaired-link
  penalty of `robot-cloud-transport-benchmark.md` is not exercised.
- `tools/chaos/kill_detector.sh --kills 5 --settle 40 --robot-container chaos-robot`; analysis by `tools/chaos/analyze_offload_bag.py`.

## Result (5 kills, `report.md`)

| kill | to OPEN (s) | within R15 bound (2.0 s) | to CLOSED (s) | CLOSED after Pod ready (s) | local frames forwarded | dropped inactive | dropped stale |
|---|---|---|---|---|---|---|---|
| 1 | 1.66 | yes | 34.16 | 2.48 | 66 | 7 | 0 |
| 2 | 1.47 | yes | 34.93 | 2.95 | 67 | 10 | 0 |
| 3 | 1.47 | yes | 34.97 | 3.01 | 67 | 8 | 0 |
| 4 | 2.52 | **no** | 35.50 | 2.49 | 68 | 11 | 0 |
| 5 | 1.48 | yes | 34.39 | 2.40 | 67 | 9 | 0 |

- Time to detect the failure and fall back to local = time to OPEN: 1.47 to 2.52 s (median 1.48 s; `open_threshold_s` is 1.5 s). The
  selector forwards local detections from the OPEN transition on. Kill 4 exceeded the R15 bound by 0.5 s (one outlier in five).
- Pod restart (kill to rollout complete): 31.7 to 33.0 s (image already on the node; the model loads in about 2 s, the rest is Pod
  scheduling, the weights init container check and the lifecycle script). There is no readinessProbe, so "ready" means containers started.
- Time to recover (kill to CLOSED): 34.2 to 35.5 s; 2.4 to 3.0 s after the Pod is Ready (`stable_window_s` 3.0 s).
- Frames lost: none on the output stream during the outage (the local detector answered; 0 stale, 0 watermark drops). The only gap is
  about `open_threshold_s` (1.5 s, roughly 3 frames at 2 Hz) between the last cloud answer and the fallback. Detections on the output
  topic over the run: 590, stamp violations (R16): 0. The orchestrator mode did not change (no mode publisher in this setup, so
  the "mode changed" column is `None`).

## Findings

- yolo11x on CPU takes about 1.0 to 1.4 s per frame in the Pod (4 CPU limit): at 2 Hz the queue saturates, `last_cloud_age_s` reached
  3.9 s and the breaker never closed. The measured run therefore uses yolo11n in the cloud. yolo11x needs a GPU or a lower image rate.
- yolo11n on the robot container (2 CPUs) takes about 0.3 to 0.4 s per frame; at 4 Hz the local standby falls behind and every local
  response is stale (`dropped_stale` grew about 6 per second). The camera of the run is 2 Hz.
- The bridge does not discover a DDS node that starts after it with the default SPDP interval (30 s): no route is created. The Pod needs
  `CYCLONEDDS_URI` with `SPDPInterval` 1 s on the bridge (added to the manifest); the robot-side bridge is started after the nodes.
- Fixed in this change: the manifest passed `model:` instead of `weights`, `input_type`, `input_topic` and `output_topic`; nothing
  activated the lifecycle node (`docker/detector/run-cloud.sh` now does); the image did not put the venv (ultralytics) on the
  `PYTHONPATH` of the ROS console scripts; the container had no `workingDir` on a read-only root.
- Not fixed (source of `src/vacuum_*`, for the owner): `projector` hangs on `activate` (lifecycle services stop answering; reproduced
  alone, without traffic); `detector`, `detection_selector`, `image_throttle` and `projector` have no autostart.
