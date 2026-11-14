# 009a: Tasks

## Phase 0: preparation

- [ ] **T901** Add `OffloadStatus` to `vacuum_interfaces`. (009a-R14)
- [x] **T902** [P] (done on Lyrical, same host; `deploy/measurements/t902-transport.md`; Q-06 recommendation: bridge, awaiting the project owner) Measure the robot ↔ K3s transport with `rmw_zenoh` and with `zenoh-bridge-ros2dds`: latency and rate with a compressed RGB image at 2 Hz. Decide Q-06. (009a-R03)
- [ ] **T903** [P] Read the 2025S2 semantic node and record the exact format of `/detections_markers` it expects. (009a-R13)

## Phase 1: tests first

- [ ] **T904** pytest tests of `selector_core`: age and watermark filters, every transition of the table and the hysteresis, with a simulated clock. (009a-R05 to R10)
- [ ] **T905** [P] `projector` test with synthetic TF: transform at the photo timestamp, drop when there is no TF. (009a-R12)

## Phase 2: implementation

- [ ] **T906** `detector`: YOLO configurable by parameter, `header` copied, empty array when nothing is detected. (009a-R02, R04)
- [ ] **T907** `selector_core` and the `detection_selector` node, until T904 passes. Publish the status and the notifications. (009a-R05 to R11, R14)
- [ ] **T908** `projector` with depth cache by timestamp, until T905 passes. (009a-R12, R13)
- [ ] **T909** `image_throttle`: compression and configurable rate. (009a-R03, R11)
- [ ] **T910** Detector Dockerfile with `ARG ROS_DISTRO`, built on the CI matrix. (009a-R17, R18)
- [ ] **T911** Terraform (`infra/modules/workloads`) with Deployment, `livenessProbe` and the transport chosen in Q-06 (proposal: `zenoh-bridge-ros2dds` with the R19 allow-list; blocked on the owner's confirmation; depends on T902, T910; the bridge runs on the robot outside `lifecycle_manager_core`, cleaning continues if it dies, allow-list without `mux`/`cmd_vel`). (009a-R17, R19)
- [ ] **T912** Launch with `offload:=true|false`. (009a-R01)

## Phase 3: verification

- [ ] **T913** `tools/chaos/kill_detector.sh` and the rosbag analysis. Record the times and counters for the report. (009a-R15, R16)
- [ ] **T914** Run T913 on Jazzy and on Lyrical. (009a-R18)

## Phase 4: transport follow-ups (Q-06, from T902)

- [ ] **T919** [devops] Repeat T902 with `scripts/transport-bench.sh --distro jazzy` (image `t902-bench:jazzy`; `rmw_zenoh` 0.2.x on Jazzy against the bridge 1.10.2), same 4 cells. Depends on T902 and T910 (distro images). (009a-R18, Q-06)
- [ ] **T920** [devops] Repeat T902 on the real robot-to-cluster link (Wi-Fi, not same host), both transports, with a real House-scene image, and record CPU of each transport. Depends on T902 and on a real image source (T909). If the bridge's age exceeds `close_threshold_s` in more than a small fraction of responses, reopen Q-06 in favour of `rmw_zenoh`. (009a-R03, R10, Q-06)
- [ ] **T921** [devops] [P] Measure inference time of yolo11x in the K3s Pod (and yolo11n locally) so the real age (transport plus inference) can be compared with `close_threshold_s` and `max_age_s`. Depends on T906, T910. (009a-R05, R10)
- [ ] **T922** [devops] [ros-dev] Add to T913 one run with `tc netem` (20 ms +/- 5 ms, 1 % loss) on the uplink and report time to CLOSED and the share of `dropped_stale`. Depends on T907, T913. (009a-R16)
- [ ] **T923** [ros-dev] [P] Contract test of the R19 allow-list (only the two perception topics cross), generated from `contracts/topics.yaml`; depends on T911. (009a-R19, R03)
