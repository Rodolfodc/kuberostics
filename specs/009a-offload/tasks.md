# 009a: Tasks

## Phase 0: preparation

- [ ] **T901** Add `OffloadStatus` to `vacuum_interfaces`. (009a-R14)
- [ ] **T902** [P] Measure the robot ↔ K3s transport with `rmw_zenoh` and with `zenoh-bridge-ros2dds`: latency and rate with a compressed RGB image at 2 Hz. Decide Q-06. (009a-R03)
- [ ] **T903** [P] Read the 2025S2 semantic node and record the exact format of `/detections_markers` it expects. (009a-R13)

## Phase 1: tests first

- [x] **T904** pytest tests of `selector_core`: age and watermark filters, every transition of the table and the hysteresis, with a simulated clock. (009a-R05 to R10)
- [x] **T905** [P] `projector` test with synthetic TF: transform at the photo timestamp, drop when there is no TF. (009a-R12)

## Phase 2: implementation

- [ ] **T906** `detector`: YOLO configurable by parameter, `header` copied, empty array when nothing is detected. (009a-R02, R04)
- [ ] **T907** `selector_core` and the `detection_selector` node, until T904 passes. Publish the status and the notifications. (009a-R05 to R11, R14)
- [ ] **T908** `projector` with depth cache by timestamp, until T905 passes. (009a-R12, R13)
- [ ] **T909** `image_throttle`: compression and configurable rate. (009a-R03, R11)
- [ ] **T910** Detector Dockerfile with `ARG ROS_DISTRO`, built on the CI matrix. (009a-R17, R18)
- [ ] **T911** Terraform (`infra/modules/workloads`) with Deployment, `livenessProbe` and the transport chosen in Q-06. (009a-R17)
- [ ] **T912** Launch with `offload:=true|false`. (009a-R01)

## Phase 3: verification

- [ ] **T913** `tools/chaos/kill_detector.sh` and the rosbag analysis. Record the times and counters for the report. (009a-R15, R16)
- [ ] **T914** Run T913 on Jazzy and on Lyrical. (009a-R18)
