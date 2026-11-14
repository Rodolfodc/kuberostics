# 009a: Technical plan

## Flow

```
                       ROBOT                                       │        K3s
RGB camera ─┬─▶ image_throttle (2 Hz, compressed) ─────────────────┼──▶ detector_cloud (yolo11x)
            │                                                      │          │
            └─▶ detector_local (yolo11n, hot standby)              │          │ Detection2DArray
                        │ Detection2DArray                         │          │ (header = photo header)
                        ▼                                          │          ▼
                 detection_selector ◀──────────────────────────────┼──────────┘
                 (circuit breaker, age, watermark)                 │
                        │ /myRobot/perception/detections           │
                        ▼                                          │
depth camera ──▶ projector (depth + TF at the photo timestamp)     │
                        │ /detections_markers                      │
                        ▼                                          │
              2025S2 semantic node (unchanged)                     │
```

## Packages

- `vacuum_detection`: `detector` (the same executable runs locally or in the cloud, with the model as a parameter),
  `detection_selector` and `projector`.
- `vacuum_detection/selector_core.py`: the circuit breaker machine as a pure Python class, without ROS, tested with pytest.
  Same pattern as the `SubmodeMachine` of spec 001.
- `deploy/k8s/`: Deployment, Service and Zenoh router (if Q-06 = a), or bridge (if Q-06 = b). Proposed default: bridge (T902, see spec, "Robot to cloud transport").
- `infra/modules/workloads/`: Terraform with the `kubernetes` provider pointing at K3s.
- `docker/detector/Dockerfile`: with `ARG ROS_DISTRO`, built on the matrix `[jazzy, lyrical]`.

## Circuit breaker

| State | Source forwarded | Event | Next |
|---|---|---|---|
| CLOSED | cloud | no cloud response with age ≤ 1.5 s for 1.5 s | OPEN (WARN) |
| OPEN | local | cloud response with age ≤ 0.5 s | HALF_OPEN |
| HALF_OPEN | local | 3 s in a row with every cloud response ≤ 0.5 s | CLOSED (INFO) |
| HALF_OPEN | local | cloud response with age > 0.5 s | OPEN |

In every state, the age (`max_age_s`) and watermark filters apply before forwarding.

## Why the projection stays on the robot

- `float32` depth at 30 Hz would be about 37 MB/s. It stays on the robot, in a short image cache indexed
  by timestamp (`message_filters.Cache`).
- TF is already on the robot. Querying at the photo timestamp requires neither sending `/tf` to the cloud nor synchronising clocks.
- The 2025S2 semantic node keeps receiving `/detections_markers` as before (Constitution, Art. II).

## Injected-failure test

Script `tools/chaos/kill_detector.sh`:

1. Starts cleaning.
2. Deletes the Pod with `kubectl delete pod`.
3. Waits for the Deployment to recreate it.
4. Repeats 5 times.

Records `OffloadStatus` in a rosbag and computes time to OPEN, time to CLOSED and drop counters.
It also checks the monotonicity of the timestamps on `/myRobot/perception/detections`.

## Risks

- Q-06: T902 measured both on Lyrical (same host, synthetic). Proposal: `zenoh-bridge-ros2dds`, awaiting the project owner; the requirements do not depend on the transport, because health is measured in the application. Missing: Jazzy, real Wi-Fi, detector inference time, CPU (T919 to T922).
- On a lossy link the bridge's age tail (p99 0.58 s, max 0.62 s without inference) is above `close_threshold_s` (0.5 s) and 45 ms from `max_age_s` (0.67 s): the breaker is expected to reclose with difficulty or stay on local. No threshold is changed; see the spec.
- yolo11n on the robot CPU, even at 2 Hz, may compete with SLAM. Mitigation: measure in week 3 and reduce the resolution if needed.
- Exact format of the `MarkerArray` expected by the 2025S2 semantic node to be confirmed (T903).

## Transport (Q-06), T902 result and what depends on it

- If the owner confirms the bridge: T911 declares in `deploy/k8s/` and Terraform a Pod with the detector plus a bridge sidecar
  (allow-list: the two perception topics, R19), exposed by Service/NodePort; the robot runs one bridge with the same
  allow-list; the robot's local graph stays on CycloneDDS with localhost discovery as in the T902 bench.
- If the owner chooses `rmw_zenoh`: T911 declares `rmw_zenohd` in the cluster and the cloud node with `RMW_IMPLEMENTATION=rmw_zenoh_cpp`;
  the robot needs its own router and per-node RMW, with the ACL to meet R19, and Art. VI/X must be re-examined (a different
  RMW for perception nodes only, two versions to qualify).
- The measurements that would settle it are T919 (Jazzy), T920 (real Wi-Fi), T921 (inference time and CPU).
