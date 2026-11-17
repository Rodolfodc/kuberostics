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
- `deploy/k8s/`: Deployment, Service and Zenoh router (if Q-06 = a), or bridge (if Q-06 = b).
- `infra/modules/workloads/`: Terraform with the `kubernetes` provider pointing at K3s.
- `docker/detector/Dockerfile`: with `ARG ROS_DISTRO`, built on the matrix `[jazzy, lyrical]`.

## Circuit breaker

| State | Source forwarded | Event | Next |
|---|---|---|---|
| CLOSED | cloud | no cloud response with age ≤ `max_age_s` (0.67 s) for `open_threshold_s` (1.5 s) | OPEN (WARN) |
| OPEN | local | cloud response with age ≤ 0.5 s | HALF_OPEN |
| HALF_OPEN | local | 3 s in a row with every cloud response ≤ 0.5 s | CLOSED (INFO) |
| HALF_OPEN | local | cloud response with age > 0.5 s | OPEN |
| HALF_OPEN | local | no cloud response for more than `open_threshold_s` (silence) | OPEN (no notification; the stable window restarts at the next R09 transition) |

Order per response: breaker health tracking (cloud only), source gate, age filter, watermark filter (see spec, "Selector
processing order and counters"). The age and watermark filters apply to the active source in every state.

Rulings (2026-11): (1) `dropped_inactive` counts the non-active source before the age filter, so every response lands in
exactly one counter. (2) CLOSED health uses `max_age_s`, not `open_threshold_s`, which removes the dead zone where a slow
cloud kept the breaker CLOSED while every response was dropped as stale. Invariants: `close_threshold_s < max_age_s ≤
open_threshold_s`. (3) The fifth row exists so that a response after a long gap cannot close the breaker at once (R10,
"every response"); it needs the silence check in `tick()` and in the HALF_OPEN branch.

Rulings implemented in PR #11 (commit 42cd673), pinned by tests in `test_selector_core.py`:
- [x] Counters: `test_009a_R07_stale_cloud_response_while_open_counts_only_as_inactive`,
  `test_009a_R05_every_response_lands_in_exactly_one_counter`.
- [x] Dead zone: `test_009a_R08_dead_zone_closed_cloud_slower_than_max_age_opens_and_local_takes_over`.
- [x] HALF_OPEN silence: `test_009a_R10_cloud_silence_in_half_open_returns_to_open`,
  `test_009a_R10_silence_in_half_open_returns_to_open_without_notification`.
- [x] Invariant: `test_009a_R08_invalid_threshold_ordering_is_rejected`.

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

- Q-06 undecided until the measurement. The requirements do not depend on the transport, because health is measured in the application.
- yolo11n on the robot CPU, even at 2 Hz, may compete with SLAM. Mitigation: measure in week 3 and reduce the resolution if needed.
- Exact format of the `MarkerArray` expected by the 2025S2 semantic node to be confirmed (T903).
