# 009a: Offloading the detector to the cloud

Status: draft for review · Week 3 (18–24/11) · Depends on: 001, 004
Reference architecture: FogROS2 (Ichnowski et al., ICRA 2023) and FogROS2-FT. Decisions D-15 and D-16 of spec 000.

## Goal

Run the large YOLO detector (yolo11x) in a K3s Deployment while the robot cleans, without the robot depending on the
cloud. When the cloud goes down, gets slow or comes back, the robot switches source by itself, with no stale, duplicated
or out-of-order detections on the semantic map. All of this on Jazzy and on Lyrical.

## Out of scope

- Full robot ↔ cloud protocol (Q-07): authentication, versioning, sending policy.
- Cloud outside the local network; TLS on the transport.
- Offloading any node of the motion loop (Constitution, Art. VI).

## Concepts used

| Problem | Solution | Origin |
|---|---|---|
| Ambiguous silence | The detector publishes an empty array when it sees nothing; health is measured by the age of the last response | End-to-end argument (Saltzer, Reed and Clark, 1984) |
| Slow or dead | Slowness above the threshold counts as a failure | FLP result (Fischer, Lynch and Paterson, 1985) |
| How long to wait | Hysteresis with different thresholds to open and close, plus a stability window | Circuit breaker (Nygard, *Release It!*, 2007) |
| Late response | Timestamp echo: age = robot clock − `header.stamp` of the photo | NTP (Mills); TCP timestamp option (RFC 7323) |
| Duplicate or out of order | Monotonic watermark by photo timestamp | *The Dataflow Model* (Akidau et al., VLDB 2015) |
| Object in the wrong place | TF queried at the photo timestamp | tf2 |

The age limit comes from physics: `max_age_s = position_tolerance_m / v_max_m_s`.
With 0.20 m tolerance and 0.30 m/s, the limit is about 0.67 s.

## Requirements (EARS)

| ID | Requirement | Verification |
|---|---|---|
| 009a-R01 | The system shall accept the launch argument `offload:=true\|false`. With `false`, only the local detector runs, and the behaviour is the same as in spec 004. | launch_testing |
| 009a-R02 | Every detector, local or in the cloud, shall publish `Detection2DArray` with a `header` identical to that of the input image, and shall publish an empty array when it detects nothing. | unit |
| 009a-R03 | While `offload:=true`, the robot shall send the cloud only the compressed RGB image, at `image_rate_hz` (default 2 Hz). Depth shall not leave the robot. | contract |
| 009a-R04 | The local detector shall stay loaded and processing from startup (hot standby). | launch_testing |
| 009a-R05 | For each response that passed the source gate (R07), the selector shall compute `age = now − header.stamp` with the robot clock and drop the response if `age > max_age_s`, incrementing `dropped_stale`. | unit |
| 009a-R06 | The selector shall forward a response only if its `header.stamp` is strictly greater than the watermark, update the watermark when forwarding and, otherwise, drop it and increment `dropped_watermark`. | unit |
| 009a-R07 | While in CLOSED, the selector shall forward only cloud responses. While in OPEN or HALF_OPEN, only local responses. A response from the other source shall be dropped, regardless of its age, incrementing `dropped_inactive`. | unit |
| 009a-R08 | When in CLOSED and no cloud response with `age ≤ max_age_s` (a healthy response, i.e. one that would be forwarded by the age filter) arrives within `open_threshold_s` (default 1.5 s, and never below `max_age_s`), the selector shall change to OPEN and emit a WARN `Notification` `CLOUD_DETECTOR_DOWN`. | unit + injected failure |
| 009a-R09 | When in OPEN and a cloud response with `age ≤ close_threshold_s` (default 0.5 s) arrives, the selector shall change to HALF_OPEN. The selector shall reject a configuration that violates `close_threshold_s < max_age_s ≤ open_threshold_s`. | unit |
| 009a-R10 | When in HALF_OPEN and every cloud response stays with `age ≤ close_threshold_s` for `stable_window_s` (default 3 s), the selector shall change to CLOSED and emit an INFO `Notification` `CLOUD_DETECTOR_RECOVERED`. If any response violates the limit, it shall go back to OPEN. If no cloud response at all arrives for longer than `open_threshold_s` while in HALF_OPEN, it shall also go back to OPEN, with no `Notification`, and the stability window shall restart from the next response that triggers R09. | unit |
| 009a-R11 | While in OPEN or HALF_OPEN, the robot shall keep sending images to the cloud at the same rate, to act as a probe. | contract |
| 009a-R12 | The `projector` shall query TF at the detection `header.stamp`, with timeout `tf_timeout_s` (default 0.1 s). If the transform does not exist, it shall drop the detection and increment `dropped_tf`. It shall never use the arrival instant. | unit |
| 009a-R13 | The `projector` shall publish `/detections_markers` in the format the 2025S2 semantic node already consumes. | contract + test with its node |
| 009a-R14 | The selector shall publish `OffloadStatus` at 1 Hz. | contract |
| 009a-R15 | When the cloud detector Pod is deleted during cleaning, the selector shall reach OPEN within `open_threshold_s + 0.5 s`, and the orchestrator mode shall not change. | injected failure |
| 009a-R16 | When the Pod comes back, the selector shall reach CLOSED, and no detection with a repeated `header.stamp` or one below the watermark shall appear on `/myRobot/perception/detections` throughout the test. | injected failure |
| 009a-R17 | The cloud detector shall run as a Deployment on K3s with a `livenessProbe`, from one image per distribution built by CI, and the infrastructure shall be declared in Terraform (`local` module). | CI + `terraform plan` |
| 009a-R18 | Requirements R01 to R16 shall hold on Jazzy and on Lyrical. | CI (matrix) |

## Selector processing order and counters (rulings of 2026-11)

Each response is processed in this order:

1. Health tracking and breaker transitions (cloud responses only; local responses only call the silence check).
2. Source gate (R07).
3. Age filter (R05).
4. Watermark filter (R06).

Every response ends in exactly one counter, so `forwarded + dropped_inactive + dropped_stale + dropped_watermark` equals
the number of responses received. Meaning, for the R15/R16 report:

| Counter | Meaning |
|---|---|
| `forwarded` | Response from the active source, fresh and newer than the watermark. |
| `dropped_inactive` | Response from the source that is not active in the current state, whatever its age or stamp. Includes the local stream while CLOSED and the cloud stream while OPEN or HALF_OPEN (the probe). |
| `dropped_stale` | Response from the active source with `age > max_age_s`. |
| `dropped_watermark` | Response from the active source, fresh, with `stamp ≤ watermark` (duplicate or out of order). |

The watermark is shared by both sources: it is a robot-clock photo stamp, so it stays valid when the source changes.

Health (R08) uses the same limit as forwarding (`max_age_s`). A cloud answering with age between `max_age_s` and
`open_threshold_s` therefore cannot keep the breaker CLOSED: all its responses are dropped as stale, the breaker opens
after `open_threshold_s` and local takes over (Art. VII: the robot keeps getting detections). Hysteresis is kept because
`close_threshold_s < max_age_s` (R09 requires the selector to reject a configuration that breaks the invariant).
`max_age_s` is derived (`position_tolerance_m / v_max_m_s`), so changing the tolerance or the speed can trip the invariant.

Accepted side effects of this ruling:
- Up to `open_threshold_s` (1.5 s) without detections while the breaker is still CLOSED after the cloud dies or turns slow:
  local responses are dropped as `dropped_inactive` in that window.
- A cloud that is only intermittently healthy (about one healthy response per 1.5 s) stays CLOSED with most detections
  dropped as stale: no flapping, but a lower detection rate.
- The local fallback still works whenever the breaker opens (Art. VII).

## Definition of done

- Unit tests of the selector (R05 to R10) and of the `projector` (R12) passing on both distributions.
- Injected-failure test (R15, R16) executed and measured: time to OPEN, time to CLOSED and drop counters.
  These numbers go into the report.
- Q-06 decided, with the measurement that justifies the choice.
