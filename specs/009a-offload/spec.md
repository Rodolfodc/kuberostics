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
| 009a-R05 | For each response received, the selector shall compute `age = now − header.stamp` with the robot clock and drop the response if `age > max_age_s`, incrementing `dropped_stale`. | unit |
| 009a-R06 | The selector shall forward a response only if its `header.stamp` is strictly greater than the watermark, update the watermark when forwarding and, otherwise, drop it and increment `dropped_watermark`. | unit |
| 009a-R07 | While in CLOSED, the selector shall forward only cloud responses. While in OPEN or HALF_OPEN, only local responses. | unit |
| 009a-R08 | When in CLOSED and no cloud response with `age ≤ open_threshold_s` (default 1.5 s) arrives within `open_threshold_s`, the selector shall change to OPEN and emit a WARN `Notification` `CLOUD_DETECTOR_DOWN`. | unit + injected failure |
| 009a-R09 | When in OPEN and a cloud response with `age ≤ close_threshold_s` (default 0.5 s) arrives, the selector shall change to HALF_OPEN. | unit |
| 009a-R10 | When in HALF_OPEN and every cloud response stays with `age ≤ close_threshold_s` for `stable_window_s` (default 3 s), the selector shall change to CLOSED and emit an INFO `Notification` `CLOUD_DETECTOR_RECOVERED`. If any response violates the limit, it shall go back to OPEN. | unit |
| 009a-R11 | While in OPEN or HALF_OPEN, the robot shall keep sending images to the cloud at the same rate, to act as a probe. | contract |
| 009a-R12 | The `projector` shall query TF at the detection `header.stamp`, with timeout `tf_timeout_s` (default 0.1 s). If the transform does not exist, it shall drop the detection and increment `dropped_tf`. It shall never use the arrival instant. | unit |
| 009a-R13 | The `projector` shall publish `/detections_markers` in the format the 2025S2 semantic node already consumes. | contract + test with its node |
| 009a-R14 | The selector shall publish `OffloadStatus` at 1 Hz. | contract |
| 009a-R15 | When the cloud detector Pod is deleted during cleaning, the selector shall reach OPEN within `open_threshold_s + 0.5 s`, and the orchestrator mode shall not change. | injected failure |
| 009a-R16 | When the Pod comes back, the selector shall reach CLOSED, and no detection with a repeated `header.stamp` or one below the watermark shall appear on `/myRobot/perception/detections` throughout the test. | injected failure |
| 009a-R17 | The cloud detector shall run as a Deployment on K3s with a `livenessProbe`, from one image per distribution built by CI, and the infrastructure shall be declared in Terraform (`local` module). | CI + `terraform plan` |
| 009a-R18 | Requirements R01 to R16 shall hold on Jazzy and on Lyrical. | CI (matrix) |

## Definition of done

- Unit tests of the selector (R05 to R10) and of the `projector` (R12) passing on both distributions.
- Injected-failure test (R15, R16) executed and measured: time to OPEN, time to CLOSED and drop counters.
  These numbers go into the report.
- Q-06 decided by the project owner, with the measurement that justifies the choice (T902 recorded below; Wi-Fi and Jazzy measurements pending, T919 to T921).

## Robot to cloud transport (Q-06): measurement and proposal

Status: **recommendation recorded, awaiting the project owner.** Q-06 stays open in spec 000 until the owner confirms.
Source: T902, `deploy/measurements/t902-transport.md` (PR of branch `feat/t902-transport-measurement`).

### What was measured (Lyrical only)

`rmw_zenoh` 0.10.6 against `zenoh-bridge-ros2dds` 1.10.2, 640x480 JPEG (80 kB, synthetic) at 2 Hz, round trip =
robot clock at reception minus the `header.stamp` echoed by a stand-in detector (the age of R05, without inference time),
600 samples per cell, one run per cell, robot and K3s Pod on the same host.

| Scenario | Transport | p50 (ms) | p99 (ms) | max (ms) |
|---|---|---|---|---|
| lan | `rmw_zenoh` | 4.05 | 4.67 | 5.69 |
| lan | bridge | 5.15 | 5.89 | 6.91 |
| wan (netem 20 ms +/- 5 ms, 1 % loss, uplink only) | `rmw_zenoh` | 149.3 | 407.7 | 430.3 |
| wan | bridge | 393.2 | 579.9 | 624.3 |

Neither transport lost an image (both over TCP) nor reordered. From the raw CSV of the bridge on `wan` (this
specification's own computation, indicative): 6.9 % of the responses have age above `close_threshold_s` (0.5 s), none above
`max_age_s` (0.67 s), and 68 % of the windows of 6 consecutive responses (3 s at 2 Hz) are entirely at or below 0.5 s.

Limits: same-host lower bound (not Wi-Fi), synthetic image, a single run per cell, `wan` is a netem model on the uplink
only, Lyrical only (Jazzy has `rmw_zenoh` 0.2.x, not measured), CPU not measured, inference time of the detector not
included. The versions measured (0.10.6, 1.10.2) differ from those written in spec 000 Q-06 (0.10.8 and 1.9.0);
the table in spec 000 was not verified against the source in this change.

### Proposal (for the project owner): `zenoh-bridge-ros2dds` as the default transport

Reasons, all structural (T902 report): the allow-list enforces R03 by construction (only the two perception topics cross);
the motion loop (`twist_mux`, `stop_guard`) stays on DDS and its RMW does not change (Art. VI); one binary (1.10.2) for
both distributions, whereas `rmw_zenoh` would have to be qualified in two different versions (Art. X); the cloud Pod is
a bridge sidecar plus an ordinary DDS node. Latency does not separate them on a good link (4 ms against 5 ms).

Conditions to revisit and prefer `rmw_zenoh`: (1) the real robot-to-cluster link is lossy, as measured by the Wi-Fi task
T920 below, with the bridge's age above `close_threshold_s` in more than a small fraction of responses; (2) the Jazzy
and Lyrical versions of `rmw_zenoh` are both qualified (T919) and the ACL of `rmw_zenoh` is shown to enforce R03;
(3) the detector inference time (T921) leaves less margin than the transport difference.

### What the bridge's lossy-link tail implies for the selector

Read against the selector rulings of 2026-11 (CLOSED health uses `max_age_s`, invariant `close_threshold_s < max_age_s <=
open_threshold_s`, silence returns HALF_OPEN to OPEN): the transport numbers are *without* inference time, which the
selector sees, because the age is measured at the robot after the whole round trip.

- **Opening is not the expected event.** The breaker opens when no cloud response with age <= `max_age_s` (0.67 s)
  arrives for 1.5 s (R08); the bridge's maximum on the lossy link was 0.62 s, so with only the transport it stays CLOSED.
  What fails first is *closing again*: `close_threshold_s` is 0.5 s, the bridge has p50 0.39 s and p99 0.58 s, so
  after any opening (for example a Pod restart) HALF_OPEN to CLOSED (R10, every response <= 0.5 s for 3 s) will take
  longer or repeat, and on a really lossy link the robot may stay on the local detector. That is the correct, designed
  behaviour (Art. VII), not a defect, but the benefit of offloading is lost there.
- **The margin to `max_age_s` is thin.** Maximum 0.62 s against 0.67 s leaves about 45 ms. Any detector inference time
  above that pushes the tail beyond `max_age_s` and those responses are dropped as `dropped_stale`. With inference above about
  0.11 s the bridge's p50 on the lossy link exceeds `close_threshold_s` (the breaker cannot close), and above about 0.28 s the
  median exceeds `max_age_s` (most responses stale, breaker opens). yolo11x on K3s inference time is not measured yet.
- **No threshold change is proposed.** `max_age_s` is physical (0.20 m / 0.30 m/s) and `close_threshold_s` keeps the
  hysteresis; loosening them to suit the link would reintroduce stale detections on the semantic map (R05). If the
  measured link and inference leave no usable window, the choice is a better transport or a smaller cloud model, or running
  with `offload:=false`, not different thresholds.
- **Consequence for the report:** R15 and R16 are measured on a clean link (the local chaos test); add one run with
  netem on the uplink (T922) so that the time to CLOSED on a lossy link, and the share of `dropped_stale`, appear in the numbers.

### Requirement added by this proposal

| ID | Requirement | Verification |
|---|---|---|
| 009a-R19 | While `offload:=true`, the transport shall let only `/myRobot/perception/rgb_throttled/compressed` leave the robot and only `/myRobot/perception/detections/cloud` come in; every other topic shall stay local, whichever transport is chosen. | contract (allow-list or ACL checked against `topics.yaml`) |

Topic names are those of the T902 bench; confirm them against `contracts/topics.yaml` when T911 is done (this change
does not modify the contract).
