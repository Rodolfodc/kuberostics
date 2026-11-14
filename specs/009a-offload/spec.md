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
| 009a-R18 | Requirements R01 to R16 and R19 shall hold on Jazzy and on Lyrical. | CI (matrix) |

## Definition of done

- Unit tests of the selector (R05 to R10) and of the `projector` (R12) passing on both distributions.
- Injected-failure test (R15, R16) executed and measured: time to OPEN, time to CLOSED and drop counters.
  These numbers go into the report.
- Q-06 decided by the project owner, with the measurement that justifies the choice (T902 recorded below; Wi-Fi and Jazzy measurements pending, T919 to T921).

## Robot to cloud transport (Q-06): measurement and proposal

Status: **recommendation recorded, awaiting the project owner.** Q-06 stays open in spec 000 until the owner confirms.
Source: T902, `deploy/measurements/t902-transport.md` (PR of branch `feat/t902-transport-measurement`).

Dependency: the selector rulings quoted below (health uses `max_age_s`, the invariant, HALF_OPEN silence) exist only in
PR #13; **merge PR #13 first**. This section also must not merge before PR #16 (T902) is settled.

The figures below were recomputed from the committed CSVs (`deploy/measurements/t902/*.csv`) with the stated subset and
convention, and may be updated by the final summary of PR #16.

### What was measured (Lyrical only)

`rmw_zenoh` 0.10.6 against `zenoh-bridge-ros2dds` 1.10.2, 640x480 JPEG (80 kB, synthetic) at 2 Hz, round trip =
robot clock at reception minus the `header.stamp` echoed by a stand-in detector (the age of R05, without inference time),
one run per cell, robot and K3s Pod on the same host. Each wan CSV has 607 numeric rows (608 lines, one empty). Two
subsets and conventions were computed: (A) the first 600 rows with the bench's index method `sorted[int(p/100*n)]`
(the figures of the T902 summary), and (B) all 607 rows with linear interpolation (`statistics.quantiles`, inclusive).
Values are rounded to 1 ms because one run does not support more.

| Scenario | Transport | p50 (ms) A / B | p95 (ms) A / B | p99 (ms) A / B | max (ms), 607 rows |
|---|---|---|---|---|---|
| wan (netem 20 ms +/- 5 ms, 1 % loss, uplink only) | `rmw_zenoh` | 149 / 150 | 310 / 307 | 408 / 366 | 430 |
| wan | bridge | 393 / 393 | 511 / 511 | 580 / 566 | 624 |

The p99 depends on the convention and the subset (bridge 566 to 580 ms, `rmw_zenoh` 366 to 408 ms): quote it as "about
0.57 to 0.58 s" for the bridge; in both conventions it is above 0.5 s. The lan cells (p50 4.05 ms `rmw_zenoh`, 5.15 ms
bridge; p99 4.67 and 5.89 ms) are quoted from the T902 summary and were not recomputed here. Neither transport lost an image
(both over TCP) nor reordered. From the bridge wan CSV: 42 of 607 responses (6.9 %) exceed 0.5 s, none exceed 0.67 s;
68 % of the windows of 6 consecutive responses (3 s at 2 Hz) are entirely at or below 0.5 s. For `rmw_zenoh`, none exceed 0.5 s.

Limits: same-host lower bound (not Wi-Fi), synthetic image, a single run per cell, `wan` is a netem model on the uplink
only, Lyrical only (Jazzy has `rmw_zenoh` 0.2.x, not measured), CPU not measured, inference time of the detector not
included. Versions: 0.10.6 is the apt candidate on Lyrical (it may simply lag the upstream release), and the Jazzy
`rmw_zenoh` apt candidate in the report is 0.2.11; spec 000 Q-06 says 0.10.8 (Lyrical) and 0.2.13 (Jazzy), and the bridge 1.9.0
against 1.10.2 measured. Spec 000's text was left untouched and not verified against the source.

### Proposal (for the project owner): `zenoh-bridge-ros2dds` as the default transport

Reasons, all structural (T902 report): the allow-list enforces R03 by construction (only the two perception topics cross);
the motion loop (`twist_mux`, `stop_guard`) stays on DDS and its RMW does not change (Art. VI); one binary (1.10.2) for
both distributions, whereas `rmw_zenoh` would have to be qualified in two different versions (Art. X); the cloud Pod is
a bridge sidecar plus an ordinary DDS node. Latency does not separate them on a good link (4 ms against 5 ms); on a lossy link the bridge is worse (risk below).

Conditions to revisit and prefer `rmw_zenoh`: (1) the real robot-to-cluster link is lossy, as measured by the Wi-Fi task
T920 below, with the bridge's age above `close_threshold_s` in more than a small fraction of responses; (2) the Jazzy
and Lyrical versions of `rmw_zenoh` are both qualified (T919) and the ACL of `rmw_zenoh` is shown to enforce R03;
(3) the detector inference time (T921) leaves less margin than the transport difference.

### What the bridge's lossy-link tail implies for the selector

Read against the selector rulings of 2026-11 (CLOSED health uses `max_age_s`, invariant `close_threshold_s < max_age_s <=
open_threshold_s`, silence returns HALF_OPEN to OPEN): the transport numbers are *without* inference time, which the
selector sees, because the age is measured at the robot after the whole round trip.

- **Opening is not the expected event; reclosing is.** The breaker opens when no cloud response with age <= `max_age_s`
  (0.67 s) arrives for 1.5 s (R08); the bridge's maximum on the lossy link was 0.62 s, so on transport alone it stays CLOSED.
  What fails is *closing again* after any opening (for example a Pod restart): R10 needs every response of the 3 s stable
  window (about 6 at 2 Hz) at or below `close_threshold_s` (0.5 s). On the bridge, 42 of 607 responses (6.9 %) exceed 0.5 s
  and p95 is about 511 ms; with independent losses about 1 - 0.93^6, roughly 35 %, of the recovery windows fail and send the
  selector back to OPEN (the sample gives 32 %: 68 % of the windows are clean). With `rmw_zenoh` the maximum is 430 ms and
  the limit is never violated. The robot keeps working on the local detector (Art. VII), but the benefit of offloading is lost.
- **The margin to `max_age_s` is thin and the inference time is missing.** Maximum 0.62 s against 0.67 s leaves about 45 ms;
  the round trip excludes inference. With yolo11x in K3s, the bridge's p99 (about 0.57 to 0.58 s) plus inference probably crosses `max_age_s`
  and R05 drops those frames as `dropped_stale`. By arithmetic on the p50 (0.39 s): inference above about 0.11 s puts the
  median above `close_threshold_s` (the breaker cannot close), above about 0.28 s above `max_age_s` (most responses stale).
  Inference time is not measured (T921).
- **This is a risk against the bridge choice, not something to wave away.** The thresholds are not changed by this
  specification, and `max_age_s` is physical (0.20 m / 0.30 m/s), but the owner must decide, if the real link is lossy,
  between: (a) re-tuning `close_threshold_s`, `stable_window_s` and/or `max_age_s` for the bridge (a spec change to R05, R09, R10
  and to the invariant `close_threshold_s < max_age_s <= open_threshold_s`, with the cost of older detections on the
  semantic map), or (b) reconsidering `rmw_zenoh` (conditions above), or (c) running `offload:=false`. Until T920 and T921
  give numbers, the proposal stays a proposal.
- **Consequence for the report:** R15 and R16 are measured on a clean link (the local chaos test); add one run with
  netem on the uplink (T922) so that the time to CLOSED on a lossy link, and the share of `dropped_stale`, appear in the numbers.
- **The bridge is a new robot-side process** outside `lifecycle_manager_core` (T911 must declare it, in the optional group
  of D-09 or equivalent): if it dies the robot must keep cleaning (Art. VII), and its allow-list must never include
  `mux` or `cmd_vel` topics (Art. VI; R19). The bridge on Jazzy is untested: "same binary for both distros" is an
  expectation until T919. **Open for the project owner (Art. III, third-party binary):** does the bridge run on the robot
  inside the optional lifecycle group (D-09), or as a separate supervised process outside `lifecycle_manager_core`?

### Requirement added by this proposal

| ID | Requirement | Verification |
|---|---|---|
| 009a-R19 | While `offload:=true`, the transport shall let only `/myRobot/perception/rgb_throttled/compressed` leave the robot and only `/myRobot/perception/detections/cloud` come in; every other topic shall stay local, whichever transport is chosen. | contract (allow-list or ACL checked against `topics.yaml`) |

The two topics are already in `specs/000-system/contracts/topics.yaml`: `/myRobot/perception/rgb_throttled/compressed`
(`sensor_msgs/msg/CompressedImage`, owner `image_throttle`) and `/myRobot/perception/detections/cloud`
(`vision_msgs/msg/Detection2DArray`, owner `detector_cloud`). T923 generates the allow-list check from that file. This change
does not modify the contract.
