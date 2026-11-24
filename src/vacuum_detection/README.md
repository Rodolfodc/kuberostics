# vacuum_detection

Robot side of spec 009a (detector offload). Rule of the package (D-18 to D-20): every decision lives in a pure
Python core with no ROS imports, tested with pytest and a simulated clock; every node is a thin `LifecycleNode`
(parameters in, messages out) managed by `lifecycle_manager_core`.

## Modules

| Module | Kind | What it does |
|---|---|---|
| `selector_core.py` | pure | Circuit breaker (CLOSED/OPEN/HALF_OPEN), source gate, age and watermark filters (009a-R05 to R10). Clock, notify callback and logger are injected. |
| `detection_selector_node.py` | node | Wires two `Detection2DArray` inputs to `SelectorCore`, republishes what it forwards, publishes `OffloadStatus` (R14), `Notification`s and telemetry. |
| `throttle_core.py` | pure | `ThrottleCore`: rate limit by photo stamp (009a-R03, R11), clock injected only for unstamped frames. |
| `image_codec.py` | pure | `encode_jpeg`: raw image buffer to JPEG bytes (numpy + OpenCV, no `cv_bridge`). |
| `image_throttle_node.py` | node | RGB `Image` in, JPEG `CompressedImage` out at `image_rate_hz`, header copied (timestamp echo), telemetry. |
| `detector_core.py` | pure | `DetectorCore`: runs an injected `DetectorModel`, filters by score, bounds the count, times the inference (009a-R02). `None` = inference failed (nothing is published), `[]` = saw nothing. |
| `yolo_model.py` | adapter | Ultralytics YOLO behind `DetectorModel`. `ultralytics`/torch are optional imports; missing ones raise `DetectorDependencyError` with instructions. |
| `detector_node.py` | node | `detector`: image (raw or compressed) in, `Detection2DArray` out with the image header copied; an empty array when nothing is detected; telemetry with inference time on `/myRobot/telemetry/<node name>`. Same executable is `detector_local` and (in K3s) `detector_cloud`. |
| `launch/detection.launch.py` | launch | `offload:=true\|false` (009a-R01), see below. |
| `ros_common.py` | helpers | QoS profiles, `stamp_to_sec`, `build_diagnostics`. No decisions. |
| `projection_core.py` | pure | Pixel + depth + TF at the photo stamp to a 3D point (009a-R12). Its node (`projector`, T908) is not written yet: blocked on the `/detections_markers` format (T903). |

## Data flow (image throttle)

```
/camera (Image, RGB) --> ThrottleCore (rate by stamp) --> encode_jpeg --> /myRobot/perception/rgb_throttled/compressed
```

The throttle knows nothing about the breaker state, so the probe keeps the same rate while OPEN or HALF_OPEN (R11).

## Data flow (selector)

```
/myRobot/perception/detections/cloud --\
                                        +--> SelectorCore --forwarded--> /myRobot/perception/detections
/myRobot/perception/detections/local --/        |  (state, counters)
                                                +--> /myRobot/perception/offload_status (1 Hz)
                                                +--> /myRobot/notifications (CLOUD_DETECTOR_DOWN / _RECOVERED)
                                                +--> /myRobot/telemetry/detection_selector
```

A timer calls `SelectorCore.tick()` at `tick_rate_hz` so that cloud silence opens the breaker without needing a
response. The clock is the node clock (robot clock); ages are `now - header.stamp`.

## Parameters

Single source: `specs/000-system/contracts/topics.yaml`, section `parameters`. `detection_selector`: `max_age_s`
(0.67), `open_threshold_s` (1.5), `close_threshold_s` (0.5), `stable_window_s` (3.0), `status_rate_hz` (1.0),
`tick_rate_hz` (10.0), `cloud_topic`, `local_topic`, `output_topic`. The invariant
`close_threshold_s < max_age_s <= open_threshold_s` is checked on configure; the node fails to configure otherwise.
The watermark is internal state, not a parameter.

`image_throttle`: `image_rate_hz` (2.0, must be > 0), `jpeg_quality` (80), `telemetry_rate_hz` (1.0),
`input_topic` (`/camera`), `output_topic`.

## Tests

```bash
scripts/test-docker.sh               # both distros (Art. X)
scripts/test-docker.sh lyrical
# pure cores only, no ROS needed:
cd src/vacuum_detection && python3 -m pytest test/test_selector_core.py test/test_projector_core.py test/test_throttle_core.py
```

Each test name cites the requirement it verifies (`test_009a_R06_...`). Node tests need rclpy and are skipped
without it.

## How to add a detector

A detector is any node that subscribes to an RGB image and publishes `vision_msgs/Detection2DArray` with the
`header` copied from the image and an empty array when nothing is detected (009a-R02, R04). To plug it in: publish
on `/myRobot/perception/detections/local` (or `/cloud`), or give `detection_selector` another topic through its
`local_topic` / `cloud_topic` parameters. Nothing else changes: the selector only looks at the source and the stamp.

## Detector and launch (spec 009a, robot side)

```
ros2 launch vacuum_detection detection.launch.py offload:=false   # detector_local + detection_selector, nothing leaves the robot
ros2 launch vacuum_detection detection.launch.py offload:=true    # the same + image_throttle; the cloud detector runs in K3s, outside this launch
```

Only `detection_selector` publishes `/myRobot/perception/detections` (contract: `max_publishers: 1`); `detector_local` always
publishes `/myRobot/perception/detections/local`. With `offload:=false` there is no cloud input, so the selector opens its
breaker after `open_threshold_s` (1.5 s) and forwards local; until then local responses count as `dropped_inactive`, and a
`CLOUD_DETECTOR_DOWN` notification is emitted once.

Parameters of `detector`: `backend` (`ultralytics`|`null`), `weights` (`yolo11n.pt` local, `yolo11x.pt` cluster, or a
path), `device`, `score_threshold`, `iou_threshold`, `image_size`, `max_detections`, `input_type` (`image`|`compressed`),
`input_topic`, `output_topic`, `telemetry_rate_hz`. Cloud: `input_type:=compressed`, `input_topic:=/myRobot/perception/
rgb_throttled/compressed`, `output_topic:=/myRobot/perception/detections/cloud`, `weights:=yolo11x.pt`.

Lifecycle (D-26, D-09): the chain is under `lifecycle_manager_optional` (a perception failure never stops the core,
Art. VII). The launch starts its own instance (`with_manager:=true`); with the skeleton sequencer use
`with_manager:=false` and add the node names to `vacuum_bringup/config/lifecycle_optional.yaml`.

Optional Python dependencies (`requirements-detector.txt`): `ultralytics` (+ torch). They are not rosdep keys and no test
needs them (fake model, `backend:=null`). Tests and the launch pass without them on both distros. torch wheels for
Python 3.14 (Lyrical) were not verified: if absent, run the detector from its container image (T910).
