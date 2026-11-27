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
| `projection_core.py` | pure | `ProjectorCore`: pixel + depth + TF at the photo stamp to a 3D point (009a-R12). |
| `detection_filter.py` | pure | `DetectionFilter` (confidence, allowed labels, NaN, mapped area) and `GridArea` (mapped-area test). Policy only. |
| `detection_set.py` | pure | `DetectionSet`: persistent detections, merge of same label within `merge_distance_m`. |
| `marker_spec.py` | pure | `build_markers`: set to plain `MarkerSpec` dataclasses (CUBE + TEXT_VIEW_FACING). |
| `projector_pipeline.py` | pure | `ProjectionPipeline`: filter, project, filter, add to the set; counters for telemetry. |
| `stamp_cache.py`, `depth_sampling.py` | pure | `StampCache` (depth by stamp, max size and age), `depth_at` (median depth around a pixel). |
| `projector_node.py` | node | Thin glue: parameters, tf2 buffer behind `lookup_tf`, depth cache, `Marker` conversion, telemetry. |

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

## Data flow (projector)

```
/myRobot/perception/detections --> filter(label, confidence) --> ProjectorCore --> filter(NaN, area) --> DetectionSet
/camera_depth --> StampCache --(depth at the photo stamp)-----^   tf2 at header.stamp (R12)               |
/topologic_map --> GridArea (mapped-area check)                                                            v
                         timer every republish_period_s: WHOLE set --> /detections_markers (CUBE + TEXT)
                         timer telemetry_rate_hz --> /myRobot/telemetry/projector
```

The projector is the producer of `/detections_markers` (2025S2 format, `specs/009a-offload/markers-format.md`):
it owns the persistent set and republishes all of it every `republish_period_s` (2.0 s), because the semantic
node drops what arrives before `/topologic_map` and keeps no history. It never sends DELETE. Per detection it
publishes a CUBE (`ns=detections`, `id=det_id`, RViz only) and a TEXT_VIEW_FACING (`ns=detections_text`,
`id=det_id+100000`, text `"<label> (<det_id>)"`) in frame `map`; the text marker is the one the consumer reads.
NaN poses are rejected before publishing (the consumer raises `ValueError` on them). The TF query uses the
detection `header.stamp` and `tf_timeout_s`; depth comes from the cache entry nearest to that stamp within
`depth_tolerance_s`. Without a map yet, `check_mapped_area` drops detections (the consumer would drop them too).
Policy lives here and not in `detection_selector`, which only judges transport health.

Counters in `/myRobot/telemetry/projector`: `projected`, `dropped_tf`, `dropped_depth`, `dropped_label`,
`dropped_confidence`, `dropped_nan`, `dropped_area`, `accepted`, `merged`, `detections`.

### How to add a projector filter

1. Add a `check_*` method to `DetectionFilter` returning `None` or a reason string (a reason becomes the
   counter `dropped_<reason>`).
2. Call it from `ProjectionPipeline.process` and add `'dropped_<reason>': 0` to its counters.
3. Add a parameter to `ProjectorNode` and to `contracts/topics.yaml` in the same commit, and a
   `test_009a_R13_...` test. The node does not change otherwise.

## Parameters

Single source: `specs/000-system/contracts/topics.yaml`, section `parameters`. `detection_selector`: `max_age_s`
(0.67), `open_threshold_s` (1.5), `close_threshold_s` (0.5), `stable_window_s` (3.0), `status_rate_hz` (1.0),
`tick_rate_hz` (10.0), `cloud_topic`, `local_topic`, `output_topic`. The invariant
`close_threshold_s < max_age_s <= open_threshold_s` is checked on configure; the node fails to configure otherwise.
The watermark is internal state, not a parameter.

`image_throttle`: `image_rate_hz` (2.0, must be > 0), `jpeg_quality` (80), `telemetry_rate_hz` (1.0),
`input_topic` (`/camera`), `output_topic`.

`projector`: topics `detections_topic`, `depth_topic` (`/camera_depth`), `map_topic` (`/topologic_map`),
`output_topic`; frames `target_frame` (`map`), `camera_frame` (empty: detection header); intrinsics `fx fy cx cy`
(589.4, 589.4, 319.5, 239.5: `camera_fx` of the myRobot model in the `vacuum_sim_bridge` profile); `tf_timeout_s` (0.1), `republish_period_s`
(2.0), `min_confidence` (0.7), `allowed_labels` (unset: all), `check_mapped_area` (true), `merge_distance_m`
(0.5), `depth_cache_size` (8), `depth_cache_max_age_s` (2.0), `depth_tolerance_s` (0.05), `depth_window_px` (2),
`telemetry_rate_hz` (1.0). Depth is `32FC1` (m) or `16UC1` (mm), same resolution as the RGB image.

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
`autostart:=true` (with `with_manager:=false`, default false) makes the launch itself configure and activate each node
through launch lifecycle events; `with_projector:=true` adds the projector. With the manager, `autostart` is ignored.

Optional Python dependencies (`requirements-detector.txt`): `ultralytics` (+ torch). They are not rosdep keys and no test
needs them (fake model, `backend:=null`). Tests and the launch pass without them on both distros. torch wheels for
Python 3.14 (Lyrical) were not verified: if absent, run the detector from its container image (T910).
