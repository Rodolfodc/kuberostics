"""Ultralytics YOLO behind `DetectorModel` (spec 009a-R02, R04). No ROS imports.

`ultralytics` (and torch) are OPTIONAL imports: only this module needs them, and only when `load_yolo` is called.
Everything else in the package, and every test, runs without them (no GPU, no weights). If they are missing, or
the wheels do not exist for the Python of a distro (Lyrical has Python 3.14), `load_yolo` raises
`DetectorDependencyError` with the instructions, and the node fails its configure step: the robot keeps working
without a local detector (Art. VII) and the cloud detector still runs from its container image.
"""
from typing import Any, List

from vacuum_detection.detector_core import Detection


class DetectorDependencyError(RuntimeError):
    """ultralytics/torch missing or unusable. The message says what to install."""


class YoloModel:
    def __init__(self, yolo: Any, device: str, iou_threshold: float, image_size: int, min_score: float):
        self._yolo = yolo
        self._device = device
        self._iou = iou_threshold
        self._size = image_size
        self._min_score = min_score

    def predict(self, image: Any) -> List[Detection]:
        # The engine already drops boxes below `conf`; the core applies the same threshold again (cheap, and it
        # keeps the rule in one place for fake models).
        result = self._yolo.predict(image, device=self._device, conf=self._min_score, iou=self._iou,
                                    imgsz=self._size, verbose=False)[0]
        names = result.names
        out = []
        for box in result.boxes:
            cx, cy, w, h = (float(v) for v in box.xywh[0].tolist())
            out.append(Detection(label=str(names[int(box.cls[0])]), score=float(box.conf[0]),
                                 cx=cx, cy=cy, w=w, h=h))
        return out


def load_yolo(weights: str, device: str = 'cpu', iou_threshold: float = 0.45, image_size: int = 640,
              min_score: float = 0.25) -> YoloModel:
    """Loads `weights` (e.g. `yolo11n.pt`, `yolo11x.pt` or a path). Raises DetectorDependencyError or ValueError."""
    try:
        from ultralytics import YOLO                      # noqa: PLC0415  (optional dependency, see module doc)
    except ImportError as exc:
        raise DetectorDependencyError(
            "the 'ultralytics' package (and torch) is not installed or not importable: "
            f'{exc}. Install it with `pip install -r requirements-detector.txt` (see the vacuum_detection '
            "README), or use backend:=null. On Python 3.14 (Lyrical) torch wheels may not exist yet; run the "
            'detector from its container image instead.') from exc
    try:
        yolo = YOLO(weights)
    except Exception as exc:                              # bad path/name, download failed, corrupt file
        raise ValueError(f'could not load YOLO weights {weights!r}: {exc!r}') from exc
    return YoloModel(yolo, device, iou_threshold, image_size, min_score)
