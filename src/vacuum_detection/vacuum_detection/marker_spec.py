"""Builds the marker specs of `/detections_markers` as plain dataclasses (spec 009a-R13, D-16). No ROS (D-18).

Format: specs/009a-offload/markers-format.md. The node only converts these to `visualization_msgs/Marker`.
Integer constants equal the ROS ones (checked in the node test).
"""
from dataclasses import dataclass
from typing import Iterable, List, Tuple

from vacuum_detection.detection_set import Detection

CUBE = 1
TEXT_VIEW_FACING = 9
ADD = 0
NS_CUBE = 'detections'
NS_TEXT = 'detections_text'          # the only one the consumer reads
TEXT_ID_OFFSET = 100000
FRAME = 'map'


@dataclass(frozen=True)
class MarkerSpec:
    ns: str
    id: int
    type: int
    action: int
    frame_id: str
    stamp: float
    x: float
    y: float
    z: float
    scale: Tuple[float, float, float]
    color: Tuple[float, float, float, float]    # r, g, b, a
    text: str = ''


def marker_text(det: Detection) -> str:
    return f'{det.label} ({det.det_id})'


def build_markers(detections: Iterable[Detection], frame_id: str = FRAME) -> List[MarkerSpec]:
    """CUBE (RViz) and TEXT (consumer) per detection; always ADD, never DELETE."""
    out = []
    for d in detections:
        out.append(MarkerSpec(NS_CUBE, d.det_id, CUBE, ADD, frame_id, d.stamp, d.x, d.y, d.z,
                              (0.2, 0.2, 0.2), (0.0, 1.0, 0.0, 0.8)))
        out.append(MarkerSpec(NS_TEXT, d.det_id + TEXT_ID_OFFSET, TEXT_VIEW_FACING, ADD, frame_id, d.stamp,
                              d.x, d.y, 0.25, (0.0, 0.0, 0.2), (1.0, 1.0, 1.0, 0.9), marker_text(d)))
    return out
