"""T906: DetectorCore with a fake model (no torch, no weights, no GPU). Verifies 009a-R02 (empty array on no
detection is the caller's rule; the core returns [] for it and None for a failed inference) and the telemetry."""
import pytest

from vacuum_detection.detector_core import Detection, DetectorCore, NullModel
from vacuum_detection.yolo_model import DetectorDependencyError, load_yolo


class FakeModel:
    def __init__(self, result=None, error=None):
        self.result, self.error, self.calls = result or [], error, []

    def predict(self, image):
        self.calls.append(image)
        if self.error:
            raise self.error
        return self.result


class Clock:
    def __init__(self, step):
        self.t, self.step = 0.0, step

    def __call__(self):
        self.t += self.step
        return self.t


def det(label, score):
    return Detection(label, score, 10.0, 20.0, 4.0, 6.0)


def test_009a_R02_nothing_detected_is_an_empty_list_not_none():
    core = DetectorCore(FakeModel([]), Clock(0.01))
    assert core.process('img') == []
    assert core.snapshot()['empty'] == 1


def test_009a_R02_below_threshold_counts_as_nothing_detected():
    core = DetectorCore(FakeModel([det('cup', 0.2)]), Clock(0.01), score_threshold=0.25)
    assert core.process('img') == []


def test_009a_R02_keeps_best_first_and_bounds_the_count():
    core = DetectorCore(FakeModel([det('a', 0.5), det('b', 0.9), det('c', 0.7)]), Clock(0.01), max_detections=2)
    assert [d.label for d in core.process('img')] == ['b', 'c']


def test_009a_R02_model_failure_returns_none_and_is_counted():
    core = DetectorCore(FakeModel(error=RuntimeError('boom')), Clock(0.01))
    assert core.process('img') is None
    snap = core.snapshot()
    assert snap['errors'] == 1 and snap['processed'] == 0 and snap['empty'] == 0


def test_009a_R02_telemetry_has_inference_time():
    core = DetectorCore(FakeModel([det('a', 0.9)]), Clock(0.05))
    core.process('img')
    core.process('img')
    snap = core.snapshot()
    assert snap['processed'] == 2 and snap['detections'] == 2
    assert snap['last_inference_ms'] == pytest.approx(50.0)
    assert snap['mean_inference_ms'] == pytest.approx(50.0)
    assert snap['max_inference_ms'] == pytest.approx(50.0)


@pytest.mark.parametrize('kwargs', [{'score_threshold': 1.5}, {'score_threshold': -0.1}, {'max_detections': 0}])
def test_009a_R02_rejects_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        DetectorCore(FakeModel(), Clock(0.01), **kwargs)


def test_009a_R02_null_model_detects_nothing():
    assert NullModel().predict('img') == []


def test_009a_R02_missing_ultralytics_gives_a_clear_error():
    try:
        import ultralytics  # noqa: F401
    except ImportError:
        with pytest.raises(DetectorDependencyError, match='ultralytics'):
            load_yolo('yolo11n.pt')
    else:
        pytest.skip('ultralytics is installed here')
