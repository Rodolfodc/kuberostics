"""room_segmenter node glue tests (003-R18, R22, R23, R26). In-process rclpy, condition waits, no launch."""
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import OccupancyGrid                        # noqa: E402
from rclpy.executors import SingleThreadedExecutor            # noqa: E402
from rclpy.parameter import Parameter                          # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy   # noqa: E402
from std_srvs.srv import Trigger                               # noqa: E402
from vacuum_interfaces.msg import RoomArray                    # noqa: E402
from vacuum_interfaces.srv import SetRoomLabel                 # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray         # noqa: E402

from grid_fixtures import two_rooms                            # noqa: E402
from vacuum_mapping.room_segmenter_node import RoomSegmenterNode   # noqa: E402

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.fixture()
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_until(executor, predicate, timeout_s=10.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end and not predicate():
        executor.spin_once(timeout_sec=0.05)
    return predicate()


def grid_msg():
    raster = two_rooms(0.7)
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = raster.resolution
    msg.info.height, msg.info.width = raster.cells.shape
    msg.info.origin.orientation.w = 1.0
    msg.data = raster.cells.flatten().tolist()
    return msg


class Harness:
    def __init__(self, tmp_path, **params):
        overrides = [Parameter('map_name', value='house'), Parameter('map_file', value=str(tmp_path / 'house.yaml')),
                     Parameter('min_votes', value=2)]
        overrides += [Parameter(k, value=v) for k, v in params.items()]
        self.node = RoomSegmenterNode(parameter_overrides=overrides)
        self.helper = rclpy.create_node('room_segmenter_test_helper')
        self.rooms, self.labels = [], []
        self.helper.create_subscription(RoomArray, '/myRobot/map/rooms', self.rooms.append, LATCHED)
        self.helper.create_subscription(OccupancyGrid, '/myRobot/map/room_labels', self.labels.append, LATCHED)
        self.map_pub = self.helper.create_publisher(OccupancyGrid, '/map', LATCHED)
        self.marker_pub = self.helper.create_publisher(MarkerArray, '/detections_markers', 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)

    def call(self, srv_type, name, request):
        client = self.helper.create_client(srv_type, name)
        assert spin_until(self.executor, client.service_is_ready)
        future = client.call_async(request)
        assert spin_until(self.executor, future.done)
        return future.result()

    def close(self):
        self.node.destroy_node()
        self.helper.destroy_node()


def start(tmp_path, **params):
    h = Harness(tmp_path, **params)
    h.node.trigger_configure()
    h.node.trigger_activate()
    h.map_pub.publish(grid_msg())
    return h


def test_003_R18_R26_start_up_with_map_file_publishes_room_labels_and_rooms(ros, tmp_path):
    h = start(tmp_path)
    assert spin_until(h.executor, lambda: h.rooms and h.labels)
    assert [r.id for r in h.rooms[-1].rooms] == ['room_01', 'room_02']
    assert h.rooms[-1].header.frame_id == 'map'
    assert set(h.labels[-1].data) >= {0, 1, 2}
    assert h.labels[-1].info.width == grid_msg().info.width
    h.close()


def test_003_R18_segment_service_before_any_map_fails_cleanly(ros, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)      # without map_file the rooms file goes to ./maps
    h = Harness(tmp_path, map_file='')
    h.node.trigger_configure()
    h.node.trigger_activate()
    response = h.call(Trigger, '/myRobot/map/segment', Trigger.Request())
    assert response.success is False
    h.close()


def test_003_R18_segment_service_runs_on_demand_when_map_file_is_not_set(ros, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    h = Harness(tmp_path, map_file='')
    h.node.trigger_configure()
    h.node.trigger_activate()
    h.map_pub.publish(grid_msg())
    assert spin_until(h.executor, lambda: h.node._raster is not None)
    assert not h.rooms
    assert h.call(Trigger, '/myRobot/map/segment', Trigger.Request()).success
    assert spin_until(h.executor, lambda: h.rooms and h.labels)
    h.close()


def test_003_R23_set_label_updates_rooms_persists_and_rejects_unknown_id(ros, tmp_path):
    h = start(tmp_path)
    assert spin_until(h.executor, lambda: h.rooms)
    ok = h.call(SetRoomLabel, '/myRobot/rooms/set_label', SetRoomLabel.Request(id='room_02', label='bedroom'))
    assert ok.success
    assert spin_until(h.executor, lambda: {r.id: r.label for r in h.rooms[-1].rooms}['room_02'] == 'bedroom')
    assert h.rooms[-1].rooms[1].label_source == 2
    bad = h.call(SetRoomLabel, '/myRobot/rooms/set_label', SetRoomLabel.Request(id='room_99', label='x'))
    assert bad.success is False
    assert 'bedroom' in (tmp_path / 'house_rooms.yaml').read_text()
    h.close()


def test_003_R22_markers_label_a_room_and_republish_is_not_double_counted(ros, tmp_path):
    h = start(tmp_path)
    assert spin_until(h.executor, lambda: h.rooms)
    x, y = h.rooms[-1].rooms[0].centroid.x, h.rooms[-1].rooms[0].centroid.y

    def marker(i, text):
        m = Marker(ns='detections_text', id=i, text=text)
        m.pose.position.x, m.pose.position.y = x, y
        return m
    assert spin_until(h.executor, lambda: h.marker_pub.get_subscription_count() > 0)
    one = MarkerArray(markers=[marker(100001, 'oven (1)')])
    for _ in range(4):
        h.marker_pub.publish(one)
        spin_until(h.executor, lambda: False, 0.2)
    assert h.rooms[-1].rooms[0].label == 'room_01'                  # one oven is one vote, min_votes is 2
    h.marker_pub.publish(MarkerArray(markers=[marker(100001, 'oven (1)'), marker(100002, 'sink (2)')]))
    assert spin_until(h.executor, lambda: h.rooms[-1].rooms[0].label == 'kitchen')
    h.close()


def test_003_R26_telemetry_is_published(ros, tmp_path):
    from diagnostic_msgs.msg import DiagnosticArray
    h = start(tmp_path)
    got = []
    h.helper.create_subscription(DiagnosticArray, '/myRobot/telemetry/room_segmenter', got.append, 10)
    assert spin_until(h.executor, lambda: got)
    assert {kv.key for kv in got[-1].status[0].values} >= {'rooms', 'segmentations', 'failures'}
    h.close()
