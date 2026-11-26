"""`room_segmenter` LifecycleNode (spec 003): thin glue over `RoomSession`.

All decisions live in the pure cores. This node converts `/map` to a `Raster`, runs the session when asked
(service `segment`, after `save`, and once at start-up when `map_file` is set), publishes the latched
`room_labels` and `rooms` (003-R18), serves `set_label` (003-R23), feeds the labeler from the
`observations_topic` markers (003-R22) and publishes telemetry (D-20).

Files: `<dir>/<map_name>_rooms.yaml` with `<dir>` = directory of `map_file` when set, else `maps`.
`explored` (exploration is cut) is not consumed.
"""
import os
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
# RCLError is not re-exported by a public rclpy module on Jazzy; this path works on Jazzy and Lyrical.
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
from vacuum_interfaces.msg import Room as RoomMsg
from vacuum_interfaces.msg import RoomArray
from vacuum_interfaces.srv import SetRoomLabel
from visualization_msgs.msg import MarkerArray

from vacuum_mapping.core.marker_observations import ObservationBuffer
from vacuum_mapping.core.room_session import RoomSession
from vacuum_mapping.core.types import UNKNOWN, Raster, classify_occupancy
from vacuum_mapping.map_io import write_map
from vacuum_mapping.rooms_io import read_rules

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def raster_from_grid(msg: OccupancyGrid) -> Raster:
    values = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
    return Raster(classify_occupancy(values), float(msg.info.resolution),
                  float(msg.info.origin.position.x), float(msg.info.origin.position.y))


class RoomSegmenterNode(LifecycleNode):
    def __init__(self, node_name: str = 'room_segmenter', **kwargs):
        super().__init__(node_name, **kwargs)
        # Defaults and meaning: specs/000-system/contracts/topics.yaml (parameters: room_segmenter, mapping).
        self.declare_parameter('door_width_m', 0.9)
        self.declare_parameter('min_room_area_m2', 2.0)
        self.declare_parameter('min_votes', 2)
        self.declare_parameter('observations_topic', '/detections_markers')
        self.declare_parameter('rules_file', 'config/room_rules.yaml')
        self.declare_parameter('map_name', 'house')
        self.declare_parameter('map_file', '')
        self.declare_parameter('start_pose_xyyaw', [0.0, 0.0, 0.0])
        self._session: Optional[RoomSession] = None
        self._raster: Optional[Raster] = None
        self._map_msg: Optional[OccupancyGrid] = None
        self._active = False
        self._startup_pending = False
        self._timer = None
        self._segmentations = 0
        self._failures = 0

    # ---------------------------------------------------------------- lifecycle
    def on_configure(self, state) -> TransitionCallbackReturn:
        log = self.get_logger()
        map_file = self.get_parameter('map_file').value
        maps_dir = os.path.dirname(os.path.abspath(map_file)) if map_file else 'maps'
        rooms_path = os.path.join(maps_dir, self.get_parameter('map_name').value + '_rooms.yaml')
        rules = self._load_rules(self.get_parameter('rules_file').value)
        pose = tuple(float(v) for v in self.get_parameter('start_pose_xyyaw').value)
        try:
            self._session = RoomSession(
                rooms_path, self.get_parameter('map_name').value, rules,
                door_width_m=self.get_parameter('door_width_m').value,
                min_room_area_m2=self.get_parameter('min_room_area_m2').value,
                min_votes=self.get_parameter('min_votes').value,
                start_pose=pose if len(pose) == 3 else None, log=log.info)
        except (ValueError, OSError, KeyError) as exc:
            log.error(f'cannot configure: {exc}')
            return TransitionCallbackReturn.FAILURE
        self._buffer = ObservationBuffer(log=log.info)
        self._maps_dir = maps_dir
        self._pub_labels = self.create_lifecycle_publisher(OccupancyGrid, '/myRobot/map/room_labels', LATCHED)
        self._pub_rooms = self.create_lifecycle_publisher(RoomArray, '/myRobot/map/rooms', LATCHED)
        self._pub_telemetry = self.create_lifecycle_publisher(
            DiagnosticArray, '/myRobot/telemetry/room_segmenter', 10)
        self._subs = [
            self.create_subscription(OccupancyGrid, '/map', self._on_map, LATCHED),
            self.create_subscription(MarkerArray, self.get_parameter('observations_topic').value,
                                     self._on_markers, 10),
        ]
        self._srvs = [
            self.create_service(Trigger, '/myRobot/map/segment', self._srv_segment),
            self.create_service(Trigger, '/myRobot/map/save', self._srv_save),
            self.create_service(SetRoomLabel, '/myRobot/rooms/set_label', self._srv_set_label),
        ]
        log.info(f'configured, rooms file {rooms_path}, {len(self._session.rooms)} rooms loaded')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._active = True
        self._timer = self.create_timer(1.0, self._publish_telemetry)
        self._startup_pending = bool(self.get_parameter('map_file').value)   # 003-R18: at start-up with map_file
        if self._startup_pending and self._raster is not None:
            self._segment('start-up')
        elif self._session.rooms:
            self._publish_rooms(None)    # previous rooms are served until a map arrives
        self.get_logger().info('activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        self._active = False
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        super().on_deactivate(state)
        self.get_logger().info('deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        for sub in self._subs:
            self.destroy_subscription(sub)
        for srv in self._srvs:
            self.destroy_service(srv)
        for pub in (self._pub_labels, self._pub_rooms, self._pub_telemetry):
            self.destroy_publisher(pub)
        self._session = None
        return TransitionCallbackReturn.SUCCESS

    # ---------------------------------------------------------------- data flow
    def _load_rules(self, rules_file: str):
        path = rules_file
        if not os.path.isabs(path):
            try:
                path = os.path.join(get_package_share_directory('vacuum_mapping'), rules_file)
            except PackageNotFoundError:
                pass
        try:
            rules = read_rules(path)
            self.get_logger().info(f'{len(rules)} room rules from {path}')
            return rules
        except (OSError, ValueError, KeyError, AttributeError) as exc:
            self.get_logger().warning(f'no room rules ({exc}); automatic labels disabled')
            return {}

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            self._raster = raster_from_grid(msg)
        except ValueError as exc:
            self.get_logger().error(f'ignoring invalid /map: {exc}')
            return
        self._map_msg = msg
        if self._active and self._startup_pending:
            self._segment('start-up')

    def _on_markers(self, msg: MarkerArray) -> None:
        if self._buffer.update(msg) and self._session is not None:
            self._session.set_observations(self._buffer.observations())
            if self._active and self._map_msg is not None:
                self._publish_rooms(self._map_msg.header.stamp)

    def _segment(self, reason: str) -> bool:
        self._startup_pending = False
        if self._raster is None or self._map_msg is None:
            self._failures += 1
            return False
        try:
            tracked = self._session.segment(self._raster)
        except (ValueError, OSError) as exc:
            self._failures += 1
            self.get_logger().error(f'segmentation failed ({reason}): {exc}')
            return False
        self._segmentations += 1
        labels = tracked.labels.astype(np.int16)
        labels[self._raster.cells == UNKNOWN] = -1                 # 003-R18: -1 on unknown cells
        out = OccupancyGrid()
        out.header = self._map_msg.header
        out.info = self._map_msg.info
        out.data = np.clip(labels, -1, 127).astype(np.int8).flatten().tolist()
        if self._active:
            self._pub_labels.publish(out)
            self._publish_rooms(self._map_msg.header.stamp)
        self.get_logger().info(f'segmented ({reason}): {len(tracked.rooms)} rooms')
        return True

    def _publish_rooms(self, stamp) -> None:
        msg = RoomArray()
        msg.header.frame_id = 'map'
        if stamp is not None:
            msg.header.stamp = stamp
        for room in self._session.rooms:
            msg.rooms.append(RoomMsg(
                index=room.index, id=room.id, label=room.label, label_source=int(room.label_source),
                centroid=Point(x=room.centroid[0], y=room.centroid[1], z=0.0), area_m2=float(room.area_m2)))
        self._pub_rooms.publish(msg)

    def _publish_telemetry(self) -> None:
        values = {'rooms': len(self._session.rooms), 'segmentations': self._segmentations,
                  'failures': self._failures, 'observations': len(self._buffer),
                  'has_map': self._raster is not None}
        status = DiagnosticStatus(name=self.get_name(), hardware_id='vacuum', level=DiagnosticStatus.OK,
                                  message='ok', values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self._pub_telemetry.publish(array)

    # ---------------------------------------------------------------- services
    def _srv_segment(self, request, response):
        response.success = self._active and self._segment('service')
        response.message = 'segmented' if response.success else 'no map received yet or node inactive'
        return response

    def _srv_save(self, request, response):
        """003-R07 minimal: write the last /map (+ rooms file) and segment it (003-R18)."""
        if not self._active or self._raster is None:
            response.success, response.message = False, 'no map received yet or node inactive'
            return response
        yaml_path = os.path.join(self._maps_dir, self.get_parameter('map_name').value + '.yaml')
        try:
            os.makedirs(self._maps_dir, exist_ok=True)
            write_map(self._raster, yaml_path)
        except OSError as exc:
            response.success, response.message = False, f'cannot write {yaml_path}: {exc}'
            return response
        response.success = self._segment('save')
        response.message = f'saved {yaml_path}' if response.success else 'saved, segmentation failed'
        return response

    def _srv_set_label(self, request, response):
        if not self._active:
            response.success, response.message = False, 'node inactive'
            return response
        response.success, response.message = self._session.set_label(request.id, request.label)
        if response.success:
            self._publish_rooms(self._map_msg.header.stamp if self._map_msg else None)
        else:
            self.get_logger().warning(f'set_label rejected: {response.message}')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoomSegmenterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except RCLError:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
