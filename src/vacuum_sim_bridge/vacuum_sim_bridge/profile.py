"""Robot numbers of the combined model (.sim/myRobotLaser.ttm) with their documented defaults.

Every value was measured on the model (see .sim/robot_report.txt). A per-robot profile file will override them
later: the nodes declare each field as a ROS parameter with the default below, so nothing is hard-coded elsewhere.

Conventions
-----------
* Model frame: the CoppeliaSim /myRobot frame. Its forward direction is local +Y, the right wheel is at +X.
* ROS frame `base_link` (REP-103): the model frame rotated +90 deg about z, so x_ros = y_model (forward) and
  y_ros = -x_model (left). Its origin is the model origin, 0.07 m behind the wheel-axle centre.
* `laser_link` and the camera frames are expressed in `base_link`.
"""
import math
from dataclasses import dataclass, fields


@dataclass(frozen=True)
class RobotProfile:
    # --- drive ---
    wheel_radius: float = 0.05            # m
    wheel_separation: float = 0.20        # m
    axle_offset_x: float = 0.07           # m, wheel-axle centre ahead of the base_link origin (ROS x)
    max_linear: float = 0.5               # m/s, the same cap python_controler applies
    max_angular: float = math.radians(90.0)  # rad/s, idem
    cmd_timeout: float = 0.5              # s without a command before the wheels are zeroed
    joint_sign: float = -1.0              # wheel travel = joint_sign * joint angle * radius (forward = negative)
    # --- laser (model: distSignal on /myRobot, 452 beams) ---
    scan_num_beams: int = 452             # expected; the real count is taken from the data (warning on mismatch)
    scan_angle_min: float = -math.pi / 2  # rad, beam 0 = right side
    scan_angle_increment: float = math.pi / 450.0  # rad (the model script steps pi/450, not 180/452 deg)
    scan_range_min: float = 0.055         # m, anything below (the model writes 0 for a miss) becomes inf
    scan_range_max: float = 10.0          # m
    laser_xyz: tuple = (0.0, 0.0, 0.16)   # m in base_link; no rotation (beam 0 = forward)
    # --- camera (kinect/rgb and kinect/depth, same pose) ---
    camera_width: int = 640
    camera_height: int = 480
    camera_fx: float = 589.4              # px; fx = fy, cx = 319.5, cy = 239.5 (informational, for downstream)
    camera_near: float = 0.01             # m
    camera_far: float = 3.5               # m
    camera_xyz: tuple = (0.0983, 0.0141, 0.077)       # m, optical frame origin in base_link
    # optical frame (x right, y down, z forward) in base_link, quaternion x y z w: pitched down 15.5 deg
    camera_quat: tuple = (-0.5628, 0.5628, -0.4168, 0.4390)
    camera_flip_horizontal: bool = True   # the sensor +x points left; same as the stock bridge
    camera_flip_vertical: bool = False    # UNVERIFIED: the stock bridge does not flip rows; check live (see tests)
    # --- frames and object paths ---
    odom_frame: str = 'my_robot_odom'     # same as the stock odometry node
    footprint_frame: str = 'base_footprint'  # odometry child frame, same as the stock node
    base_frame: str = 'base_link'
    laser_frame: str = 'laser_link'
    camera_frame: str = 'camera_link'
    footprint_to_base_z: float = 0.08     # m, model origin height above the floor
    robot_path: str = '/myRobot'
    left_motor_path: str = '/myRobot/leftMotor'
    right_motor_path: str = '/myRobot/rightMotor'
    rgb_sensor_path: str = '/myRobot/kinect/rgb'
    depth_sensor_path: str = '/myRobot/kinect/depth'
    scan_signal: str = 'signal.distSignal'  # float-array property on robot_path


def profile_fields():
    return [(f.name, f.default) for f in fields(RobotProfile)]
