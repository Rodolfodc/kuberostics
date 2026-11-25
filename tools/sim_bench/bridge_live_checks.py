#!/usr/bin/env python3
"""Live checks of the vacuum_sim_bridge nodes against ground truth from the simulator (local-only, Art. V).
Runs inside the ROS container (rclpy + coppeliasim_zmqremoteapi_client), ROS_DOMAIN_ID 77, with the four bridges and
twist_mux running (tools/sim_bench/bridge_live_start.sh) and the simulator loaded by bridge_live_setup.py.
Usage: bridge_live_checks.py [scan] [camera] [drive] [turn] [tf]   (default: all, in that order)
Ground truth: a probe ray sensor created in the scene (removed at the end) and the world pose of the model."""
import math, sys, time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformListener
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

FX = 589.4
sim = RemoteAPIClient().require('sim')
robot = sim.getObject('/myRobot')
ref = sim.getObject('/myRobot/LaserScanner_2D/Laserscanner_ref')
rgb_h = sim.getObject('/myRobot/kinect/rgb')
dep_h = sim.getObject('/myRobot/kinect/depth')
left_j, right_j = sim.getObject('/myRobot/leftMotor'), sim.getObject('/myRobot/rightMotor')


def save_png(path, arr):
    import zlib, struct
    a = np.ascontiguousarray(arr.astype(np.uint8)); h, w = a.shape[:2]; ch = 1 if a.ndim == 2 else 3
    raw = b''.join(b'\x00' + a[r].tobytes() for r in range(h))
    def chunk(t, d):
        c = struct.pack('>I', len(d)) + t + d
        return c + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    open(path, 'wb').write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0 if ch == 1 else 2, 0, 0, 0))
                           + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))


def out(s=''):
    print(s, flush=True)


def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]


def rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Rig(Node):
    def __init__(self):
        super().__init__('live_checks')
        self.scan = self.cam = self.dep = self.odom = None
        self.odoms = []
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 10)
        self.create_subscription(Image, '/camera', lambda m: setattr(self, 'cam', m), 10)
        self.create_subscription(Image, '/camera_depth', lambda m: setattr(self, 'dep', m), 10)
        self.create_subscription(Odometry, '/myrobot/odom', self._odom, 10)
        self.tele = self.create_publisher(Twist, '/myRobot/mux/teleop', 10)
        self.cmd = []
        self.create_subscription(Twist, '/myRobot/cmd_vel', lambda m: self.cmd.append((time.time(), m.linear.x, m.angular.z)), 10)
        self.tfbuf = Buffer(); self.tfl = TransformListener(self.tfbuf, self)

    def _odom(self, m):
        self.odom = m; self.odoms.append((time.time(), m))

    def fresh(self, name, timeout=20.0):
        setattr(self, name, None); t0 = time.time()
        while getattr(self, name) is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return getattr(self, name)

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, v, w):
        m = Twist(); m.linear.x = float(v); m.angular.z = float(w); self.tele.publish(m)


class Pub:
    """Teleop publisher in its own process (10 Hz, wall clock): the simulator calls here take ~0.26 s each when the
    laser makes the steps slow, so publishing from this loop would starve twist_mux (timeout 0.5 s)."""
    proc = None

    @classmethod
    def set(cls, v, w):
        cls.stop()
        import subprocess
        msg = '{linear: {x: %s}, angular: {z: %s}}' % (v, w)
        cls.proc = subprocess.Popen(['ros2', 'topic', 'pub', '-r', '10', '/myRobot/mux/teleop', 'geometry_msgs/msg/Twist', msg],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    @classmethod
    def stop(cls):
        if cls.proc:
            cls.proc.terminate(); cls.proc.wait(); cls.proc = None


def gt_pose():
    p = sim.getObjectPose(robot, -1)
    return p[0], p[1], yaw_of(p[3:])      # model yaw; ROS heading = model yaw + 90 deg


_hidden = []


def hide_robot(on):
    """Makes the robot model not detectable so the probe ray cannot hit its own laser/kinect bodies."""
    if on:
        _hidden.append(sim.getModelProperty(robot))
        sim.setModelProperty(robot, _hidden[-1] | sim.modelproperty_not_detectable)
    else:
        sim.setModelProperty(robot, _hidden.pop())


def make_ray(offset=0.0, rng=10.0):
    hide_robot(True)
    s = sim.createProximitySensor(sim.proximitysensor_ray_subtype, 16, 0, [1, 1, 1, 1, 1, 0, 0, 0],
                                  [offset, rng, 0.01, 0.01, 0.01, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    return s


def ray_world(sensor, origin, direction, rng=10.0):
    """distance along `direction` (world, unit) from `origin` to the first hit, or None; the sensor is aimed with its z axis."""
    d = np.asarray(direction, float); d /= np.linalg.norm(d)
    z = np.array([0, 0, 1.0]); ax = np.cross(z, d); s = np.linalg.norm(ax); c = float(z @ d)
    if s < 1e-9:
        q = [0, 0, 0, 1.0] if c > 0 else [1.0, 0, 0, 0]
    else:
        ax /= s; ang = math.acos(max(-1, min(1, c))); q = [*(ax * math.sin(ang / 2)), math.cos(ang / 2)]
    sim.setObjectPose(sensor, [*map(float, origin), *map(float, q)], -1)
    r = sim.checkProximitySensor(sensor, sim.handle_all)
    return r[1] if r[0] == 1 else None


def check_scan(rig, tag):
    out(f'== SCAN [{tag}] ==')
    m = rig.fresh('scan')
    refp = sim.getObjectPose(ref, -1); R = rot(refp[3:]); o = np.array(refp[:3])
    ray = make_ray()
    n = len(m.ranges); inc = m.angle_increment
    out(f'frame={m.header.frame_id} n={n} angle_min={math.degrees(m.angle_min):.2f} deg inc={math.degrees(inc):.4f} deg angle_max={math.degrees(m.angle_max):.2f} range=[{m.range_min},{m.range_max}]')
    raw = sim.getFloatArrayProperty(robot, 'signal.distSignal', {'noError': True})
    zeros = [i for i, x in enumerate(raw) if x == 0]
    infs = [i for i, x in enumerate(m.ranges) if math.isinf(x)]
    out(f'raw zeros={len(zeros)} bridge inf={len(infs)} same set={zeros == infs} zeros-left-in-scan={sum(1 for x in m.ranges if x == 0)} nan={sum(1 for x in m.ranges if math.isnan(x))}')
    errs = []; rows = []
    # the model's sweep: beam i at -90 deg + i*pi/450 (laser frame x forward, CCW positive)
    for i in list(range(0, n, 25)) + [n - 1]:
        a = -math.pi / 2 + i * math.pi / 450
        d = R @ np.array([math.cos(a), math.sin(a), 0])
        gt = ray_world(ray, o + d * 0.0, d)
        bridge = m.ranges[i]
        rows.append((i, math.degrees(a), gt, bridge, raw[i]))
        if gt is not None and math.isfinite(bridge):
            errs.append(bridge - gt)
    sim.removeObjects([ray]); hide_robot(False)
    out(' idx  angle   gt_from_ref  bridge   raw')
    for i, a, gt, b, r in rows:
        out(f'{i:4d} {a:7.1f}  {("%.3f" % gt) if gt is not None else "  miss"}  {b:7.3f}  {r:6.3f}')
    if errs:
        out(f'bridge - gt (m): mean {np.mean(errs):+.3f} median {np.median(errs):+.3f} min {min(errs):+.3f} max {max(errs):+.3f} over {len(errs)} beams')
    # mirror test: if the angle axis were reversed the profile would disagree with GT; report correlation both ways
    pairs = [(r[2], r[3]) for r in rows if r[2] is not None and math.isfinite(r[3])]
    rev = []
    for i, a, gt, b, r in rows:
        j = n - 1 - i
        if gt is not None and math.isfinite(m.ranges[j]):
            rev.append(m.ranges[j] - gt)
    if rev:
        out(f'(for comparison, reversed beam order would give mean |err| {np.mean(np.abs(rev)):.3f} m vs {np.mean(np.abs(errs)):.3f} m as published)')


def pixel_dir(R, c, r):
    f = R @ np.array([0, 0, 1.0])
    up = np.array([0, 0, 1.0]) - (np.array([0, 0, 1.0]) @ f) * f; up /= np.linalg.norm(up)
    right = np.cross(f, up)
    d = f + ((c - 319.5) / FX) * right - ((r - 239.5) / FX) * up
    return f, d / np.linalg.norm(d)


def check_camera(rig, tag):
    out(f'== CAMERA [{tag}] ==')
    cam = rig.fresh('cam'); dep = rig.fresh('dep')
    out(f'/camera {cam.width}x{cam.height} {cam.encoding} step={cam.step} frame={cam.header.frame_id}; /camera_depth {dep.width}x{dep.height} {dep.encoding} step={dep.step}')
    ci = np.frombuffer(bytes(cam.data), np.uint8).reshape(cam.height, cam.width, 3)
    di = np.frombuffer(bytes(dep.data), np.float32).reshape(dep.height, dep.width)
    out(f'rgb mean={ci.mean():.1f} std={ci.std():.1f}; depth finite={np.isfinite(di).mean()*100:.1f}% min={np.nanmin(di):.3f} max={np.nanmax(di):.3f} nan={np.isnan(di).mean()*100:.1f}%')
    tagf = tag.replace(' ', '_')
    save_png(f'/tmp/cam_{tagf}_rgb.png', ci)
    save_png(f'/tmp/cam_{tagf}_depth.png', np.clip(np.nan_to_num(di, nan=3.5) / 1.5 * 255, 0, 255))
    # raw frames from the simulator, then which flip reproduces the bridge output
    rawc, rc = sim.getVisionSensorImg(rgb_h); rawd, _ = sim.getVisionSensorDepth(dep_h)
    rc_arr = np.frombuffer(rawc, np.uint8).reshape(rc[1], rc[0], 3)
    rd_arr = np.frombuffer(rawd, np.float32).reshape(rc[1], rc[0])
    metres = (0.01 + rd_arr * 3.49)
    for name, fh, fv in (('none', 0, 0), ('flipH', 1, 0), ('flipV', 0, 1), ('flipH+flipV', 1, 1)):
        a = rc_arr
        if fh: a = a[:, ::-1]
        if fv: a = a[::-1]
        e = np.abs(a.astype(int) - ci.astype(int)).mean()
        b = metres
        if fh: b = b[:, ::-1]
        if fv: b = b[::-1]
        ed = np.nanmean(np.abs(np.where(rd_arr >= 0.999, np.nan, b) - di))
        out(f'  published vs raw sim buffer with {name:12s}: rgb mean abs diff {e:6.2f}   depth mean abs diff {ed:.4f} m')
    # ground truth z-depth at sample pixels (upright image: row 0 = top, column 0 = robot-left)
    sp = sim.getObjectPose(dep_h, -1); Rs = rot(sp[3:]); o = np.array(sp[:3])
    # the sensor looks along its +z; "right of image" must be the robot's right (model +X direction at yaw 0)
    ray = make_ray()
    out('  pixel(c,r)   gt_zdepth  bridge_depth   (upright: row 0 = top, col 0 = left)')
    res = []
    for c, r in ((320, 470), (320, 10), (320, 240), (20, 240), (620, 240), (20, 470), (620, 470), (20, 10), (620, 10)):
        f, d = pixel_dir(Rs, c, r)
        gtd = ray_world(ray, o + d * 0.0, d)
        gz = gtd * float(d @ f) if gtd is not None else None
        b = float(di[r, c])
        res.append((c, r, gz, b))
        out(f'  ({c:3d},{r:3d})   {("%.3f" % gz) if gz is not None else "  none"}      {b:.3f}')
    grid = []
    for r in range(40, 440, 80):
        for c in range(40, 640, 100):
            f, d = pixel_dir(Rs, c, r); gtd = ray_world(ray, o, d)
            if gtd is not None:
                gz = gtd * float(d @ f)
                if gz < 3.3:
                    grid.append((c, r, gz))
    sim.removeObjects([ray]); hide_robot(False)
    if grid:
        for name, tf in (('as published', lambda c, r: (c, r)), ('mirrored left-right', lambda c, r: (639 - c, r)),
                         ('upside down', lambda c, r: (c, 479 - r)), ('rotated 180', lambda c, r: (639 - c, 479 - r))):
            e = [abs(di[tf(c, r)[1], tf(c, r)[0]] - g) for c, r, g in grid if np.isfinite(di[tf(c, r)[1], tf(c, r)[0]])]
            out(f'  grid of {len(grid)} ground-truth pixels vs image {name:20s}: median |err| {np.median(e):.3f} m, mean {np.mean(e):.3f} m')
    ok = [abs(g - b) for _, _, g, b in res if g is not None and math.isfinite(b)]
    out(f'  mean |gt - bridge| = {np.mean(ok):.3f} m over {len(ok)} pixels (max {max(ok):.3f})' if ok else '  no comparable pixels')
    # floor-wall transition row in depth vs in rgb luminance (is rgb row order the same as depth row order?)
    col = slice(250, 390)
    dprof = np.nanmean(np.where(np.isfinite(di[:, col]), di[:, col], 4.0), axis=1)
    lum = ci[:, col].astype(float).mean(axis=(1, 2))
    rd_ = int(np.argmax(np.abs(np.diff(dprof)))); rl_ = int(np.argmax(np.abs(np.diff(lum))))
    out(f'  strongest vertical edge: depth row {rd_}, rgb luminance row {rl_} (should be close if both share the row order)')
    return ci, di


def drive(rig, v, w, secs_sim, label, odom_start=None):
    """Publishes on /myRobot/mux/teleop at 10 Hz (real time) for `secs_sim` of simulated time. Returns samples."""
    out(f'-- drive {label}: v={v} w={w} for {secs_sim:.1f} s of simulation time')
    rig.cmd.clear()
    Pub.set(v, w); rig.spin(2.0)        # let the CLI publisher start and the command reach the bridge
    t0s = sim.getSimulationTime(); rows = []; vel = []
    tmax = time.time() + 600
    while time.time() < tmax:
        rig.spin(0.01)
        ts = sim.getSimulationTime()
        x, y, yaw = gt_pose()
        lv, rv = sim.getJointVelocity(left_j), sim.getJointVelocity(right_j)
        od = rig.odom
        rows.append((ts - t0s, x, y, yaw, lv, rv, (od.pose.pose.position.x, od.pose.pose.position.y,
                    2 * math.atan2(od.pose.pose.orientation.z, od.pose.pose.orientation.w)) if od else (None,) * 3))
        if ts - t0s >= secs_sim:
            break
    Pub.stop(); rig.spin(1.0)
    return rows


def settle(rig, secs=1.5):
    Pub.stop(); rig.spin(secs)


def report_straight(rows):
    t, x, y, yaw, lv, rv, od = zip(*rows)
    t = np.array(t); x = np.array(x); y = np.array(y)
    # forward = model +Y rotated by yaw: at the start yaw ~ -90 deg, so travel is along world +X
    fx, fy = -math.sin(yaw[0] + 0), math.cos(yaw[0])        # model +Y in world
    s = (x - x[0]) * fx + (y - y[0]) * fy
    lat = -(x - x[0]) * fy + (y - y[0]) * fx
    w_lin = -np.array(lv) * 0.05, -np.array(rv) * 0.05
    sel = t > 2.0
    sp = (s[-1] - s[sel][0]) / (t[-1] - t[sel][0])
    steps = len(rows)
    out(f'samples={steps} sim time {t[-1]:.2f} s; distance forward {s[-1]:.3f} m (lateral {lat[-1]:+.3f} m); yaw change {math.degrees(wrap(yaw[-1]-yaw[0])):+.2f} deg')
    out(f'speed t>2s from ground truth: {sp:.3f} m/s; wheel linear speed (joint velocity*0.05) left mean {np.mean(w_lin[0][sel]):.3f} right mean {np.mean(w_lin[1][sel]):.3f} m/s; mean |L-R| {np.mean(np.abs(w_lin[0][sel]-w_lin[1][sel])):.4f} m/s, max |L-R| {np.max(np.abs(w_lin[0][sel]-w_lin[1][sel])):.4f} m/s')
    zeros = int(np.sum((np.abs(w_lin[0]) < 0.02) & (t > 2.0)))
    out(f'samples with the left wheel near zero after 2 s: {zeros} of {int(sel.sum())}')
    if od[0][0] is not None and od[-1][0] is not None:
        dox, doy = od[-1][0] - od[0][0], od[-1][1] - od[0][1]
        out(f'odometry (odom frame) delta x={dox:+.3f} y={doy:+.3f} yaw={math.degrees(wrap(od[-1][2]-od[0][2])):+.2f} deg  vs ground truth forward {s[-1]:.3f} lateral {lat[-1]:+.3f}')
    return s[-1], lat[-1]


def main():
    which = sys.argv[1:] or ['scan', 'camera', 'drive', 'turn', 'tf']
    rclpy.init(); rig = Rig()
    try:
        run(rig, which)
    finally:
        Pub.stop()          # never leave the `ros2 topic pub` subprocess running


def run(rig, which):
    out(f'sim state {sim.getSimulationState()} time {sim.getSimulationTime():.2f}')
    for _ in range(30):
        rig.spin(0.1)
    if rig.odom is None:
        out('NO ODOM RECEIVED'); return
    settle(rig)
    if 'scan' in which:
        check_scan(rig, 'start pose, facing +x')
    if 'camera' in which:
        check_camera(rig, 'start pose')
    if 'drive' in which:
        rig.fresh('odom')
        o0 = rig.odom.pose.pose
        out(f'odom at start: x={o0.position.x:.3f} y={o0.position.y:.3f} yaw={math.degrees(2*math.atan2(o0.orientation.z,o0.orientation.w)):.2f} deg')
        g0 = gt_pose()
        rows = drive(rig, 0.2, 0.0, 8.0, 'straight 0.2 m/s')
        settle(rig, 3)
        report_straight(rows)
        g1 = gt_pose(); rig.fresh('odom')
        s = ((g1[0] - g0[0]) * -math.sin(g0[2]) + (g1[1] - g0[1]) * math.cos(g0[2]))
        l = (-(g1[0] - g0[0]) * math.cos(g0[2]) - (g1[1] - g0[1]) * math.sin(g0[2]))   # +x ros left = model -x
        od = rig.odom.pose.pose
        out(f'after stop+settle: ground truth moved {s:.3f} m forward; odom reports x={od.position.x:.3f} y={od.position.y:.3f} (|odom|={math.hypot(od.position.x, od.position.y):.3f}) yaw={math.degrees(2*math.atan2(od.orientation.z, od.orientation.w)):+.2f} deg')
        out(f'  odom length error {math.hypot(od.position.x, od.position.y) - abs(s):+.3f} m; gt ROS-left offset {l:+.3f}')
    if 'turn' in which:
        rig.fresh('odom'); g0 = gt_pose(); o0 = rig.odom.pose.pose
        y0 = 2 * math.atan2(o0.orientation.z, o0.orientation.w)
        out('-- turn: positive angular.z 0.5 rad/s until ground-truth yaw changed by 90 deg')
        Pub.set(0.0, 0.5); rig.spin(2.0)
        t0s = sim.getSimulationTime(); done = False; samples = []
        while sim.getSimulationTime() - t0s < 12:
            rig.spin(0.01)
            g = gt_pose(); dyaw = wrap(g[2] - g0[2]); samples.append((sim.getSimulationTime() - t0s, dyaw))
            if abs(dyaw) >= math.radians(90):
                break
        settle(rig, 3); g1 = gt_pose(); rig.fresh('odom'); o1 = rig.odom.pose.pose
        dy_gt = math.degrees(wrap(g1[2] - g0[2])); dy_od = math.degrees(wrap(2 * math.atan2(o1.orientation.z, o1.orientation.w) - y0))
        rate = (samples[-1][1] - samples[0][1]) / max(1e-6, samples[-1][0] - samples[0][0])
        out(f'ground truth yaw change {dy_gt:+.2f} deg (commanded rate 0.5 rad/s = 28.6 deg/s, measured {math.degrees(rate):+.1f} deg/s over {samples[-1][0]:.1f} s); odom yaw change {dy_od:+.2f} deg (CCW positive expected)')
        out(f'turn centre drift: gt moved {math.hypot(g1[0]-g0[0], g1[1]-g0[1]):.3f} m; odom position {o1.position.x:+.3f},{o1.position.y:+.3f}')
        if 'scan' in which:
            check_scan(rig, 'after the 90 deg turn')
        if 'camera' in which:
            check_camera(rig, 'after the 90 deg turn')
    if 'tf' in which:
        out('== TF ==')
        rig.spin(1.0)
        for p, c in (('my_robot_odom', 'base_footprint'), ('base_footprint', 'base_link'), ('my_robot_odom', 'base_link'),
                     ('base_link', 'laser_link'), ('base_link', 'camera_link')):
            try:
                t = rig.tfbuf.lookup_transform(p, c, rclpy.time.Time())
                tr, q = t.transform.translation, t.transform.rotation
                out(f'{p} -> {c}: xyz=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f}) yaw={math.degrees(2*math.atan2(q.z,q.w)):+.1f} deg q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})')
            except Exception as e:
                out(f'{p} -> {c}: MISSING ({e})')
        rp = sim.getObjectPose(robot, -1); Rr = rot(rp[3:]); orr = np.array(rp[:3])
        Rb = Rr @ rot([0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)])    # base_link axes (model rotated +90 deg about z) in world
        def in_base(pos, Rw):
            return Rb.T @ (np.array(pos) - orr), Rb.T @ Rw
        pl, Rl = in_base(sim.getObjectPose(ref, -1)[:3], rot(sim.getObjectPose(ref, -1)[3:]))
        out(f'GT laser_ref in base_link: xyz={np.round(pl, 3)} x-axis={np.round(Rl[:, 0], 3)}')
        cp = sim.getObjectPose(dep_h, -1); pc, Rc = in_base(cp[:3], rot(cp[3:]))
        out(f'GT kinect depth sensor in base_link: xyz={np.round(pc, 3)} optical(+z) axis={np.round(Rc[:, 2], 3)} +x axis={np.round(Rc[:, 0], 3)} +y axis={np.round(Rc[:, 1], 3)}')
        try:
            t = rig.tfbuf.lookup_transform('base_link', 'camera_link', rclpy.time.Time()).transform.rotation
            Rt = rot([t.x, t.y, t.z, t.w])
            out(f'TF camera_link axes in base_link: x={np.round(Rt[:, 0], 3)} y={np.round(Rt[:, 1], 3)} z={np.round(Rt[:, 2], 3)}')
            out(f'  angle between TF z axis and GT optical axis: {math.degrees(math.acos(max(-1, min(1, float(Rt[:, 2] @ Rc[:, 2]))))):.2f} deg')
        except Exception as e:
            out(f'camera TF missing {e}')
    out('done')


main()
