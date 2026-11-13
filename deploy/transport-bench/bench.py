#!/usr/bin/env python3
"""T902 transport bench (009a-R03, Q-06). Two roles:

  pub  (robot): publishes a compressed RGB image at --rate Hz on the contract topic, stamped with now, and
                subscribes to the cloud detections. RTT = now - header.stamp of the echoed Detection2DArray
                (the same timestamp-echo age that 009a-R05 uses, so there is a single clock).
  echo (cloud): stands in for the detector: copies the image header into a Detection2DArray and publishes it.

QoS follows contracts/topics.yaml: image = sensor (best_effort, depth 5), detections = default (reliable, depth 10).
The CSV has one row per sent image: seq, send_time_ns, rtt_ms (empty when no reply by the end), payload_bytes.
"""
import argparse, csv, io, statistics, time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

IMG = '/myRobot/perception/rgb_throttled/compressed'
DET = '/myRobot/perception/detections/cloud'
QOS_SENSOR = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=5)
QOS_DEFAULT = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=10)


def make_jpeg(w=640, h=480, quality=80):
    """Smooth synthetic 640x480 scene (gradient + shapes) -> a JPEG of realistic size."""
    from PIL import Image, ImageDraw
    img = Image.linear_gradient('L').resize((w, h)).convert('RGB')
    d = ImageDraw.Draw(img)
    for i in range(40):
        x, y = (i * 97) % w, (i * 53) % h
        d.ellipse([x, y, x + 60, y + 40], fill=((i * 31) % 255, (i * 67) % 255, (i * 13) % 255))
    noise = Image.effect_noise((w, h), 40).convert('RGB')   # sensor-like texture, so the size is realistic
    img = Image.blend(img, noise, 0.25)
    buf = io.BytesIO()
    img.save(buf, 'JPEG', quality=quality)
    return buf.getvalue()


class Echo(Node):
    def __init__(self):
        super().__init__('t902_echo')
        self.pub = self.create_publisher(Detection2DArray, DET, QOS_DEFAULT)
        self.create_subscription(CompressedImage, IMG, self.on_img, QOS_SENSOR)
        self.n = 0

    def on_img(self, msg):
        out = Detection2DArray()
        out.header = msg.header          # timestamp echo (009a-R02)
        for _ in range(3):
            d = Detection2D()
            d.header = msg.header
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = 'cup'
            h.hypothesis.score = 0.9
            d.results.append(h)
            out.detections.append(d)
        self.pub.publish(out)
        self.n += 1
        if self.n % 20 == 0:
            self.get_logger().info(f'echoed {self.n}')


class Pub(Node):
    def __init__(self, rate, duration, csv_path, warmup):
        super().__init__('t902_pub')
        self.jpeg = make_jpeg()
        self.pub = self.create_publisher(CompressedImage, IMG, QOS_SENSOR)
        self.create_subscription(Detection2DArray, DET, self.on_det, QOS_DEFAULT)
        self.rows = {}                    # seq -> [send_ns, rtt_ms|None]
        self.seq = 0
        self.rate = rate
        self.warmup, self.duration, self.csv_path = warmup, duration, csv_path
        self.t0 = time.monotonic()
        self.first_reply_seq = None
        self.order_violations = 0
        self.last_recv_seq = -1
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'payload {len(self.jpeg)} bytes')

    def tick(self):
        m = CompressedImage()
        now = time.time_ns()
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(now, 10**9)
        m.header.frame_id = str(self.seq)
        m.format = 'jpeg'
        m.data = self.jpeg
        self.rows[self.seq] = [now, None]
        self.seq += 1
        self.pub.publish(m)

    def on_det(self, msg):
        now = time.time_ns()
        sent = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        seq = int(msg.header.frame_id)
        if seq in self.rows and self.rows[seq][1] is None:
            self.rows[seq][1] = (now - sent) / 1e6
            if self.first_reply_seq is None:
                self.first_reply_seq = seq
                self.get_logger().info(f'first reply seq={seq} after {time.monotonic() - self.t0:.1f}s')
            if seq < self.last_recv_seq:
                self.order_violations += 1
            self.last_recv_seq = max(self.last_recv_seq, seq)

    def done(self):
        # run until `duration` seconds after the first reply (or give up after warmup if none ever arrives)
        if self.first_reply_seq is None:
            return time.monotonic() - self.t0 > self.warmup
        return self.seq - self.first_reply_seq >= self.duration * self.rate + 8

    def report(self, label):
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['seq', 'send_time_ns', 'rtt_ms', 'payload_bytes'])
            for s, (t, r) in sorted(self.rows.items()):
                w.writerow([s, t, '' if r is None else f'{r:.3f}', len(self.jpeg)])
        first = self.first_reply_seq
        steady = [(s, self.rows[s][1]) for s in sorted(self.rows) if first is not None and s >= first]
        # drop the last 4 s of sends: their replies may legitimately still be in flight
        tail = int(4 * self.rate)
        steady = steady[:-tail] if len(steady) > tail else steady
        got = sorted(r for _, r in steady if r is not None)
        n, lost = len(steady), len(steady) - len(got)
        nan = float('nan')

        def pct(p):
            return got[min(len(got) - 1, int(p / 100 * len(got)))] if got else nan

        span = n / self.rate
        print(f'SUMMARY {label} payload_B={len(self.jpeg)} sent={n} received={len(got)} lost={lost} '
              f'loss_pct={100 * lost / max(n, 1):.2f} reordered={self.order_violations} '
              f'recv_rate_hz={len(got) / max(span, 1e-9):.3f} rtt_ms_min={got[0] if got else nan:.2f} '
              f'p50={pct(50):.2f} p95={pct(95):.2f} p99={pct(99):.2f} max={got[-1] if got else nan:.2f} '
              f'mean={statistics.fmean(got) if got else nan:.2f}', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('role', choices=['pub', 'echo'])
    ap.add_argument('--rate', type=float, default=2.0)
    ap.add_argument('--duration', type=float, default=300.0)
    ap.add_argument('--warmup', type=float, default=60.0, help='max seconds to wait for the first reply')
    ap.add_argument('--csv', default='/out/rtt.csv')
    ap.add_argument('--label', default='run')
    a = ap.parse_args()
    rclpy.init()
    if a.role == 'echo':
        rclpy.spin(Echo())
        return
    node = Pub(a.rate, a.duration, a.csv, a.warmup)
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    node.report(a.label)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
