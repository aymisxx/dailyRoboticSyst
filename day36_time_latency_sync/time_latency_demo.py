#!/usr/bin/env python3
import csv
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import ClockType

from std_msgs.msg import Header
from builtin_interfaces.msg import Time as RosTime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path("day36_time_latency_sync")
RESULTS = ROOT / "results"


def now(node: Node) -> Time:
    return node.get_clock().now()


def to_sec(obj) -> float:
    return obj.nanoseconds * 1e-9


def ros_time_from_time(t: Time) -> RosTime:
    return RosTime(
        sec=int(t.nanoseconds // 1_000_000_000),
        nanosec=int(t.nanoseconds % 1_000_000_000),
    )


def time_from_ros_time(rt: RosTime, clock_type: ClockType) -> Time:
    return Time(
        nanoseconds=int(rt.sec * 1_000_000_000 + rt.nanosec),
        clock_type=clock_type,
    )


class FakeCamPublisher(Node):
    def __init__(self):
        super().__init__("fake_cam_pub")
        self.pub = self.create_publisher(Header, "/cam", 10)
        self.rate_hz = 30.0
        self.delay_ms = 80.0  # set 0.0 for run B
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

    def tick(self):
        t_meas = now(self)
        if self.delay_ms > 0:
            time.sleep(self.delay_ms / 1000.0)

        msg = Header()
        msg.stamp = ros_time_from_time(t_meas)
        msg.frame_id = "cam"
        self.pub.publish(msg)


class FakeImuPublisher(Node):
    def __init__(self):
        super().__init__("fake_imu_pub")
        self.pub = self.create_publisher(Header, "/imu", 50)
        self.rate_hz = 200.0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

    def tick(self):
        t_meas = now(self)
        msg = Header()
        msg.stamp = ros_time_from_time(t_meas)
        msg.frame_id = "imu"
        self.pub.publish(msg)


class SkewLogger(Node):
    def __init__(self):
        super().__init__("skew_logger")

        self.cam_buf = deque(maxlen=4000)
        self.imu_buf = deque(maxlen=40000)

        self.create_subscription(Header, "/cam", self.on_cam, 10)
        self.create_subscription(Header, "/imu", self.on_imu, 50)

        self.t0 = now(self)
        self.rows = []
        self.done = False  # <-- signal to main/executor

        self.timer = self.create_timer(0.1, self.pair_and_log)
        self.get_logger().info("Logging for 20 seconds...")

    def on_cam(self, msg: Header):
        self.cam_buf.append(msg)

    def on_imu(self, msg: Header):
        self.imu_buf.append(msg)

    def pair_and_log(self):
        if self.done:
            return

        t_now = now(self)
        t_rel = to_sec(t_now - self.t0)

        if t_rel > 20.0:
            self.finish()
            self.done = True
            self.timer.cancel()  # stop callbacks cleanly
            return

        if not self.cam_buf or not self.imu_buf:
            return

        ct = self.get_clock().clock_type
        cam = self.cam_buf[-1]
        cam_t = time_from_ros_time(cam.stamp, ct)

        imu_best = min(
            self.imu_buf,
            key=lambda m: abs(time_from_ros_time(m.stamp, ct).nanoseconds - cam_t.nanoseconds),
        )
        imu_t = time_from_ros_time(imu_best.stamp, ct)

        cam_age = to_sec(t_now - cam_t)
        imu_age = to_sec(t_now - imu_t)
        stamp_mismatch = to_sec(cam_t - imu_t)

        self.rows.append([t_rel, cam_age, imu_age, stamp_mismatch])

    def finish(self):
        RESULTS.mkdir(parents=True, exist_ok=True)

        out_csv = RESULTS / "skew.csv"
        out_age = RESULTS / "skew_age.png"
        out_mismatch = RESULTS / "skew_stamp_mismatch.png"

        with out_csv.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_rel_s", "cam_age_s", "imu_age_s", "cam_minus_imu_stamp_s"])
            w.writerows(self.rows)

        t = [r[0] for r in self.rows]
        cam_age = [r[1] for r in self.rows]
        imu_age = [r[2] for r in self.rows]
        mismatch = [r[3] for r in self.rows]

        plt.figure()
        plt.plot(t, cam_age, label="cam_age")
        plt.plot(t, imu_age, label="imu_age")
        plt.xlabel("time (s)")
        plt.ylabel("age (s)")
        plt.legend()
        plt.title("Message age shows latency (+ jitter)")
        plt.tight_layout()
        plt.savefig(out_age, dpi=160)

        plt.figure()
        plt.plot(t, mismatch, label="cam_stamp - imu_stamp")
        plt.xlabel("time (s)")
        plt.ylabel("seconds")
        plt.legend()
        plt.title("Timestamp mismatch shows sync quality")
        plt.tight_layout()
        plt.savefig(out_mismatch, dpi=160)

        self.get_logger().info(f"Saved: {out_csv}")
        self.get_logger().info(f"Saved: {out_age}")
        self.get_logger().info(f"Saved: {out_mismatch}")


def main():
    rclpy.init()

    cam = FakeCamPublisher()
    imu = FakeImuPublisher()
    logger = SkewLogger()

    ex = MultiThreadedExecutor()
    ex.add_node(cam)
    ex.add_node(imu)
    ex.add_node(logger)

    try:
        # spin until logger says it's done
        while rclpy.ok() and not logger.done:
            ex.spin_once(timeout_sec=0.2)
    finally:
        # clean node teardown
        for n in (cam, imu, logger):
            ex.remove_node(n)
            n.destroy_node()
        # shutdown exactly once, safely
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
