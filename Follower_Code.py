#!/usr/bin/env python3
"""
Project template.

The PupperInterface class wraps all ROS 2 work: it subscribes to /detections
in a background thread and exposes a small synchronous API that the main loop
can call without dealing with callbacks or executors directly.

API:
    node.get_detections()                -> list[Detection]  (most recent frame)
    node.seconds_since_last_detection()  -> float
    node.set_velocity(linear_x, linear_y, angular_z)   publish Twist to cmd_vel

Fill in the main loop below.
"""

from math import dist
import os
import signal
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time

from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray


IMAGE_WIDTH = 700  # pixels; matches the equirectangular image published by hailo_detection.py
DEFAULT_LABELS_PATH = os.path.join(os.path.dirname(__file__), 'coco.txt')


@dataclass
class Detection:
    """One detected object from the current frame."""
    class_id: int
    class_name: str
    confidence: float
    center_x: float  # pixels
    center_y: float
    size_x: float
    size_y: float

    @property
    def normalized_x(self) -> float:
        """Center x mapped to [-0.5, 0.5] (0 = image center, +0.5 = right edge)."""
        return (self.center_x / IMAGE_WIDTH) - 0.5


class PupperInterface(Node):
    """
    ROS 2 node that owns all subscriptions/publishers for the lab.

    Detections arrive on a background spin thread and are cached under a lock
    so that the main thread can grab a consistent snapshot at any time.
    """

    def __init__(self, labels_path: str = DEFAULT_LABELS_PATH):
        super().__init__('pupper_interface_node')

        self._class_names = self._load_labels(labels_path)

        self._lock = threading.Lock()
        self._latest_detections: List[Detection] = []
        self._last_detection_time: Optional[Time] = None

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self._detection_callback,
            10,
        )

    def _load_labels(self, path: str) -> List[str]:
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return f.read().splitlines()
        except FileNotFoundError:
            self.get_logger().warning(
                f'Labels file not found at {path}; class_name will fall back to the numeric id.'
            )
            return []

    def _lookup_class_name(self, class_id: int) -> str:
        if 0 <= class_id < len(self._class_names):
            return self._class_names[class_id]
        return str(class_id)

    # --- internal callback (runs on the spin thread) --------------------
    def _detection_callback(self, msg: Detection2DArray):
        dets: List[Detection] = []
        for d in msg.detections:
            if not d.results:
                continue
            hyp = d.results[0]
            try:
                class_id = int(hyp.hypothesis.class_id)
            except ValueError:
                class_id = -1
            dets.append(Detection(
                class_id=class_id,
                class_name=self._lookup_class_name(class_id),
                confidence=float(hyp.hypothesis.score),
                center_x=float(d.bbox.center.position.x),
                center_y=float(d.bbox.center.position.y),
                size_x=float(d.bbox.size_x),
                size_y=float(d.bbox.size_y),
            ))
        with self._lock:
            self._latest_detections = dets
            self._last_detection_time = self.get_clock().now()

    # --- public API ------------------------------------------------------
    def get_detections(self) -> List[Detection]:
        """Snapshot of detections from the most recent frame (possibly empty)."""
        with self._lock:
            return list(self._latest_detections)

    def seconds_since_last_detection(self) -> float:
        """Seconds since the last detection frame; +inf if none received yet."""
        with self._lock:
            t = self._last_detection_time
        if t is None:
            return float('inf')
        return (self.get_clock().now() - t).nanoseconds / 1e9

    def set_velocity(self, linear_x: float = 0.0, linear_y: float = 0.0,
                     angular_z: float = 0.0):
        """Publish a body-frame velocity command to the RL controller.

        linear_x:  forward/back (m/s)
        linear_y:  lateral / strafe (m/s)
        angular_z: yaw rate (rad/s)
        """
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.linear.y = float(linear_y)
        cmd.angular.z = float(angular_z)
        self._cmd_pub.publish(cmd)


def main():
    # Disable rclpy's default SIGINT handler so Ctrl+C doesn't tear down the
    # context before we get a chance to publish a zero-velocity stop command.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PupperInterface()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    stop_requested = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_requested.set())

    try:


        start_time = time.time()
        log = []
        dt = 0.01

        KP_LINEAR = 0.00002
        KI_LINEAR = 0.000001
        KD_LINEAR = 0.0000032

        KP_ANGULAR = 5
        KI_ANGULAR = 5
        KD_ANGULAR = 0.7

        STOP_AREA = 43000

        lin_prev_error = 0.0
        ang_prev_error = 0.0
        ang_err_sum    = 0.0
        error_sum      = 0.0

        while rclpy.ok() and not stop_requested.is_set():
            detections = node.get_detections()

            print("\n--- NEW FRAME ---")
            print(f"Total detections: {len(detections)}")

            human_found = False
            for det in detections:
                print(f"  - {det.class_name} (id={det.class_id}, conf={det.confidence:.2f})")
                if det.class_id == 0:
                    area = det.size_x * det.size_y
                    x = det.center_x
                    if area <= 0:
                        break
                    human_found = True
                    Dist = 2.03561799e+06 * (area ** -1.82128344)
                    error = (STOP_AREA - area)
                    print(">>> HUMAN DETECTED <<<")
                    print(f"    x = {det.center_x:.1f}")
                    print(f"    area = {area:.1f}")
                    print(f"    x_norm = {det.normalized_x:.3f}")
                    print(f"    error={error:.6f}")
                    print(f"    distance = {Dist:.2f}")
                    break

            if human_found:
                vel = 0.0
                w   = 0.0

                ang_error = det.normalized_x
                ang_err_sum += det.normalized_x * dt
                err_der = (ang_error - ang_prev_error) / dt
                w = KP_ANGULAR*det.normalized_x + KD_ANGULAR*err_der + KI_ANGULAR*ang_err_sum
                ang_prev_error = ang_error

                error = STOP_AREA - area
                error_sum += error*dt
                derivative = (error - lin_prev_error) / dt
                vel = KP_LINEAR * error + KD_LINEAR*derivative + KI_LINEAR*error_sum
                lin_prev_error = error

                print(f"FOLLOWING | vel={vel:.3f} | w={w:.3f}")
                node.set_velocity(linear_x=vel, linear_y=0.0, angular_z=w)
            else:
                print("No Human = CONSTANT VELOCITY")
                node.set_velocity(linear_x=0.5, linear_y=0.0, angular_z=-0.0)

            current_time = time.time()

            vel_log = vel if 'vel' in locals() else 0.0
            w_log   = w if 'w' in locals() else 0.0

            log.append([
                current_time,
                det.normalized_x if human_found else 0.0,
                area if human_found else 0.0,
                vel_log,
                w_log
            ])

            time.sleep(0.01)


    except Exception as e:
        node.get_logger().error(f'Main loop error: {e}')
    finally:
        import numpy as np

        np.savetxt(
            "log.csv",
            log,
            delimiter=",",
            header="time,normalized_x,area,velocity,angular",
            comments=""
        )

        print("Saved log to log.csv")


        # Stop the robot. Publish zero velocity a few times with small sleeps
        # so the controller actually receives it before we tear the node down.
        for _ in range(5):
            node.set_velocity(0.0, 0.0, 0.0)
            time.sleep(0.1)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()