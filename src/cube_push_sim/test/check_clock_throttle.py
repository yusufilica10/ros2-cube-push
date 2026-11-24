#!/usr/bin/env python3
"""Manual ROS integration check: rate limiting, exact stamps, and reset forwarding."""
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def main():
    process = subprocess.Popen([
        'ros2', 'run', 'cube_push_sim', 'clock_throttle', '--ros-args',
        '-r', '/simulation/clock_raw:=/test_clock/input',
        '-r', '/clock:=/test_clock/output',
    ])
    rclpy.init()
    node = rclpy.create_node('check_clock_throttle')
    pub = node.create_publisher(Clock, '/test_clock/input', qos_profile_sensor_data)
    received = []
    sub = node.create_subscription(
        Clock, '/test_clock/output',
        lambda msg: received.append(msg.clock.sec * 1000 + msg.clock.nanosec // 1000000),
        qos_profile_sensor_data,
    )

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)

    try:
        end = time.monotonic() + 15
        while (pub.get_subscription_count() == 0 or sub.get_publisher_count() == 0) and time.monotonic() < end:
            spin(.05)
        assert pub.get_subscription_count() > 0, 'Clock relay not discovered'
        # DDS matching can precede delivery readiness; establish the first tick.
        end = time.monotonic() + 3
        while not received and time.monotonic() < end:
            pub.publish(Clock())
            spin(.05)
        assert received == [0], received
        for milliseconds in list(range(2, 102, 2)) + [0, 2, 20]:
            msg = Clock()
            msg.clock.nanosec = milliseconds * 1000000
            pub.publish(msg)
            spin(.025)
        spin(.3)
        expected = [0, 20, 40, 60, 80, 100, 0, 20]
        assert received == expected, (received, expected)
        print('Clock relay passed: 50 Hz simulation-time limit, exact stamps, backward reset.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


if __name__ == '__main__':
    main()
