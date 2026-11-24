#!/usr/bin/env python3
"""Terminal WASD control; stop automatically when keyboard input expires."""
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist


def main():
    if not sys.stdin.isatty():
        raise SystemExit('Bu komutu etkileşimli bir terminalde çalıştırın.')
    rclpy.init()
    node = rclpy.create_node('teleop_wasd')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    settings = termios.tcgetattr(sys.stdin)
    deadline = 0.0
    command = Twist()
    print('W: ileri | S: geri | A/D: yerinde dön | SPACE: dur | Q: çıkış\n'
          'Hareket için tuşu basılı tut. Tuş tekrarı kesilince robot durur.\n', flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1).lower()
                command = Twist()
                if key in ('q', '\x03'):
                    break
                if key == 'w':
                    command.linear.x = 0.8
                elif key == 's':
                    command.linear.x = -0.8
                elif key == 'a':
                    command.angular.z = 1.1
                elif key == 'd':
                    command.angular.z = -1.1
                deadline = time.monotonic() + 0.3
            if time.monotonic() > deadline:
                command = Twist()
            pub.publish(command)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
