#!/usr/bin/env python3
"""Start Nav2 after simulation sensors/odometry have actually begun publishing."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.srv import ManageLifecycleNodes

class Gate(Node):
    def __init__(self):
        super().__init__('navigation_ready_gate');self.scans=0;self.odom=False
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.pose,qos_profile_sensor_data)
        self.client=self.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
    def scan(self,msg):self.scans+=1
    def pose(self,msg):self.odom=True

def main():
    rclpy.init();n=Gate();deadline=time.monotonic()+120
    try:
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.1)
            if n.scans>=3 and n.odom and n.client.service_is_ready():break
        else:raise RuntimeError('Simulation sensors did not become ready')
        n.get_logger().info('Sensors ready; starting Nav2 lifecycle nodes')
        req=ManageLifecycleNodes.Request();req.command=ManageLifecycleNodes.Request.STARTUP
        f=n.client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=120)
        if not f.done() or not f.result().success:raise RuntimeError('Nav2 startup failed')
        n.get_logger().info('Nav2 startup complete')
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
