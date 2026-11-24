#!/usr/bin/env python3
"""Exclusive Nav2 / mission velocity ownership with a wall-time watchdog."""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class VelocityMux(Node):
    def __init__(self):
        super().__init__('velocity_mux');self.mode='nav';self.last={'nav':(0,Twist()),'push':(0,Twist())}
        self.pub=self.create_publisher(Twist,'/control/cmd_vel',10)
        self.create_subscription(Twist,'/cmd_vel_smoothed',lambda m:self.receive('nav',m),10)
        self.create_subscription(Twist,'/mission/cmd_vel',lambda m:self.receive('push',m),10)
        self.create_subscription(String,'/mission/control_mode',self.control,10)
        self.create_timer(.05,self.tick)
    def receive(self,key,msg):self.last[key]=(time.monotonic(),msg)
    def control(self,msg):
        if msg.data in ('nav','push','stop'):
            if msg.data!=self.mode:self.last['push']=(0,Twist())
            self.mode=msg.data
    def tick(self):
        msg=Twist()
        if self.mode in self.last:
            stamp,old=self.last[self.mode]
            if time.monotonic()-stamp<1.:
                msg.linear.x=max(-.8,min(.8,old.linear.x));msg.angular.z=max(-1.5,min(1.5,old.angular.z))
        self.pub.publish(msg)

def main():
    rclpy.init();n=VelocityMux()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.pub.publish(Twist());n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
