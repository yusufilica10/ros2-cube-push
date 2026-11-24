#!/usr/bin/env python3
"""Manual integration test: fresh sim.launch.py ONLY, no mission/SLAM/Nav2.

Example: source env.sh; python3 src/cube_push_sim/test/sim_drive_calibration.py
The initial area must be unchanged. Simulator model poses are an offline oracle.
"""
import json
import math
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def physics_pose():
    result=subprocess.run(['gz','topic','-t','/world/competition/dynamic_pose/info','-e','-n','1','--json-output'],capture_output=True,text=True,check=True,timeout=10)
    return next(p for p in json.loads(result.stdout)['pose'] if p['name']=='cube_pusher')

def yaw(q):
    if isinstance(q,dict):return math.atan2(2*(q.get('w',1)*q.get('z',0)+q.get('x',0)*q.get('y',0)),1-2*(q.get('y',0)**2+q.get('z',0)**2))
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def main():
    rclpy.init();node=Node('drive_calibration',parameter_overrides=[Parameter('use_sim_time',value=True)])
    samples=[];node.create_subscription(Odometry,'/odom',lambda m:samples.append(m),10)
    pub=node.create_publisher(Twist,'/cmd_vel',10)
    def spin_for(seconds,linear=0.,angular=0.):
        start=node.get_clock().now().nanoseconds;deadline=time.monotonic()+max(20,seconds*5)
        while (node.get_clock().now().nanoseconds-start)*1e-9<seconds:
            if time.monotonic()>deadline:raise RuntimeError('Simulation clock stalled')
            msg=Twist();msg.linear.x=float(linear);msg.angular.z=float(angular);pub.publish(msg);rclpy.spin_once(node,timeout_sec=.02)
    try:
        deadline=time.monotonic()+30
        while not samples and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if not samples:raise RuntimeError('No odometry')
        if any(name in ['cube_mission','controller_server','slam_toolbox'] for name in node.get_node_names()):raise RuntimeError('Close mission, Nav2 and SLAM before running calibration')
        p=physics_pose()['position']
        if math.hypot(p['x']+2.4,p['y']+1.4)>.1:raise RuntimeError('Start with a fresh simulation in the defined initial pose')
        initial=yaw(samples[-1].pose.pose.orientation);before=yaw(physics_pose()['orientation'])
        spin_for(3.,angular=.4);spin_for(1.)
        measured=yaw(samples[-1].pose.pose.orientation)-initial;actual=yaw(physics_pose()['orientation'])-before
        measured=math.atan2(math.sin(measured),math.cos(measured));actual=math.atan2(math.sin(actual),math.cos(actual))
        report={'angular':{'odometry_rad':measured,'physics_rad':actual,'absolute_error_rad':abs(measured-actual)}}
        before=physics_pose()['position'];p=samples[-1].pose.pose.position;ox,oy=p.x,p.y
        spin_for(4.,linear=.1);spin_for(1.)
        p=samples[-1].pose.pose.position;measured=math.hypot(p.x-ox,p.y-oy);p=physics_pose()['position'];actual=math.hypot(p['x']-before['x'],p['y']-before['y'])
        report['linear']={'odometry_m':measured,'physics_m':actual,'absolute_error_m':abs(measured-actual)}
        report['passed']=report['angular']['absolute_error_rad']<.02 and report['linear']['absolute_error_m']<.02
        out=Path(__file__).resolve().parents[3]/'docs/drive_calibration_latest.json';out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    finally:
        pub.publish(Twist());node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
