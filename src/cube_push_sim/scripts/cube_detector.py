#!/usr/bin/env python3
"""RGB color candidates with ground-plane/known-height filtering; no simulator poses."""
import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_point


from cube_geometry import detect_cubes, detect_zones, bearing_candidates


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')
        self.declare_parameter('camera_height', .5)
        self.declare_parameter('min_area', 25)
        self.declare_parameter('process_hz', 5.0)
        self.bridge = CvBridge()
        self.k = None
        self.last_processed = None
        self.pending = []
        self.create_timer(.05, self.process_pending, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.results = self.create_publisher(String, '/perception/cubes', 10)
        self.debug = self.create_publisher(Image, '/perception/debug_image', 2)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image', self.image, QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info('Cube detector ready: RGB -> /perception/cubes, /perception/debug_image')

    def info(self, msg):
        self.k = msg.k

    def image(self, msg):
        self.pending.append(msg)
        self.pending = self.pending[-2:]

    def process_pending(self):
        # Pick the newest frame whose exposure-time odometry has arrived.
        # A wall timer keeps this queue draining when the VM runs slowly.
        for msg in reversed(self.pending):
            if self.tf.can_transform('odom','camera_link',Time.from_msg(msg.header.stamp)):
                stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
                self.pending=[m for m in self.pending if m.header.stamp.sec*1000000000+m.header.stamp.nanosec>stamp]
                self.process_image(msg)
                return

    def process_image(self, msg):
        if self.k is None:
            return
        now = self.get_clock().now().nanoseconds
        exposure = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        interval = 1e9/max(.1,float(self.get_parameter('process_hz').value))
        if self.last_processed is not None and 0 <= exposure-self.last_processed < interval*.9:
            return
        self.last_processed = exposure
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return
        detections = detect_cubes(bgr,self.k,float(self.get_parameter('camera_height').value),int(self.get_parameter('min_area').value))
        transforms = None
        zones=[]
        try:
            stamp=Time.from_msg(msg.header.stamp)
            # Local odometry is sampled at exposure time. AMCL's latest map
            # correction is applied separately (its TF is future-dated).
            odom_camera=self.tf.lookup_transform('odom','camera_link',stamp)
            odom_base=self.tf.lookup_transform('odom','base_footprint',stamp)
            map_odom=self.tf.lookup_transform('map','odom',Time())
            correction_stamp=map_odom.header.stamp.sec+map_odom.header.stamp.nanosec*1e-9
            if now*1e-9-correction_stamp>2.:
                raise TransformException('Localization correction is stale')
            def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            base_yaw=yaw(map_odom.transform.rotation)+yaw(odom_base.transform.rotation)
            zones=detect_zones(bgr,self.k,base_yaw,float(self.get_parameter('camera_height').value))
            transforms=(odom_camera,map_odom)
        except TransformException:
            pass
        for d in detections+zones:
            d['position_map'] = None
            if transforms is not None:
                p = PointStamped();p.header.stamp = msg.header.stamp;p.header.frame_id='camera_link'
                p.point.x=d['position_camera']['x'];p.point.y=d['position_camera']['y'];p.point.z=d['position_camera']['z']
                mapped=do_transform_point(do_transform_point(p,transforms[0]),transforms[1]).point
                d['position_map']={'x':round(mapped.x,3),'y':round(mapped.y,3),'z':round(mapped.z,3)}
        payload={'stamp':{'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec},
                 'frame_id':'camera_link','method':'rgb_cube_top_plane','detections':detections,'zones':zones,'candidates':bearing_candidates(bgr,self.k)}
        self.results.publish(String(data=json.dumps(payload)))
        debug=bgr.copy()
        for d in detections:
            x,y,w,h=d['bbox'];cv2.rectangle(debug,(x,y),(x+w,y+h),(0,255,255),2)
            cv2.putText(debug,f"{d['color']} ~{d['range_m']:.2f}m",(x,max(18,y-7)),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
        for z in zones:
            x,y,w,h=z['bbox'];cv2.rectangle(debug,(x,y),(x+w,y+h),(255,255,255),1)
            cv2.putText(debug,z['color']+' target',(x,max(12,y-3)),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
        image=self.bridge.cv2_to_imgmsg(debug,'bgr8');image.header=msg.header;self.debug.publish(image)


def main():
    rclpy.init();node=CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
