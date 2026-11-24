#!/usr/bin/env python3
"""RGB color candidates with ground-plane/known-height filtering; no simulator poses."""
import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_point


from cube_geometry import detect_cubes, detect_zones


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
        self.create_timer(.1, self.process_pending)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.results = self.create_publisher(String, '/perception/cubes', 10)
        self.debug = self.create_publisher(Image, '/perception/debug_image', 2)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image', self.image, qos_profile_sensor_data)
        self.get_logger().info('Cube detector ready: RGB -> /perception/cubes, /perception/debug_image')

    def info(self, msg):
        self.k = msg.k

    def image(self, msg):
        self.pending.append(msg)
        self.pending = self.pending[-3:]

    def process_pending(self):
        if len(self.pending) < 2:
            return
        self.process_image(self.pending.pop(0))

    def process_image(self, msg):
        if self.k is None:
            return
        now = self.get_clock().now().nanoseconds
        interval = 1e9/max(.1,float(self.get_parameter('process_hz').value))
        if self.last_processed is not None and 0 <= now-self.last_processed < interval:
            return
        self.last_processed = now
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return
        detections = detect_cubes(bgr,self.k,float(self.get_parameter('camera_height').value),int(self.get_parameter('min_area').value))
        transform = None
        try:
            transform = self.tf.lookup_transform('map','camera_link',Time.from_msg(msg.header.stamp),timeout=Duration(seconds=0.0))
        except TransformException:
            pass  # No map coordinate is better than inventing a stale transform.
        zones=[]
        try:
            pose=self.tf.lookup_transform('map','base_footprint',Time.from_msg(msg.header.stamp))
            q=pose.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            zones=detect_zones(bgr,self.k,yaw,float(self.get_parameter('camera_height').value))
        except TransformException:
            pass
        for d in detections+zones:
            d['position_map'] = None
            if transform is not None:
                p = PointStamped();p.header.stamp = msg.header.stamp;p.header.frame_id='camera_link'
                p.point.x=d['position_camera']['x'];p.point.y=d['position_camera']['y'];p.point.z=d['position_camera']['z']
                mapped=do_transform_point(p,transform).point
                d['position_map']={'x':round(mapped.x,3),'y':round(mapped.y,3),'z':round(mapped.z,3)}
        payload={'stamp':{'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec},
                 'frame_id':'camera_link','method':'rgb_cube_top_plane','detections':detections,'zones':zones}
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
