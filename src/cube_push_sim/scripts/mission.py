#!/usr/bin/env python3
"""Camera-feedback cube delivery with Nav2 positioning and exclusive velocity control.

No simulator pose topics/services are used. Arena waypoints are public fixed layout.
"""
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException


def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def yaw_of(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def clamp(x,a,b):return max(a,min(b,x))
def delivered_inside(center,target,margin=.025):
    # Half diagonal bounds a rotated 18 cm cube for any orientation.
    limit=.30-.09*math.sqrt(2)-margin
    return abs(center[0]-target[0])<=limit and abs(center[1]-target[1])<=limit

def arena_clearance(x,y):
    distances=[3-abs(x),2-abs(y)]
    for wx,ymin,ymax in [(-.35,-2.,-.55),(-.35,.55,2.),(.9,-2.,-1.),(.9,.2,2.)]:
        distances.append(math.hypot(x-wx,y-clamp(y,ymin,ymax))-.05)
    return min(distances)

class MissionFailure(RuntimeError):pass

class Mission(Node):
    def __init__(self):
        super().__init__('cube_mission')
        for k,v in [('autostart',False),('duration',480.),('colors',['green','blue','red']),
                    ('map_offset_x',2.4),('map_offset_y',1.4),('push_speed',.25),
                    ('report_file','/tmp/cube_mission_latest.json')]:self.declare_parameter(k,v)
        self.offset=(float(self.get_parameter('map_offset_x').value),float(self.get_parameter('map_offset_y').value))
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose');self.nav_handle=None
        self.clearers=[self.create_client(ClearEntireCostmap,name) for name in ['/local_costmap/clear_entirely_local_costmap','/global_costmap/clear_entirely_global_costmap']]
        self.vel=self.create_publisher(Twist,'/mission/cmd_vel',10)
        self.mode_pub=self.create_publisher(String,'/mission/control_mode',10)
        self.status_pub=self.create_publisher(String,'/mission/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.cloud=self.create_publisher(PointCloud2,'/perception/cube_obstacles',2)
        self.create_subscription(String,'/perception/cubes',self.observe,10)
        self.create_subscription(Odometry,'/odom',self.odometry,10)
        self.create_service(Trigger,'/mission/start',self.start)
        self.create_service(Trigger,'/mission/stop',self.stop)
        self.create_timer(.5,self.publish_obstacles)
        self.nav_active=False
        self.create_subscription(Bool,'/lifecycle_manager_navigation/managed_nodes_activated',self.nav_state,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.requested=bool(self.get_parameter('autostart').value);self.aborted=False;self.running=False
        self.observations={};self.pairs={};self.odom=None;self.started=None;self.sim_started=None
        self.state='READY';self.active=None;self.mode='nav';self.delivered=[];self.failures=[];self.events=[];self.last_camera_wall=0
        self.create_timer(1.,self.publish_status)
        self.get_logger().info('Ready: /mission/start and /mission/stop (std_srvs/Trigger)')
    def start(self,req,res):
        res.success=not self.running and self.started is None
        if res.success:self.requested=True
        res.message='Mission requested' if res.success else 'Already started; reset simulation for a new run'
        return res
    def stop(self,req,res):self.aborted=True;self.set_mode('stop');res.success=True;res.message='Stop requested';return res
    def nav_state(self,msg):self.nav_active=msg.data
    def odometry(self,msg):self.odom=msg
    def observe(self,msg):
        data=json.loads(msg.data);self.last_camera_wall=time.monotonic()
        stamp=data['stamp']['sec']+data['stamp']['nanosec']*1e-9
        for d in data['detections']:
            if d.get('position_map') is not None:self.observations[d['color']]={'data':d,'wall':time.monotonic(),'stamp':stamp}
        for z in data.get('zones',[]):
            d=next((d for d in data['detections'] if d['color']==z['color']),None)
            if d and d.get('position_map') and z.get('position_map'):
                self.pairs[z['color']]={'cube':d,'zone':z,'stamp':stamp,'wall':time.monotonic()}
    def world(self,x,y):return (x+self.offset[0],y+self.offset[1])
    def simtime(self):return self.get_clock().now().nanoseconds*1e-9
    def set_mode(self,mode):self.mode=mode;self.mode_pub.publish(String(data=mode))
    def command(self,v=0.,w=0.):
        self.mode_pub.publish(String(data=self.mode));m=Twist();m.linear.x=float(clamp(v,-.8,.8));m.angular.z=float(clamp(w,-1.5,1.5));self.vel.publish(m)
    def event(self,state,**details):
        self.state=state;entry={'state':state,'color':self.active,'elapsed_s':round(time.monotonic()-self.started,2) if self.started else 0,**details};self.events.append(entry)
        self.get_logger().info(json.dumps(entry));self.publish_status();self.write_report()
    def write_report(self):
        report={'state':self.state,'delivered':self.delivered,'failures':self.failures,'elapsed_s':round(time.monotonic()-self.started,2) if self.started else 0,'events':self.events}
        p=Path(self.get_parameter('report_file').value);p.parent.mkdir(parents=True,exist_ok=True)
        tmp=p.with_suffix(p.suffix+'.tmp');tmp.write_text(json.dumps(report,indent=2));tmp.replace(p)
    def publish_status(self):
        elapsed=time.monotonic()-self.started if self.started else 0
        self.status_pub.publish(String(data=json.dumps({'state':self.state,'active_color':self.active,'elapsed_s':round(elapsed,1),'sim_elapsed_s':round(self.simtime()-self.sim_started,1) if self.sim_started else 0,'delivered':self.delivered,'failures':self.failures})))
    def publish_obstacles(self):
        # Only visually observed cubes; omit the active cube during controlled contact.
        points=[]
        for color,item in self.observations.items():
            if color==self.active:continue
            if time.monotonic()-item['wall']>120:continue
            p=item['data']['position_map']
            for dx in [-.09,-.045,0,.045,.09]:
                for dy in [-.09,-.045,0,.045,.09]:points.append((p['x']+dx,p['y']+dy,.09))
        h=Header();h.frame_id='map';h.stamp=self.get_clock().now().to_msg()
        self.cloud.publish(point_cloud2.create_cloud_xyz32(h,points))
    def check(self):
        if self.aborted:raise MissionFailure('Stopped by operator')
        if self.started and time.monotonic()-self.started>float(self.get_parameter('duration').value):raise MissionFailure('480-second wall-time mission deadline reached')
    def spin(self,seconds=.05):
        until=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<until:
            self.check();self.mode_pub.publish(String(data=self.mode));rclpy.spin_once(self,timeout_sec=min(.05,max(0,until-time.monotonic())))
    def pose(self):
        try:
            t=self.tf.lookup_transform('map','base_footprint',Time())
            return t.transform.translation.x,t.transform.translation.y,yaw_of(t.transform.rotation)
        except TransformException:return None
    def cube(self,color,max_age=.8):
        item=self.observations.get(color)
        if item and self.simtime()-item['stamp']<max_age and time.monotonic()-item['wall']<4:
            d=item['data'];p=d['position_map'];return (p['x'],p['y'],d)
        return None
    def pair(self,color):
        p=self.pairs.get(color)
        if p and self.simtime()-p['stamp']<.6 and time.monotonic()-p['wall']<3:return p
        return None
    def wait_future(self,future,timeout):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:self.spin()
        if not future.done():raise MissionFailure('Action response timed out')
        return future.result()
    def goto(self,x,y,yaw,timeout=100):
        self.set_mode('nav');self.command()
        for client in self.clearers:
            if client.service_is_ready():self.wait_future(client.call_async(ClearEntireCostmap.Request()),3)
        self.spin(.5)
        g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.header.stamp=self.get_clock().now().to_msg()
        g.pose.pose.position.x=float(x);g.pose.pose.position.y=float(y);g.pose.pose.orientation.z=math.sin(yaw/2);g.pose.pose.orientation.w=math.cos(yaw/2)
        self.event('NAVIGATE',goal=[round(x,3),round(y,3),round(yaw,3)])
        h=self.wait_future(self.nav.send_goal_async(g),10)
        if not h.accepted:raise MissionFailure('Nav2 rejected approach goal')
        self.nav_handle=h;f=h.get_result_async();end=time.monotonic()+timeout
        while not f.done() and time.monotonic()<end:self.spin()
        if not f.done():
            self.wait_future(h.cancel_goal_async(),5);self.nav_handle=None;raise MissionFailure('Nav2 approach timed out')
        self.nav_handle=None;result=f.result()
        if result.status!=4:raise MissionFailure(f'Nav2 failed ({result.status}): {result.result.error_msg}')
    def acquire(self,color,timeout=16):
        self.event('ACQUIRE');self.set_mode('push');start=time.monotonic();end=start+timeout;stable=[];last_stamp=None
        while time.monotonic()<end:
            c=self.cube(color)
            if c:
                stamp=self.observations[color]['stamp']
                if stamp!=last_stamp:stable.append(c);last_stamp=stamp
                self.command(0.,clamp(1.5*c[2]['bearing_rad'],-.3,.3) if abs(c[2]['bearing_rad'])>.06 else 0.)
                if len(stable)>=5 and abs(c[2]['bearing_rad'])<.08:
                    recent=stable[-5:]
                    if max(p[0] for p in recent)-min(p[0] for p in recent)<.05 and max(p[1] for p in recent)-min(p[1] for p in recent)<.05:
                        self.command();return c
            else:
                known=self.observations.get(color);pose=self.pose()
                if known and pose:
                    p=known['data']['position_map'];angle=wrap(math.atan2(p['y']-pose[1],p['x']-pose[0])-pose[2])
                    self.command(0.,clamp(1.5*angle,-.5,.5))
                else:self.command(0.,.5)
            self.spin()
        raise MissionFailure(f'No stable camera detection for {color}')
    def backoff(self,distance=.22):
        self.set_mode('push');self.command();self.spin(.2)
        if self.odom is None:raise MissionFailure('No odometry for backing away')
        p=self.odom.pose.pose.position;x,y=p.x,p.y;yaw=yaw_of(self.odom.pose.pose.orientation);end=time.monotonic()+20
        while time.monotonic()<end:
            p=self.odom.pose.pose.position
            if math.hypot(p.x-x,p.y-y)>=distance:break
            self.command(-.14,clamp(1.5*wrap(yaw-yaw_of(self.odom.pose.pose.orientation)),-.3,.3));self.spin()
        self.command();self.spin(.3)
    def push_to(self,color,dest,match_zone=False):
        aligned=False
        for attempt in range(3):
            c=self.cube(color) or self.acquire(color)
            dx,dy=dest[0]-c[0],dest[1]-c[1];length=math.hypot(dx,dy)
            if length<.07 and not match_zone:return
            if length<.001:raise MissionFailure('Approach direction is undefined')
            heading=math.atan2(dy,dx);unit=(dx/length,dy/length)
            candidate=(c[0]-.58*unit[0]-self.offset[0],c[1]-.58*unit[1]-self.offset[1])
            standoff=.58 if min(3-abs(candidate[0]),2-abs(candidate[1]))>.50 else .40
            approach=(c[0]-standoff*unit[0],c[1]-standoff*unit[1])
            staging=(c[0]-.78*unit[0],c[1]-.78*unit[1])
            other_clearance=[]
            for other,oy in [('blue',-1.2),('green',0.),('red',1.2)]:
                if other!=color and other not in self.delivered:
                    op=self.world(-1.6,oy);other_clearance.append(math.hypot(staging[0]-op[0],staging[1]-op[1]))
            current=self.pose()
            already_aligned=bool(current and abs(wrap(heading-current[2]))<.12 and abs((c[0]-current[0])*unit[1]-(c[1]-current[1])*unit[0])<.06)
            if not already_aligned and arena_clearance(staging[0]-self.offset[0],staging[1]-self.offset[1])>.43 and min(other_clearance,default=10.)>.65:
                self.goto(*staging,heading)
            current=self.pose()
            if not (already_aligned and current and math.hypot(current[0]-approach[0],current[1]-approach[1])<.06):
                self.goto(*approach,heading)
            c=self.acquire(color);pose=self.pose()
            dx,dy=dest[0]-c[0],dest[1]-c[1];length=math.hypot(dx,dy)
            if length<.07 and not match_zone:return
            desired=math.atan2(dy,dx)
            error=wrap(desired-pose[2]) if pose else math.pi
            if abs(error)<.30:
                unit=(dx/length,dy/length);heading=desired;aligned=True;break
            self.event('REALIGN',attempt=attempt+1,yaw_error=round(error,3))
            self.backoff(.22);self.spin(.6)
        if not aligned:raise MissionFailure('Could not align behind the visually observed cube')
        self.event('PUSH',destination=[round(v,3) for v in dest]);self.set_mode('push')
        end=time.monotonic()+100;lost=None;last_log=0.0
        while time.monotonic()<end:
            self.check();c=self.cube(color);pose=self.pose()
            if c is None or pose is None:
                self.command();lost=lost or time.monotonic()
                if time.monotonic()-lost>5:raise MissionFailure('Camera/TF lost during push; stopped')
                self.spin();continue
            lost=None;pair=self.pair(color) if match_zone else None
            goal=dest
            if pair:
                p=pair['cube']['position_map'];c=(p['x'],p['y'],pair['cube']);z=pair['zone']['position_map'];goal=(z['x'],z['y'])
            dx,dy=goal[0]-c[0],goal[1]-c[1];remaining=math.hypot(dx,dy)
            along=dx*unit[0]+dy*unit[1];cross=abs(dx*unit[1]-dy*unit[0])
            reached=(remaining<.04 and pair is not None) if match_zone else (remaining<.06 or (along<.035 and cross<.08))
            if reached:
                self.command();self.event('PUSH_REACHED',cube=[round(c[0],3),round(c[1],3)]);self.backoff();return
            desired=math.atan2(dy,dx);error=wrap(desired-pose[2]);bearing=c[2]['bearing_rad']
            w=clamp(3.0*error+.5*bearing,-.5,.5)
            speed=float(self.get_parameter('push_speed').value)
            if c[2]['range_m']>.40:speed=min(speed,.12)
            if match_zone and remaining<.30:
                if pair is None:
                    self.command();self.spin();continue
                speed=min(speed,.08 if remaining>.10 else .04)
            if abs(bearing)>.4 or abs(error)>.40:speed=0.
            elif abs(bearing)>.30:speed=min(speed,.10)
            if time.monotonic()-last_log>2:
                self.get_logger().info(json.dumps({'phase':'push_feedback','target_source':'camera_zone' if pair else 'map_waypoint','cube':[round(c[0],3),round(c[1],3)],'remaining':round(remaining,3),'yaw_error':round(error,3),'bearing':round(bearing,3),'velocity':[round(speed,3),round(w,3)]}));last_log=time.monotonic()
            self.command(speed,w);self.spin()
        self.command();raise MissionFailure('Push progress timeout')
    def verify(self,color,target):
        self.event('VERIFY');self.set_mode('push');self.command();end=time.monotonic()+10;valid=0;last=None
        while time.monotonic()<end:
            pair=self.pair(color)
            if pair and pair['stamp']!=last:
                last=pair['stamp'];c=pair['cube']['position_base'];z=pair['zone']['position_base'];theta=pair['zone']['yaw_base']
                dx,dy=c['x']-z['x'],c['y']-z['y']
                relative=(math.cos(theta)*dx+math.sin(theta)*dy,-math.sin(theta)*dx+math.cos(theta)*dy)
                valid=valid+1 if delivered_inside(relative,(0.,0.)) else 0
                if valid>=4:
                    self.event('DELIVERY_CONFIRMED',relative_error_m=[round(v,4) for v in relative],frames=valid)
                    p=pair['cube']['position_map'];return p['x'],p['y'],pair['cube']
            self.spin()
        raise MissionFailure('Camera could not confirm cube fully inside matching visible target')
    def ready(self):
        self.event('WAITING_FOR_NAV2');end=time.monotonic()+90
        while time.monotonic()<end:
            if self.nav_active and self.nav.server_is_ready() and self.pose() and self.odom and self.last_camera_wall:return
            self.spin()
        raise MissionFailure('Navigation, localization or camera not ready')
    def run(self):
        self.running=True
        try:
            self.ready();self.started=time.monotonic();self.sim_started=self.simtime();self.event('MISSION_STARTED')
            for color in self.get_parameter('colors').value:
                if color not in ['red','green','blue']:raise MissionFailure('Unknown cube color')
                self.active=color;y={'red':1.2,'green':0.,'blue':-1.2}[color]
                try:
                    if color=='blue':self.goto(*self.world(-2.25,-.4),-math.pi/2)
                    self.goto(*self.world(-2.25,y),0.)
                    self.acquire(color)
                    route=[(.60,0.),(1.8,-.55)] if color=='green' else [(-.9,0.),(.60,0.),(1.8,-.55)]
                    for waypoint in route:self.push_to(color,self.world(*waypoint))
                    self.push_to(color,self.world(2.35,y),match_zone=True)
                    c=self.verify(color,self.world(2.35,y));self.delivered.append(color)
                    self.event('DELIVERED',cube=[round(c[0],3),round(c[1],3)])
                    # Return through both gaps before approaching the next initial region.
                    if len(self.delivered)<len(self.get_parameter('colors').value):
                        self.goto(*self.world(1.6,-.4),math.pi)
                        self.goto(*self.world(.25,0),math.pi)
                        self.goto(*self.world(-1.0,-.4),math.pi)
                except MissionFailure as exc:
                    self.failures.append({'color':color,'reason':str(exc)});raise
            self.event('COMPLETE')
        except MissionFailure as exc:
            self.set_mode('stop');self.command()
            self.event('FAILED',reason=str(exc))
        except Exception as exc:
            self.set_mode('stop');self.command()
            self.event('FAILED',reason=f'{type(exc).__name__}: {exc}')
        finally:
            self.set_mode('stop');self.command()
            if self.nav_handle is not None:
                f=self.nav_handle.cancel_goal_async()
                until=time.monotonic()+3
                while not f.done() and time.monotonic()<until:rclpy.spin_once(self,timeout_sec=.05)
            self.set_mode('stop');self.command();self.running=False
            self.write_report()
            self.publish_status()

def main():
    rclpy.init();n=Mission()
    try:
        while rclpy.ok():
            rclpy.spin_once(n,timeout_sec=.1)
            if n.requested and n.started is None and not n.running:n.requested=False;n.run()
    except KeyboardInterrupt:
        n.set_mode('stop');n.command()
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
