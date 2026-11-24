"""Camera-only top-plane geometry for known 18 cm cubes; no simulator state."""
import math
import cv2
import numpy as np

CUBE_SIZE=.18

def plane_points(pixels,k,plane_height=.18,camera_height=.5,camera_pitch=.95,camera_x=.18):
    pixels=np.asarray(pixels,dtype=float).reshape(-1,2)
    forward=np.ones(len(pixels));left=-(pixels[:,0]-k[2])/k[0];up=-(pixels[:,1]-k[5])/k[4]
    rx=math.cos(camera_pitch)*forward+math.sin(camera_pitch)*up
    rz=-math.sin(camera_pitch)*forward+math.cos(camera_pitch)*up
    t=(plane_height-camera_height)/rz
    return np.stack([camera_x+t*rx,t*left],axis=1)

def project_top(center,yaw,k,camera_height=.5,camera_pitch=.95,camera_x=.18,plane_height=.18,size=.18):
    corners=np.array([[x,y] for x,y in [(-size/2,-size/2),(-size/2,size/2),(size/2,size/2),(size/2,-size/2)]])
    c,s=math.cos(yaw),math.sin(yaw);points=corners@np.array([[c,s],[-s,c]])+center
    x=points[:,0]-camera_x;z=plane_height-camera_height
    depth=math.cos(camera_pitch)*x-math.sin(camera_pitch)*z
    up=math.sin(camera_pitch)*x+math.cos(camera_pitch)*z
    return np.stack([k[2]-k[0]*points[:,1]/depth,k[5]-k[4]*up/depth],axis=1)

def bearing_candidates(bgr,k,camera_pitch=.95,min_area=25):
    """Bearing-only hints, including clipped tops; never assign metric positions."""
    if min(k[0],k[4])<=0:return []
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV);result=[]
    for color,bands in {'red':[(0,10),(170,179)],'green':[(35,85)],'blue':[(95,135)]}.items():
        mask=np.zeros(hsv.shape[:2],np.uint8)
        for lo,hi in bands:mask|=cv2.inRange(hsv,(lo,100,170),(hi,255,255))
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area=cv2.contourArea(contour)
            if area<min_area:continue
            x,y,w,h=cv2.boundingRect(contour)
            if min(w,h)<4:continue
            u=x+(w-1)/2;v=y+(h-1)/2
            forward=math.cos(camera_pitch)-math.sin(camera_pitch)*(v-k[5])/k[4]
            left=-(u-k[2])/k[0]
            if forward<=0:continue
            result.append({'color':color,'bearing_rad':math.atan2(left,forward),
                           'bbox':[x,y,w,h],'area_px':float(area)})
    return sorted(result,key=lambda d:-d['area_px'])


def detect_cubes(bgr,k,camera_height=.5,min_area=25,camera_pitch=.95,camera_x=.18):
    if min(k[0],k[4],camera_height-.18)<=0:return []
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV);detections=[]
    for color,bands in {'red':[(0,10),(170,179)],'green':[(35,85)],'blue':[(95,135)]}.items():
        mask=np.zeros(hsv.shape[:2],np.uint8)
        # In this model the illuminated top is bright; vertical faces are darker.
        # Pastel target zones have low saturation and do not enter this mask.
        for low,high in bands:mask|=cv2.inRange(hsv,(low,100,170),(high,255,255))
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area=cv2.contourArea(contour)
            if area<min_area:continue
            x,y,w,h=cv2.boundingRect(contour)
            if x<=1 or y<=1 or x+w>=bgr.shape[1]-1 or y+h>=bgr.shape[0]-1:continue
            world=plane_points(contour.reshape(-1,2),k,camera_height=camera_height,camera_pitch=camera_pitch,camera_x=camera_x)
            if not np.isfinite(world).all():continue
            rect=cv2.minAreaRect(world.astype(np.float32));(forward,left),(sx,sy),angle=rect
            if not (.145<=sx<=.215 and .145<=sy<=.215):continue
            if min(sx,sy)/max(sx,sy)<.8:continue
            if cv2.contourArea(world.astype(np.float32))<.020:continue
            dx=forward-camera_x;dz=.09-camera_height
            cx=math.cos(camera_pitch)*dx-math.sin(camera_pitch)*dz
            cz=math.sin(camera_pitch)*dx+math.cos(camera_pitch)*dz
            detections.append({'color':color,'bbox':[x,y,w,h],'pixel_center':[x+(w-1)/2,y+(h-1)/2],
                               'bearing_rad':round(math.atan2(left,forward),4),'range_m':round(math.hypot(forward,left),3),
                               'estimated_height_m':.18,'top_size_m':[round(sx,3),round(sy,3)],
                               'position_base':{'x':float(forward),'y':float(left),'z':.09},
                               'position_camera':{'x':float(cx),'y':float(left),'z':float(cz)}})
    return sorted(detections,key=lambda d:(d['color'],d['range_m']))


def detect_zones(bgr,k,base_yaw,camera_height=.5,camera_pitch=.95,camera_x=.18):
    """Fit far edges of known 60 cm floor squares even when the near edge is hidden."""
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV);zones=[]
    theta=-base_yaw;c,s=math.cos(theta),math.sin(theta)
    rotation=np.array([[c,s],[-s,c]])
    for color,bands in {'red':[(0,10),(170,179)],'green':[(35,85)],'blue':[(95,135)]}.items():
        mask=np.zeros(hsv.shape[:2],np.uint8)
        for lo,hi in bands:mask|=cv2.inRange(hsv,(lo,25,160),(hi,110,255))
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            if cv2.contourArea(contour)<100:continue
            x,y,w,h=cv2.boundingRect(contour)
            if y<=1:continue  # The far edge itself must be in view.
            pixels=contour.reshape(-1,2)
            points=plane_points(pixels,k,plane_height=.002,camera_height=camera_height,camera_pitch=camera_pitch,camera_x=camera_x)
            if not np.isfinite(points).all():continue
            local=points@rotation.T;lo,hi=local.min(axis=0),local.max(axis=0)
            center=[];valid=True
            for axis,forward_component in [(0,c),(1,-s)]:
                span=hi[axis]-lo[axis]
                if span>.67 or span<.15:valid=False;break
                if abs(forward_component)<.10:
                    if span<.53:valid=False;break
                    center.append((lo[axis]+hi[axis])/2);continue
                far=hi[axis] if forward_component>0 else lo[axis]
                support=(np.abs(local[:,axis]-far)<.012)&(pixels[:,0]>1)&(pixels[:,0]<bgr.shape[1]-2)&(pixels[:,1]>1)
                if support.sum()<8 or np.ptp(local[support,1-axis])<.08:valid=False;break
                center.append(far-.30 if forward_component>0 else far+.30)
            if not valid:continue
            forward,left=np.asarray(center)@rotation
            if not .10<forward<1.3:continue
            dx=forward-camera_x;dz=.002-camera_height
            zones.append({'color':color,'bbox':[x,y,w,h],
                          'position_base':{'x':float(forward),'y':float(left),'z':.002},
                          'position_camera':{'x':float(math.cos(camera_pitch)*dx-math.sin(camera_pitch)*dz),'y':float(left),'z':float(math.sin(camera_pitch)*dx+math.cos(camera_pitch)*dz)},
                          'yaw_base':theta,'observed_spans_m':[float(v) for v in hi-lo]})
    return zones
