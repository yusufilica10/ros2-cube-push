"""Check geometric ranging, flat target rejection, hue wrapping and safe cropping."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
import unittest
import cv2
import numpy as np
from cube_geometry import project_top,detect_cubes,detect_zones
from mission import delivered_inside,wrap
from evaluate_run import evaluate
K=np.array([160.,0,160.,0,160.,120.,0,0,1.])

def image_of_cube(color,center=(.50,0),angle=.2):
    points=project_top(center,angle,K)
    image=np.zeros((240,320,3),np.uint8)
    cv2.fillConvexPoly(image,np.rint(points).astype(np.int32),color)
    return image

class DetectionTest(unittest.TestCase):
    def test_three_colors_with_metric_fit(self):
        for name,color in [('red',(0,0,255)),('green',(0,255,0)),('blue',(255,0,0))]:
            detections=detect_cubes(image_of_cube(color),K)
            self.assertEqual([d['color'] for d in detections],[name])
            self.assertAlmostEqual(detections[0]['position_base']['x'],.5,delta=.04)
    def test_zone_center_survives_near_edge_occlusion_and_side_clipping(self):
        image=np.zeros((240,320,3),np.uint8)
        points=project_top((.45,-.15),.9,K,plane_height=.002,size=.6)
        cv2.fillConvexPoly(image,np.rint(points).astype(np.int32),(255,191,191))
        cv2.rectangle(image,(0,175),(319,239),(0,0,0),-1)
        zones=detect_zones(image,K,-.9)
        self.assertEqual(len(zones),1)
        self.assertAlmostEqual(zones[0]['position_base']['x'],.45,delta=.015)
        self.assertAlmostEqual(zones[0]['position_base']['y'],-.15,delta=.015)
    def test_real_occluded_zone_rejects_cube_antialias_fringe(self):
        image=cv2.imread(str(Path(__file__).parent/'fixtures/zone_occluded.png'))
        zones=detect_zones(image,K,-.9037394259000339,camera_pitch=1.1)
        self.assertEqual(len(zones),1)
        self.assertAlmostEqual(zones[0]['position_base']['x'],.5916641465,delta=.015)
        self.assertAlmostEqual(zones[0]['position_base']['y'],-.0050947020,delta=.015)
    def test_zone_far_edge_clipping_is_rejected(self):
        image=np.zeros((240,320,3),np.uint8)
        cv2.rectangle(image,(30,0),(280,130),(255,191,191),-1)
        self.assertEqual(detect_zones(image,K,-.9),[])
    def test_clipped_cube_is_not_ranged(self):
        im=np.zeros((240,320,3),np.uint8);cv2.rectangle(im,(0,100),(80,200),(255,0,0),-1)
        self.assertEqual(detect_cubes(im,K),[])
    def test_flat_zone_rejected(self):
        im=np.zeros((240,320,3),np.uint8);cv2.rectangle(im,(60,160),(260,170),(0,0,255),-1)
        self.assertEqual(detect_cubes(im,K),[])
    def test_red_hue_wrap(self):
        im=image_of_cube((0,0,255));hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);hsv[hsv[:,:,1]>0]=(178,255,255)
        self.assertEqual(detect_cubes(cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR),K)[0]['color'],'red')
    def test_full_containment_includes_rotation_margin(self):
        self.assertTrue(delivered_inside((2.35,1.2),(2.35,1.2)))
        self.assertFalse(delivered_inside((2.55,1.2),(2.35,1.2)))
    def test_independent_oracle_checks_all_corners(self):
        poses=[]
        for color,y in [('red',1.2),('green',0.),('blue',-1.2)]:
            poses.append({'name':color+'_cube','position':{'x':2.35,'y':y,'z':.09},'orientation':{'w':1.}})
        self.assertEqual(evaluate({'pose':poses})['delivered_count'],3)
        poses[0]['position']['x']=2.60
        self.assertEqual(evaluate({'pose':poses})['delivered_count'],2)
    def test_angle_wrap(self):self.assertAlmostEqual(wrap(2*np.pi+.2),.2)

if __name__=='__main__':unittest.main()
