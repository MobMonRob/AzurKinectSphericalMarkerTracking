import open3d as o3d
import cv2
import numpy as np
import argparse
import matplotlib.pyplot as plt
import os
import sys
import json
from statistics import median


# can be used instead of AzureMKVReader
class AzureSensor:
    def __init__(self,config):
        pass
    def run(config):
        sensor = o3d.io.AzureKinectSensor(config)
        if not sensor.connect(0):
            raise RuntimeError('Failed to connect to sensor')
        return sensor
    
pwd = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(pwd, '..'))
#from initialize_config import initialize_config

class AzureMKVReader:
    def __init__(self):
        pass
    def run():
        input = "../newdata/WFOV-UNBINNED.mkv"
        reader = o3d.io.AzureKinectMKVReader()
        reader.open(input)
        if not reader.is_opened():
            raise RuntimeError("Unable to open file {}".format(input))
        return reader

class GetArgs:
    def __init__(self):
        parser = argparse.ArgumentParser(description='Azure kinect mkv recorder.')
        parser.add_argument('--config', type=str, help='input json kinect config')
        parser.add_argument('--list',
                            action='store_true',
                            help='list available azure kinect sensors')
        parser.add_argument('--device',
                            tyinputpe=int,
                            default=0, 
                            help='input kinect device id')
        parser.add_argument('-a',
                            '--align_depth_to_color',
                            action='store_true',
                            help='enable align depth image to color')
        self.args = parser.parse_args()
        
    def readconf(self):
        if self.args.list:
            o3d.io.AzureKinectSensor.list_devices()
            exit()
    
        if self.args.config is not None:
            config = o3d.io.read_azure_kinect_sensor_config(self.args.config)
        else:
            config = o3d.io.read_azure_kinect_sensor_config("default_config.json")
        
        return config



def getMarkerDistance(x,y,img):
    #x=5
    #y=30
    markerDist = []
    color,depth = np.asarray(img.color).astype(np.uint8),np.asarray(img.depth).astype(np.float32)
    for i in range(-7,7):
        for j in range(-7,7):
            arr= np.asarray(depth)
            val = np.asarray(depth)[y+i, x+j]
            if val == 0:
                continue
            markerDist.append(val)
            #print(val)
    result = median(markerDist)
    return result

def main():
    #config = GetArgs()
    pointclouds = []
    pcd  = o3d.geometry.PointCloud
    #sensor = AzureSensor.run(config.readconf())
    sensor = AzureMKVReader.run()
    metadata = sensor.get_metadata()
    o3d.io.write_azure_kinect_mkv_metadata(
                'intrinsic.json', metadata)
    while True:
        raw = sensor.next_frame() #capture_frame(False)
        if raw is None:
            continue
        imgd = np.zeros((512, 512, 3))
        #img_depth_old = np.asarray(rgbd.depth)
        color,depth = np.asarray(raw.color).astype(np.uint8),np.asarray(raw.depth).astype(np.float32)/1000.0
        img = np.asarray(raw.color,dtype=np.uint8)

        highest=0
        #print (np.max(imgd))

        imgd = cv2.normalize(depth,None,0,255,cv2.NORM_MINMAX,dtype=cv2.CV_8U)

        #cv2.waitKey(0)
        
        img = imgd

        #print (img)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Convert to grayscale.
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        minDist = 20
        param1 = 10
        param2 = 20
        minRad = 5
        maxRad = 20

        ret,thresh = cv2.threshold(gray,1,255,cv2.THRESH_BINARY_INV)
        cv2.imshow("thresh",thresh)
        cv2.waitKey(1)
        #HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) -> circles
        detected_circles = cv2.HoughCircles(thresh,
                        cv2.HOUGH_GRADIENT, 1, minDist, param1 =param1,
                    param2 = param2, minRadius = minRad, maxRadius = maxRad)

        # Draw circles that are detected.
        if detected_circles is not None:

            # Convert the circle parameters a, b and r to integers.
            detected_circles = np.uint16(np.around(detected_circles))

            for pt in detected_circles[0, :]:
                a, b, r = pt[0], pt[1], pt[2]
                if gray[b][a]== 0:
                    if r >= 12:
                        exit
                    dist = getMarkerDistance(a,b,raw)
                    print("Dist: ",dist,"x: ",a,"y: ",b)

                else:
                  
                    #print(gray[b][a])
                    # Draw the circumference of the circle.
                    cv2.circle(img, (a, b), r, (0, 0, 255), 2)
                    # Draw a small circle (of radius 1) to show the center.
                    cv2.circle(img, (a, b), 1, (0, 0, 255), 1)
                #pcd = o3d.geometry.PointCloud.create_from_rgbd_image()
                color = o3d.geometry.Image(color)
                depth = o3d.geometry.Image(depth)
                rgbd_new = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, depth_scale=1.0,convert_rgb_to_intensity=False)    
                intr = o3d.camera.PinholeCameraIntrinsic(1280, 720, 603.4254150390625, 603.6671142578125, 635.36102294921875, 365.50311279296875)
                #intr = o3d.io.read_pinhole_camera_intrinsic("../newdata/NFOV-UNBINNED.mkv")
                pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_new,intr)
                #o3d.visualization.draw_geometries([pcd])
                #pointclouds.appennd(pcd)
                #cv2.imshow("Detected Circle", img)
                #print 
                #cv2.waitKey(1)
if __name__=='__main__':
    main()