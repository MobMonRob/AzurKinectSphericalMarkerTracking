import open3d as o3d
import cv2
import numpy as np
import argparse
import matplotlib.pyplot as plt

class AzureSensor:
    def __init__(self,config):
        pass
    def run(config):
        sensor = o3d.io.AzureKinectSensor(config)
        if not sensor.connect(0):
            raise RuntimeError('Failed to connect to sensor')
        return sensor

class GetArgs:
    def __init__(self):
        parser = argparse.ArgumentParser(description='Azure kinect mkv recorder.')
        parser.add_argument('--config', type=str, help='input json kinect config')
        parser.add_argument('--list',
                            action='store_true',
                            help='list available azure kinect sensors')
        parser.add_argument('--device',
                            type=int,
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
            config = o3d.io.AzureKinectSensorConfig()
        
        return config



def main():
    config = GetArgs()
    
    sensor = AzureSensor.run(config.readconf())

    while True:
        rgbd = sensor.capture_frame(False)
        if rgbd is None:
            continue
        imgd = np.zeros((512, 512, 3))
        img_depth_old = np.asarray(rgbd.depth)
        
        img = np.asarray(rgbd.color,dtype=np.uint8)

        highest=0
        #print (np.max(imgd))
        imgd = cv2.normalize(img_depth_old,None,0,255,cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        cv2.waitKey(0)
        
        img = imgd

        #print (img)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Convert to grayscale.
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = imgd
        # Blur using 3 * 3 kernel.
        gray_blurred = cv2.blur(gray, (3, 3))

        # Apply Hough transform on the blurred image.
        detected_circles = cv2.HoughCircles(gray_blurred,
                        cv2.HOUGH_GRADIENT, 1, 20, param1 = 50,
                    param2 = 30, minRadius = 1, maxRadius = 40)

        # Draw circles that are detected.
        if detected_circles is not None:

            # Convert the circle parameters a, b and r to integers.
            detected_circles = np.uint16(np.around(detected_circles))

            for pt in detected_circles[0, :]:
                a, b, r = pt[0], pt[1], pt[2]

                # Draw the circumference of the circle.
                cv2.circle(img, (a, b), r, (0, 255, 0), 2)

                # Draw a small circle (of radius 1) to show the center.
                cv2.circle(img, (a, b), 1, (0, 0, 255), 3)
                cv2.imshow("Detected Circle", img)
                #print 
                cv2.waitKey(0)
        else:
            cv2.imshow("NO Circles", img)
                #print 
            cv2.waitKey(1)
if __name__=='__main__':
    main()