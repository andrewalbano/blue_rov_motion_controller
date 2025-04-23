#!/usr/bin/env python3
import matplotlib.pyplot
import rospy
import numpy as np
# import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, TwistStamped, Pose, Twist, PointStamped
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from std_msgs.msg import Bool, Int8, Float32MultiArray, Int16MultiArray

# Jack added this:
from sensor_msgs.msg import Image as sensor_image
import json
import matplotlib

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import torch

bridge = CvBridge()


class SonarTopic():
    def __init__(self):

        self.image = None
        self.sonar_image = None
        self.camera_image = None
        self.mouse_coords = None
        self.squeeze = None

        self.sub_sonar = rospy.Subscriber('/oculus_sonar/ping_image', sensor_image, self.sonar_callback)
        self.sub_camrea = rospy.Subscriber('/camera/image_raw', sensor_image, self.camera_callback)
        # self.mouse = rospy.Subscriber('/oculus_sonar/ping_image/mouse_click', PointStamped, self.mouse_points)
        # Definitely need different subscribers for the mouse and sonar image ^^
        
        # creating publishers
        self.compressed_image = rospy.Publisher('/sonar_scan_image', sensor_image, queue_size=10)
        # self.cp2 = rospy.Publisher('/sonar_scan_image_approx', sensor_image, queue_size=10)
        # self.compressed_image = rospy.Publisher('/sonar_scan_image', Bool, queue_size=10)

        self.positive_detection = None

        self.sonar = None

        self.sonar_range = 120
        self.sonar_arc = 130*np.pi/180

        self.azimuth = None
        self.range = None
        self.theta = None
        # self.array=None

        self.x = 0
        self.y = 0
        self.transform = None
        
    
    def sonar_callback(self,msg:sensor_image):
        self.sonar_image = sensor_image()
        self.sonar_image = msg
        self.image_array = bridge.imgmsg_to_cv2(self.sonar_image, desired_encoding='passthrough')

        self.im_width = self.sonar_image.width
        self.im_height = self.sonar_image.height        

    def camera_callback(self,msg:sensor_image):
        self.camera_image = sensor_image()
        self.camera_image = msg

    def cylindrical2cartesian(self): #,msg:Int16MultiArray):
        self.squeeze = sensor_image()
        # Takes every pixel in the image and converts into cartesian coordinates
        # Each pixel keeps it's value
        transform = np.zeros((503,516))

        # for i in range(self.image_array.shape[0]): # rows
        #     for j in range(self.image_array.shape[1]): # columns
        #         theta = (self.sonar_arc/self.sonar_image.width) * j - self.sonar_arc/2
        #         # theta_vec[j] = theta

        #         radius = (self.sonar_range/self.im_height) * i
        #         # radius_vec[i] = radius

        #         # Picture centering: Add or subtract to x, y to move picture frame
        #         x = radius * np.math.cos(theta)
        #         y = radius * np.math.sin(theta) + 125            
                
        #         x_int, y_int = int(x), int(y) # Improving image resoultion by approximating pixels:

        #         # x_int_vec[i] = i
        #         # y_int_vec[j] = j

        #         transform[x_int,y_int] = self.image_array[i,j] # Maps the grayscale value for each pixel from the cylindrical image to the cartesian image

        #         # Improving image resoultion by approximating pixels:
        self.image_array = self.image_array[0:503,0:503]
        theta_span = np.arange(start=0,stop=self.image_array.shape[1],step=1)
        radius_span = np.arange(start=0,stop=self.image_array.shape[0],step=1)

        theta = (self.sonar_arc/self.sonar_image.width) * theta_span - self.sonar_arc/2
        theta = np.expand_dims(theta,axis=1)
        radius = (self.sonar_range/self.im_height) * radius_span
        radius = np.expand_dims(radius,axis=1)

        # Picture centering: Add or subtract to x, y to move picture frame
        # print(theta.shape,radius.shape)
        
        x = np.outer(radius,np.cos(theta))
        y = np.outer(radius,np.sin(theta)) + 125            
        
        print("XY",x.shape,y.shape,self.image_array.shape)
        # print(np.outer(x,y))
        # print(int(x[-1].astype(int)))
        # transform = np.empty(shape=(int(x[-1].astype(int)),int(y[-1].astype(int)))) # Maps the grayscale value for each pixel from the cylindrical image to the cartesian
        # transform.setflags(write=1)
        print("Tr",transform.shape)
        mask = self.image_array < 0
        # print(int(x[-1].astype(int),int(y[-1].astype(int))))
        transform[mask.astype(int)] = self.image_array

        positive_detection = transform
        positive_detection = np.roll(positive_detection,axis=1,shift=100)
        positive_detection = positive_detection[0:125,100:350]

        self.squeeze = bridge.cv2_to_imgmsg(transform.astype(np.uint8),encoding='passthrough')

        self.compressed_image.publish(self.squeeze)
        return positive_detection
        

    # def add_row(array):

        
def main():
    rospy.init_node('get_sonar_image')
    rospy.loginfo('Started get_sonar_image')
    # print(Image.width, Image.height)
    bag = SonarTopic()
    
    i = 0
    while not rospy.is_shutdown():
        if not (bag.sonar_image == None):
            rospy.loginfo("We have an image")
            cart = bag.cylindrical2cartesian()
            cv2.imwrite('transform.png', cart)
            
            break 
        else:
            # print(bag.mouse_coords)
            rospy.loginfo_throttle(3,"no image")
            
        # rospy.sleep(0.01)
    
    rospy.loginfo("broke rospy loop, ready to save")            


if __name__ == "__main__": 
        
    # function call 
    main() 