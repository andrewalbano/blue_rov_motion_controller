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
import dvl_client
import json
import matplotlib
import PIL
from PIL import Image as im

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

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
        self.mouse = rospy.Subscriber('/oculus_sonar/ping_image/mouse_click', PointStamped, self.mouse_points)
        # Definitely need different subscribers for the mouse and sonar image ^^
        
        # creating publishers
        # self.compressed_image = rospy.Publisher('/sonar_scan_image', sensor_image, queue_size=10)
        # self.cp2 = rospy.Publisher('/sonar_scan_image_approx', sensor_image, queue_size=10)
        # self.compressed_image = rospy.Publisher('/sonar_scan_image', Bool, queue_size=10)


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
        self.image_array = bridge.imgmsg_to_cv2(self.sonar_image, desired_encoding='mono8')

        # self.mouse_coords = PointStamped()
        # self.mouse_coords = mouse

        self.im_width = self.sonar_image.width
        self.im_height = self.sonar_image.height
        # self.im_width = msg.width
        # self.im_height = msg.height

        # self.im_width = self.image_array.shape[1]
        # self.im_height = self.image_array.shape[0]


        

    def camera_callback(self,msg:sensor_image):
        self.camera_image = sensor_image()
        self.camera_image = msg

    def mouse_points(self,msg:PointStamped):
        self.mouse_coords = PointStamped()
        self.mouse_coords = msg

        self.theta = (self.sonar_arc/self.sonar_image.width) * self.mouse_coords.point.x - self.sonar_arc/2
        self.range = (self.sonar_range/self.im_height) * self.mouse_coords.point.y
        self.azimuth = self.range*np.math.sin(self.theta)

    def scatter(self):
        x = np.zeros(self.image_array.shape[0])
        y = np.zeros(self.image_array.shape[0])

        for i in range(self.image_array.shape[0]):
            for j in range(self.image_array.shape[0]):
                # theta = (self.sonar_arc/self.sonar_image.width) * i - self.sonar_arc/2
                # radius = (self.sonar_range/self.im_height) * j
                # self.azimuth = self.range*np.math.sin(self.theta)
                if self.image_array[i,j] > 0:
                    x[i] = i * np.math.cos(j)
                    y[j] = i * np.math.sin(j)
                else:
                    x[i],y[i] = 0, 0
                
                # 1-to-1 correspondance for (i,j) -> (x,y)
                
                
        matplotlib.pyplot.scatter(y,x,s=4)
        matplotlib.pyplot.show()

    def cylindrical2cartesian(self): #,msg:Int16MultiArray):
        self.squeeze = sensor_image()
        
        # Takes every pixel in the image and converts into cartesian coordinates
        # Each pixel keeps it's value
        transform = np.zeros((516,516))
        add_pixels = 0
        positive_detection = []
        # radius_vec = np.zeros(self.image_array.shape[0])
        # x_int_vec = radius_vec
        # theta_vec = np.zeros(self.image_array.shape[1])
        # y_int_vec = theta_vec

        for i in range(self.image_array.shape[0]): # rows
            for j in range(self.image_array.shape[1]): # columns
                theta = (self.sonar_arc/self.sonar_image.width) * j - self.sonar_arc/2
                # theta_vec[j] = theta

                radius = (self.sonar_range/self.im_height) * i
                # radius_vec[i] = radius

                # Picture centering: Add or subtract to x, y to move picture frame
                x = radius * np.math.cos(theta)
                y = radius * np.math.sin(theta) + 125            
                
                x_int, y_int = int(x), int(y) # Improving image resoultion by approximating pixels:

                # x_int_vec[i] = i
                # y_int_vec[j] = j

                transform[x_int,y_int] = self.image_array[i,j] # Maps the grayscale value for each pixel from the cylindrical image to the cartesian image

                # Improving image resoultion by approximating pixels:
    
                if self.image_array[i,j] > 40:
                    # print("t=",type(positive_detection))
                    positive_detection.append([x_int,y_int])  

        # Use these lines to adjust picture position on screen:
        transform = np.roll(transform,axis=1,shift=100)
        transform = transform[0:125,100:350]

        self.squeeze = bridge.cv2_to_imgmsg(transform.astype(np.uint8),encoding='mono8')

        # self.compressed_image.publish(self.squeeze)
        
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

            # image_array = bridge.imgmsg_to_cv2(bag.sonar_image, desired_encoding='mono8')
            # print(image_array.shape)
            # Need to extract x,y coordinates of every pixel in this array ^, then scatter

                        




            # print(bag.sonar_image.width)
            # print(bag.mouse_coords)
            # print(bag.azimuth,bag.range,bag.theta*180/np.pi)
            # print(bag.sonar_arc,bag.im_width,bag.mouse_coords.point.x)
            # print(bag.im_width,bag.im_height)

            # image_array = np.flip(image_array,axis=0)

            # picture = cv2.imwrite('pic.png', image_array)
            # # print(image_array[200,250:270])

            # crop = image_array[100:300,50:400]
            # cropped = cv2.imwrite('crop.png', crop)

            # # bag.cylindrical2cartesian()

            # color = image_array
            # # print(color)
            # print(color.shape)

            # plot = bag.scatter()

            
            # print("cart=",cart.shape,cart.size)
            carte = cv2.imwrite('transform.png', cart2)



            # for i in range(color.shape[0]):
            #     for j in range(color.shape[1]):
            #         if color[i,j] < 0:
            #             color[i,j] = 0

            # blot = cv2.imwrite('blot.png', color)
            
            # break 
        else:
            # print(bag.mouse_coords)
            rospy.loginfo_throttle(3,"no image")
            

        # rospy.sleep(0.01)
    
    rospy.loginfo("broke rospy loop, ready to save")            


if __name__ == "__main__": 
        
    # function call 
    main() 