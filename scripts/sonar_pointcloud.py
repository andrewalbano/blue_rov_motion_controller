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
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image as sensor_image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import json
import matplotlib
from pcl_msgs.msg import Vertices
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
        self.sub_camera = rospy.Subscriber('/camera/image_raw', sensor_image, self.camera_callback)
        
        # creating publishers
        self.compressed_image = rospy.Publisher('/sonar_scan_image', sensor_image, queue_size=10)
        self.cloud_map = rospy.Publisher('/point_cloud2', PointCloud2, queue_size=100)

        self.sonar = None

        self.sonar_range =120
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

        self.im_array_rgb = bridge.imgmsg_to_cv2(self.camera_image, desired_encoding='passthrough')

        self.im_rgb_width = self.camera_image.width
        self.im_rg_height = self.camera_image.height    

    def cylindrical2cartesian(self):
        self.squeeze = None #sensor_image()
        self.cloud = None # PointCloud()

        # Takes every pixel in the image and converts into cartesian coordinates
        # Each pixel keeps it's value
        transform = np.zeros((503,2*516))
        
        theta_span = np.arange(start=0,stop=self.image_array.shape[1],step=1)
        radius_span = np.arange(start=0,stop=self.image_array.shape[0],step=1)

        theta = (self.sonar_arc/self.sonar_image.width) * theta_span - self.sonar_arc/2
        theta = np.expand_dims(theta,axis=1)
        radius = (self.sonar_range/self.im_height) * radius_span
        radius = np.expand_dims(radius,axis=1)

        c = 2
        x = 3*np.outer(radius,np.cos(theta)) + 50
        y = c*np.outer(radius,np.sin(theta))+250
        print("max",np.max(x),np.max(y))
        transform[x.astype(int),y.astype(int)] = self.image_array[:,:] # Maps the grayscale value for each pixel from the cylindrical image to the cartesian image 
        # transform[:,5], transform[:,-5]= 255, 255
        # transform[5,:], transform[-5,:] = 255, 255
        transform[:,516:] = self.image_array[0:503,:]
        # For a certain threshold, turn all pixels with color less than threshold to zero:
        thresh = 100
        mask = transform[:,0:516] > thresh
        filtered = np.multiply(mask.astype(float),transform[:,0:516])
        # Multiplies the above-threshold pixels by 1, and the below-threshold pixels by 0

        positive_detection = filtered
        positive_detection = np.roll(positive_detection,axis=(0,1),shift=(0,0))
        # positive_detection = positive_detection[:/2,:/2]

        self.squeeze = bridge.cv2_to_imgmsg(transform.astype(np.uint8),encoding='passthrough')
       
        self.compressed_image.publish(self.squeeze)

        # Have [x,y] points for sonar, just need to create a list of [x,y,mono_value] for all the pixels in the image
        

        pc_list = [] # (shape=(positive_detection.shape[0]*positive_detection.shape[1],3))
        # positive_detection = np.roll(positive_detection,axis=1,shift=-100)
        positive_detection = positive_detection.astype(int)
        rows = positive_detection.shape[0]
        columns = positive_detection.shape[1]
        i, j = 0, 0
        scale = 1
        for i in range(rows):
            for j in range(columns):
                if positive_detection[int(i/scale),int(j/scale)] > thresh:
                    pc_list_index = [i,j-125,0]
                    pc_list.append(pc_list_index)
                else:
                    pass

            
        # pc_list[2] = np.ndarray.flatten(positive_detection).astype(list)
        # print(pc_list)
        # print(len(pc_list),250*125,pc_list[-1],pc_list[-2])
    
        if True:
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            # PointField('mono', 12, PointField.UINT32, 1),
            ]

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"

            pc2 = point_cloud2.create_cloud(header=header, fields=fields, points=pc_list)
            self.cloud_map.publish(pc2)
        return positive_detection

    
def main():
    rospy.init_node('get_sonar_image')
    rospy.loginfo('Started get_sonar_image')
    bag = SonarTopic()
    
    while not rospy.is_shutdown():
        if not (bag.sonar_image == None):
            rospy.loginfo("We have an image")
            cart = bag.cylindrical2cartesian()

            # color = bag.interpret_rgb()
            # cv2.imwrite('transform.png', cart)
            
            # break 
        else:
            rospy.loginfo_throttle(3,"no image")
            
        rospy.sleep(0.01)
    
    rospy.loginfo("broke rospy loop, ready to save")            


if __name__ == "__main__": 
        
    # function call 
    main() 