#!/usr/bin/env python3
import numpy as np
from sonar import SonarTopic
import rospy
from sensor_msgs.msg import PointCloud
from math import cos as c
from math import sin as s
from sensor_msgs.msg import Image as sensor_image

class MapBuilder_0(SonarTopic):
    # Purpose: Uses dvl state estimation and sonar scan data to project map in rviz
    # Need to save map as a rostopic, then echo continuosly in rviz for display on 2D grid

    def __init__(self):
        self.state = np.zeros((3,1))
        self.sonar = SonarTopic()
        self.image = SonarTopic.sonar_callback(self,sensor_image)

        self.map = rospy.Publisher('/map', PointCloud, queue_size=10) # What message type?
        # We want to plot the sonar detections in the fixed frame, in every iteration of the while loop
        # The output needs to be a list of XY coordinates, that is constantly changing size based on the
        # number of XY pixels which meet the threshold
        self.yaw = self.state[2]
        self.psi = 0
        
    
    def local2FF(self):
        self.Rot2D = np.array([c(self.psi), -s(self.psi)],[s(self.psi), c(self.psi)])
        XY_local = self.sonar.cylindrical2cartesian() # (N,2)
        XY_FF2ROV = np.array([self.state[0], self.state[1]])  # (1,2)
        XY_FF2ROV = np.zeros((1,2))
        # Solve equations for each pixel coordinates, unless batch processing possible?
        XY_FF = XY_FF2ROV + self.Rot2D * XY_local.T # Shape (2,N)
        self.map.publish(XY_FF) # Published the list of XY pixels to the ROS topic for use in RViz
        return XY_local

def main():
    rospy.init_node('get_sonar_image')
    rospy.loginfo('Started get_sonar_image')
    # print(Image.width, Image.height)
    bag = MapBuilder_0()
    
    i = 0
    while not rospy.is_shutdown():
        if True:
            rospy.loginfo("We have an image")
            cart = bag.cylindrical2cartesian()
            print(bag.local2FF)

            # break 
        else:
            # print(bag.mouse_coords)
            rospy.loginfo_throttle(3,"no image")
            

        # rospy.sleep(0.01)
    
    rospy.loginfo("broke rospy loop, ready to save")            


if __name__ == "__main__": 
        
    # function call 
    main() 