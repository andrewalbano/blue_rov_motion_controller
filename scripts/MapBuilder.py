#!/usr/bin/env python3
import rospy
import numpy as np
# import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose, Twist
from tf.transformations import euler_from_quaternion, euler_matrix
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
from std_msgs.msg import Bool, Int8, Float32MultiArray, PointCloud
import time
from sonar import SonarTopic

class MapBuilder(SonarTopic):
    # Purpose: Uses dvl state estimation and sonar scan data to project map in rviz
    # Need to save map as a rostopic, then echo continuosly in rviz for display on 2D grid

    def __init__(self):
        self.current_pose = PoseStamped()
        self.current_roll,self.current_pitch,self.current_yaw =0,0,0
        self.sonar = SonarTopic()
        self.map = rospy.Publisher('/map', PointCloud, queue_size=1)# What message type?
        self.state_sub = rospy.Subscriber('/state', PoseWithCovarianceStamped, self.position_callback)
        # We want to plot the sonar detections in the fixed frame, in every iteration of the while loop
        # The output needs to be a list of XY coordinates, that is constantly changing size based on the
        # number of XY pixels which meet the threshold
        # self.yaw = self.state['yaw']
        self.Rot2D = np.array([np.cos(self.current_yaw), -np.sin(self.current_yaw)],[np.sin(self.current_yaw), np.cos(self.current_yaw)])
    
    def position_callback(self, msg:PoseWithCovarianceStamped):

        # Extract position (x, y) from the message
        self.current_pose.header.frame_id = msg.header.frame_id
        self.current_pose.pose.position = msg.pose.pose.position
        self.current_pose.pose.orientation= msg.pose.pose.orientation
        
        # Extract orientation (quaternion) and convert to Euler angles (roll, pitch, yaw)
        q = msg.pose.pose.orientation

        self.current_roll,self.current_pitch,self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # self.attitude_transform = np.array(euler_matrix(self.current_roll,self.current_pitch,self.current_yaw))[:3,:3]
        # rospy.loginfo(self.attitude_transform.shape)

    def local2FF(self):
        XY_local = self.sonar.cylindrical2cartesian() # (N,2)
        XY_FF2ROV = np.array([self.current_pose.pose.position.x, self.current_pose.pose.position.y])  # (1,2)
        XY_FF2ROV = np.zeros((1,2))
        # Solve equations for each pixel coordinates, unless batch processing possible?
        XY_FF = XY_FF2ROV + self.Rot2D * XY_local.T # Shape (2,N)
        # rospy.loginfo_throttle(5,XY_FF)
        
        # print(XY_FF)
        # self.map.publish(XY_FF) # Published the list of XY pixels to the ROS topic for use in RViz


def main(): 
    
    # Initialize the ROS node
    rospy.init_node('map_builder')
    map = MapBuilder()
       
    while not rospy.is_shutdown():
        map.local2FF()
        


        
