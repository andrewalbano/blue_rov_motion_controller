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

class MapBuilder_0(SonarTopic):
    # Purpose: Uses dvl state estimation and sonar scan data to project map in rviz
    # Need to save map as a rostopic, then echo continuosly in rviz for display on 2D grid

    def __init__(self):
        self.state = np.zeros((1,3))
        self.sonar = SonarTopic()
        self.map = rospy.Publisher('/map', tuple, queue_size=10) # What message type?
        # We want to plot the sonar detections in the fixed frame, in every iteration of the while loop
        # The output needs to be a list of XY coordinates, that is constantly changing size based on the
        # number of XY pixels which meet the threshold
        self.yaw = self.state[2]
        psi = 0
        self.Rot2D = np.array([np.cos(psi), -np.sin(psi)],[np.sin(psi), np.cos(psi)])
    
    def local2FF(self):
        XY_local = self.sonar.cylindrical2cartesian() # (N,2)
        XY_FF2ROV = np.array([self.state[0], self.state[1]])  # (1,2)
        XY_FF2ROV = np.zeros((1,2))
        # Solve equations for each pixel coordinates, unless batch processing possible?
        XY_FF = XY_FF2ROV + self.Rot2D * XY_local.T # Shape (2,N)
        self.map.publish(XY_FF) # Published the list of XY pixels to the ROS topic for use in RViz