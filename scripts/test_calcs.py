#!/usr/bin/env python3
import rospy
import numpy as np
# import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose, Twist
from tf.transformations import euler_from_quaternion, euler_matrix
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from std_msgs.msg import Bool, Int8, Float32MultiArray
x =0
y = 0 
z =0 
roll = 0.1
pitch = 0.2
yaw = 0.3

pose = np.array([x,y,z,roll,pitch,yaw]).reshape(-1,1)


def coordinate_transform_matrix(pose):  
    # may need to double check that this returns the same answer
    R = np.array(euler_matrix(pose[3],pose[4],pose[5]))[:3,:3]
    zeros = np.zeros((3,3))
    
    R2 = np.block([
            [R,zeros],
            [zeros, R]
            ])
    print(R2)

    return R 
    


def euler_to_rotation_matrix(phi, theta, psi):
    # Precompute sines and cosines of the Euler angles
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    ctheta = np.cos(theta)
    stheta = np.sin(theta)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)
    
    # Construct the rotation matrix
    rotation_matrix = np.array([
        [cpsi * ctheta, spsi * ctheta, -stheta],
        [-spsi * cphi + cpsi * stheta * sphi, cpsi * cphi + spsi * stheta * sphi, ctheta * sphi],
        [spsi * sphi + cpsi * stheta * cphi, -cpsi * sphi + spsi * stheta * cphi, ctheta * cphi]
    ]).T
    
    return rotation_matrix


def coriolis_matrix():
    zeros = np.zeros((3,3))
    v = [2,3,4,1,1,1]
    # print(v[:3])
    M1 = create_skew_symmetric_matrix(v[:3])


    print(create_skew_symmetric_matrix(np.multiply(np.array([[1,2,-1*3]]),v[3:])[0]))

    # print(M1)

def create_skew_symmetric_matrix(v):
    M = np.array([
        [0,v[2],-1*v[1]],
        [-v[2],0,v[0]],
        [v[1],-1*v[0],0]
    ])
    return M

coriolis_matrix()
# R1 = coordinate_transform_matrix(pose)
# R2 = euler_to_rotation_matrix(roll, pitch, yaw)

# print(R1)

# print(R2)
        
