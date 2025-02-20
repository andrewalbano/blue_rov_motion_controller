#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from std_msgs.msg import Bool, Int8, Float32MultiArray

print("Test 1")
current_yaw = 0
desired_yaw = 90

current_yaw = current_yaw*np.pi/180
desired_yaw = desired_yaw *np.pi/180

yaw_error = desired_yaw-current_yaw 


if yaw_error >np.pi:
        yaw_error -=2*np.pi
elif yaw_error <-np.pi:
        yaw_error +=2*np.pi

print(yaw_error*180/np.pi)


print("Test 2")
current_yaw = 0
desired_yaw = 180

current_yaw = current_yaw*np.pi/180
desired_yaw = desired_yaw *np.pi/180

yaw_error = desired_yaw-current_yaw 


if yaw_error >np.pi:
        yaw_error -=2*np.pi
elif yaw_error <-np.pi:
        yaw_error +=2*np.pi

print(yaw_error*180/np.pi)


print("Test 3")
current_yaw = 0
desired_yaw = 270

current_yaw = current_yaw*np.pi/180
desired_yaw = desired_yaw *np.pi/180

yaw_error = desired_yaw-current_yaw 


if yaw_error >np.pi:
        yaw_error -=2*np.pi
elif yaw_error <-np.pi:
        yaw_error +=2*np.pi

print(yaw_error*180/np.pi)


print("Test 4")
current_yaw = 0
desired_yaw = 360

current_yaw = current_yaw*np.pi/180
desired_yaw = desired_yaw *np.pi/180

yaw_error = desired_yaw-current_yaw 


if yaw_error >np.pi:
        yaw_error -=2*np.pi
elif yaw_error <-np.pi:
        yaw_error +=2*np.pi

print(yaw_error*180/np.pi)


# above tests passed



print("Test 1")
current_yaw = 90
desired_yaw = 0

current_yaw = current_yaw*np.pi/180
desired_yaw = desired_yaw *np.pi/180

yaw_error = desired_yaw-current_yaw 


if yaw_error >np.pi:
        yaw_error -=2*np.pi
elif yaw_error <-np.pi:
        yaw_error +=2*np.pi

print(yaw_error*180/np.pi)


print("Test 2")
current_yaw = 180
desired_yaw = 0

current_yaw = current_yaw*np.pi/180
desired_yaw = desired_yaw *np.pi/180

yaw_error = desired_yaw-current_yaw 


if yaw_error >np.pi:
        yaw_error -=2*np.pi
elif yaw_error <-np.pi:
        yaw_error +=2*np.pi

print(yaw_error*180/np.pi)


print("Test 3")
current_yaw = 270
desired_yaw = 0

current_yaw = current_yaw*np.pi/180
desired_yaw = desired_yaw *np.pi/180

yaw_error = desired_yaw-current_yaw 


if yaw_error >np.pi:
        yaw_error -=2*np.pi
elif yaw_error <-np.pi:
        yaw_error +=2*np.pi

print(yaw_error*180/np.pi)


print("Test 4")
current_yaw = 360
desired_yaw = 0

current_yaw = current_yaw*np.pi/180
desired_yaw = desired_yaw *np.pi/180

yaw_error = desired_yaw-current_yaw 


if yaw_error >np.pi:
        yaw_error -=2*np.pi
elif yaw_error <-np.pi:
        yaw_error +=2*np.pi

print(yaw_error*180/np.pi)

# above tests passed






print("Test 1")
current_yaw = 0
desired_yaw = 270

current_yaw = current_yaw*np.pi/180
desired_yaw = desired_yaw *np.pi/180

yaw_error = desired_yaw-current_yaw 


if yaw_error >np.pi:
        yaw_error -=2*np.pi
elif yaw_error <-np.pi:
        yaw_error +=2*np.pi

print(yaw_error*180/np.pi)


print("Test 2")
current_yaw = 180
desired_yaw = 270

current_yaw = current_yaw*np.pi/180
desired_yaw = desired_yaw *np.pi/180

yaw_error = desired_yaw-current_yaw 


if yaw_error >np.pi:
        yaw_error -=2*np.pi
elif yaw_error <-np.pi:
        yaw_error +=2*np.pi

print(yaw_error*180/np.pi)


print("Test 3")
current_yaw = 270
desired_yaw = 0

current_yaw = current_yaw*np.pi/180
desired_yaw = desired_yaw *np.pi/180

yaw_error = desired_yaw-current_yaw 


if yaw_error >np.pi:
        yaw_error -=2*np.pi
elif yaw_error <-np.pi:
        yaw_error +=2*np.pi

print(yaw_error*180/np.pi)


print("Test 4")
current_yaw = 360
desired_yaw = 0

current_yaw = current_yaw*np.pi/180
desired_yaw = desired_yaw *np.pi/180

yaw_error = desired_yaw-current_yaw 


if yaw_error >np.pi:
        yaw_error -=2*np.pi
elif yaw_error <-np.pi:
        yaw_error +=2*np.pi

print(yaw_error*180/np.pi)