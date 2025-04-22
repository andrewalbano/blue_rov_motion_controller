#!/usr/bin/env python3
import rospy
import numpy as np
# import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose, Twist, TwistWithCovarianceStamped
from tf.transformations import euler_from_quaternion, euler_matrix
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
from std_msgs.msg import Bool, Int8, Float32MultiArray, String, Float32
import time

import termios
import tty
import sys
from select import select

"""
Example of how to set target depth in depth hold mode with pymavlink
"""

import time
import math
# Import mavutil
from pymavlink import mavutil
# Imports for attitude
from pymavlink.quaternion import QuaternionBase
class attitude_test:
    def __init__(self):
        #set up publishers
        self.mode_change = rospy.Publisher('/BlueRov2/mode/set', String, queue_size=10)
        self.arm_ROV = rospy.Publisher('/BlueRov2/arm', Bool, queue_size=10)
        self.command_ROV = rospy.Publisher('/BlueRov2/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        #key bindings from input
        self.armBindings= {
            'a':(1),
            'l':(0),
        }
        self.modeBindings= {
            'm':('MANUAL'),
            's':('STABILIZE'),
            'p':('POSHOLD')
        }
        
        #set up publishers
        self.mode_change = rospy.Publisher('/BlueRov2/mode/set', String, queue_size=10)
        self.arm_ROV = rospy.Publisher('/BlueRov2/arm', Bool, queue_size=10)
        self.command_ROV = rospy.Publisher('/BlueRov2/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)



    def getKey(self, settings, timeout):
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def keyboard_check(self):
            self.mode_change.publish(self.modeBindings.get(key))


def set_target_depth(depth):
    """ Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

    """
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=( # ignore everything except z position
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), lat_int=0, lon_int=0, alt=depth, # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )

def set_target_attitude(master,roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode.

    'roll', 'pitch', and 'yaw' are angles in degrees.

    """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
    )

# connect to bluerov
def setup_signal(master):
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()
        
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)

    rospy.loginfo("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    rospy.loginfo('Armed!')
    
def initialize_operating_mode(master,mode):
    if mode == "manual":
        master.set_mode(19) 
    elif mode =="depth hold":
        master.set_mode('ALT_HOLD')

def main(): 
    rospy.init_node("test_attitude")
    rospy.loginfo("startin attitude test")
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14548') 
    setup_signal(master)
    initialize_operating_mode(master=master, mode = "depth hold")
    
    cmd = attitude_test()

    global boot_time 
    boot_time = time.time()
    # Wait a heartbeat before sending commands
   
    # set a depth target
    # set_target_depth(-0.5)
    while not rospy.is_shutdown():
        roll_angle =0 
        pitch_angle =0
        yaw_angle = 90
        set_target_attitude(master,roll_angle, pitch_angle, yaw_angle)
        time.sleep(5)

        yaw_angle = 180
        set_target_attitude(master,roll_angle, pitch_angle, yaw_angle)
        time.sleep(5)

        # go for a spin
        # (set target yaw from 0 to 500 degrees in steps of 10, one update per second)
        # roll_angle = pitch_angle = 0
        # for yaw_angle in range(0, 90, 10):
        #     master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
        #                                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        #     set_target_attitude(roll_angle, pitch_angle, yaw_angle)
        #     time.sleep(3) # wait for a second

        # # spin the other way with 3x larger steps
        # for yaw_angle in range(500, 0, -30):
        #     set_target_attitude(roll_angle, pitch_angle, yaw_angle)
        #     time.sleep(1)

        # clean up (disarm) at the end
        # master.arducopter_disarm()
        # master.motors_disarmed_wait()
        # Send heartbeat from a GCS (types are define as enum in the dialect file).
       
    
        # rospy.rate.sleep()   
        
    

if __name__ == "__main__":

    main()
    