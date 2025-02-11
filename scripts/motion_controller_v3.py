#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped
from tf.transformations import euler_from_quaternion
# from mavros_msgs.msg import OverrideRCIn
import time
# from pymavlink import mavutil
import threading
from std_msgs.msg import Bool

CONTROL_CLIP = (1400, 1600)

class MotionControl:
    def __init__(self):
        
        self.current_pose = PoseStamped()
        self.waypoints = PoseArray()

        self.current_waypoint = PoseStamped()
        self.waypoint_index = 0 
        self.num_waypoints = 0

        # for plotting
        self.x_vals = []
        self.y_vals = []
        self.yaws = []

        # P Controller Gains
        self.Kp_position = 0.1 # Proportional gain for position control (x, y)
        self.Kp_yaw = 0.1     # Proportional gain for yaw control

        # errors ... might need to initialize as very large values
        self.error_x = 0
        self.error_y = 0
        self.error_z = 0
        self.error_roll = 0
        self.error_pitch = 0
        self.error_yaw = 0

        # Control values
        self.x_control = 0
        self.y_control = 0
        self.z_control = 0
        self.roll_control = 0
        self.pitch_control = 0
        self.yaw_control = 0

        # Simulated Control values
        self.velocity_command = TwistStamped()
        self.velocity_command.header.frame_id = "base_link"  # Example frame id

        # reached waypoint threshold
        self.tolerance = 0.2
        
        # turn on or off
        self.invoked = False

        self.frequency =10 
        self.rate = rospy.Rate(self.frequency)  # 10 Hz

        # creating subscribers
        self.sub1 = rospy.Subscriber('motion_control_state',Bool, self.on_off_callback)
        self.sub2 = rospy.Subscriber('/dvl/local_position', PoseWithCovarianceStamped, self.position_callback)
        self.sub3 = rospy.Subscriber('target_waypoints_list', PoseArray, self.waypoint_list_callback)

        # creating publishers
        self.pub1 = rospy.Publisher('velocity_command', TwistStamped, queue_size=10)

    # whenever the button is hit, toggle the controller on/off
    def on_off_callback(self,msg:Bool):
        self.invoked = msg.data
        if self.invoked: 
            rospy.loginfo("Motion controller activated")
        elif not self.invoked:
            rospy.loginfo("Motion controller deactivated")

        # if self.invoked:
        #     self.invoked = False
        #     rospy.loginfo("Motion controller turned off")
        # elif self.invoked == False:
        #     rospy.loginfo("Motion controller turned on")
        #     self.invoked = True      

    def position_callback(self, msg:PoseWithCovarianceStamped):

        # Extract position (x, y) from the message
        self.current_pose.header.frame_id = msg.header.frame_id
        self.current_pose.pose.position = msg.pose.pose.position
        self.current_pose.pose.orientation= msg.pose.pose.orientation
       
        # Extract orientation (quaternion) and convert to Euler angles (roll, pitch, yaw)
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def waypoint_list_callback(self, msg:PoseArray):
        # checks if it is a list and get how many waypoint
        if isinstance(msg.poses, list):
            self.num_waypoints = len(msg.poses)
            rospy.loginfo("Received " + str(self.num_waypoints) + " waypoints")
        else:
            self.num_waypoints = 1
            rospy.loginfo("Received 1 waypoint")


        # assigns waypoints
        self.waypoints = msg

    def get_current_waypoint(self):
        #  accounts for the case where there is ponly 1 waypoint 
        if self.num_waypoints == 1:
            self.waypoint_index = 0
            self.current_waypoint.header.frame_id = self.waypoints.header.frame_id
            self.current_waypoint.pose = self.waypoints.poses[0]
        elif self.num_waypoints > 1:
            self.current_waypoint.header.frame_id = self.waypoints.header.frame_id
            self.current_waypoint.pose = self.waypoints.poses[self.waypoint_index]

        return self.current_waypoint

    def calculate_control(self): # default to first waypoint

        # self.current_waypoint = self.get_current_waypoint()
        # map frame error calculations
        # distance from waypoint (in map frame)
        self.error_x = self.current_waypoint.pose.position.x - self.current_pose.pose.position.x
        self.error_y = self.current_waypoint.pose.position.y - self.current_pose.pose.position.y
        self.error_z = self.current_waypoint.pose.position.z - self.current_pose.pose.position.z


        # in map frame
        # get the current roll, pitch, yaw
        current_roll, current_pitch, current_yaw = euler_from_quaternion([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w])

        # get the target roll, pitch, yaw 
        target_roll, target_pitch, target_yaw = euler_from_quaternion([self.current_waypoint.pose.orientation.x,self.current_waypoint.pose.orientation.y, self.current_waypoint.pose.orientation.z, self.current_waypoint.pose.orientation.w])



        # Transform the position errors to the body frame using the robot's yaw
        ex = np.cos(current_yaw) * self.error_x  + np.sin(current_yaw) * self.error_y
        ey = -np.sin(current_yaw) * self.error_x  + np.cos(current_yaw) * self.error_y
        ez = self.error_z



        # define the desired yaw as the yaw that will transform the x axis to point at the waypoint
        desired_yaw = np.arctan2(self.error_y, self.error_x) 

        # Yaw error: difference between current yaw and desired yaw
        yaw_error = desired_yaw - current_yaw

        # Normalize yaw error to [-pi, pi] range
        if yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        elif yaw_error < -np.pi:
            yaw_error += 2 * np.pi

        # Proportional control for position (x, y) and yaw
        self.x_control = self.Kp_position * ex
        self.y_control = self.Kp_position * ey
        self.z_control = self.Kp_position * ez
        self.yaw_control = self.Kp_yaw * yaw_error

    def send_sim_control(self):
        # Populate the TwistStamped
        if self.invoked:
            self.velocity_command.twist.linear.x = self.x_control  
            self.velocity_command.twist.linear.y = self.y_control  
            self.velocity_command.twist.linear.z = self.z_control 
            self.velocity_command.twist.angular.x= self.roll_control
            self.velocity_command.twist.angular.y =self.pitch_control
            self.velocity_command.twist.angular.z =self.yaw_control  
        else:
            self.velocity_command.twist.linear.x = 0
            self.velocity_command.twist.linear.y = 0  
            self.velocity_command.twist.linear.z = 0 
            self.velocity_command.twist.angular.x = 0
            self.velocity_command.twist.angular.y = 0
            self.velocity_command.twist.angular.z = 0  

        # Publish the message
        self.pub1.publish(self.velocity_command)
    
    def reached_waypoint_position(self):
        waypoint_distance = np.sqrt((self.current_pose.pose.position.x - self.current_waypoint.pose.position.x)**2 + (self.current_pose.pose.position.y - self.current_waypoint.pose.position.y)**2 + (self.current_pose.pose.position.z - self.current_waypoint.pose.position.z)**2)               
        return waypoint_distance < self.tolerance
    

def main(): 
    # Initialize the ROS node
    rospy.init_node('waypoint_follower')
    
    # initialize motion controller
    controller = MotionControl()
    
    while not rospy.is_shutdown():
        
        if controller.invoked:
            rospy.loginfo_throttle(30,"controller is active")
            # rospy.loginfo("controller is turned on")
            # rospy.loginfo("The current z position is:\n " + str(controller.current_pose.pose.position.z)) 
            controller.get_current_waypoint()
            # rospy.loginfo("The current z waypoint position is:\n " + str(controller.current_waypoint.pose.position.z)) 
            controller.calculate_control()
            # rospy.loginfo("The current x control is:\n " + str(controller.x_control)) 
            controller.send_sim_control()    

            if controller.reached_waypoint_position():
                if controller.waypoint_index < controller.num_waypoints-1:
                    rospy.loginfo(f"Reached waypoint {controller.waypoint_index +1}")
                    controller.waypoint_index +=1
                else:
                    rospy.loginfo(f"Reached the last waypoint, holding postion at waypoint {controller.waypoint_index +1}")
                

        elif controller.invoked == False:
            # rospy.loginfo("controller is turned off")
            rospy.loginfo_throttle(30,"controller is inactive")
            controller.invoked = False
            controller.x_control = 0.0
            controller.send_sim_control()
        else:
            rospy.loginfo("controller is weird")

        rospy.sleep(0.1)    

    
if __name__ == "__main__":
    main()