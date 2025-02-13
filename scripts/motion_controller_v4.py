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
        # self.current_transform = 
        self.waypoints = PoseArray()

        self.current_waypoint = PoseStamped()
        self.waypoint_index = 0 
        self.num_waypoints = 0

        # for plotting
        self.x_vals = []
        self.y_vals = []
        self.yaws = []

        # P Controller Gains
        # x-y gains
        self.Kp_xy = 0.3 # Proportional gain for position control (x, y)
        self.Kd_xy = 0.3 # Derivative gain for position control (x, y)
        
        # z gains
        self.Kp_z = 0.5 # Proportional gain for depth control z
        self.Kd_z = 0.3 # Derivative gain for position control z

        # yaw gains 
        self.Kp_yaw = 0.5 #  Proportional gain for yaw control
        self.Kd_yaw = 0.5 #  Proportional gain for yaw control


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
        self.position_threshold = 0.2
        degrees_threshold = 3
        self.yaw_threshold = degrees_threshold * np.pi / 180

        
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
        #  accounts for the case where there is only 1 waypoint 
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
        # distance from waypoint (in NED frame)
        self.error_x = self.current_waypoint.pose.position.x - self.current_pose.pose.position.x
        self.error_y = self.current_waypoint.pose.position.y - self.current_pose.pose.position.y
        self.error_z = self.current_waypoint.pose.position.z - self.current_pose.pose.position.z
        

        # in NED frame
        # get the current roll, pitch, yaw
        current_roll, current_pitch, current_yaw = euler_from_quaternion([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w])

        # get the target roll, pitch, yaw 
        target_roll, target_pitch, target_yaw = euler_from_quaternion([self.current_waypoint.pose.orientation.x,self.current_waypoint.pose.orientation.y, self.current_waypoint.pose.orientation.z, self.current_waypoint.pose.orientation.w])

        # calculate the yaw error 
        self.error_yaw = target_yaw - current_yaw


        # Transform the position errors to the body frame using the robot's yaw
        ex = np.cos(current_yaw) * self.error_x  + np.sin(current_yaw) * self.error_y
        ey = -np.sin(current_yaw) * self.error_x  + np.cos(current_yaw) * self.error_y
        ez = self.error_z

        # Normalize yaw error to [-pi, pi] range
        if self.error_yaw > np.pi:
            self.error_yaw -= 2 * np.pi
        elif self.error_yaw < -np.pi:
            self.error_yaw+= 2 * np.pi


        # Proportional control for position (x, y) and yaw
        self.x_control = self.Kp_xy * ex
        self.y_control = self.Kp_xy * ey
        self.z_control = self.Kp_z * ez

        self.yaw_control = self.Kp_yaw * self.error_yaw

        # if self.current_waypoint == 4:
        # rospy.loginfo(f"Error in NED frame: x,y,yaw: {self.error_x}, {self.error_y}, {self.error_yaw}")
        # rospy.loginfo(f"Error in Body frame: x,y,yaw: {ex}, {ey}, {self.error_yaw}")
        # rospy.loginfo(f"control signal: x,y,yaw: {self.z_control}, {self.y_control}, {self.yaw_control}")

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
    
    def reached_position(self):
        waypoint_distance = np.linalg.norm((self.error_x, self.error_y, self.error_z))
        # waypoint_distance = np.sqrt((self.current_pose.pose.position.x - self.current_waypoint.pose.position.x)**2 + (self.current_pose.pose.position.y - self.current_waypoint.pose.position.y)**2 + (self.current_pose.pose.position.z - self.current_waypoint.pose.position.z)**2)               
        return waypoint_distance < self.position_threshold
    
    def reached_orientation(self):
        waypoint_orientation_distance = self.error_yaw
    
        return waypoint_orientation_distance < self.yaw_threshold
    
    def reached_waypoint(self):
        if self.reached_position() and self.reached_orientation():
            return True
        else:
            return False

'''
# Function to send control inputs to the robot (e.g., through MAVLink or a ROS topic)
def send_control(x_control, y_control, yaw_control, master):
    # Here we would send the control commands to the robot
    # Example: using MAVLink RC Override
    x_pwm = int(np.clip(1500+x_control, CONTROL_CLIP[0], CONTROL_CLIP[1]))
    y_pwm = int(np.clip(1500+y_control, CONTROL_CLIP[0], CONTROL_CLIP[1]))
    yaw_pwm = int(np.clip(1500+yaw_control, CONTROL_CLIP[0], CONTROL_CLIP[1]))
    rospy.loginfo("Updated control: x=%.2f, y=%.2f, yaw=%.2f", x_pwm, y_pwm, yaw_pwm)
    set_rc_channel_pwm(5, master, pwm=x_pwm)
    set_rc_channel_pwm(6, master, pwm=y_pwm) #lateral control
    set_rc_channel_pwm(4, master, pwm=yaw_pwm)


def set_rc_channel_pwm(channel_id, master, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.
'''

def main(): 
    # Initialize the ROS node
    rospy.init_node('waypoint_follower')

    '''
    rospy.init_node('waypoint_follower', anonymous=True)
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()
    

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')
    '''

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

            # use this for hardware
            # send_control(controller.x_control, controller.y_control, controller.yaw_control, master):
              

            if controller.reached_waypoint():
                if controller.waypoint_index < controller.num_waypoints-1:
                    rospy.loginfo(f"Reached waypoint {controller.waypoint_index +1}: {controller.current_waypoint.pose.position.x}, {controller.current_waypoint.pose.position.y}, {controller.current_waypoint.pose.position.z}")
                    controller.waypoint_index +=1
                    controller.get_current_waypoint()
                    rospy.loginfo(f"Heading to waypoint {controller.waypoint_index +1}: {controller.current_waypoint.pose.position.x}, {controller.current_waypoint.pose.position.y}, {controller.current_waypoint.pose.position.z}")
                else:
                    rospy.loginfo_throttle(15,f"Reached the last waypoint, holding postion at waypoint {controller.waypoint_index +1}")
            
            # if controller.current_waypoint == 4:
            #     rospy.loginfo(f"error, x,y,yaw: {controller.error_x}, {controller.error_y}, {controller.error_yaw}")
               


        elif controller.invoked == False:
            # rospy.loginfo("controller is turned off")
            rospy.loginfo_throttle(30,"controller is inactive")
            controller.invoked = False
            controller.send_sim_control()
            
            
            # use this for hardware
            # controller.x_control = 0.0
            # controller.y_control = 0.0
            # controller.z_control = 0.0
            # controller.roll_control = 0.0
            # controller.pitch_control = 0.0
            # controller.yaw_control = 0.0
            # send_control(controller.x_control, controller.y_control, controller.yaw_control, master):
        else:
            rospy.loginfo("controller is weird")

        rospy.sleep(0.1)    

    
if __name__ == "__main__":
    main()