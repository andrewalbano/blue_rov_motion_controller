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
        self.current_pose = PoseStamped()
        self.waypoints = PoseArray()

        self.current_waypoint = PoseStamped()
        self.waypoint_index = 0 

        # for plotting
        self.x_vals = []
        self.y_vals = []
        self.yaws = []

        # P Controller Gains
        self.Kp_position = 0.1 # Proportional gain for position control (x, y)
        self.Kp_yaw = 1     # Proportional gain for yaw control

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
        # self.waypoint_distance = None

        # turn on or off
        self.invoked = Bool()
        self.invoked = False

        self.frequency =10 
        self.rate = rospy.Rate(self.frequency)  # 10 Hz

        # creating subscribers
        self.sub1 = rospy.Subscriber('motion_control_state', Bool, self.on_off_callback)
        self.sub2 = rospy.Subscriber('current_state_test', PoseWithCovarianceStamped, self.position_callback)
        # self.sub1 = rospy.Subscriber('/dvl/local_position', PoseWithCovarianceStamped, self.position_callback)
        self.sub3 = rospy.Subscriber('target_waypoints_list', PoseArray, self.waypoint_list_callback)

        # creating publishers
        self.pub1 = rospy.Publisher('velocity_command', TwistStamped, queue_size=10)

    def on_off_callback(self,msg:Bool):
        self.invoked = msg
        # if self.invoked:
        #     rospy.loginfo("Motion controller turned on")
        # else:
        #     rospy.loginfo("Motion controller turned off")
        # global controller_on
        # if msg:
        #     controller_on = True
        #     rospy.loginfo("Motion controller turned on")
        # elif not msg:
        #     controller_on= False 
        #     rospy.loginfo("Motion controller turned off")

    def position_callback(self, msg:PoseWithCovarianceStamped):

        # Extract position (x, y) from the message
        self.current_pose.header.frame_id = msg.header.frame_id
        self.current_pose.pose.position = msg.pose.pose.position
        self.current_pose.pose.orientation= msg.pose.pose.orientation
       
        # Extract orientation (quaternion) and convert to Euler angles (roll, pitch, yaw)
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # storing in array for plotting via class instead of global, may not need this because of rviz
        self.x_vals.append(self.current_pose.pose.position.x)
        self.y_vals.append(self.current_pose.pose.position.y)
        self.yaws.append(yaw)
    
    def waypoint_list_callback(self, msg:PoseArray):
        self.waypoints = msg

    def get_current_waypoint(self):
        self.current_waypoint.header.frame_id = self.waypoints.header.frame_id
        self.current_waypoint.pose = self.waypoints.poses[self.waypoint_index]

    def update_plot(self):
        plt.ion()  # Turn on interactive mode
        fig, ax = plt.subplots()

        while not rospy.is_shutdown():
            if len(self.x_vals) > 0:
            
                ax.clear()  # Clear the previous plot
                
                # Plot the robot's trajectory (x, y)
                ax.plot(self.x_vals[-50:], self.y_vals[-50:], 'b-', label="Robot Path")
                
                # Plot the robot's orientation (yaw) as a direction indicator
                ax.arrow(x=self.x_vals[-1], y=self.y_vals[-1], dx=0.1 * np.cos(self.yaws[-1]), dy=0.1 * np.sin(self.yaws[-1]),
                            head_width=0.1, head_length=0.1, fc='r', ec='r', label="Yaw Direction")

                for wp in self.waypoints: 
                    ax.scatter(wp[0], wp[1], s=4, c='green')

                # Set axis limits and labels
                ax.set_xlim(-5, 5)  # Set x-axis limit (adjust as needed)
                ax.set_ylim(-5, 5)  # Set y-axis limit (adjust as needed)
                ax.set_xlabel("X (meters)")
                ax.set_ylabel("Y (meters)")
                ax.set_title("Robot's Position and Yaw")
                ax.legend()

                # Redraw the plot
                plt.draw()
                plt.pause(0.1)  # Pause to update the plot``
    
    def calculate_control(self): # default to first waypoint

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
        self.yaw_control = self.Kp_yaw * yaw_error

    # Function to check if the robot has reached the waypoint (within a tolerance)
    def reached_waypoint(self):
        waypoint_distance = np.sqrt((self.current_pose.pose.position.x - self.current_waypoint.pose.position.x)**2 + (self.current_pose.pose.position.y - self.current_waypoint.pose.position.y)**2)               
        return waypoint_distance < self.tolerance
    

    def send_sim_control(self):
        # Populate the TwistStamped
        if self.invoked:
            self.velocity_command.twist.linear.x = self.x_control  
            self.velocity_command.twist.linear.y = self.y_control  
            self.velocity_command.twist.linear.z = self.z_control 
            self.velocity_command.twist.angular.x=self.roll_control
            self.velocity_command.twist.angular.y =self.pitch_control
            self.velocity_command.twist.angular.z =self.yaw_control  
        else:
            self.velocity_command.twist.linear.x = 0
            self.velocity_command.twist.linear.y = 0  
            self.velocity_command.twist.linear.z = 0 
            self.velocity_command.twist.angular.x= 0
            self.velocity_command.twist.angular.y = 0
            self.velocity_command.twist.angular.z = 0  

        # Publish the message
        self.pub1.publish(self.velocity_command)
        # rospy.loginfo(self.velocity_command)
    
    def listener(self):

        if self.invoked == True:
        # if controller_on == True:   
            # rospy.loginfo_throttle(60,"Motion controller is enabled")

            # update current waypoint every iteration and print current waypoint every 10 seconds
            self.get_current_waypoint()
            # rospy.loginfo_throttle(10, "current waypoint: \n", controller.current_waypoint)

            # calculate the new values to send
            self.calculate_control()

            # # Check if we need to rotate to the correct heading first
            # # If yaw control (heading) is significant, we focus on turning first
            # if abs(controller.yaw_control) > 15.0:  # Threshold for turning (adjust as necessary)
            #     # Prioritize turning to the correct heading
            #     rospy.loginfo(f"Turning to heading. Current yaw error: {controller.yaw_control}")
                
            #     # send_control(0, 0, yaw_control, master)  # Only send yaw control (rotation)
            # else:
            #     # Once the heading is correct, translate towards the waypoint
            #     rospy.loginfo(f"Yaw aligned. Translating towards waypoint.")
            #     # send_control(x_control, y_control, 0, master)  # Only send translation control

            self.send_sim_control()

            # if the wayppoint is reached increment waypoint index, should consider using .pop 
            if self.reached_waypoint():
                try:
                    if self.waypoint_index < len(self.waypoints.poses):
                        rospy.loginfo(f"Reached waypoint {self.waypoint_index +1}")
                        self.waypoint_index +=1
                    
                    else:
                        rospy.loginfo("Holding postion and orientation at final waypoint") 
                        # self.invoked = False # temporary for testing turns off the motion self once all goals are reached
                except:
                    rospy.loginfo("Holding postion and orientation at final waypoint") 
                       
        else:
            # rospy.loginfo("self_state: ", self.invoked)
            # rospy.loginfo_throttle(30,"Motion self is disabled") 
            self.x_control = 0
            self.y_control = 0
            self.z_control = 0
            self.roll_control = 0
            self.pitch_control = 0
            self.yaw_control = 0
            self.send_sim_control()


        rospy.sleep(0.1)    
    # send control signals 
    '''
    # Function to send control inputs to the robot (e.g., through MAVLink or a ROS topic)
    def send_control(self, master):
        # Here we would send the control commands to the robot
        # Example: using MAVLink RC Override
        x_pwm = int(np.clip(1500+self.x_control, CONTROL_CLIP[0], CONTROL_CLIP[1]))
        y_pwm = int(np.clip(1500+self.y_control, CONTROL_CLIP[0], CONTROL_CLIP[1]))
        yaw_pwm = int(np.clip(1500+self.yaw_control, CONTROL_CLIP[0], CONTROL_CLIP[1]))
        rospy.loginfo("Updated control: x=%.2f, y=%.2f, yaw=%.2f", x_pwm, y_pwm, yaw_pwm)
        set_rc_channel_pwm(5, master, pwm=x_pwm)
        set_rc_channel_pwm(6, master, pwm=y_pwm) #lateral control
        set_rc_channel_pwm(4, master, pwm=yaw_pwm)

    def set_rc_channel_pwm(self, channel_id, master, pwm=1500):
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


# generate patterns
# def create_semicircle(center, radius, arc):

def generate_lawnmower_pattern(width, height, strip_width, spacing):
    """
    Generate a lawnmower pattern for a given width, height, strip width, and spacing.

    Parameters:
    - width: The width of the area to be covered (meters).
    - height: The height of the area to be covered (meters).
    - strip_width: The width of each mowing strip (meters).
    - spacing: The vertical spacing between each mow row (meters).

    Returns:
    - A 2D numpy array of shape (N, 2) representing the lawnmower pattern waypoints.
    """
    # Initialize an empty list to store waypoints
    waypoints = []

    # We will generate waypoints row by row
    y_position = 0  # Start from the top of the field (y = 0)
    
    while y_position < height:
        # Calculate how many strips we can fit horizontally in this row
        num_strips = int(np.ceil(width / strip_width))
        
        if int(y_position / spacing) % 2 == 0:
            # Left to right direction
            for i in range(num_strips):
                x_position = i * strip_width
                # Ensure x_position doesn't exceed width
                x_position = min(x_position, width)
                waypoints.append([x_position, y_position])
        else:
            # Right to left direction
            for i in range(num_strips - 1, -1, -1):
                x_position = i * strip_width
                # Ensure x_position doesn't exceed width
                x_position = min(x_position, width)
                waypoints.append([x_position, y_position])
        
        # Move down by the spacing after completing a row
        y_position += spacing
        
    # Convert the list of waypoints into a numpy array
    waypoints_array = np.array(waypoints)

    return waypoints_array

def create_semicircle(center,radius,angle,increments):
    angles = np.linspace(-angle/2,angle/2,increments)
    # Only care about X,Y, and yaw return values
    POSE = np.array([radius*np.cos(angles), radius*np.sin(angles)]) + center[:,None]
    return POSE.T

def orbit_mode(center,radius,angle,increments):
    # Modified version of create_semicircle, where the robot will always point towards the center of the arc
    angles = np.linspace(-angle/2,angle/2,increments)
    # Only care about X,Y, and yaw return values
    POSE = np.array([radius*np.cos(angles), radius*np.sin(angles), ]) + center[:,None]
    return POSE.T

'''
def listener():
        # if controller.invoked == True:
        if controller_on == True:   
            rospy.loginfo_throttle(60,"Motion controller is enabled")

            # update current waypoint every iteration and print current waypoint every 10 seconds
            controller.get_current_waypoint()
            # rospy.loginfo_throttle(10, "current waypoint: \n", controller.current_waypoint)

            # calculate the new values to send
            controller.calculate_control()

            # # Check if we need to rotate to the correct heading first
            # # If yaw control (heading) is significant, we focus on turning first
            # if abs(controller.yaw_control) > 15.0:  # Threshold for turning (adjust as necessary)
            #     # Prioritize turning to the correct heading
            #     rospy.loginfo(f"Turning to heading. Current yaw error: {controller.yaw_control}")
                
            #     # send_control(0, 0, yaw_control, master)  # Only send yaw control (rotation)
            # else:
            #     # Once the heading is correct, translate towards the waypoint
            #     rospy.loginfo(f"Yaw aligned. Translating towards waypoint.")
            #     # send_control(x_control, y_control, 0, master)  # Only send translation control

            controller.send_sim_control()

            # if the wayppoint is reached increment waypoint index, should consider using .pop 
            if controller.reached_waypoint():
                try:
                    if controller.waypoint_index < len(controller.waypoints.poses):
                        rospy.loginfo(f"Reached waypoint {controller.waypoint_index +1}")
                        controller.waypoint_index +=1
                    
                    else:
                        rospy.loginfo("Holding postion and orientation at final waypoint") 
                        # controller.invoked = False # temporary for testing turns off the motion controller once all goals are reached
                except:
                    rospy.loginfo("Holding postion and orientation at final waypoint") 
                       
        else:
            # rospy.loginfo("controller_state: ", controller.invoked)
            rospy.loginfo_throttle(30,"Motion controller is disabled") 
            controller.x_control = 0
            controller.y_control = 0
            controller.z_control = 0
            controller.roll_control = 0
            controller.pitch_control = 0
            controller.yaw_control = 0
            controller.send_sim_control()


        rospy.sleep(0.1)    
    # Signal to stop the visualization thread and close the plot
'''
def main(): 
    # Initialize the ROS node
    rospy.init_node('waypoint_follower') #, anonymous=True)

    # initializing mavutil
    '''
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
        # rospy.loginfo(controller.invoked)
        if controller.invoked:
            # rospy.loginfo("controller is on")
            # update current waypoint every iteration and print current waypoint every 10 seconds
            controller.get_current_waypoint()
            # rospy.loginfo_throttle(10, "current waypoint: \n", controller.current_waypoint)

            # calculate the new values to send
            # controller.calculate_control()

            # # Check if we need to rotate to the correct heading first
            # # If yaw control (heading) is significant, we focus on turning first
            # if abs(controller.yaw_control) > 15.0:  # Threshold for turning (adjust as necessary)
            #     # Prioritize turning to the correct heading
            #     rospy.loginfo(f"Turning to heading. Current yaw error: {controller.yaw_control}")
                
            #     # send_control(0, 0, yaw_control, master)  # Only send yaw control (rotation)
            # else:
            #     # Once the heading is correct, translate towards the waypoint
            #     rospy.loginfo(f"Yaw aligned. Translating towards waypoint.")
            #     # send_control(x_control, y_control, 0, master)  # Only send translation control
            controller.x_control = 0.1
            controller.send_sim_control()

            # # if the wayppoint is reached increment waypoint index, should consider using .pop 
            # if controller.reached_waypoint():
            #     try:
            #         if controller.waypoint_index < len(controller.waypoints.poses):
            #             rospy.loginfo(f"Reached waypoint {controller.waypoint_index +1}")
            #             controller.waypoint_index +=1
                    
            #         else:
            #             # rospy.loginfo("Holding postion and orientation at final waypoint") 
                        
            #             controller.invoked = False # temporary for testing turns off the motion self once all goals are reached
            #     except:
            #         controller.invoked = False
            #         rospy.loginfo("Holding postion and orientation at final waypoint") 
                        
        
        else:
            # rospy.loginfo("controller is off")

            controller.invoked = False
            # rospy.loginfo("controller_state: ", controller.invoked)
            # rospy.loginfo_throttle(30,"Motion controller is disabled") 
            # controller.x_control = 0
            # controller.y_control = 0
            # controller.z_control = 0
            # controller.roll_control = 0
            # controller.pitch_control = 0
            # controller.yaw_control = 0
            # controller.send_sim_control()


        rospy.sleep(0.1)    

    
if __name__ == "__main__":
    main()