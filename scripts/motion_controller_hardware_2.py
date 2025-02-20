#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from std_msgs.msg import Bool, Int8, Float32MultiArray

class MotionControl:
    def __init__(self): 
        #  if used for simulating set to true, if used for hardware set to false
        self.rviz_sim = False
        self.sitl = False
        self.hardware = False

        self.algorithm = 1 #whcih control loop to use 

            

        # turn contrlller on and off, off just sends control when simulaing = 0
        self.invoked = False

        # control clip
        self.control_clip = (1200, 1800)    
        self.upper_control_clip = (1550, 1800)    
        self.lower_control_clip = (1100, 1450)    

        # rates and frequencies
        self.frequency =10 
        self.rate = rospy.Rate(self.frequency)  # 10 Hz
        self.dt = 1 /self.frequency


        # current pose 
        self.current_pose = PoseStamped() # stores the current pose
        self.last_pose = PoseStamped() # stores the last pose

        self.current_velocity =TwistStamped() #store the current velocity

        # for holding a pose between waypoints basically like hitting pause 
        self.hold_pose = False
        self.hold_pose_waypoint = Pose()
        
        
        # waypoint management 
        self.waypoints = PoseArray()    # stores the list of waypoints
        self.current_waypoint = PoseStamped()   # stores the current waypoint
        self.waypoint_index = 0     # stores the index of the current waypoint
        self.num_waypoints = 0  # srores the number of waypoints

        # controller gaines
        # x-y gains
        self.Kp_xy = 0# Proportional gain for position control (x, y)
        self.Kd_xy = 0 # Derivative gain for position control (x, y)
        self.Ki_xy = 0 # Integral gain for position control (x, y)

        # z gains
        self.Kp_z = 0 # Proportional gain for depth control z
        self.Kd_z = 0 # Derivative gain for position control z
        self.Ki_z = 0 # Integral gain for position control z

        # yaw gains 
        self.Kp_yaw = 0#  Proportional gain for yaw control
        self.Kd_yaw = 0 #  Proportional gain for yaw control
        self.Ki_yaw = 0 # Integral gain for yaw control



        # Error management 
        # Error from waypoint - current pose for each degree of freedom in NED frame
        self.error_x = 0
        self.error_y = 0
        self.error_z = 0
        self.error_roll = 0
        self.error_pitch = 0
        self.error_yaw = 0

        # # erros in body frame
        # self.ex = 0
        # self.ey = 0
        # self.ez = 0
    
        # Previous error values for derivative calculation, in NED frame
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        self.prev_error_yaw = 0.0

        # erivatives of error in NED FRAME
        self.dedx = 0
        self.dedy = 0
        self.dedz = 0
        self.dedyaw = 0

        # Accumulated error for integral calculation, in NED Sframe
        self.sum_error_x = 0.0
        self.sum_error_y = 0.0
        self.sum_error_z = 0.0
        self.sum_error_yaw = 0.0

        self.anti_windup_clip = [0,500]
    
        # Desired velocity from calculate_control
        self.x_control = 0
        self.y_control = 0
        self.z_control = 0
        self.roll_control = 0
        self.pitch_control = 0
        self.yaw_control = 0

        # pwm setpoint
        self.x_pwm = 0
        self.y_pwm = 0
        self.z_pwm = 0
        self.yaw_pwm = 0
        self.roll_pwm = 0
        self.pitch_pwm = 0


        # Waypoint Thresholds
        self.position_threshold = 0.5
        degrees_threshold = 5
        self.yaw_threshold = degrees_threshold * np.pi / 180
    


        # Simulated Control values used in the kinematic simulation only 
        self.velocity_command = TwistStamped()
        self.velocity_command.header.frame_id = "base_link"  # Example frame id
        
        self.current_velocity = TwistStamped()
        self.velocity_command.header.frame_id = "base_link"  # Example frame id



        # creating subscribers
        self.sub1 = rospy.Subscriber('motion_control_state',Bool, self.on_off_callback)
        self.sub2 = rospy.Subscriber('/dvl/local_position', PoseWithCovarianceStamped, self.position_callback)
        self.sub3 = rospy.Subscriber('target_waypoints_list', PoseArray, self.waypoint_list_callback)
        self.sub4 = rospy.Subscriber('wapoint_index_reset', Int8, self.waypoint_index_callback)
        self.sub5 = rospy.Subscriber('hold_pose', Bool, self.hold_pose_callback)
        self.sub6 = rospy.Subscriber('hold_pose_waypoint', PoseStamped, self.hold_pose_waypoint_callback)
        self.sub7 = rospy.Subscriber('controller_gains', Float32MultiArray, self.controller_gains_callback)
        self.sub8 = rospy.Subscriber('sitl_current_velocity', TwistStamped, self.velocity_callback)

        # creating publishers, for the RViz only simulation
        self.pub1 = rospy.Publisher('velocity_command', TwistStamped, queue_size=10)

    # whenever the button is hit, toggle the controller on/off
    def on_off_callback(self,msg:Bool):
        self.invoked = msg.data
        if self.invoked: 
            rospy.loginfo("Motion controller activated")
        elif not self.invoked:
            rospy.loginfo("Motion controller deactivated")

    def waypoint_index_callback(self,msg:Int8):
        self.waypoint_index = msg.data
        rospy.loginfo(f"Waypoint index reset to waypoint {self.waypoint_index+1}")
    
    def hold_pose_waypoint_callback(self,msg:PoseStamped):
        self.hold_pose_waypoint = msg.pose

    def hold_pose_callback(self,msg:Bool):
        self.hold_pose = msg.data
        # rospy.loginfo(f"Waypoint index reset to waypoint {self.waypoint_index+1}")

    def controller_gains_callback(self,msg: Float32MultiArray):
        self.Kp_xy = msg.data[0]
        self.Kd_xy = msg.data[1]
        self.Ki_xy = msg.data[2]
        
        # z gains
        self.Kp_z = msg.data[3]
        self.Kd_z = msg.data[4]
        self.Ki_z = msg.data[5]

        # yaw gains 
        self.Kp_yaw =msg.data[6]
        self.Kd_yaw = msg.data[7]
        self.Ki_yaw = msg.data[8]

        rospy.loginfo(f"Controller gains (kp,kd,ki):\nxy: ({self.Kp_xy}, {self.Kd_xy}, {self.Ki_xy})\nz: ({self.Kp_z}, {self.Kd_z}, {self.Ki_z})\nyaw: ({self.Kp_yaw}, {self.Kd_yaw}, {self.Ki_yaw})")

    def position_callback(self, msg:PoseWithCovarianceStamped):

        # Extract position (x, y) from the message
        self.current_pose.header.frame_id = msg.header.frame_id
        self.current_pose.pose.position = msg.pose.pose.position
        self.current_pose.pose.orientation= msg.pose.pose.orientation
       
        # Extract orientation (quaternion) and convert to Euler angles (roll, pitch, yaw)
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def velocity_callback(self, msg:TwistStamped):
        self.current_velocity = msg

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


    def calculate_control(self): 
        
        # THE POSITION CONTROLLER

        # Calculating errors in NED FRAME
        # distance from waypoint (in NED FRAME)
        self.error_x = self.current_waypoint.pose.position.x - self.current_pose.pose.position.x
        self.error_y = self.current_waypoint.pose.position.y - self.current_pose.pose.position.y
        self.error_z = self.current_waypoint.pose.position.z - self.current_pose.pose.position.z
        

        # in NED FRAME
        # assume roll and pitch are negligible
        # get the current roll, pitch, yaw
        current_roll, current_pitch, current_yaw = euler_from_quaternion([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w])

        # get the target roll, pitch, yaw 
        target_roll, target_pitch, target_yaw = euler_from_quaternion([self.current_waypoint.pose.orientation.x,self.current_waypoint.pose.orientation.y, self.current_waypoint.pose.orientation.z, self.current_waypoint.pose.orientation.w])

        # calculate the yaw error 
        self.error_yaw = target_yaw - current_yaw


        # Calculate derivatives of error in NED FRAME
        self.dedx = (self.error_x - self.prev_error_x) / self.dt
        self.dedy = (self.error_y - self.prev_error_y) / self.dt
        self.dedz = (self.error_z - self.prev_error_z) / self.dt
        self.dedyaw = (self.error_yaw - self.prev_error_yaw) / self.dt


        # calulating integral for each error in NED FRAME
        self.sum_error_x += self.calculate_integral(a = self.prev_error_x, b = self.error_x, h = self.dt)
        self.sum_error_y += self.calculate_integral(a = self.prev_error_y, b = self.error_y, h = self.dt)
        self.sum_error_z += self.calculate_integral(a = self.prev_error_z, b = self.error_z, h = self.dt)
        self.sum_error_yaw += self.calculate_integral(a = self.prev_error_yaw, b = self.error_yaw, h = self.dt)
        
        # anti windup for integral
        self.sum_error_x = np.clip(self.sum_error_x, self.anti_windup_clip[0],self.anti_windup_clip[1])
        self.sum_error_y = np.clip(self.sum_error_y, self.anti_windup_clip[0],self.anti_windup_clip[1])
        self.sum_error_z = np.clip(self.sum_error_z, self.anti_windup_clip[0],self.anti_windup_clip[1])
        self.sum_error_yaw = np.clip(self.sum_error_yaw, self.anti_windup_clip[0],self.anti_windup_clip[1])


        # Update previous errors in NED FRAME
        self.prev_error_x = self.error_x
        self.prev_error_y = self.error_y
        self.prev_error_z = self.error_z
        self.prev_error_yaw = self.error_yaw
        

        # Transforming ERRORS to BODY FRAME...note: yaw and z error are already in body frame, we assume roll and pitch are negligible
        # proportional error
        ex = np.cos(current_yaw) * self.error_x  + np.sin(current_yaw) * self.error_y
        ey = -np.sin(current_yaw) * self.error_x  + np.cos(current_yaw) * self.error_y

        # derivative error
        dedx = np.cos(current_yaw) * self.dedx  + np.sin(current_yaw) * self.dedy
        dedy = -np.sin(current_yaw) * self.dedx + np.cos(current_yaw) * self.dedy

        # integral error
        sum_ex = np.cos(current_yaw) * self.sum_error_x  + np.sin(current_yaw) * self.sum_error_y
        sum_ey = -np.sin(current_yaw) * self.sum_error_x  + np.cos(current_yaw) * self.sum_error_y

        # Normalize yaw error to [-pi, pi] range
        if self.error_yaw > np.pi:
            self.error_yaw -= 2 * np.pi
        elif self.error_yaw < -np.pi:
            self.error_yaw+= 2 * np.pi


        # Proportional-Integral-Derivative control for position x, y, z and yaw
        self.x_control = self.Kp_xy * ex + self.Kd_xy * dedx + self.Ki_xy * sum_ex
        self.y_control = self.Kp_xy * ey + self.Kd_xy * dedy + self.Ki_xy * sum_ey
        self.z_control = self.Kp_z * self.error_z + self.Kd_z * self.dedz + self.Ki_z * self.sum_error_z
        self.yaw_control = self.Kp_yaw * self.error_yaw + self.Kd_yaw * self.dedyaw + self.Ki_yaw * self.sum_error_yaw


        if self.algorithm ==2:
            # Velocity Controller
            vx_error = self.x_control - self.current_velocity.twist.linear.x 
            vy_error = self.y_control - self.current_velocity.twist.linear.y
            vz_error = self.z_control - self.current_velocity.twist.linear.z 
            vyaw_error = self.yaw_control - self.current_velocity.twist.angular.z 


            kp_v_xy = 50
            kp_v_z = 50
            kp_v_yaw = 50
    
            self.x_control = kp_v_xy*vx_error
            self.y_control = kp_v_xy*vy_error
            self.z_control = kp_v_z*vz_error
            self.yaw_control = kp_v_yaw*vyaw_error


         


    # used for my RViz only simulation mode
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
    
    def set_pwm(self):
        if self.algorithm ==1:

            self.x_pwm = int(np.clip(1500+self.x_control, self.control_clip[0],  self.control_clip[1]))
            self.y_pwm = int(np.clip(1500+self.y_control, self.control_clip[0],  self.control_clip[1]))
            self.z_pwm = int(np.clip(1500+self.z_control, self.control_clip[0],  self.control_clip[1]))
            self.yaw_pwm = int(np.clip(1500+self.yaw_control, self.control_clip[0],  self.control_clip[1]))

        if self.algorithm ==2:

            self.x_pwm = int(np.clip(1500+self.x_control, self.vx_control_clip[0],  self.vx_control_clip[1]))
            self.y_pwm = int(np.clip(1500+self.y_control, self.vx_control_clip[0],  self.vx_control_clip[1]))
            self.z_pwm = int(np.clip(1500+self.z_control, self.vx_control_clip[0],  self.vx_control_clip[1]))
            self.yaw_pwm = int(np.clip(1500+self.yaw_control, self.vx_control_clip[0], self.vx_control_clip[1]))


    def reached_position(self):
        waypoint_distance = np.linalg.norm((self.error_x, self.error_y, self.error_z))
        return waypoint_distance < self.position_threshold
    
    def reached_orientation(self):
        waypoint_orientation_distance = self.error_yaw
    
        return waypoint_orientation_distance < self.yaw_threshold
    
    def reached_waypoint(self):
        if self.reached_position() and self.reached_orientation():
            return True
        else:
            return False

    def set_test_mode(self, mode):
        # rviz only
        if self.algorithm ==1:
            if mode == "rviz": 
                self.rviz_sim = True
                self.sitl = False
                self.hardware = False
                # x-y gains
                self.Kp_xy = 0.8 # Proportional gain for position control (x, y)
                self.Kd_xy = 0.1 # Derivative gain for position control (x, y)
                self.Ki_xy = 0 # Integral gain for position control (x, y)

                # z gains
                self.Kp_z = 0.8 # Proportional gain for depth control z
                self.Kd_z = 0.1 # Derivative gain for position control z
                self.Ki_z = 0 # Integral gain for position control z

                # yaw gains 
                self.Kp_yaw = 0.8 #  Proportional gain for yaw control
                self.Kd_yaw = 0.1 #  Proportional gain for yaw control
                self.Ki_yaw = 0 # Integral gain for yaw control


            elif mode == "sitl": 
                self.rviz_sim = False
                self.sitl = True
                self.hardware = False
                # x-y gains
                self.Kp_xy = 10 # Proportional gain for position control (x, y)
                self.Kd_xy = 0 # Derivative gain for position control (x, y)
                self.Ki_xy = 0 # Integral gain for position control (x, y)

                # z gains
                self.Kp_z = 10 # Proportional gain for depth control z
                self.Kd_z = 0 # Derivative gain for position control z
                self.Ki_z = 0 # Integral gain for position control z

                # yaw gains 
                self.Kp_yaw = 10 #  Proportional gain for yaw control
                self.Kd_yaw = 0 #  Proportional gain for yaw control
                self.Ki_yaw = 0 # Integral gain for yaw control

                # control clip
                self.control_clip = (1200, 1800)    
                self.upper_control_clip = (1550, 1800)    
                self.lower_control_clip = (1100, 1450)
                self.anti_windup_clip = [0,500]

            elif mode == "hardware":
                self.rviz_sim = False 
                self.sitl = False
                self.hardware = True

                # x-y gains
                self.Kp_xy = 10 # Proportional gain for position control (x, y)
                self.Kd_xy = 0 # Derivative gain for position control (x, y)
                self.Ki_xy = 0 # Integral gain for position control (x, y)

                # z gains
                self.Kp_z = 10 # Proportional gain for depth control z
                self.Kd_z = 0 # Derivative gain for position control z
                self.Ki_z = 0 # Integral gain for position control z

                # yaw gains 
                self.Kp_yaw = 10 #  Proportional gain for yaw control
                self.Kd_yaw = 0 #  Proportional gain for yaw control
                self.Ki_yaw = 0 # Integral gain for yaw control

                # control clip
                self.control_clip = (1200, 1800)    
                self.upper_control_clip = (1550, 1800)    
                self.lower_control_clip = (1100, 1450)   
                self.anti_windup_clip = [0,500] 

        
        elif self.algorithm ==2:
            if mode == "rviz": 
                self.rviz_sim = True
                self.sitl = False
                self.hardware = False
                # x-y gains
                self.Kp_xy = 0.8 # Proportional gain for position control (x, y)
                self.Kd_xy = 0.1 # Derivative gain for position control (x, y)
                self.Ki_xy = 0 # Integral gain for position control (x, y)

                # z gains
                self.Kp_z = 0.8 # Proportional gain for depth control z
                self.Kd_z = 0.1 # Derivative gain for position control z
                self.Ki_z = 0 # Integral gain for position control z

                # yaw gains 
                self.Kp_yaw = 0.8 #  Proportional gain for yaw control
                self.Kd_yaw = 0.1 #  Proportional gain for yaw control
                self.Ki_yaw = 0 # Integral gain for yaw control


            elif mode == "sitl": 
                self.rviz_sim = False
                self.sitl = True
                self.hardware = False
                # x-y gains
                self.Kp_xy = 5 # Proportional gain for position control (x, y)
                self.Kd_xy = 0 # Derivative gain for position control (x, y)
                self.Ki_xy = 0 # Integral gain for position control (x, y)

                # z gains
                self.Kp_z = 0 # Proportional gain for depth control z
                self.Kd_z = 0 # Derivative gain for position control z
                self.Ki_z = 0 # Integral gain for position control z

                # yaw gains 
                self.Kp_yaw = 5 #  Proportional gain for yaw control
                self.Kd_yaw = 0 #  Proportional gain for yaw control
                self.Ki_yaw = 0 # Integral gain for yaw control
                # control clip
                self.control_clip = (-2, 2)  
                self.vx_control_clip = (1200,1800)  
                self.upper_control_clip = (1550, 1800)    
                self.lower_control_clip = (1100, 1450)   
                self.anti_windup_clip = [0,500] 

            elif mode == "hardware":
                self.rviz_sim = False 
                self.sitl = False
                self.hardware = True

                # x-y gains
                self.Kp_xy = 10 # Proportional gain for position control (x, y)
                self.Kd_xy = 0 # Derivative gain for position control (x, y)
                self.Ki_xy = 0 # Integral gain for position control (x, y)

                # z gains
                self.Kp_z = 10 # Proportional gain for depth control z
                self.Kd_z = 0 # Derivative gain for position control z
                self.Ki_z = 0 # Integral gain for position control z

                # yaw gains 
                self.Kp_yaw = 10 #  Proportional gain for yaw control
                self.Kd_yaw = 0 #  Proportional gain for yaw control
                self.Ki_yaw = 0 # Integral gain for yaw control
                # control clip
                self.control_clip = (1200, 1800)    
                self.upper_control_clip = (1550, 1800)    
                self.lower_control_clip = (1100, 1450)   
                self.anti_windup_clip = [0,500] 
            
    def calculate_integral(self,a,b,h, method="trapezoidal"):
        # a = previous error
        # b = current error
        # h = time step

        if method == "trapezoidal":
            area = h*(a+b)/2
        return area
    
    def reset_errors(self):
        # previous error
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        self.prev_error_yaw = 0.0

         # erivatives of error in NED FRAME
        self.dedx = 0
        self.dedy = 0
        self.dedz = 0
        self.dedyaw = 0

        # Accumulated error for integral calculation, in NED frame
        self.sum_error_x = 0.0
        self.sum_error_y = 0.0
        self.sum_error_z = 0.0
        self.sum_error_yaw = 0.0

# Function to send control inputs to the robot (e.g., through MAVLink or a ROS topic)
def send_control(x_pwm, y_pwm, z_pwm, yaw_pwm, master):
    set_rc_channel_pwm(3, master, pwm= z_pwm) #throttle or depth
    set_rc_channel_pwm(4, master, pwm=yaw_pwm) # yaw
    set_rc_channel_pwm(5, master, pwm=x_pwm)  # forward
    set_rc_channel_pwm(6, master, pwm=y_pwm) # lateral control
   

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


def main(): 
    # Initialize the ROS node
    rospy.init_node('waypoint_follower')

    # initialize motion controller
    controller = MotionControl()

    controller.algorithm = 2

    # if mode = rviz use kinematic_sim.launch
    controller.set_test_mode(mode="sitl")   

    if controller.sitl or controller.hardware: 
        rospy.loginfo_once('Controller is in hardware mode')
    
        # rospy.init_node('waypoint_follower', anonymous=True)
        master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
        # Wait a heartbeat before sending commands
        master.wait_heartbeat()
        

        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1,
            1, 0, 0, 0, 0, 0, 0)

        rospy.loginfo("Waiting for the vehicle to arm")
        master.motors_armed_wait()
        rospy.loginfo('Armed!')
    else:
        rospy.loginfo_once('Controller is in simulation mode')
    
    while not rospy.is_shutdown():
        
        if controller.invoked:
            rospy.loginfo_throttle(30,"controller is active")
           
            if controller.hold_pose:
                controller.current_waypoint.pose = controller.hold_pose_waypoint
            else:
                controller.get_current_waypoint()
                

            
            controller.calculate_control()


            if controller.sitl or controller.hardware:

            
                controller.set_pwm()
    
                send_control(controller.x_pwm,controller.y_pwm, controller.z_pwm, controller.yaw_pwm, master)
                # rospy.loginfo_throttle(5, "current x,y,z,yaw: {controller.current_pose.pose.position.x}, {controller.current_pose.pose.position.y}, {controller.current_pose.pose.position.z}, {controller.current_pose.pose.orientation.z}")
                rospy.loginfo_throttle(2,f"current waypoint position:\n {controller.current_waypoint.pose.position}")

                rospy.loginfo_throttle(2, 
                                        f"current x,y,z: {controller.current_pose.pose.position.x:.2f}, "
                                        f"{controller.current_pose.pose.position.y:.2f}, "
                    
                                        f"{controller.current_pose.pose.position.z:.2f}")
                    
                rospy.loginfo_throttle(2,"Control (pwm): x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",controller.x_pwm, controller.y_pwm, controller.z_pwm, controller.yaw_pwm) 

                rospy.loginfo_throttle(2,"Velocity: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",controller.current_velocity.twist.linear.x, controller.current_velocity.twist.linear.y, controller.current_velocity.twist.linear.z, controller.current_velocity.twist.angular.z) 

    
                    
            elif controller.rviz_sim: 
                controller.send_sim_control() 


            if controller.hold_pose:
                rospy.loginfo_throttle(15,f"Holding postion")
            elif controller.reached_waypoint():
                if controller.waypoint_index < controller.num_waypoints-1:
                    rospy.loginfo(f"Reached waypoint {controller.waypoint_index +1}: {controller.current_waypoint.pose.position.x}, {controller.current_waypoint.pose.position.y}, {controller.current_waypoint.pose.position.z}")
                    controller.waypoint_index +=1

                    rospy.loginfo(f"Error Windup: ({controller.sum_error_x}, {controller.sum_error_y}, {controller.sum_error_z}, {controller.sum_error_yaw})")
                    # controller.reset_errors()
                    # rospy.loginfo(f"Reset errors")


                    controller.get_current_waypoint()
                    rospy.loginfo(f"Heading to waypoint {controller.waypoint_index +1}: {controller.current_waypoint.pose.position.x}, {controller.current_waypoint.pose.position.y}, {controller.current_waypoint.pose.position.z}")
                else:
                    rospy.loginfo_throttle(15,f"Reached the last waypoint, holding postion at waypoint {controller.waypoint_index +1}")


        elif not controller.invoked:
            rospy.loginfo_throttle(30,"controller is  disabled")

            controller.invoked = False
            if controller.rviz_sim:
                controller.send_sim_control() # sends 0 velocity if simulating
            
        
        else:
            rospy.logwarn_throttle(10,"controller is in an unexpectated state")

        rospy.sleep(0.1)    

    
if __name__ == "__main__":
    main()