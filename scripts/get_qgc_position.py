#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from std_msgs.msg import Bool, Int8
from navpy import lla2ned

class MotionControl:
    def __init__(self): 
        #  if used for simulating set to true, if used for hardware set to false
        self.sim = True 

        # turn contrlller on and off, off just sends control when simulaing = 0
        self.invoked = False

        # control clip
        # self.control_clip = (1400, 1600)    
        self.control_clip = (1300, 1800) 

        # rates and frequencies
        self.frequency =10 
        self.rate = rospy.Rate(self.frequency)  # 10 Hz
        self.dt = 1 /self.frequency


        # current pose 
        self.current_pose = PoseStamped() # stores the current pose

        # for holding a pose between waypoints basically like hitting pause 
        self.hold_pose = False
        self.hold_pose_waypoint = Pose()
        
        
        # waypoint management 
        self.waypoints = PoseArray()    # stores the list of waypoints
        self.current_waypoint = PoseStamped()   # stores the current waypoint
        self.waypoint_index = 0     # stores the index of the current waypoint
        self.num_waypoints = 0  # srores the number of waypoints


        #Controllers gains
        # x-y gains
        self.Kp_xy = 0.8 # Proportional gain for position control (x, y)
        self.Kd_xy = 0.1 # Derivative gain for position control (x, y)
        
        # z gains
        self.Kp_z = 0.8 # Proportional gain for depth control z
        self.Kd_z = 0.1 # Derivative gain for position control z

        # yaw gains 
        self.Kp_yaw = 0.8 #  Proportional gain for yaw control
        self.Kd_yaw = 0.1 #  Proportional gain for yaw control



        # Error management 
        # Error from waypoint - current pose for each degree of freedom
        self.error_x = 0
        self.error_y = 0
        self.error_z = 0
        self.error_roll = 0
        self.error_pitch = 0
        self.error_yaw = 0

        # Previous error values for derivative calculation
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        self.prev_error_yaw = 0.0


    
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
        self.position_threshold = 0.2
        degrees_threshold = 3
        self.yaw_threshold = degrees_threshold * np.pi / 180
    


        # Simulated Control values used in the kinematic simulation only 
        self.velocity_command = TwistStamped()
        self.velocity_command.header.frame_id = "base_link"  # Example frame id



        # creating subscribers
        self.sub1 = rospy.Subscriber('motion_control_state',Bool, self.on_off_callback)
        self.sub2 = rospy.Subscriber('/dvl/local_position', PoseWithCovarianceStamped, self.position_callback)
        self.sub3 = rospy.Subscriber('target_waypoints_list', PoseArray, self.waypoint_list_callback)
        self.sub4 = rospy.Subscriber('wapoint_index_reset', Int8, self.waypoint_index_callback)
        self.sub5 = rospy.Subscriber('hold_pose', Bool, self.hold_pose_callback)
        self.sub6 = rospy.Subscriber('hold_pose_waypoint', PoseStamped, self.hold_pose_waypoint_callback)

        # creating publishers, for the simulation
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

    # desired velocity output
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
        
        # get time step and start recording new time...feel like there is a better way
        # self.dt = rospy.get_time() - self.time
        # self.time = rospy.get_time()

        # Calculate derivatives of error
        dedx = (ex - self.prev_error_x) / self.dt
        dedy = (ey - self.prev_error_y) / self.dt
        dedz = (ez - self.prev_error_z) / self.dt
        dedyaw = (self.error_yaw - self.prev_error_yaw) / self.dt
        

        # Update previous errors
        self.prev_error_x = ex
        self.prev_error_y = ey
        self.prev_error_z = ez
        self.prev_error_yaw = self.error_yaw

        # Proportional-Derivative control for position (x, y, z) and yaw
        self.x_control = self.Kp_xy * ex + self.Kd_xy * dedx
        self.y_control = self.Kp_xy * ey + self.Kd_xy * dedy
        self.z_control = self.Kp_z * ez + self.Kd_z * dedz
        self.yaw_control = self.Kp_yaw * self.error_yaw + self.Kd_yaw * dedyaw


    
        # if self.current_waypoint == 4:
        # rospy.loginfo(f"Error in NED frame: x,y,yaw: {self.error_x}, {self.error_y}, {self.error_yaw}")
        # rospy.loginfo(f"Error in Body frame: x,y,yaw: {ex}, {ey}, {self.error_yaw}")
        # rospy.loginfo(f"control signal: x,y,yaw: {self.z_control}, {self.y_control}, {self.yaw_control}")

    # used for my simulation
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
        self.x_pwm = int(np.clip(1500+self.x_control, self.control_clip[0],  self.control_clip[1])) #this clips it to a min or max value defined by control clip
        self.y_pwm = int(np.clip(1500+self.y_control, self.control_clip[0],  self.control_clip[1]))
        self.yaw_pwm = int(np.clip(1500+self.yaw_control, self.control_clip[0],  self.control_clip[1]))
        # rospy.loginfo("Updated control: x=%.2f, y=%.2f, yaw=%.2f", self.x_pwm, self.y_pwm, self.yaw_pwm)

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



# creating publishers, for the simulation
# self.pub1 = rospy.Publisher('', TwistStamped, queue_size=10)?

def main(): 
    # Initialize the ROS node
    rospy.init_node('qgc_position_test')

    master = mavutil.mavlink_connection('udpin:0.0.0.0:14552')

    # Wait a heartbeat before sending commands
    master.wait_heartbeat()
    
    # master.mav.command_long_send(
    #     master.target_system,
    #     master.target_component,
    #     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command
    #     0,  # Confirmation
    #     mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,  # Request specific message ID (BATTERY_STATUS)
    #     0, 0, 0, 0, 0, 0  # Unused parameters
    # )   

     # Listen for BATTERY_STATUS messages
    while not rospy.is_shutdown():
        master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # Command
                0,  # Confirmation
                mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,  # Request specific message ID (BATTERY_STATUS)
                0, 0, 0, 0, 0, 0  # Unused parameters
            )  
         
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if not msg:
            continue

        # rospy.loginfo_throttle(3, f"NED: {msg}")
    



        rospy.sleep(0.1)


        

if __name__ == "__main__":
    main()