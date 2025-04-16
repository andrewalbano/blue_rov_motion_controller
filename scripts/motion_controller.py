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

class MotionControl:
    def __init__(self,master, system = "hardware"): 
        self.system = system
        self.state = "Disabled"
        self.send_signal = "manual"
        self.master = master
        self.boot_time = time.time()
        self.mode = 1
        self.delta = 5
        self.target_depth = None

        self.num_errors = 15 # stores the last n errors for integral term
        if system == "hardware": 
            # POSITION CONTROLLER
            # x-y gains
            self.Kp_x = 0.75
            self.Kd_x = 0
            self.Ki_x = 0

            self.Kp_y = 0.75
            self.Kd_y = 0
            self.Ki_y = 0

            # z gains
            self.Kp_z = 0
            self.Kd_z = 0 
            self.Ki_z = 0 

            # yaw gains 
            self.Kp_yaw = 0.1
            self.Kd_yaw = 0 
            self.Ki_yaw = 0

            # saturation 
            self.velocity_anti_windup_clip = [0,10]  # prevents the integral error from becoming too large, might need to split this up into multiple degree of freedoms
            self.linear_velocity_clip = [-0.3,0.3] #min and max velocity setpoints
            self.angular_velocity_clip = [-20*np.pi/180, 20*np.pi/180] #[-1,1]#[-10*np.pi/180, 10*np.pi/180] # min and max angular velocity setpoints
        


            # xy
            self.kp_v_x = 2000
            self.kd_v_x = 0
            self.ki_v_x = 500

            self.kp_v_y = 2000
            self.kd_v_y = 0
            self.ki_v_y = 500

            # z gains
            self.kp_v_z = 300
            self.kd_v_z = 0
            self.ki_v_z = 0

            # yaw
            self.kp_v_yaw = 2000
            self.kd_v_yaw = 0
            self.ki_v_yaw = 500

            # saturation 
            self.pwm_anti_windup_clip = [0,10]  # 500# prevents the integral error from becoming too large, might need to split this up into multiple degree of freedoms
            self.pwm_anti_windup_clip_angular = [0,10]
            self.linear_pwm_clip = [1000, 2000] #min and max pwm setpoints
            self.angular_pwm_clip = [1000, 2000] # min and max angular velocity setpoints
            self.pwm_clip = [-1000,1000]
        
        elif system  == "sitl": 
        
            # POSITION CONTROLLER
            # x-y gains
            self.Kp_x = 0.25
            self.Kd_x = 0
            self.Ki_x = 0

            self.Kp_y = 0.25
            self.Kd_y = 0
            self.Ki_y = 0

            # z gains
            self.Kp_z = 0
            self.Kd_z = 0 
            self.Ki_z = 0 

            # yaw gains 
            self.Kp_yaw = 0.1 # 0.2
            self.Kd_yaw = 0 
            self.Ki_yaw = 0

            # saturation 
            self.velocity_anti_windup_clip = [0,10]  # prevents the integral error from becoming too large, might need to split this up into multiple degree of freedoms
            self.linear_velocity_clip = [-0.3,0.3] #min and max velocity setpoints
            self.angular_velocity_clip = [-20*np.pi/180, 20*np.pi/180] #[-1,1]#[-10*np.pi/180, 10*np.pi/180] # min and max angular velocity setpoints

            # xy
            # self.kp_v_x = 200
            # self.kd_v_x = 0
            # self.ki_v_x = 150

            # self.kp_v_y = 200
            # self.kd_v_y = 0
            # self.ki_v_y = 150


            # z gains
            self.kp_v_z = 300
            self.kd_v_z = 0
            self.ki_v_z = 50

            # yaw
            self.kp_v_yaw = 200
            self.kd_v_yaw = 0
            self.ki_v_yaw = 150
            

            # # xy
            self.kp_v_x = 1800
            self.kd_v_x = 50
            self.ki_v_x = 100

            self.kp_v_y = 1800
            self.kd_v_y = 50
            self.ki_v_y = 100


            # # z gains
            # self.kp_v_z = 300
            # self.kd_v_z = 0
            # self.ki_v_z = 50

            # # yaw
            # self.kp_v_yaw = 200
            # self.kd_v_yaw = 0
            # self.ki_v_yaw = 150
            


            # saturation 
            self.pwm_anti_windup_clip = [-6,6]

            # self.pwm_anti_windup_clip = [0,10]
            self.pwm_anti_windup_clip_angular = [-20,20]
              # 500# prevents the integral error from becoming too large, might need to split this up into multiple degree of freedoms
            self.linear_pwm_clip = [1000, 2000] #min and max pwm setpoints
            self.angular_pwm_clip = [1000, 2000] # min and max angular velocity setpoints

            self.pwm_clip = [-1000,1000]
          
        elif system  == "custom sim": 
        
            # POSITION CONTROLLER
            # x-y gains
            self.Kp_x = 0.3
            self.Kd_x = 0
            self.Ki_x = 0

            self.Kp_y = 0.3
            self.Kd_y = 0
            self.Ki_y = 0

            # z gains
            self.Kp_z = 0.3
            self.Kd_z = 0 
            self.Ki_z = 0 

            # yaw gains 
            self.Kp_yaw = 0.3 # 0.2
            self.Kd_yaw = 0 
            self.Ki_yaw = 0

            # saturation 
            self.velocity_anti_windup_clip = [0,10]  # prevents the integral error from becoming too large, might need to split this up into multiple degree of freedoms
            self.linear_velocity_clip = [-0.2,0.2] #min and max velocity setpoints
            self.angular_velocity_clip = [-10*np.pi/180, 10*np.pi/180] #[-1,1]#[-10*np.pi/180, 10*np.pi/180] # min and max angular velocity setpoints

            # xy
            self.kp_v_x = 200
            self.kd_v_x = 0
            self.ki_v_x = 150

            self.kp_v_y = 200
            self.kd_v_y = 0
            self.ki_v_y = 150


            # z gains
            self.kp_v_z = 300
            self.kd_v_z = 0
            self.ki_v_z = 50

            # yaw
            self.kp_v_yaw = 200
            self.kd_v_yaw = 0
            self.ki_v_yaw = 150

            # saturation 
            self.pwm_anti_windup_clip = [0,50]
            self.pwm_anti_windup_clip_angular = [0,50]
              # 500# prevents the integral error from becoming too large, might need to split this up into multiple degree of freedoms
            self.linear_pwm_clip = [1000, 2000] #min and max pwm setpoints
            self.angular_pwm_clip = [1000, 2000] # min and max angular velocity setpoints

            self.pwm_clip = [-1000,1000]
          
        
        # set the rates and frequencies
        self.frequency = 40
        self.rate = rospy.Rate(self.frequency)  # 10 Hz
        self.dt = 1/self.frequency


        # storing pose information for current pose and last pose
        self.current_pose = PoseStamped()
        self.current_roll = 0
        self.current_pitch = 0
        self.current_yaw = 0
        self.attitude_transform = np.eye(3)
        self.heading = 0
        

        self.last_pose = PoseStamped()

        #  storing information about current velocity 
        self.current_velocity = Twist()
           

        # waypoint management 
        self.current_waypoint = None
        # self.current_waypoint = PoseStamped()   # stores the current waypoint
        self.waypoint_yaw = 0
        self.waypoint_index = 0     # stores the index of the current waypoint

        self.lookahead_waypoint = PoseStamped()

        # # Waypoint Thresholds
        # self.position_threshold = 0.5   
        # self.z_threshold = 1
        # degrees_threshold = 10
        # self.yaw_threshold = degrees_threshold * np.pi / 180



        # Control setpoints

        # storing the setpoints for publishing to topic
        self.pwm_setpoint = Twist()
        self.velocity_setpoint = TwistStamped()
        self.velocity_setpoint.header.frame_id = "base_link"
        

        # Velocity setpoint
        self.vx_setpoint = 0
        self.vy_setpoint = 0
        self.vz_setpoint = 0
        self.vyaw_setpoint = 0


        # pwm setpoint
        self.x_pwm = 0
        self.y_pwm = 0
        self.z_pwm = 500 # 500 when in manual mode , 0 when in rc channel pwm mode
        self.yaw_pwm = 0
  

    
        # storing errors
        # calculate position errors
        self.error_x = 0
        self.error_y = 0
        self.error_z = 0
        self.error_yaw = 0

        # Previous error values for derivative calculations for velocity setpoint
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        self.prev_error_yaw = 0.0

        # derivatives of error for velocity
        self.dedx = 0
        self.dedy = 0
        self.dedz = 0
        self.dedyaw = 0

        # Accumulated error for integral calculation for velocity
        self.sum_error_x = 0.0
        self.sum_error_y = 0.0
        self.sum_error_z = 0.0
        self.sum_error_yaw = 0.0



        # calculate velocity errors
        self.error_vx = 0
        self.error_vy = 0
        self.error_vz = 0
        self.error_vyaw = 0

        # Previous error values for derivative calculations for velocity setpoint
        self.prev_error_vx = 0.0
        self.prev_error_vy = 0.0
        self.prev_error_vz = 0.0
        self.prev_error_vyaw = 0.0

        # derivatives of error for velocity
        self.vdedx = 0
        self.vdedy = 0
        self.vdedz = 0
        self.vdedyaw = 0

        # Accumulated error for integral calculation for velocity
        self.sum_error_vx = 0.0
        self.sum_error_vy = 0.0
        self.sum_error_vz = 0.0
        self.sum_error_vyaw = 0.0

        self.last_n_errors_vx = [] 
        self.last_n_errors_vy = [] 
        self.last_n_errors_vz = [] 
        self.last_n_errors_vyaw = [] 

    
        
        self.pub16= rospy.Publisher('v_magnitude_setpoint', Float32, queue_size=1)
        self.pub17= rospy.Publisher('v_magnitude', Float32, queue_size=1)

        self.pub2 = rospy.Publisher('velocity_setpoint', TwistStamped, queue_size=1)
        self.pub3 = rospy.Publisher('pwm_setpoint', Twist, queue_size=1)
        self.pub4= rospy.Publisher('vx_setpoint', Float32, queue_size=1)
        self.pub5= rospy.Publisher('vy_setpoint', Float32, queue_size=1)
        self.pub6= rospy.Publisher('vz_setpoint', Float32, queue_size=1)
        self.pub7= rospy.Publisher('vyaw_setpoint', Float32, queue_size=1)
        self.pub8= rospy.Publisher('x_pwm', Float32, queue_size=1)
        self.pub9= rospy.Publisher('y_pwm', Float32, queue_size=1)
        self.pub10= rospy.Publisher('z_pwm', Float32, queue_size=1)
        self.pub11= rospy.Publisher('yaw_pwm', Float32, queue_size=1)

        self.pub12= rospy.Publisher('vx', Float32, queue_size=1)
        self.pub13= rospy.Publisher('vy', Float32, queue_size=1)
        self.pub14= rospy.Publisher('vz', Float32, queue_size=1)
        self.pub15= rospy.Publisher('vyaw', Float32, queue_size=1)

        self.pub16= rospy.Publisher('x_error', Float32, queue_size=1)
        self.pub17= rospy.Publisher('y_error', Float32, queue_size=1)
        self.pub18= rospy.Publisher('yaw_error', Float32, queue_size=1)

        self.pub19= rospy.Publisher('vx_error', Float32, queue_size=1)
        self.pub20= rospy.Publisher('vy_error', Float32, queue_size=1)
        self.pub21= rospy.Publisher('vyaw_error', Float32, queue_size=1)

        self.pub22= rospy.Publisher('vx_accumulated_error', Float32, queue_size=1)
        self.pub23= rospy.Publisher('vy_accumulated_error', Float32, queue_size=1)
        self.pub24= rospy.Publisher('vyaw_accumulated_error', Float32, queue_size=1)
       
       


        
        # creating subscribers
        self.sub2 = rospy.Subscriber('/state', PoseWithCovarianceStamped, self.position_callback)
        self.sub7 = rospy.Subscriber('controller_gains', Float32MultiArray, self.gui_info_callback)
        # self.sub8 = rospy.Subscriber('/dvl/twist', TwistStamped, self.velocity_callback)
        self.sub8 = rospy.Subscriber('/twist', TwistWithCovarianceStamped, self.velocity_callback)
        self.sub9 = rospy.Subscriber('motion_controller_state', String, self.controller_state_callback)

        # switch based on whichever is being used for the waypoint

        # self.waypoint_sub = rospy.Subscriber('current_goal_waypoint', PoseStamped, self.current_waypoint_callback)
        self.waypoint_sub = rospy.Subscriber('lookahead_waypoint', PoseStamped, self.current_waypoint_callback)
    
       
        
        
    # might need to initialize certain params for the state that im in 
    def controller_state_callback(self, msg:String):
        self.last_state = self.state
        self.state = msg.data
        rospy.loginfo(f"{self.state} mode is active")
        if self.state == "Disabled":
            self.erase_target_depth()
            self.current_waypoint =None

        elif self.state == "Initialized": 
            self.target_depth = self.current_pose.pose.position.z  
            self.current_waypoint =None
            self.erase_target_depth()


        elif self.state == "velocity":
            self.erase_target_depth()
            self.vx_setpoint = 0
            self.vy_setpoint  = 0
            self.vz_setpoint  = 0
            self.vyaw_setpoint  = 0

        elif self.state == "manual pwm":           
            self.erase_target_depth()
            self.x_pwm = 0
            self.y_pwm = 0
            self.z_pwm = 500
            self.yaw_pwm = 0
        

        elif self.state == "waypoint": 
            self.erase_target_depth()
            self.current_waypoint = PoseStamped()

            self.current_waypoint.pose.position.x = self.current_pose.pose.position.x
            self.current_waypoint.pose.position.y = self.current_pose.pose.position.y
            self.current_waypoint.pose.position.z = self.current_pose.pose.position.z
            self.current_waypoint.pose.orientation.x = self.current_pose.pose.orientation.x
            self.current_waypoint.pose.orientation.y = self.current_pose.pose.orientation.y    
            self.current_waypoint.pose.orientation.z = self.current_pose.pose.orientation.z   
            self.current_waypoint.pose.orientation.w = self.current_pose.pose.orientation.w              
    
        elif self.state == "Joystick":
            self.erase_target_depth()
    
    def current_waypoint_callback(self,msg:PoseStamped):
        '''function gets the current waypoint from the ros topic'''
        self.current_waypoint = PoseStamped()   # stores the current waypoint
        self.current_waypoint = msg
        _,_,self.waypoint_yaw = euler_from_quaternion([self.current_waypoint.pose.orientation.x,self.current_waypoint.pose.orientation.y, self.current_waypoint.pose.orientation.z, self.current_waypoint.pose.orientation.w])
        # rospy.loginfo("motion controller received new target waypoint")

    def gui_info_callback(self,msg: Float32MultiArray):
        ''' changes various parameters '''
        if msg.data[0]==1:
            
            self.Kp_x = msg.data[1]
            self.Kd_x = msg.data[2]
            self.Ki_x = msg.data[3]

            self.Kp_y = msg.data[4]
            self.Kd_y = msg.data[5]
            self.Ki_y = msg.data[6]
            
            # z gains
            self.Kp_z = msg.data[7]
            self.Kd_z = msg.data[8]
            self.Ki_z = msg.data[9]

            # yaw gains 
            self.Kp_yaw = msg.data[10]
            self.Kd_yaw = msg.data[11]
            self.Ki_yaw = msg.data[12]
            self.reset_position_errors()

            rospy.loginfo(f"Position controller gains (kp,kd,ki):\nx: ({self.Kp_x}, {self.Kd_x}, {self.Ki_x})\ny: ({self.Kp_y}, {self.Kd_y}, {self.Ki_y})\nz: ({self.Kp_z}, {self.Kd_z}, {self.Ki_z})\nyaw: ({self.Kp_yaw}, {self.Kd_yaw}, {self.Ki_yaw})")

        elif msg.data[0]==2:
            
            self.kp_v_x = msg.data[1]
            self.kd_v_x = msg.data[2]
            self.ki_v_x = msg.data[3]
            
            self.kp_v_y = msg.data[4]
            self.kd_v_y = msg.data[5]
            self.ki_v_y = msg.data[6]
            
            # z gains
            self.kp_v_z = msg.data[7]
            self.kd_v_z = msg.data[8]
            self.ki_v_z = msg.data[9]

            # yaw gains 
            self.kp_v_yaw = msg.data[10]
            self.kd_v_yaw = msg.data[11]
            self.ki_v_yaw = msg.data[12]

            
            self.reset_velocity_errors()

            rospy.loginfo(f"Velocity controller gains (kp,kd,ki):\nx: ({self.kp_v_x}, {self.kd_v_x}, {self.ki_v_x})\ny: ({self.kp_v_y}, {self.kd_v_y}, {self.ki_v_y})\nz: ({self.kp_v_z}, {self.kd_v_z}, {self.ki_v_z})\nyaw: ({self.kp_v_yaw}, {self.kd_v_yaw}, {self.ki_v_yaw})")

        elif msg.data[0]==3:
            self.linear_velocity_clip = [-1*msg.data[1], msg.data[1]]
            rospy.loginfo(f"Set max linear velocity to {msg.data[1]} m/s")

        elif msg.data[0]==9:
            self.angular_velocity_clip = [-1*msg.data[1]*np.pi/180, msg.data[1]*np.pi/180]

            # self.angular_velocity_clip = [-1*msg.data[1], msg.data[1]]
            # self.linear_pwm_clip = [msg.data[2],msg.data[3]]
            # self.angular_pwm_clip = [msg.data[2],msg.data[3]]

            rospy.loginfo(f"Set max angular velocity to {msg.data[1]} deg/s")

        elif msg.data[0]==4:
            self.pwm_clip = [-1*msg.data[1], msg.data[1]]
            rospy.loginfo(f"Setting max PWM signal to {msg.data[1]/1000}% of maximum")

        elif msg.data[0]==5:
            self.vx_setpoint = msg.data[1]
            self.vy_setpoint = msg.data[2]
            self.vz_setpoint = msg.data[3]
            # self.vz_setpoint = 0

            self.vyaw_setpoint = msg.data[4]*np.pi/180

            self.vx_setpoint = round(np.clip(self.vx_setpoint, self.linear_velocity_clip[0],self.linear_velocity_clip[1]),3)
            self.vy_setpoint = round(np.clip(self.vy_setpoint, self.linear_velocity_clip[0],self.linear_velocity_clip[1]),3)
            self.vz_setpoint = round(np.clip(self.vz_setpoint, self.linear_velocity_clip[0],self.linear_velocity_clip[1]),3)
            self.vyaw_setpoint = np.clip(self.vyaw_setpoint, self.angular_velocity_clip[0],self.angular_velocity_clip[1])
            self.reset_velocity_errors()
            
        
            rospy.loginfo(f"Preparing for velocity test\nvx setpoint = {self.vx_setpoint} m/s\nvy setpoint = {self.vy_setpoint} m/s\nvz setpoint = {self.vz_setpoint} m/s\nangular velocity setpoint = { round(self.vyaw_setpoint*180/np.pi,3)} deg/s = {self.vyaw_setpoint} rad/s")

        elif msg.data[0]==6:
        
            self.target_depth = msg.data[1]

            rospy.loginfo(f"Setting target depth to {self.target_depth} m below the surface")
        
        elif msg.data[0]==7:
            
            self.x_pwm = int(msg.data[1])
            self.y_pwm = int(msg.data[2])
            self.z_pwm= int(msg.data[3])
            # self.z_pwm = 500
            self.yaw_pwm = int(msg.data[4])
            rospy.loginfo("Received manual PWM signals")

    def position_callback(self, msg:PoseWithCovarianceStamped):

        # Extract position (x, y) from the message
        self.current_pose.header.frame_id = msg.header.frame_id
        self.current_pose.pose.position = msg.pose.pose.position
        self.current_pose.pose.orientation= msg.pose.pose.orientation
       
        # Extract orientation (quaternion) and convert to Euler angles (roll, pitch, yaw)
        q = msg.pose.pose.orientation
        self.current_roll,self.current_pitch,self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def velocity_callback(self, msg:TwistWithCovarianceStamped):
        # self.current_velocity = msg.twist

        # vyaw = self.current_velocity.angular.z*180/np.pi
        # self.pub12.publish(self.current_velocity.linear.x)
        # self.pub13.publish(self.current_velocity.linear.y)
        # self.pub14.publish(self.current_velocity.linear.z)
        # self.pub15.publish(vyaw)

        self.current_velocity = msg.twist.twist

        vyaw = self.current_velocity.angular.z*180/np.pi
        self.pub12.publish(self.current_velocity.linear.x)
        self.pub13.publish(self.current_velocity.linear.y)
        self.pub14.publish(self.current_velocity.linear.z)
        self.pub15.publish(vyaw)

        v_xy_magnitude = np.linalg.norm([self.current_velocity.linear.x, self.current_velocity.linear.y])
        self.pub17.publish(v_xy_magnitude)
    
    def get_lookahead_waypoint(self):
    
        self.lookahead_waypoint.header.frame_id = self.waypoints.header.frame_id
        self.lookahead_waypoint.pose = self.waypoints.poses[self.waypoint_index+1]
        # _,_,self.waypoint_yaw = euler_from_quaternion([self.current_waypoint.pose.orientation.x,self.current_waypoint.pose.orientation.y, self.current_waypoint.pose.orientation.z, self.current_waypoint.pose.orientation.w])        

        return self.lookahead_waypoint

    def publish_velocity_setpoints(self):
        self.velocity_setpoint.twist.linear.x = self.vx_setpoint
        self.velocity_setpoint.twist.linear.y = self.vy_setpoint
        self.velocity_setpoint.twist.linear.z = self.vz_setpoint
        self.velocity_setpoint.twist.angular.x= 0
        self.velocity_setpoint.twist.angular.y = 0
        self.velocity_setpoint.twist.angular.z = self.vyaw_setpoint

        vyaw = self.vyaw_setpoint *180/np.pi

        v_xy_magnitude_setpoint = np.linalg.norm([self.vx_setpoint,self.vy_setpoint])

        self.pub2.publish(self.velocity_setpoint)
        self.pub4.publish(self.vx_setpoint)
        self.pub5.publish(self.vy_setpoint)
        self.pub6.publish(self.vz_setpoint)
        self.pub7.publish(vyaw)

        self.pub16.publish(v_xy_magnitude_setpoint)

        self.pub19.publish(self.error_vx)
        self.pub20.publish(self.error_vy)
        self.pub21.publish(self.error_vyaw)

        self.pub22.publish(self.sum_error_vx)
        self.pub23.publish(self.sum_error_vy)
        self.pub24.publish(self.sum_error_vyaw)
       
    def publish_pwm_commands(self):
        self.pwm_setpoint.linear.x = self.x_pwm
        self.pwm_setpoint.linear.y = self.y_pwm
        self.pwm_setpoint.linear.z = self.z_pwm
        self.pwm_setpoint.angular.x = 0
        self.pwm_setpoint.angular.y = 0
        self.pwm_setpoint.angular.z = self.yaw_pwm

        self.pub3.publish(self.pwm_setpoint)
        self.pub8.publish(self.x_pwm)
        self.pub9.publish(self.y_pwm)
        self.pub10.publish(self.z_pwm)
        self.pub11.publish(self.yaw_pwm)

    
    #  calculating errors
    def calculate_integral(self,a,b,h, method="trapezoidal"):
        # a = previous error
        # b = current error
        # h = time step

        if method == "trapezoidal":
            area = h*(a+b)/2
        return area
    
    def reset_position_errors(self):
        # previous error
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        self.prev_error_yaw = 0.0

        # Derivatives of error in NED FRAME
        self.dedx = 0
        self.dedy = 0
        self.dedz = 0
        self.dedyaw = 0

        # Accumulated error for integral calculation, in NED frame
        self.sum_error_x = 0.0
        self.sum_error_y = 0.0
        self.sum_error_z = 0.0
        self.sum_error_yaw = 0.0

    def reset_velocity_errors(self):
    
        # previous error
        self.prev_error_vx = 0.0
        self.prev_error_vy = 0.0
        self.prev_error_vz = 0.0
        self.prev_error_vyaw = 0.0

         # erivatives of error in NED FRAME
        self.vdedx = 0
        self.vdedy = 0
        self.vdedz = 0
        self.vdedyaw = 0

        # Accumulated error for integral calculation, in NED frame
        self.sum_error_vx = 0.0
        self.sum_error_vy = 0.0
        self.sum_error_vz = 0.0
        self.sum_error_vyaw = 0.0

        # self.last_n_errors_vx = [] 
        # self.last_n_errors_vy = [] 
        # self.last_n_errors_vz = [] 
        # self.last_n_errors_vyaw = [] 

    def calculate_position_errors(self):

        # calculate the current yaw
        _,_, self.current_yaw = euler_from_quaternion([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w])

        # calculate the target yaw
        # _,_,self.target_yaw = euler_from_quaternion([self.current_waypoint.pose.orientation.x,self.current_waypoint.pose.orientation.y, self.current_waypoint.pose.orientation.z, self.current_waypoint.pose.orientation.w])

        self.target_yaw = self.waypoint_yaw 

        
        # calculate position errors in the NED FRAME
        self.error_x = self.current_waypoint.pose.position.x - self.current_pose.pose.position.x
        self.error_y = self.current_waypoint.pose.position.y - self.current_pose.pose.position.y
        self.error_z = self.current_waypoint.pose.position.z - self.current_pose.pose.position.z

        # calculate the yaw error to final orientation
        self.error_yaw = self.target_yaw - self.current_yaw
        

         # Normalize yaw error to [-pi, pi] range
        if self.error_yaw > np.pi:
            self.error_yaw -= 2 * np.pi
        elif self.error_yaw < -np.pi:
            self.error_yaw+= 2 * np.pi
        # rospy.loginfo(f"yaw error = {self.error_yaw}")

        # Calculate the derivative of the error in the NED FRAME
        self.dedx = (self.error_x - self.prev_error_x) / self.dt
        self.dedy = (self.error_y - self.prev_error_y) / self.dt
        self.dedz = (self.error_z - self.prev_error_z) / self.dt
        self.dedyaw = (self.error_yaw - self.prev_error_yaw) / self.dt # to final orientation

        # calulating integral of the error in the NED FRAME using the trapezoidal method for the area

        # calulating integral of the error in the NED FRAME using the trapezoidal method for the area
        self.sum_error_x +=self.sum_error_vx
        self.sum_error_y += self.sum_error_vy
        self.sum_error_z += self.sum_error_vz
        self.sum_error_yaw += self.sum_error_vyaw# to final orientation

        # self.sum_error_x += self.calculate_integral(a = self.prev_error_x, b = self.error_x, h = self.dt)
        # self.sum_error_y += self.calculate_integral(a = self.prev_error_y, b = self.error_y, h = self.dt)
        # self.sum_error_z += self.calculate_integral(a = self.prev_error_z, b = self.error_z, h = self.dt)
        # self.sum_error_yaw += self.calculate_integral(a = self.prev_error_yaw, b = self.error_yaw, h = self.dt) # to final orientation

        
        # anti windup for integral
        self.sum_error_x = np.clip(self.sum_error_x, self.velocity_anti_windup_clip[0],self.velocity_anti_windup_clip[1])
        self.sum_error_y = np.clip(self.sum_error_y, self.velocity_anti_windup_clip[0],self.velocity_anti_windup_clip[1])
        self.sum_error_z = np.clip(self.sum_error_z, self.velocity_anti_windup_clip[0],self.velocity_anti_windup_clip[1])
        self.sum_error_yaw = np.clip(self.sum_error_yaw, self.velocity_anti_windup_clip[0],self.velocity_anti_windup_clip[1]) # to final orientation

        # Update previous errors in NED FRAME
        self.prev_error_x = self.error_x
        self.prev_error_y = self.error_y
        self.prev_error_z = self.error_z
        self.prev_error_yaw = self.error_yaw #to final orientation

        
    def calculate_velocity_errors(self):
        # velocity errors  Assumed to be velocity in the robot frame
        self.error_vx = self.vx_setpoint - self.current_velocity.linear.x 
        self.error_vy = self.vy_setpoint- self.current_velocity.linear.y
        self.error_vz = self.vz_setpoint - self.current_velocity.linear.z 
        self.error_vyaw = self.vyaw_setpoint - self.current_velocity.angular.z 
        

        # Calculate the derivative of the velocity error in the NED FRAME
        self.vdedx = (self.error_vx - self.prev_error_vx) / self.dt
        self.vdedy = (self.error_vy - self.prev_error_vy) / self.dt
        self.vdedz = (self.error_vz - self.prev_error_vz) / self.dt
        self.vdedyaw = (self.error_vyaw - self.prev_error_vyaw) / self.dt
        
        # calulating integral of the error in the NED FRAME using the trapezoidal method for the area

        self.sum_error_vx += self.error_vx
        self.sum_error_vy += self.error_vy
        self.sum_error_vz += self.error_vz
        self.sum_error_vyaw += self.error_vyaw

        # self.sum_error_vx += self.calculate_integral(a = self.prev_error_vx, b = self.error_vx, h = self.dt)
        # self.sum_error_vy += self.calculate_integral(a = self.prev_error_vy, b = self.error_vy, h = self.dt)
        # self.sum_error_vz += self.calculate_integral(a = self.prev_error_vz, b = self.error_vz, h = self.dt)
        # self.sum_error_vyaw += self.calculate_integral(a = self.prev_error_vyaw, b = self.error_vyaw, h = self.dt)

        # anti windup for integral
        self.sum_error_vx = np.clip(self.sum_error_vx, self.pwm_anti_windup_clip[0],self.pwm_anti_windup_clip[1])
        self.sum_error_vy = np.clip(self.sum_error_vy, self.pwm_anti_windup_clip[0],self.pwm_anti_windup_clip[1])
        self.sum_error_vz = np.clip(self.sum_error_vz, self.pwm_anti_windup_clip[0],self.pwm_anti_windup_clip[1])
        self.sum_error_vyaw = np.clip(self.sum_error_vyaw, self.pwm_anti_windup_clip_angular[0],self.pwm_anti_windup_clip_angular[1])

        # Update previous velocity errors 
        self.prev_error_vx = self.error_vx
        self.prev_error_vy = self.error_vy
        self.prev_error_vz = self.error_vz
        self.prev_error_vyaw = self.error_vyaw

        
        # # just for the simulation 
        # if self.system == "sitl":
        #     if self.vx_setpoint == 0:
        #         self.sum_error_vx = 0
        #     if self.vy_setpoint == 0:
        #         self.sum_error_vy = 0
        #     if self.vz_setpoint == 0:
        #         self.sum_error_vz = 0
        #     if self.vyaw_setpoint == 0:
        #         self.sum_error_vyaw = 0

        # attempted an alternative method for integral here

        '''
        # rospy.loginfo_throttle(1,"getting to integral")
        # calulating approximate integral of the error in the NED FRAME, just the running sum 
        # self.last_n_errors_vx.append(self.error_vx)
        # self.last_n_errors_vy.append(self.error_vx)
        # self.last_n_errors_vz.append(self.error_vx)
        # self.last_n_errors_vyaw.append(self.error_vx)

        # # if len(self.last_n_errors_vx)> self.num_errors:
        # #     self.last_n_errors_vx.pop(0)
        # #     self.last_n_errors_vy.pop(0)
        # #     self.last_n_errors_vz.pop(0)
        # #     self.last_n_errors_vyaw.pop(0)

        # if len(self.last_n_errors_vx) > 0:
        #     self.sum_error_vx = sum(self.last_n_errors_vx)
        #     self.sum_error_vy = sum(self.last_n_errors_vy)
        #     self.sum_error_vz = sum(self.last_n_errors_vy)
        #     self.sum_error_vyaw = sum(self.last_n_errors_vyaw)
        # else:
        #     self.sum_error_vx = 0
        #     self.sum_error_vy = 0
        #     self.sum_error_vz = 0
        #     self.sum_error_vyaw =0
       

        # rospy.loginfo_throttle(1,"getting past integral")
        '''    
    
    
    #   calculating velocity setpoints from position errors
    def calc_x_velocity_setpoint(self):
        # Transforming ERRORS to BODY FRAME...note: yaw and z error are already in body frame, we assume roll and pitch are negligible
        ex = np.cos(self.current_yaw) * self.error_x  + np.sin(self.current_yaw) * self.error_y
        
        # derivative error
        dedx = np.cos(self.current_yaw) * self.dedx  + np.sin(self.current_yaw) * self.dedy
        
        # integral error
        sum_ex = np.cos(self.current_yaw) * self.sum_error_x  + np.sin(self.current_yaw) * self.sum_error_y

        self.vx_setpoint = self.Kp_x * ex + self.Kd_x * dedx + self.Ki_x * sum_ex
       

        # calculate control and saturate to max desired velocities
        self.vx_setpoint = self.Kp_x * ex + self.Kd_x * dedx + self.Ki_x * sum_ex

        # saturating to max velcoity 
        self.vx_setpoint = np.clip(self.vx_setpoint, self.linear_velocity_clip[0],self.linear_velocity_clip[1]) 

        self.pub16.publish(ex)
 
    def calc_y_velocity_setpoint(self):
        
        # Transforming ERRORS to BODY FRAME...note: yaw and z error are already in body frame, we assume roll and pitch are negligible
        ey = -np.sin(self.current_yaw) * self.error_x  + np.cos(self.current_yaw) * self.error_y

        # derivative error
        dedy = -np.sin(self.current_yaw) * self.dedx + np.cos(self.current_yaw) * self.dedy

        # integral error
        sum_ey = -np.sin(self.current_yaw) * self.sum_error_x  + np.cos(self.current_yaw) * self.sum_error_y


        # saturate to max desired velocities
        self.vy_setpoint = self.Kp_y * ey + self.Kd_y * dedy + self.Ki_y * sum_ey

        self.vy_setpoint = round(np.clip(self.vy_setpoint, self.linear_velocity_clip[0],self.linear_velocity_clip[1]),3)

        self.pub16= rospy.Publisher('x_error', Float32, queue_size=1)
        self.pub17.publish(ey)
     
    def calc_z_velocity_setpoint(self):
        # less calculations here because it is already in the right frame
        self.vz_setpoint = self.Kp_z * self.error_z + self.Kd_z * self.dedz + self.Ki_z * self.sum_error_z
        self.vz_setpoint = round(np.clip(self.vz_setpoint, self.linear_velocity_clip[0],self.linear_velocity_clip[1]),3)
       
    def calc_yaw_velocity_setpoint_heading(self):
        # use this if you want to face towards the waypoint

        # calculate heading to get to waypoint
        self.heading = np.arctan2(np.array(self.error_y),np.array(self.error_x))
        # heading error
        self.heading_error = self.heading - self.current_yaw

        # normalize heading
        if self.heading_error > np.pi:
            self.heading_error -= 2 * np.pi
        elif self.heading_error < -np.pi:
            self.heading_error += 2 * np.pi  
            
        self.vyaw_setpoint = self.Kp_yaw * self.heading_error
        self.vyaw_setpoint = np.clip(self.vyaw_setpoint, self.angular_velocity_clip[0],self.angular_velocity_clip[1])

    def calc_yaw_velocity_setpoint_final(self):
        # use this if the intermediate waypoint is the desired 
        self.error_yaw *= 180/np.pi
        self.dedyaw *= 180/np.pi
        self.sum_error_yaw *= 180/np.pi

        self.vyaw_setpoint = self.Kp_yaw * self.error_yaw + self.Kd_yaw * self.dedyaw + self.Ki_yaw * self.sum_error_yaw
        self.vyaw_setpoint = np.clip(self.vyaw_setpoint, self.angular_velocity_clip[0],self.angular_velocity_clip[1])
        # rospy.loginfo(f"v yaw setpoint = {self.vyaw_setpoint*180/np.pi} ")
        self.pub16= rospy.Publisher('x_error', Float32, queue_size=1)
        self.pub17= rospy.Publisher('y_error', Float32, queue_size=1)

        self.pub18.publish(self.error_yaw)




    # calculate pwm signals from velocity errors
    
    def calc_x_pwm(self):
        x_control = self.kp_v_x * self.error_vx+ self.kd_v_x * self.vdedx + self.ki_v_x * self.sum_error_vx
        if self.send_signal == "manual":
            self.x_pwm = int(np.clip(x_control,self.pwm_clip[0], self.pwm_clip[1]))
        elif self.send_signal == "rc":
            self.x_pwm = int(np.clip(1500+x_control, self.linear_pwm_clip[0],self.linear_pwm_clip[1]))
        
        
        if self.system == "sitl" and self.vx_setpoint ==0:
            self.x_pwm = 0
            

        # rospy.loginfo_throttle(2,f"x accumulated error: {round(self.sum_error_vx,3)}")
        # rospy.loginfo_throttle(2,f"x integral contribution: {round(self.ki_v_x * self.sum_error_vx,2)}")
        # rospy.loginfo_throttle(2,f"x_pwm = {self.x_pwm}")

    def calc_y_pwm(self):
        y_control = self.kp_v_y * self.error_vy+ self.kd_v_y * self.vdedy + self.ki_v_y * self.sum_error_vy
        if self.send_signal == "manual":
            self.y_pwm = int(np.clip(y_control,self.pwm_clip[0], self.pwm_clip[1]))
        elif self.send_signal == "rc":
            self.y_pwm = int(np.clip(1500+y_control, self.linear_pwm_clip[0],self.linear_pwm_clip[1]))

        if self.system == "sitl" and self.vy_setpoint ==0:
            self.y_pwm = 0


        # rospy.loginfo_throttle(2,f"y accumulated error: {round(self.sum_error_vy,3)}")
        # rospy.loginfo_throttle(2,f"y integral contribution: {round(self.ki_v_y * self.sum_error_vy,2)}")
        # rospy.loginfo_throttle(2,f"y_pwm = {self.y_pwm}")
                               
        # self.y_pwm = int(np.clip(1500+y_control, self.linear_pwm_clip[0],self.linear_pwm_clip[1]))

    def calc_z_pwm(self):
        z_control = self.kp_v_z * self.error_vz + self.kd_v_z * self.vdedz + self.ki_v_z * self.sum_error_vz
        if self.send_signal == "manual":
            self.z_pwm = int(np.clip(500-z_control,0, 1000)) # mau need to check the plus or minus sign
        elif self.send_signal == "rc":
            self.z_pwm = int(np.clip(1500-z_control, self.linear_pwm_clip[0],self.linear_pwm_clip[1]))
        # self.z_pwm = int(np.clip(1500-z_control, self.linear_pwm_clip[0],self.linear_pwm_clip[1])) # may need to fix this i think it needs to be between certain values 

    def calc_yaw_pwm(self):
        yaw_control = self.kp_v_yaw * self.error_vyaw + self.kd_v_yaw * self.vdedyaw + self.ki_v_yaw * self.sum_error_vyaw
        if self.send_signal == "manual":
            self.yaw_pwm = int(np.clip(yaw_control,self.pwm_clip[0], self.pwm_clip[1]))
        elif self.send_signal == "rc":
            self.yaw_pwm = int(np.clip(1500+yaw_control, self.linear_pwm_clip[0],self.linear_pwm_clip[1]))
     
    def calc_all_pwm(self):
        self.calc_x_pwm()
        self.calc_y_pwm()
        self.calc_z_pwm()
        self.calc_yaw_pwm()

    def velocity_controller(self):
        # outputs a joystick signal to send to the robot tries to reach setpoint velocity

        # compute velocity error compared to setpoint
        self.calculate_velocity_errors()
        # computeall pwm signals to send 
        self.calc_all_pwm()

        # for logging to graphs
        self.publish_velocity_setpoints()
        self.publish_pwm_commands()

    def manual_pwm_user_defined(self):

        self.publish_velocity_setpoints()
        self.publish_pwm_commands()

    def set_target_depth(self, depth):
        """ Sets the target depth while in depth-hold mode.

        Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

        'depth' is technically an altitude, so set as negative meters below the surface
            -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

        """
        self.master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self.boot_time)), # ms since boot
            self.master.target_system, self.master.target_component,
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
    
    def erase_target_depth(self):
        """ Sets the target depth while in depth-hold mode.

        Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

        'depth' is technically an altitude, so set as negative meters below the surface
            -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.

        """
        self.master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self.boot_time)), # ms since boot
            self.master.target_system, self.master.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=(
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
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
            ), lat_int=0, lon_int=0, alt=0, # (x, y WGS84 frame pos - not used), z [m]
            vx=0, vy=0, vz=0, # velocities in NED frame [m/s] (not used)
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
            # accelerations in NED frame [N], yaw, yaw_rate
            #  (all not supported yet, ignored in GCS Mavlink)
        )
    
    
    def set_target_attitude(self, roll=0, pitch=0, yaw=0):
        """ Sets the target attitude while in depth-hold mode.

        'roll', 'pitch', and 'yaw' are angles in rad.

        """
        self.master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self.boot_time)), # ms since boot
            self.master.target_system, self.master.target_component,
            # allow throttle to be controlled by depth_hold mode
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
            QuaternionBase([roll,pitch,yaw]),
            0, 0, 0, 0 # roll rate, pitch rate, yaw rate, thrust
        )

    def carrot_chasing(self):
        # let d  = crosstrack error 
        self.get_current_waypoint()
        self.get_lookahead_waypoint()
        r1 = self.current_waypoint.pose.position.x - self.current_pose.pose.position.x
        r2 = self.current_waypoint.pose.position.y - self.current_pose.pose.position.y
        ru = np.linalg.norm(np.array(r1,r2))
        theta = np.arctan2(self.lookahead_waypoint.pose.position.y- self.current_waypoint.pose.position.y,self.lookahead_waypoint.pose.position.x- self.current_waypoint.pose.position.x) 
        theta_u = np.arctan2(self.current_pose.pose.position.y-self.current_waypoint.pose.position.y, self.current_pose.pose.position.x-self.current_waypoint.pose.position.x)
        beta = theta-theta_u
        R = np.sqrt((ru**2)-(ru*np.sin(beta)**2))
        x_t_prime = (R + self.delta) * np.cos(theta)
        y_t_prime = (R + self.delta) * np.sin(theta)
        carrot_point = (x_t_prime, y_t_prime)
        self.heading = np.arctan2(y_t_prime-self.current_pose.pose.position.y,x_t_prime-self.current_pose.pose.position.x)
        u = self.Kp_yaw()

        
# Function to send control inputs to the robot (e.g., through MAVLink or a ROS topic)
# uses manual mode
def send_control_manual(master, x_pwm, y_pwm, z_pwm = 500, yaw_pwm = 0):
    # if x_pwm == 999:
    #     x_pwm = 32767
    
    # if z_pwm ==500:
    #     z_pwm = 32767
    master.mav.manual_control_send(
    master.target_system,
    x_pwm,
    y_pwm,
    z_pwm,
    yaw_pwm,
    0)

#uses RC channel
def send_control(x_pwm, y_pwm, z_pwm, yaw_pwm, master):
    # master.mav.manual_control_send(
    # master.target_system,
    # x_pwm,
    # y_pwm,
    # z_pwm,
    # yaw_pwm,
    # 0)

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


    # # set the desired operating mode
    # DEPTH_HOLD = 'ALT_HOLD'
    # DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
    # while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    # master.set_mode(DEPTH_HOLD)


def main(): 
    
    # Initialize the ROS node
    rospy.init_node('waypoint_follower')
    try:
    
        system = "sitl"    

        # custom
        attitude_control = "built in"

        if system == "hardware":
            ignore_depth_control = True
            master = mavutil.mavlink_connection('udpin:0.0.0.0:14548') 
            setup_signal(master)
            initialize_operating_mode(master=master, mode = "depth hold")
            
        elif system == "sitl":
            ignore_depth_control = False
            master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
            setup_signal(master)
            initialize_operating_mode(master=master, mode = "depth hold") 
        elif system == "custom sim":
            ignore_depth_control = True
            master = mavutil.mavlink_connection('udpin:0.0.0.0:14548') 

        # master = mavutil.mavlink_connection('udpin:0.0.0.0:14554')
        # master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
        # master = mavutil.mavlink_connection('udpin:0.0.0.0:14445')

        # this is the GCS url
        # master = mavutil.mavlink_connection('udpin:0.0.0.0:14548')
        # master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')

        # system = "sitl"


        # initialize motion controller
        # controller = MotionControl(master = master,system="custom sim")
        controller = MotionControl(master = master,system=system)

        # set which position control method to use
        controller.mode = 1
        # log info time step between debugging lines
        update_status_interval = 1

        while not rospy.is_shutdown():
            if not controller.state == "Disabled":

                if controller.state == "Initialized":
                    try:
                        if ignore_depth_control:
                            rospy.loginfo_throttle(update_status_interval ,"want to specify a depth here")
                        else:
                            controller.set_target_depth(-1*controller.target_depth)
                    except: 
                        rospy.logwarn("Depth not set")
                    # # rospy.loginfo_throttle(3,f"current target depth is {controller.target_depth}")
                    # controller.set_target_depth(-1*controller.target_depth)
                

                elif controller.state == "manual pwm":
                    # publish setpoints
                    controller.manual_pwm_user_defined()
                    send_control_manual(master, x_pwm = controller.x_pwm, y_pwm = controller.y_pwm,z_pwm = controller.z_pwm, yaw_pwm = controller.yaw_pwm)

                elif controller.state == "velocity":
                
                    controller.velocity_controller()
                    if controller.vz_setpoint == 0:
                        
                        send_control_manual(master, x_pwm = controller.x_pwm, y_pwm = controller.y_pwm,z_pwm = 500, yaw_pwm = controller.yaw_pwm)
                        
                        # controller.set_target_depth(-1*controller.target_depth)

                    else: 
                        send_control_manual(master, x_pwm = controller.x_pwm, y_pwm = controller.y_pwm,z_pwm = controller.z_pwm, yaw_pwm = controller.yaw_pwm)

                    # rospy.loginfo_throttle(update_status_interval,"Setpoint Velocity: x=%.2f, y=%.2f, z=%.2f" ,controller.vx_setpoint, controller.vy_setpoint, controller.vz_setpoint) 
                    # rospy.loginfo_throttle(update_status_interval,"Current Velocity: x=%.2f, y=%.2f, z=%.2f" ,controller.current_velocity.linear.x, controller.current_velocity.linear.y, controller.current_velocity.linear.z) # controller.current_velocity.angular.z) 
                    # rospy.loginfo_throttle(update_status_interval,"Control (pwm): x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",controller.x_pwm, controller.y_pwm, controller.z_pwm, controller.yaw_pwm) 

                elif controller.state == "waypoint":
                    rospy.loginfo_throttle(10,"waypoint follower is active")

                    if not controller.current_waypoint == None:
                    
                        controller.calculate_position_errors()
                      
                        controller.calc_x_velocity_setpoint()
                        controller.calc_y_velocity_setpoint()
                        controller.calc_yaw_velocity_setpoint_final()

                        controller.velocity_controller()


                        if attitude_control == "built in":
                        # test to see if target attitude works if not switch to yaw controller
                            controller.set_target_attitude(yaw = controller.target_yaw)
                            send_control_manual(master, x_pwm = controller.x_pwm,y_pwm = controller.y_pwm,z_pwm=500, yaw_pwm = 0)
                            # controller.calc_yaw_velocity_setpoint_final()
                        elif attitude_control == "custom":
                    
                            # for sending the actual control via manual mode
                            # send_control_manual(master, controller.x_pwm,controller.y_pwm)
                            send_control_manual(master, x_pwm = controller.x_pwm,y_pwm = controller.y_pwm,z_pwm=500, yaw_pwm = controller.yaw_pwm)
                        
                        # set depth
                        if ignore_depth_control:
                            rospy.loginfo_throttle(update_status_interval ,"want to specify a depth here")
                        else:
                            controller.set_target_depth(-1*controller.current_waypoint.pose.position.z)
                            # send_control_manual(master, x_pwm = controller.x_pwm,y_pwm = controller.y_pwm,z_pwm=controller.z_pwm, yaw_pwm = controller.yaw_pwm)
                        

                        rospy.loginfo_throttle(5,"got to the end of motion controller")


                        # publish the calculated setpoints
                        # controller.publish_velocity_setpoints()
                        # controller.publish_pwm_commands()
                    
                        # rospy.loginfo_throttle(2,"Current Pose: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f" ,controller.current_pose.pose.position.x,controller.current_pose.pose.position.y,controller.current_pose.pose.position.z, controller.current_yaw)
                        # rospy.loginfo_throttle(2,"Current waypoint: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",controller.current_waypoint.pose.position.x,controller.current_waypoint.pose.position.y,controller.current_waypoint.pose.position.z, controller.waypoint_yaw)
                        
                        # rospy.loginfo_throttle(2,"Setpoint Velocity: x=%.2f, y=%.2f, z=%.2f" ,controller.vx_setpoint, controller.vy_setpoint, controller.vz_setpoint) 
                        # rospy.loginfo_throttle(2,"Current Velocity: x=%.2f, y=%.2f, z=%.2f" ,controller.current_velocity.linear.x, controller.current_velocity.linear.y, controller.current_velocity.linear.z) # controller.current_velocity.angular.z) 
                        # rospy.loginfo_throttle(2,"Control (pwm): x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",controller.x_pwm, controller.y_pwm, controller.z_pwm, controller.yaw_pwm) 
                
                    else:
                        rospy.logwarn_throttle(1, "no waypoint received by motion controller")

                elif controller.state == "Joystick":
                    rospy.loginfo_throttle(10,"Joystick mode is active")

                else: 
                    rospy.logerr("Motion controller is in an unexpected state")
            else: 
                rospy.loginfo_throttle(5,"motion controller is disabled")
            
           
           
            # Send heartbeat from a GCS (types are define as enum in the dialect file).
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    
            controller.rate.sleep()   

    except Exception as error:
        rospy.logerr("An error occurred:", error)
        
         
          

    
if __name__ == "__main__":

    main()