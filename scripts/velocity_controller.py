#!/usr/bin/env python3
import rospy
import numpy as np
# import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose, Twist
from tf.transformations import euler_from_quaternion, euler_matrix
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
from std_msgs.msg import Bool, Int8, Float32MultiArray
import time

class MotionControl:
    def __init__(self,master, mode = "hardware"): 
        self.send_signal = "manual"
        self.master = master
        self.boot_time = time.time()
        self.mode =1
        self.delta =5
        self.target_depth = 1


        # Activating/deactivating controller
        self.invoked = False

        # saturation 
        self.velocity_anti_windup_clip = [0,10]  # prevents the integral error from becoming too large, might need to split this up into multiple degree of freedoms
        self.linear_velocity_clip = [-0.5,0.5] #min and max velocity setpoints
        self.angular_velocity_clip = [-20*np.pi/180, 20*np.pi/180] #[-1,1]#[-10*np.pi/180, 10*np.pi/180] # min and max angular velocity setpoints

        # xy
        self.kp_v_x = 2000    
        self.kd_v_x = 0
        self.ki_v_x = 500

        self.kp_v_y = 2000
        self.kd_v_y = 0
        self.ki_v_y = 500


        # z gains
        self.kp_v_z = 0
        self.kd_v_z = 0
        self.ki_v_z = 0

        # yaw
        self.kp_v_yaw = 100
        self.kd_v_yaw = 0
        self.ki_v_yaw = 0

        # saturation 
        self.pwm_anti_windup_clip = [0,50]  # 500# prevents the integral error from becoming too large, might need to split this up into multiple degree of freedoms
        self.linear_pwm_clip = [1000, 2000] #min and max pwm setpoints
        self.angular_pwm_clip = [1000, 2000] # min and max angular velocity setpoints

    
        # used to activate the manual veocity setpoint mode, when true it does not update the velocity setpoint based on a position
        self.velocity_setpoint_testing = False

        self.pwm_testing = False

        # set the rates and frequencies
        self.frequency = 10 
        self.rate = rospy.Rate(self.frequency)  # 10 Hz
        self.dt = 1/self.frequency


        #  storing information about current velocity 
        self.current_velocity = Twist()
        
    
        # Control setpoints
        # storing the setpoints for publishing to topic
        self.pwm_setpoint = Twist()
        self.velocity_setpoint = Twist()
        

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
        # current error
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

        # creating subscribers

        self.invoked_sub = rospy.Subscriber('velocity_controller_state',Bool, self.on_off_callback)
        self.params_sub = rospy.Subscriber('velocity_controller_params',Float32MultiArray, self.controller_params_callback)




        self.sub1 = rospy.Subscriber('motion_control_state',Bool, self.on_off_callback)
              
        self.sub7 = rospy.Subscriber('controller_gains', Float32MultiArray, self.controller_gains_callback)
        # self.sub8 = rospy.Subscriber('sitl_current_velocity', TwistStamped, self.velocity_callback)
        self.sub8 = rospy.Subscriber('/dvl/twist', TwistStamped, self.velocity_callback)

        # creating publishers, for the RViz only simulation
        # self.pub1 = rospy.Publisher('velocity_command', TwistStamped, queue_size=10)
        
        # publishingg controller setpoints
        self.pub2 = rospy.Publisher('velocity_setpoint', Twist, queue_size=10)
        self.pub3 = rospy.Publisher('pwm_setpoint', Twist, queue_size=10)

    # whenever the button is hit, toggle the controller on/off
    def on_off_callback(self,msg:Bool):
        self.invoked = msg.data
        if self.invoked: 
            rospy.loginfo("Velocity_controller activated")
        elif not self.invoked:
            rospy.loginfo("Velocity_controller deactivated")

    def controller_params_callback(self,msg: Float32MultiArray):


        if msg.data[0]==2:
            
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

            self.filter_coefficient = msg.data[13]
            self.reset_errors()

            rospy.loginfo(f"PWM controller gains (kp,kd,ki):\nx: ({self.kp_v_x}, {self.kd_v_x}, {self.ki_v_x})\ny: ({self.kp_v_y}, {self.kd_v_y}, {self.ki_v_y})\nz: ({self.kp_v_z}, {self.kd_v_z}, {self.ki_v_z})\nyaw: ({self.kp_v_yaw}, {self.kd_v_yaw}, {self.ki_v_yaw})")

        elif msg.data[0]==3:
            self.linear_velocity_clip = [-1*msg.data[1], msg.data[1]]
            self.angular_velocity_clip = [-1*msg.data[1], msg.data[1]]
            self.linear_pwm_clip = [msg.data[2],msg.data[3]]
            self.angular_pwm_clip = [msg.data[2],msg.data[3]]

            rospy.loginfo(f"Set velocity and PWM clips")

        elif msg.data[0]==4:
            self.velocity_setpoint_testing = True
            self.pwm_testing = False
            self.vx_setpoint = msg.data[1]
            self.vy_setpoint = msg.data[2]
            # self.vz_setpoint = msg.data[3]
            self.vz_setpoint = 0

            self.vyaw_setpoint = msg.data[4]

            self.vx_setpoint = np.clip(self.vx_setpoint, self.linear_velocity_clip[0],self.linear_velocity_clip[1])
            self.vy_setpoint = np.clip(self.vy_setpoint, self.linear_velocity_clip[0],self.linear_velocity_clip[1])
            self.vz_setpoint = np.clip(self.vz_setpoint, self.linear_velocity_clip[0],self.linear_velocity_clip[1])
            self.vyaw_setpoint = np.clip(self.vyaw_setpoint, self.angular_velocity_clip[0],self.angular_velocity_clip[1])
            
            self.target_depth = msg.data[3]

            # reset the integral errors
            # self.sum_error_vx = 0.0
            # self.sum_error_vy = 0.0
            # self.sum_error_vz = 0.0
            # self.sum_error_vyaw = 0.0
            self.reset_errors()
        
        
            rospy.loginfo(f"Preparing for velocity test\nvx setpoint = {self.vx_setpoint}\nvy setpoint = {self.vy_setpoint}\nvz setpoint = {self.vz_setpoint}\nangular velocity setpoint = {self.vyaw_setpoint}")

        elif msg.data[0]==5:
            self.velocity_setpoint_testing = False
            self.pwm_testing = False
            self.vx_setpoint = 0
            self.vy_setpoint = 0
            self.vz_setpoint = 0
            self.vyaw_setpoint = 0

            self.reset_errors()
        
            rospy.loginfo(f"Ended velocity test mode")
        
        elif msg.data[0]==6:
            self.pwm_testing = True
            self.velocity_setpoint_testing = False
            self.x_pwm = int(msg.data[1])
            self.y_pwm = int(msg.data[2])
            # self.z_pwm= int(msg.data[3])
            self.z_pwm = 500
            self.yaw_pwm = int(msg.data[4])
            rospy.loginfo("Joystick mode")

            self.target_depth = msg.data[3]

            
        
        elif msg.data[0]==7:
            self.pwm_setpoint_testing = False
            self.velocity_setpoint_testing = False
            self.x_pwm = 0
            self.y_pwm = 0
            self.z_pwm = 500
            self.yaw_pwm = 0
            rospy.loginfo(f"Ended joystick test mode")

    def velocity_callback(self, msg:TwistStamped):
        self.current_velocity = msg.twist
        
    def publish_velocity_setpoints(self):
        
        # self.velocity_command.twist.linear.x = self.vx_setpoint
        # self.velocity_command.twist.linear.y = self.vy_setpoint
        # self.velocity_command.twist.linear.z = self.vz_setpoint
        # self.velocity_command.twist.angular.x= 0
        # self.velocity_command.twist.angular.y = 0
        # self.velocity_command.twist.angular.z = self.vyaw_setpoint

        self.velocity_setpoint.linear.x = self.vx_setpoint
        self.velocity_setpoint.linear.y = self.vy_setpoint
        self.velocity_setpoint.linear.z = self.vz_setpoint
        self.velocity_setpoint.angular.x= 0
        self.velocity_setpoint.angular.y = 0
        self.velocity_setpoint.angular.z = self.vyaw_setpoint


        self.pub2.publish(self.velocity_setpoint)

    def publish_pwm_commands(self):
        self.pwm_setpoint.linear.x = self.x_pwm
        self.pwm_setpoint.linear.y = self.y_pwm
        self.pwm_setpoint.linear.z = self.z_pwm
        self.pwm_setpoint.angular.x = 0
        self.pwm_setpoint.angular.y = 0
        self.pwm_setpoint.angular.z = self.yaw_pwm
        self.pub3.publish(self.pwm_setpoint)

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

        self.last_n_errors_vx = [] 
        self.last_n_errors_vy = [] 
        self.last_n_errors_vz = [] 
        self.last_n_errors_vyaw = [] 


# Current functions

    def calc_yaw_velocity_setpoint_heading(self):

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

        self.vyaw_setpoint = self.Kp_yaw * self.error_yaw * self.Kd_yaw * self.dedyaw * self.Ki_yaw * self.sum_error_yaw
        self.vyaw_setpoint = np.clip(self.vyaw_setpoint, self.angular_velocity_clip[0],self.angular_velocity_clip[1])
    
    def calculate_yaw_pwm(self):
        # proportional error
        self.error_vyaw = self.vyaw_setpoint - self.current_velocity.angular.z
        # derivative error
        self.vdedyaw = (self.error_vyaw - self.prev_error_vyaw) / self.dt
        # accumulated error
        self.sum_error_vyaw += self.calculate_integral(a = self.prev_error_vyaw, b = self.error_vyaw, h = self.dt)
        # anti windup for integral
        self.sum_error_vyaw = np.clip(self.sum_error_vyaw, self.pwm_anti_windup_clip[0],self.pwm_anti_windup_clip[1])

        # Update previous velocity errors 
        self.prev_error_vyaw = self.error_vyaw

        #  update pwm setpoints and clip it to max pwm
        yaw_control = self.kp_v_yaw * self.error_vyaw + self.kd_v_yaw * self.vdedyaw + self.ki_v_yaw * self.sum_error_vyaw
        self.yaw_pwm = int(np.clip(1500+yaw_control, self.linear_pwm_clip[0],self.linear_pwm_clip[1]))# 1200,1800))#self.vx_control_clip[0], self.vx_control_clip[1]))

    def calculate_position_errors(self):

        # calculate the current yaw
        _,_, self.current_yaw = euler_from_quaternion([self.current_pose.pose.orientation.x,self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w])

        # calculate the target yaw
        _,_,self.target_yaw = euler_from_quaternion([self.current_waypoint.pose.orientation.x,self.current_waypoint.pose.orientation.y, self.current_waypoint.pose.orientation.z, self.current_waypoint.pose.orientation.w])

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


        # Calculate the derivative of the error in the NED FRAME
        self.dedx = (self.error_x - self.prev_error_x) / self.dt
        self.dedy = (self.error_y - self.prev_error_y) / self.dt
        self.dedz = (self.error_z - self.prev_error_z) / self.dt
        self.dedyaw = (self.error_yaw - self.prev_error_yaw) / self.dt # to final orientation

        # calulating integral of the error in the NED FRAME using the trapezoidal method for the area
        self.sum_error_x += self.calculate_integral(a = self.prev_error_x, b = self.error_x, h = self.dt)
        self.sum_error_y += self.calculate_integral(a = self.prev_error_y, b = self.error_y, h = self.dt)
        self.sum_error_z += self.calculate_integral(a = self.prev_error_z, b = self.error_z, h = self.dt)
        self.sum_error_yaw += self.calculate_integral(a = self.prev_error_yaw, b = self.error_yaw, h = self.dt) # to final orientation

        

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
        if self.mode == 1:

            self.vx_setpoint = np.clip(self.vx_setpoint, self.linear_velocity_clip[0],self.linear_velocity_clip[1])
        else: 
            self.x_pwm= np.clip(self.vx_setpoint, -1000,1000)    

    def calc_y_velocity_setpoint(self):
        
        # Transforming ERRORS to BODY FRAME...note: yaw and z error are already in body frame, we assume roll and pitch are negligible
        ey = -np.sin(self.current_yaw) * self.error_x  + np.cos(self.current_yaw) * self.error_y

        # derivative error
        dedy = -np.sin(self.current_yaw) * self.dedx + np.cos(self.current_yaw) * self.dedy

        # integral error
        sum_ey = -np.sin(self.current_yaw) * self.sum_error_x  + np.cos(self.current_yaw) * self.sum_error_y


        # saturate to max desired velocities
        self.vy_setpoint = self.Kp_y * ey + self.Kd_y * dedy + self.Ki_y * sum_ey
        if self.mode == 1:

            self.vy_setpoint = np.clip(self.vy_setpoint, self.linear_velocity_clip[0],self.linear_velocity_clip[1])
        else: 
            self.y_pwm= np.clip(self.vy_setpoint, -1000,1000)   
     

    def calc_z_velocity_setpoint(self):
        # self.yaw_control = self.Kp_yaw * self.error_yaw + self.Kd_yaw * self.dedyaw + self.Ki_yaw * self.sum_error_yaw
        self.vz_setpoint = self.Kp_z * self.error_z + self.Kd_z * self.dedz + self.Ki_z * self.sum_error_z
        self.vz_setpoint = np.clip(self.vz_setpoint, self.linear_velocity_clip[0],self.linear_velocity_clip[1])
      
    def calculate_velocity_errors(self):
        # velocity errors  Assumed to be velocity in the robot frame
        self.error_vx = self.vx_setpoint - self.current_velocity.linear.x 
        self.error_vy = self.vy_setpoint- self.current_velocity.linear.y
        self.error_vz = self.vz_setpoint - self.current_velocity.linear.z 
        self.error_vyaw = self.vyaw_setpoint - self.current_velocity.angular.z 
        

        # Calculate the derivative of the velocity error in the NED FRAME
        # self.current_filter_estimate_x_pwm = (self.filter_coefficient*self.previous_filter_estimate_x_pwm)+(1-self.filter_coefficient)*(self.error_vx - self.prev_error_vx)
        # self.previous_filter_estimate_x_pwm = self.current_filter_estimate_x_pwm

        # self.vdedx = self.current_filter_estimate_x_pwm / self.dt

        self.vdedx = (self.error_vx - self.prev_error_vx) / self.dt
        self.vdedy = (self.error_vy - self.prev_error_vy) / self.dt
        self.vdedz = (self.error_vz - self.prev_error_vz) / self.dt
        self.vdedyaw = (self.error_vyaw - self.prev_error_vyaw) / self.dt
      
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
        
        # calulating integral of the error in the NED FRAME using the trapezoidal method for the area

        self.sum_error_vx += self.calculate_integral(a = self.prev_error_vx, b = self.error_vx, h = self.dt)
        self.sum_error_vy += self.calculate_integral(a = self.prev_error_vy, b = self.error_vy, h = self.dt)
        self.sum_error_vz += self.calculate_integral(a = self.prev_error_vz, b = self.error_vz, h = self.dt)
        self.sum_error_vyaw += self.calculate_integral(a = self.prev_error_vyaw, b = self.error_vyaw, h = self.dt)

        # anti windup for integral
        self.sum_error_vx = np.clip(self.sum_error_vx, self.pwm_anti_windup_clip[0],self.pwm_anti_windup_clip[1])
        self.sum_error_vy = np.clip(self.sum_error_vy, self.pwm_anti_windup_clip[0],self.pwm_anti_windup_clip[1])
        self.sum_error_vz = np.clip(self.sum_error_vz, self.pwm_anti_windup_clip[0],self.pwm_anti_windup_clip[1])
        self.sum_error_vyaw = np.clip(self.sum_error_vyaw, self.pwm_anti_windup_clip[0],self.pwm_anti_windup_clip[1])

        # Update previous velocity errors 
        self.prev_error_vx = self.error_vx
        self.prev_error_vy = self.error_vy
        self.prev_error_vz = self.error_vz
        self.prev_error_vyaw = self.error_vyaw

    def calc_x_pwm(self):
        x_control = self.kp_v_x * self.error_vx+ self.kd_v_x * self.vdedx + self.ki_v_x * self.sum_error_vx
        if self.send_signal == "manual":
            self.x_pwm = int(np.clip(x_control,-1000, 1000))
        elif self.send_signal == "rc":
            self.x_pwm = int(np.clip(1500+x_control, self.linear_pwm_clip[0],self.linear_pwm_clip[1]))

    def calc_y_pwm(self):
        y_control = self.kp_v_y * self.error_vy+ self.kd_v_y * self.vdedy + self.ki_v_y * self.sum_error_vy
        if self.send_signal == "manual":
            self.y_pwm = int(np.clip(y_control,-1000, 1000))
        elif self.send_signal == "rc":
            self.y_pwm = int(np.clip(1500+y_control, self.linear_pwm_clip[0],self.linear_pwm_clip[1]))
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
            self.yaw_pwm = int(np.clip(yaw_control,-1000, 1000))
        elif self.send_signal == "rc":
            self.yaw_pwm = int(np.clip(1500+yaw_control, self.linear_pwm_clip[0],self.linear_pwm_clip[1]))
        # self.yaw_pwm = int(np.clip(1500+yaw_control, self.linear_pwm_clip[0],self.linear_pwm_clip[1])) # may need to adjust the clip values

    def calc_all_pwm(self):
        self.calc_x_pwm()
        self.calc_y_pwm()
        self.calc_z_pwm()
        self.calc_yaw_pwm()

    def position_controller_RTR(self):
        # tries to reach waypoint
        self.calculate_position_errors()

        self.calc_x_velocity_setpoint()
        # self.calc_y_velocity_setpoint()

        # self.set_target_depth(-1*self.current_waypoint.pose.position.z)

        self.calc_z_velocity_setpoint()
        if self.reached_position():
            self.calc_yaw_velocity_setpoint_final()
        else:
            self.calc_yaw_velocity_setpoint_heading()
            if self.heading_error > np.pi/20:
                self.vx_setpoint = 0

    

    def velocity_controller(self):
        # outputs a joystick signal to send to the robot tries to reach setpoint velocity


        # rospy.loginfo_throttle(10,"velocity controller is active")
        # compute velocity error compared to setpoint
        self.calculate_velocity_errors()
        # computeall pwm signals to send 
        self.calc_all_pwm()

        # controller.calculate_pwm_output()

        # for logging to graphS
        self.publish_velocity_setpoints()
        self.publish_pwm_commands()

        # for sending the actual control via manual mode
        # send_control_manual(self.x_pwm,self.y_pwm, self.z_pwm, self.yaw_pwm, master)

        # rospy.loginfo_throttle(2,"error: x=%.2f, y=%.2f, z=%.2f" ,self.error_vx, self.error_vy, self.error_vz) 
        # rospy.loginfo_throttle(10,"velocity controller is active")
        # rospy.loginfo_throttle(2,"Setpoint Velocity: x=%.2f, y=%.2f, z=%.2f" ,self.vx_setpoint, self.vy_setpoint, self.vz_setpoint) 
        # rospy.loginfo_throttle(2,"Current Velocity: x=%.2f, y=%.2f, z=%.2f" ,self.current_velocity.linear.x, self.current_velocity.linear.y, self.current_velocity.linear.z) # self.current_velocity.angular.z) 
        # rospy.loginfo_throttle(2,"Control (pwm): x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",self.x_pwm, self.y_pwm, self.z_pwm, self.yaw_pwm) 

    def manual_pwm_user_defined(self):

        rospy.loginfo_throttle(10,"PWM defined by user")
        # publishing information for plotting
        self.publish_velocity_setpoints()
        self.publish_pwm_commands()

        rospy.loginfo_throttle(2,"Current Velocity: x=%.2f, y=%.2f, z=%.2f" ,self.current_velocity.linear.x, self.current_velocity.linear.y, self.current_velocity.linear.z) # self.current_velocity.angular.z) 
        rospy.loginfo_throttle(2,"Control (pwm): x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",self.x_pwm, self.y_pwm, self.z_pwm, self.yaw_pwm) 
    
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

        
    
def send_control_manual(master, x_pwm, y_pwm, z_pwm = 500, yaw_pwm = 0):
    master.mav.manual_control_send(
    master.target_system,
    x_pwm,
    y_pwm,
    z_pwm,
    yaw_pwm,
    0)


# Function to send control inputs to the robot (e.g., through MAVLink or a ROS topic)
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
    
    # set the desired operating mode
    DEPTH_HOLD = 'ALT_HOLD'
    # DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
    # while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)

    

    
def main(): 
    throttle_time = 2
    # Initialize the ROS node
    rospy.init_node('velocity_controller')
    rospy.loginfo("Initialized velocity controller node")

    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    # initialize motion controller
    controller = MotionControl(master = master)

    setup_signal(master)
    
    

    while not rospy.is_shutdown():

        
        if controller.invoked:
            
            rospy.loginfo_throttle(10,"velocity controller is invoked")
            
            controller.velocity_controller()
        
            send_control_manual(master, controller.x_pwm,controller.y_pwm, controller.yaw_pwm)

            rospy.loginfo_throttle(throttle_time,"Setpoint Velocity: x=%.2f, y=%.2f, z=%.2f" ,controller.vx_setpoint, controller.vy_setpoint, controller.vz_setpoint) 
            rospy.loginfo_throttle(throttle_time,"Current Velocity: x=%.2f, y=%.2f, z=%.2f" ,controller.current_velocity.linear.x, controller.current_velocity.linear.y, controller.current_velocity.linear.z) # controller.current_velocity.angular.z) 
            rospy.loginfo_throttle(throttle_time,"Control signal: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",controller.x_pwm, controller.y_pwm, controller.z_pwm, controller.yaw_pwm) 


        elif not controller.invoked:
            rospy.loginfo_throttle(throttle_time,"velocity controller is  disabled")
         
            # controller.set_target_depth(-1*1)
           

    
        else:
            rospy.logwarn_throttle(throttle_time,"velocity controller is in an unexpectated state")


        controller.rate.sleep()    

    
if __name__ == "__main__":

    main()