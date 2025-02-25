#!/usr/bin/env python3
import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose, Twist
import time 


class Visualiser:
    def __init__(self):
        self.sliding_window = True
        # self.fig, self.ax1 = plt.subplots()
        # self.ln, = plt.plot([], [], 'r')
        # self.ln1, = plt.plot([], [], 'b')


        # self.fig, (self.ax1,self.ax2,self.ax3,self.ax4) = plt.subplots(2)
        self.fig, (self.ax1,self.ax2) = plt.subplots(2)

        self.ln, = self.ax1.plot([], [], 'r')
        self.ln1, = self.ax1.plot([], [], 'b')

        self.ln2, = self.ax2.plot([], [], 'b')



        self.vx_data = [] 
        self.vy_data = [] 
        self.vz_data = [] 
        self.vyaw_data = []
        self.v_time = []


        self.vx_setpoint_data = []
        self.vy_setpoint_data = []
        self.vz_setpoint_data = []
        self.vyaw_setpoint_data = []
        self.v_setpoint_time = []

        self.x_pwm =[]
        self.y_pwm =[]
        self.z_pwm =[]
        self.yaw_pwm =[]
        self.pwm_time = []
        



        self.start_time = time.time()
        


    def plot_init(self):
        self.ax1.set_xlim(0, 100)
        self.ax1.set_ylim(-10, 10)

        self.ax2.set_xlim(0, 100)
        self.ax2.set_ylim(-10, 10)

        # self.ax3.set_xlim(0, 100)
        # self.ax3.set_ylim(-10, 10)

        # self.ax4.set_xlim(0, 100)
        # self.ax4.set_ylim(-10, 10)
        return self.ln
    
    
    def getYaw(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] 
        return yaw   

    def odom_callback(self, msg):
        yaw_angle = self.getYaw(msg.pose.pose)
        self.y_data.append(yaw_angle)
        x_index = len(self.x_data)
        self.x_data.append(x_index+1)

    def pwm_callback(self,msg=Twist):
        self.x_pwm.append(msg.linear.x)
        self.y_pwm.append(msg.linear.y)
        self.z_pwm.append(msg.linear.z)
        self.yaw_pwm.append(msg.angular.z)

        cur_time = time.time()-self.start_time
        self.pwm_time.append(cur_time)


    def velocity_callback(self,msg=Twist):
        self.vx_data.append(msg.linear.x)
        self.vy_data.append(msg.linear.y)
        self.vz_data.append(msg.linear.z)
        self.vyaw_data.append(msg.angular.z)

        cur_time = time.time()-self.start_time
        self.v_time.append(cur_time)

        # x_index = len(self.vx_data)
        # self.vx_x_data.append(x_index+1)
    
    def velocity_setpoint_callback(self,msg=Twist):
        self.vx_setpoint_data.append(msg.linear.x)
        self.vy_setpoint_data.append(msg.linear.y)
        self.vz_setpoint_data.append(msg.linear.z)
        self.vyaw_setpoint_data.append(msg.angular.z)

        cur_time = time.time()-self.start_time
        self.v_setpoint_time.append(cur_time)

        # x_index = len(self.vx_x_setpoint_data)
        # self.vx_x_setpoint_data.append(x_index+1)
        
    
    def update_plot(self, frame):
        # if len(self.v_setpoint_time)>10:
        self.ln.set_data(self.v_time, self.vx_data)
        self.ln1.set_data(self.v_setpoint_time, self.vx_setpoint_data)
        self.ln2.set_data(self.pwm_time, self.x_pwm)
    
        if len(self.pwm_time)>10:
            if self.sliding_window:
                minx = max(self.v_setpoint_time)-15                           
            else:
                minx = 0

            self.ax1.set_xlim(minx, max(self.v_setpoint_time)+10)
                        
            miny_1= min(self.vx_data)
            miny_2= min(self.vx_setpoint_data)
            miny = min(miny_1, miny_2)
            maxy_1= max(self.vx_data)
            maxy_2= max(self.vx_setpoint_data)
            maxy = max(maxy_1, maxy_2)
            self.ax1.set_ylim(miny-0.5,maxy+0.5)

                            

            self.ax2.set_xlim(minx, max(self.v_setpoint_time)+10)

            # miny_2= min(self.x_pwm_data)
            miny = min(self.x_pwm)
            maxy = max(self.x_pwm)
            self.ax2.set_ylim(min(self.x_pwm)-25,maxy+25)


            
        return self.ln, self.ln1, self.ln2


rospy.init_node('velocity_plot')
rospy.loginfo("initialized velocity plot node")

vis = Visualiser()
sub1 = rospy.Subscriber('velocity_setpoint', Twist, vis.velocity_setpoint_callback)
sub2 = rospy.Subscriber('/dvl/twist', Twist, vis.velocity_callback)
sub3 = rospy.Subscriber('pwm_setpoint', Twist, vis.pwm_callback)

while not rospy.is_shutdown():
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    plt.show(block=True)
plt.close() 