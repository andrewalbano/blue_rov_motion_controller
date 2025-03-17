#!/usr/bin/env python3
import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose, Twist, Point

import time 


class Visualiser:
    def __init__(self):
        self.sliding_window = True
        # self.fig, self.ax1 = plt.subplots()
        # self.ln, = plt.plot([], [], 'r')
        # self.ln1, = plt.plot([], [], 'b')


        # self.fig, (self.ax1,self.ax2,self.ax3,self.ax4) = plt.subplots(2)
        self.fig, (self.ax1,self.ax2, self.ax3, self.ax4 ,self.ax5,self.ax6) = plt.subplots(6)


        self.ln1, = self.ax1.plot([], [], 'r', label='x velocity')
        self.ln2, = self.ax1.plot([], [], 'b', label='x setpoint velocity')

        self.ln3, = self.ax2.plot([], [], 'r',label='y velocity')
        self.ln4, = self.ax2.plot([], [], 'b',label='y setpoint velocity')

        self.ln5, = self.ax3.plot([], [], 'r',label='z velocity')
        self.ln6, = self.ax3.plot([], [], 'b',label='z setpoint velocity')
        
        self.ln7, = self.ax4.plot([], [], 'r',label='angular velocity')
        self.ln8, = self.ax4.plot([], [], 'b',label='angular setpoint velocity')

        self.ln9, = self.ax5.plot([], [], 'r', label='x pwm')
        self.ln10, = self.ax5.plot([], [], 'b', label='y pwm')
        self.ln11, = self.ax5.plot([], [], 'g', label='z pwm')
        self.ln12, = self.ax5.plot([], [], 'k', label='yaw pwm')

        self.ln13,= self.ax6.plot([], [], 'b', label='x NED velocity')
        self.ln14,= self.ax6.plot([], [], 'r', label='y NED velocity')

        self.x_ned_data = [] 
        self.y_ned_data = [] 
        self.ned_time = []

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

        self.fig.tight_layout()
        self.ax1.set_title("x velocity")
        self.ax1.set_xlim(0, 100)
        self.ax1.set_ylim(-10, 10)
        self.ax1.legend(handles=[self.ln1,self.ln2])

        self.ax2.set_title("y velocity")
        self.ax2.set_xlim(0, 100)
        self.ax2.set_ylim(-10, 10)
        self.ax2.legend(handles=[self.ln3,self.ln4])

        self.ax3.set_title("z velocity")
        self.ax3.set_xlim(0, 100)
        self.ax3.set_ylim(-10, 10)
        self.ax3.legend(handles=[self.ln5,self.ln6])

        self.ax4.set_title("angular velocity")
        self.ax4.set_xlim(0, 100)
        self.ax4.set_ylim(-10, 10)
        self.ax4.legend(handles=[self.ln7,self.ln8])


        self.ax5.set_title("pwm signals")
        self.ax5.set_xlim(0, 100)
        self.ax5.set_ylim(1000,2000)
        self.ax5.legend(handles=[self.ln9,self.ln10, self.ln11,self.ln12])
        # return self.ln1
    
    
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


    def NED_callback(self,msg=Point):
        self.x_ned_data.append(msg.x)
        self.y_ned_data.append(msg.y)
        # self.x_ned_data.append(msg.x)
        # self.vy_data.append(msg.linear.y)
        # self.vz_data.append(msg.linear.z)
        # self.vyaw_data.append(msg.angular.z)

        cur_time = time.time()-self.start_time
        self.ned_time.append(cur_time)

        # x_index = len(self.vx_data)
        # self.vx_x_data.append(x_index+1)

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
        # rospy.loginfo(msg.linear.z)



        cur_time = time.time()-self.start_time
        self.v_setpoint_time.append(cur_time)

        # x_index = len(self.vx_x_setpoint_data)
        # self.vx_x_setpoint_data.append(x_index+1)
        
    
    def update_plot(self, frame):
        # if len(self.v_setpoint_time)>10:
        self.ln1.set_data(self.v_time, self.vx_data)
        self.ln2.set_data(self.v_setpoint_time, self.vx_setpoint_data)

        self.ln3.set_data(self.v_time, self.vy_data)
        self.ln4.set_data(self.v_setpoint_time, self.vy_setpoint_data)

        self.ln5.set_data(self.v_time, self.vz_data)
        self.ln6.set_data(self.v_setpoint_time, self.vz_setpoint_data)

        self.ln7.set_data(self.v_time, self.vyaw_data)
        self.ln8.set_data(self.v_setpoint_time, self.vyaw_setpoint_data)

        self.ln9.set_data(self.pwm_time, self.x_pwm)
        self.ln10.set_data(self.pwm_time, self.y_pwm)
        self.ln11.set_data(self.pwm_time, self.z_pwm)
        self.ln12.set_data(self.pwm_time, self.yaw_pwm)

        self.ln13.set_data(self.ned_time, self.x_ned_data)
        self.ln14.set_data(self.ned_time, self.y_ned_data)



    
        if len(self.pwm_time)>10:
            if self.sliding_window:
                minx = max(self.v_setpoint_time)-30                           
            else:
                minx = 0

            # x limits
            self.ax1.set_xlim(minx, max(self.v_setpoint_time)+10)
            self.ax2.set_xlim(minx, max(self.v_setpoint_time)+10)
            self.ax3.set_xlim(minx, max(self.v_setpoint_time)+10)
            self.ax4.set_xlim(minx, max(self.v_setpoint_time)+10)
            self.ax5.set_xlim(minx, max(self.v_setpoint_time)+10)
            self.ax6.set_xlim(minx, max(self.v_setpoint_time)+10)
                        
            # y limits
            miny = -1
            maxy = 1

            miny_1= min(self.vx_data)
            miny_2= min(self.vx_setpoint_data)
            miny = min(miny_1, miny_2)
            maxy_1= max(self.vx_data)
            maxy_2= max(self.vx_setpoint_data)
            maxy = max(maxy_1, maxy_2)
            self.ax1.set_ylim(miny-0.5,maxy+0.5)
                            

            miny_1= min(self.vy_data)
            miny_2= min(self.vy_setpoint_data)
            miny = min(miny_1, miny_2)
            maxy_1= max(self.vy_data)
            maxy_2= max(self.vy_setpoint_data)
            maxy = max(maxy_1, maxy_2)
            self.ax2.set_ylim(miny-0.5,maxy+0.5)

            miny_1= min(self.vz_data)
            miny_2= min(self.vz_setpoint_data)
            miny = min(miny_1, miny_2)
            maxy_1= max(self.vz_data)
            maxy_2= max(self.vz_setpoint_data)
            maxy = max(maxy_1, maxy_2)
            self.ax3.set_ylim(miny-0.5,maxy+0.5)

            miny_1= min(self.vyaw_data)
            miny_2= min(self.vyaw_setpoint_data)
            miny = min(miny_1, miny_2)
            maxy_1= max(self.vyaw_data)
            maxy_2= max(self.vyaw_setpoint_data)
            maxy = max(maxy_1, maxy_2)
            self.ax4.set_ylim(miny-0.5,maxy+0.5)

          

            # miny = min(self.x_pwm,self.y_pwm,self.z_pwm, self.yaw_pwm)
            # maxy = max(self.x_pwm,self.y_pwm,self.z_pwm, self.yaw_pwm)
            # self.ax5.set_ylim(1000,2000)
            self.ax5.set_ylim(-1000,1000)

            miny = -1
            maxy = 1
            self.ax1.set_ylim(miny-0.5,maxy+0.5)
            self.ax2.set_ylim(miny-0.5,maxy+0.5)
            self.ax3.set_ylim(miny-0.5,maxy+0.5)

            miny = -0.5
            maxy = 0.5
            self.ax4.set_ylim(miny-0.5,maxy+0.5)

            miny = -0.5
            maxy = 0.5
            self.ax6.set_ylim(miny-0.5,maxy+0.5)
           


            
        # return self.ln1, self.ln2, self.ln3, self.ln4, self.ln5, self.ln6


rospy.init_node('velocity_plot')
rospy.loginfo("initialized velocity plot node")

vis = Visualiser()
sub1 = rospy.Subscriber('velocity_setpoint', Twist, vis.velocity_setpoint_callback)
sub2 = rospy.Subscriber('/dvl/twist', Twist, vis.velocity_callback)
sub3 = rospy.Subscriber('pwm_setpoint', Twist, vis.pwm_callback)
sub4 = rospy.Subscriber('sitl_velocity_xyz', Point, vis.NED_callback) 

while not rospy.is_shutdown():
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
    plt.show(block=True)
plt.close() 