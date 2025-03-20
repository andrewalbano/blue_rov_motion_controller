#!/usr/bin/env python3
import rospy
import numpy as np
# import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,PoseArray, TwistStamped, Pose, Twist
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion, euler_matrix
from mavros_msgs.msg import OverrideRCIn
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
from std_msgs.msg import Bool, Int8, Float32MultiArray
import time

class NavigationPlanner():
    def __init__(self):
        queue_size = 1
        self.frequency = 10
        self.rate = rospy.Rate(self.frequency)
        self.map_sub = rospy.Subscriber('motion_control_state',OccupancyGrid, self.map_callback)
        self.current_pose_sub = rospy.Subscriber('/state', PoseWithCovarianceStamped, self.current_pose_callback)
        # self.path_plan_pub = rospy.Publisher('occupancy_map', OccupancyGrid, queue_size=queue_size)

        self.map = None
        self.map_grid = None
        self.start = None
        self.goal = None

# G = [];
#     for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
#         G[iind] = [];
#         for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
#             G[iind][jind] = {
#                 i:iind,j:jind, // mapping to graph array
#                 x:xpos,y:ypos, // mapping to map coordinates
#                 parent:null, // pointer to parent in graph along motion path
#                 distance:10000, // distance to start via path through parent
#                 visited:false, // flag for whether the node has been visited
#                 priority:null, // visit priority based on fscore
#                 queued:false, // flag for whether the node has been queued for visiting
#                 costCalculated:false //check if a cost has been calculated for the cell
#             };

      

    def map_callback(self,msg:OccupancyGrid):
        self.map = msg
        self.map_grid = np.array(self.map.data).reshape(self.map.info.width, self.map.info.height)

    def current_pose_callback(self,msg:PoseWithCovarianceStamped):
        self.current_pose = msg
        

    def set_start(self,start_pose):
        self.start = start_pose

    def set_goal(self,goal_pose):
        self.goal = goal_pose

  
    def get_grid_square(self,x,y):
        row = int(np.floor((x/self.map.info.resolution) + np.abs(self.map.info.origin.position.x/self.map.info.resolution)))
        col = int(np.floor((y/self.map.info.resolution) + np.abs(self.map.info.origin.position.y/self.map.info.resolution)))

    def collision(self, row, col):
        if self.map_grid[row,col] == 0:
            return False
        elif self.map_grid[row,col] > 0:
            # for now obstacles are guarenteed as anything greater than 0
            return True
        elif self.map_grid[row,col] == -1:
            # for now if unbknown set as collision
            return True

     

class Node():
    def __init__(self,row, col, resolution, start_x, start_y):
        self.i = row
        self.j = col
        self.x = row*resolution
        
        row = int(np.floor((x/self.map.info.resolution) + np.abs(self.map.info.origin.position.x/self.map.info.resolution)))
        col = int(np.floor((y/self.map.info.resolution) + np.abs(self.map.info.origin.position.y/self.map.info.resolution)))
    
        
    
    
def main(): 

     
    print(f"row =  {row} , col = {col}")
    # # Initialize the ROS node
    # rospy.init_node('navigation_planner')
    # nav = NavigationPlanner()

    

    # while not rospy.is_shutdown():

      

    #     nav.rate.sleep()    

    
if __name__ == "__main__":

    main()