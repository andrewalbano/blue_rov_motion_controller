"""!
The state machine that implements the logic.
"""
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot, QTimer
import time
import numpy as np
import rclpy
import camerameasurements
import cv2

from  motion_controller_v6 import MotionControl
import rospy




while not rospy.is_shutdown():
    # controller.time = rospy.get_time()
    
    

    rospy.sleep(0.1)   

class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the blueROV2
    """

    def __init__(self, rxarm, camera):
        """!
        @brief      Constructs a new instance.

        @param      rxarm   The rxarm
        @param      planner  The planner
        @param      camera   The camera
        """
        self.rxarm = rxarm
        self.camera = camera
        self.camerameasurements = camerameasurements
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.waypoints = [
            [-np.pi/2,       -0.5,      -0.3,          0.0,        0.0],
            [0.75*-np.pi/2,   0.5,       0.3,     -np.pi/3,    np.pi/2],
            [0.5*-np.pi/2,   -0.5,      -0.3,      np.pi/2,        0.0],
            [0.25*-np.pi/2,   0.5,       0.3,     -np.pi/3,    np.pi/2],
            [0.0,             0.0,       0.0,          0.0,        0.0],
            [0.25*np.pi/2,   -0.5,      -0.3,          0.0,    np.pi/2],
            [0.5*np.pi/2,     0.5,       0.3,     -np.pi/3,        0.0],
            [0.75*np.pi/2,   -0.5,      -0.3,          0.0,    np.pi/2],
            [np.pi/2,         0.5,       0.3,     -np.pi/3,        0.0],
            [0.0,             0.0,       0.0,          0.0,        0.0]]
        self.waypoints_record = []

        self.calibration_point = np.array([4*50,5.5*50,0])

        self.image_point = np.zeros(shape=(1,3))

        self.moving_time = rxarm.moving_time
        self.accel_time = rxarm.accel_time
        self.i = 0
        

    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

            This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    TODO: Add states and functions as needed.
        """

        # IMPORTANT: This function runs in a loop. If you make a new state, it will be run every iteration.
        #            The function (and the state functions within) will continuously be called until the state changes.

        if self.next_state == "initialize_motion_controller":
            self.motion_control()

        if self.next_state == "DVL_boundary_scan": 
            # Boundary scan could be implemented with waypoints to define rotations, but easier to have ROV rotate at constant rate, no need for error correction
            self.DVL_build_map()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()

        if self.next_state == "detect":
            self.detect()

        if self.next_state == "manual":
            self.manual()
        
        if self.next_state ==  "record":
            # print("recording")
            self.record()

        if self.next_state == "repeat":
            self.repeat()
        
        if self.next_state == "home":
            self.home()

        if self.next_state == "motion_control":
            self.motion_control()
        
        # if controller.current_waypoint == 4:
        #     rospy.loginfo(f"error, x,y,yaw: {controller.error_x}, {controller.error_y}, {controller.error_yaw}")
            


        


    """Functions run for each state"""

    def motion_control(self):
        # Must run in loop until next state

        # initialize motion controller
        self.controller = MotionControl()

        while self.controller.invoked() and not self.controller.reached_waypoint():
            if self.controller.invoked: # Call waypoint follower state here:
                rospy.loginfo_throttle(30,"controller is active")
                # rospy.loginfo("controller is turned on")
                # rospy.loginfo("The current z position is:\n " + str(controller.current_pose.pose.position.z)) 
                if self.controller.hold_pose:
                    self.controller.current_waypoint.pose = self.controller.hold_pose_waypoint
                else:
                    self.controller.get_current_waypoint()
            
                # rospy.loginfo("The current z waypoint position is:\n " + str(controller.current_waypoint.pose.position.z)) 
                self.controller.calculate_control()
                # rospy.loginfo("The current x control is:\n " + str(controller.x_control)) 
                self.controller.send_sim_control()  

                # use this for hardware
                # send_control(controller.x_control, controller.y_control, controller.yaw_control, master):
                if self.controller.hold_pose:
                    rospy.loginfo_throttle(15,f"Holding postion")
                elif self.controller.reached_waypoint():
                    if self.controller.waypoint_index < self.controller.num_waypoints-1:
                        rospy.loginfo(f"Reached waypoint {self.controller.waypoint_index +1}: {self.controller.current_waypoint.pose.position.x}, {self.controller.current_waypoint.pose.position.y}, {self.controller.current_waypoint.pose.position.z}")
                        self.controller.waypoint_index +=1
                        self.controller.get_current_waypoint()
                        rospy.loginfo(f"Heading to waypoint {self.controller.waypoint_index +1}: {self.controller.current_waypoint.pose.position.x}, {self.controller.current_waypoint.pose.position.y}, {self.controller.current_waypoint.pose.position.z}")
                    else:
                        rospy.loginfo_throttle(15,f"Reached the last waypoint, holding postion at waypoint {self.controller.waypoint_index +1}")
            
            # If controller is not on, send state back to idle
            elif self.controller.invoked == False: # Define controller "off" state here:
                # rospy.loginfo("controller is turned off")
                rospy.loginfo_throttle(30,"controller is inactive")
                self.controller.invoked = False
                self.controller.send_sim_control()
            
            
            # use this for hardware
            # controller.x_control = 0.0
            # controller.y_control = 0.0
            # controller.z_control = 0.0
            # controller.roll_control = 0.0
            # controller.pitch_control = 0.0
            # controller.yaw_control = 0.0
            # send_control(controller.x_control, controller.y_control, controller.yaw_control, master):
            
            else: # Might get stuck in this state if not careful
                rospy.loginfo("controller is weird")
                    
        self.next_state = "idle"

    def DVL_build_map(self):
        
        self.rxarm.arm.go_to_home_pose(moving_time=self.moving_time,
                             accel_time=self.accel_time,
                             blocking=True)
        self.next_state = "idle"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check rxarm and restart program"
        self.current_state = "estop"
        self.rxarm.disable_torque()

    def execute(self):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        self.status_message = "State: Execute - Executing motion plan"
        self.rxarm.arm.go_to_home_pose(moving_time=self.moving_time,
                             accel_time=self.accel_time,
                             blocking=True)
        
        i = 0
        while i < len(self.waypoints) and self.next_state == "execute":
            # self.joints = self.rxarm.get_positions()
            self.rxarm.set_positions(self.waypoints[i])
            time.sleep(3)
            i += 1

        self.next_state = "idle"

    

    def record(self):
        i = 0
        speedoflight = 3e8
        while self.next_state == "record":

            if i >= speedoflight and i % speedoflight == 0:
                self.waypoints_record.append(self.rxarm.get_positions())
                # print("i=",i)
                print(len(self.waypoints_record),self.waypoints_record)

            i += 10
            # print("i=",i)

    def repeat(self):
        self.status_message = "State: Repeat - Executing motion plan"
        i = 0
        while i < len(self.waypoints_record) and self.next_state == "repeat":
            self.rxarm.set_positions(self.waypoints_record[i])
            time.sleep(3)
            i += 1

        self.next_state = "home"

    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"
        print("Click atleast 7 points on screeen")
        self.index = 0
        while self.image_point.shape[0] < 7:
            
            temp = np.array([self.camera.last_click[0], self.camera.last_click[1], self.camera.DepthFrameRaw[self.camera.last_click[1]][self.camera.last_click[0]]])
            if self.index == 0:
                self.image_point = temp
            else:
                self.image_point = np.append(temp,axis=0)
            self.index += 1

        print(self.image_point)
        # AffineTransform = self.camera.getAffineTransform(self.calibration_point,self.image_point)
        # print(AffineTransform)
        self.next_state = "idle"

        """TODO Perform camera calibration routine here"""
        self.status_message = "Calibration - Completed Calibration"       


    """ TODO """
    def detect(self):
        """!
        @brief      Detect the blocks
        """
        time.sleep(1)

    def initialize_rxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.current_state = "initialize_rxarm"
        self.status_message = "RXArm Initialized!"
        if not self.rxarm.initialize():
            print('Failed to initialize the rxarm')
            self.status_message = "State: Failed to initialize the rxarm!"
            time.sleep(5)
        self.next_state = "idle"

    

class StateMachineThread(QThread):
    """!
    @brief      Runs the state machine
    """
    updateStatusMessage = pyqtSignal(str)
    
    def __init__(self, state_machine, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      state_machine  The state machine
        @param      parent         The parent
        """
        QThread.__init__(self, parent=parent)
        self.sm=state_machine

    def run(self):
        """!
        @brief      Update the state machine at a set rate
        """
        while True:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            time.sleep(0.05)