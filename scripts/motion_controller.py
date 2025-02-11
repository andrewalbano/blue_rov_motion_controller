#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from tf.transformations import euler_from_quaternion
from mavros_msgs.msg import OverrideRCIn
import time
from pymavlink import mavutil
import threading

CONTROL_CLIP = (1400, 1600)

def update_plot():
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()

    while not rospy.is_shutdown():
        if len(x_vals) > 0:
            # x = current_position['x']
            # y = current_position['y']
            # yaw = current_position['yaw']

            ax.clear()  # Clear the previous plot
            
            # Plot the robot's trajectory (x, y)
            ax.plot(x_vals[-50:], y_vals[-50:], 'b-', label="Robot Path")
            
            # Plot the robot's orientation (yaw) as a direction indicator
            ax.arrow(x=x_vals[-1], y=y_vals[-1], dx=0.1 * np.cos(yaws[-1]), dy=0.1 * np.sin(yaws[-1]),
                        head_width=0.1, head_length=0.1, fc='r', ec='r', label="Yaw Direction")

            for wp in waypoints: 
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

# P Controller Gains
Kp_position = 200.0  # Proportional gain for position control (x, y)
Kp_yaw = 80       # Proportional gain for yaw control


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

# Set of waypoints (x, y) in the global frame
# waypoints = generate_lawnmower_pattern(4, 3, strip_width=.75, spacing=1)
waypoints = create_semicircle(np.array([0, 0]), radius=2, angle=np.pi/2, increments=10)

x_vals = []
y_vals = [] 
yaws = []

# Initialize current position and yaw
current_position = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

# Callback function to update the current position and yaw from /dvl/local_position
def position_callback(msg):
    global current_position

    # Extract position (x, y) from the message
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # Extract orientation (quaternion) and convert to Euler angles (roll, pitch, yaw)
    q = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # Update the global position and yaw
    current_position['x'] = x
    current_position['y'] = y
    current_position['yaw'] = yaw

    x_vals.append(x)
    y_vals.append(y)
    yaws.append(yaw)

# Function to calculate control inputs based on P controller in the body frame
def calculate_control(current_position, waypoint):
    # Global errors (in global frame)
    ex_g = waypoint[0] - current_position['x']
    ey_g = waypoint[1] - current_position['y']
    
    # Transform errors to the body frame using the robot's yaw
    ex = np.cos(current_position['yaw']) * ex_g + np.sin(current_position['yaw']) * ey_g
    ey = -np.sin(current_position['yaw']) * ex_g + np.cos(current_position['yaw']) * ey_g
    
    # Yaw error: difference between current yaw and desired yaw
    desired_yaw = np.arctan2(ey_g, ex_g)  # Desired yaw angle toward waypoint
    yaw_error = desired_yaw - current_position['yaw']
    # Normalize yaw error to [-pi, pi] range
    if yaw_error > np.pi:
        yaw_error -= 2 * np.pi
    elif yaw_error < -np.pi:
        yaw_error += 2 * np.pi

    # Proportional control for position (x, y) and yaw
    x_control = Kp_position * ex
    y_control = Kp_position * ey
    yaw_control = Kp_yaw * yaw_error

    return x_control, y_control, yaw_control

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

# Function to check if the robot has reached the waypoint (within a tolerance)
def reached_waypoint(current_position, waypoint, tolerance=0.2):
    distance = np.sqrt((current_position['x'] - waypoint[0]) ** 2 + (current_position['y'] - waypoint[1]) ** 2)
    return distance < tolerance

# Thread for real-time visualization using Matplotlib
def visualization_thread():
    plt.ion()  # Enable interactive mode
    plt.show()
    while not visualization_stop_event.is_set():
        plt.draw()
        plt.pause(0.1)  # Update plot every 0.1 seconds

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

def listener():
    # Initialize the ROS node
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


    # Subscribe to the local position topic
    rospy.Subscriber('/dvl/local_position', PoseWithCovarianceStamped, position_callback)

    # Start the real-time plotting in a separate thread
    plot_thread = threading.Thread(target=update_plot)
    plot_thread.daemon = True  # Make sure the thread exits when the main program exits
    plot_thread.start()

    # Start following the waypoints
    current_waypoint_index = 0
    while not rospy.is_shutdown():
        waypoint = waypoints[current_waypoint_index]
        
        # Calculate control inputs to reach the current waypoint
        x_control, y_control, yaw_control = calculate_control(current_position, waypoint)
        
        # Check if we need to rotate to the correct heading first
        # If yaw control (heading) is significant, we focus on turning first
        if abs(yaw_control) > 15.0:  # Threshold for turning (adjust as necessary)
            # Prioritize turning to the correct heading
            rospy.loginfo(f"Turning to heading. Current yaw error: {yaw_control}")
            send_control(0, 0, yaw_control, master)  # Only send yaw control (rotation)
        else:
            # Once the heading is correct, translate towards the waypoint
            rospy.loginfo(f"Yaw aligned. Translating towards waypoint.")
            send_control(x_control, y_control, 0, master)  # Only send translation control

        # Check if the robot has reached the current waypoint
        if reached_waypoint(current_position, waypoint, tolerance=0.2):
            rospy.loginfo(f"Reached waypoint {current_waypoint_index + 1}: {waypoint}")
            current_waypoint_index += 1

            # If all waypoints are reached, stop the robot
            if current_waypoint_index >= len(waypoints):
                rospy.loginfo("All waypoints reached. Stopping.")
                send_control(0, 0, 0, master)  # Stop the robot
                break

        # Wait for the next iteration
        rospy.sleep(0.1)

    # Signal to stop the visualization thread and close the plot
# Ensure the plotting thread finishes

def main(): 
    listener()
if __name__ == "__main__":
    main()