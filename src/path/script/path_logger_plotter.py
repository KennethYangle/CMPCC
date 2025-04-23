#!/usr/bin/env python3

import matplotlib
matplotlib.use('TkAgg') # Or your preferred backend

import rospy
import numpy as np
import matplotlib.pyplot as plt
# --- Import 3D Toolkit ---
from mpl_toolkits.mplot3d import Axes3D
# --- ------------------- ---
import matplotlib.cm as cm
from matplotlib.colors import Normalize
from collections import deque
import threading
import math
import time

# Import ROS messages
from swarm_msgs.msg import DiscreteTrajectory, DiscreteTrajectoryPoint
from geometry_msgs.msg import PoseStamped, TwistStamped

# --- Configuration ---
MAX_HISTORY = rospy.get_param("~max_history", 500)
MIN_VEL = rospy.get_param("~min_velocity", 0.0)
MAX_VEL = rospy.get_param("~max_velocity", 5.0)
UPDATE_RATE = rospy.get_param("~plot_update_rate", 10.0)
ENABLE_PLOTTING = rospy.get_param("~enable_plotting", True)

# --- Global Variables ---
ref_traj_points = []
drone_history = deque(maxlen=MAX_HISTORY)
latest_velocity = (0.0, 0.0, 0.0)
has_ref_traj = False
has_velocity = False
ref_traj_lock = threading.Lock()
drone_history_lock = threading.Lock()
velocity_lock = threading.Lock()

# Matplotlib (ax will be 3D)
fig = None
ax = None # Will be assigned Axes3D object
cbar = None
plot_initialized = False

# --- ROS Callbacks ---
# (Callbacks ref_traj_callback, velocity_callback, pose_callback remain IDENTICAL)
def ref_traj_callback(msg): # Collects X, Y, Z
    global ref_traj_points, has_ref_traj
    with ref_traj_lock:
        if not msg.points: ref_traj_points, has_ref_traj = [], False
        else: ref_traj_points, has_ref_traj = msg.points, True

def velocity_callback(msg): # Collects vx, vy, vz
    global latest_velocity, has_velocity
    with velocity_lock: latest_velocity, has_velocity = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z), True

def pose_callback(msg): # Calculates speed using vx, vy, vz
    global drone_history
    if not has_velocity: return
    vx, vy, vz = 0.0, 0.0, 0.0
    with velocity_lock: vx, vy, vz = latest_velocity
    speed = math.sqrt(vx**2 + vy**2 + vz**2)
    with drone_history_lock:
        drone_history.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, speed, msg.header.stamp.to_sec()))


# --- Plotting Function (Modified for 3D) ---
def update_plot():
    """Updates the matplotlib 3D plot. Assumes called from main thread."""
    global fig, ax, cbar, plot_initialized

    if not ENABLE_PLOTTING or plt is None: return

    # --- Initialize 3D Plot on first call ---
    if not plot_initialized:
        try:
            plt.ion()
            fig = plt.figure() # Create figure instance first
            # --- Create 3D Axes ---
            ax = fig.add_subplot(111, projection='3d')
            # --- ---------------- ---
            fig.canvas.manager.set_window_title("3D Trajectory Plot")
            plot_initialized = True
            rospy.loginfo("Matplotlib 3D plot initialized.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Matplotlib 3D plotting: {e}")
            return

    # --- Get data (including Z coordinate) ---
    ref_x, ref_y, ref_z = [], [], [] # Add ref_z
    with ref_traj_lock:
        if has_ref_traj:
            for p in ref_traj_points:
                ref_x.append(p.position.x)
                ref_y.append(p.position.y)
                ref_z.append(p.position.z) # Get Z

    drone_x, drone_y, drone_z, drone_speeds = [], [], [], [] # Add drone_z
    with drone_history_lock:
        if drone_history:
            data = list(drone_history)
            for item in data:
                drone_x.append(item[0])
                drone_y.append(item[1])
                drone_z.append(item[2]) # Get Z
                drone_speeds.append(item[3])

    # --- Perform 3D Plotting ---
    try:
        ax.clear()

        # Plot reference trajectory (3D)
        if ref_x:
            ax.plot(ref_x, ref_y, ref_z, 'b--', linewidth=1.5, label='Reference Traj') # Pass Z

        # Plot drone trajectory (3D Scatter)
        if drone_x:
            norm = Normalize(vmin=MIN_VEL, vmax=MAX_VEL)
            cmap = cm.get_cmap('jet')
            # --- Use 3D Scatter ---
            sc = ax.scatter(drone_x, drone_y, drone_z, c=drone_speeds, cmap=cmap, norm=norm, s=10, label='Drone Path (Speed)')
            # --- -------------- ---

            # Add/Update Colorbar
            if cbar is None and fig is not None:
                try:
                     cbar = fig.colorbar(sc, ax=ax, shrink=0.6) # Adjust shrink factor for 3D
                     cbar.set_label('Speed (m/s)')
                except Exception as e:
                     rospy.logwarn_throttle(10.0, f"Could not create 3D colorbar: {e}")
                     cbar = "error"
            elif cbar is not None and cbar != "error":
                try:
                     cbar.update_normal(sc)
                except Exception as e:
                     rospy.logwarn_throttle(10.0, f"Could not update 3D colorbar: {e}")


        # Plot settings for 3D
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)") # Add Z label
        ax.set_title("Real-time 3D Trajectory Tracking")
        ax.grid(True)
        ax.legend()
        # Optional: Set axis limits if needed, e.g., ax.set_zlim(0, 5)
        # Optional: Set initial view angle
        # ax.view_init(elev=20., azim=-35)

        # Set aspect ratio for 3D (can be tricky)
        # Matplotlib's 3D axis scaling isn't always truly 'equal' visually.
        # A common approach is to make the ranges roughly equal if known
        # Or manually set limits based on data range
        try: # Setting limits might fail if data is empty
             all_x = ref_x + drone_x
             all_y = ref_y + drone_y
             all_z = ref_z + drone_z
             if all_x and all_y and all_z: # Check if lists are not empty
                 max_range = np.array([max(all_x)-min(all_x), max(all_y)-min(all_y), max(all_z)-min(all_z)]).max() / 2.0
                 mid_x = (max(all_x)+min(all_x)) * 0.5
                 mid_y = (max(all_y)+min(all_y)) * 0.5
                 mid_z = (max(all_z)+min(all_z)) * 0.5
                 ax.set_xlim(mid_x - max_range, mid_x + max_range)
                 ax.set_ylim(mid_y - max_range, mid_y + max_range)
                 ax.set_zlim(mid_z - max_range, mid_z + max_range)
             else: # Fallback if no data yet
                 ax.axis('auto')
        except Exception as e:
             rospy.logwarn_throttle(10, f"Could not set 3D axis limits: {e}")
             ax.axis('auto') # Fallback


        # Request redraw and process GUI events
        plt.draw()
        plt.pause(0.001)

    except Exception as e:
        rospy.logerr_throttle(5.0, f"Error during 3D plot update: {e}")


# --- Main Execution ---
if __name__ == '__main__':
    rospy.init_node('path_logger_plotter', anonymous=True)

    # Subscribers
    rospy.Subscriber("/stitched_trajectory", DiscreteTrajectory, ref_traj_callback, queue_size=5)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback, queue_size=10)
    rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, velocity_callback, queue_size=10)

    rospy.loginfo("Path Logger/Plotter Node Started.")
    rospy.loginfo(f"Plotting Enabled: {ENABLE_PLOTTING}")

    # --- Main Loop for Plotting ---
    if ENABLE_PLOTTING:
        plot_rate = rospy.Rate(UPDATE_RATE)
        rospy.loginfo(f"Starting plot loop at {UPDATE_RATE:.1f} Hz.")
        # Plot initialization happens in the first call to update_plot
        while not rospy.is_shutdown():
            update_plot() # Call plotting function from main thread
            try:
                plot_rate.sleep()
            except rospy.ROSTimeMovedBackwardsException:
                 rospy.logwarn("ROS time moved backwards, adjusting sleep.")
                 time.sleep(1.0 / UPDATE_RATE if UPDATE_RATE > 0 else 0.1)
            except rospy.ROSInterruptException:
                 rospy.loginfo("ROS interrupt received.")
                 break
    else:
        rospy.loginfo("Plotting disabled. Node will only process callbacks.")
        rospy.spin()

    # --- Cleanup ---
    if ENABLE_PLOTTING and plot_initialized:
        try:
             plt.ioff()
             plt.close(fig)
             rospy.loginfo("Matplotlib window closed.")
        except Exception as e:
             rospy.logerr(f"Error closing plot: {e}")