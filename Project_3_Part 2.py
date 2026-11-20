import signal
import logging
import sys
import time
import random
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger 

# Define the default URI for communication with the Crazyflie
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')

# Flight parameters
MAX_HEIGHT = 0.8 # 
DEFAULT_HEIGHT = 0.4  
MIN_DISTANCE = 0.2  
BOX_LIMIT = .7
GOAL_X = 2.5
BUFFER = 0.05  
VELOCITY = 0.15  
MAX_RUN_TIME = 180
COLLISION_COUNT = 0


# Position Tracking
position_estimate = [0, 0,0]  
position_x, position_y, position_z = [], [], []  

# Global flag to stop flight safely
keep_flying = True  


"---------------------------------------------------------------------------"
"""Handles Ctrl+C (SIGINT) to land the drone safely."""
def signal_handler(sig, frame):
    global keep_flying
    print("\nCtrl+C detected! Landing the drone safely...")
    keep_flying = False  # Stop the main loop
"---------------------------------------------------------------------------"


# Register the Ctrl+C signal handler
signal.signal(signal.SIGINT, signal_handler)


"---------------------------------------------------------------------------"
"""Check if an object is too close to the drone."""
def is_close(range):
    return range is not None and range < MIN_DISTANCE
"---------------------------------------------------------------------------"


"---------------------------------------------------------------------------"
""" Updates position estimate and stores data for visualization """
def log_pos_callback(timestamp, data, logconf):
    global position_estimate, position_x, position_y, position_z
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']
    position_x.append(position_estimate[0])
    position_y.append(position_estimate[1])
    position_z.append(position_estimate[2])
"---------------------------------------------------------------------------"


"---------------------------------------------------------------------------"
"""Plots the drone's movement in 2D space."""
def plot_position_data():
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(projection='3d')

    ax.plot(position_x, position_y, position_z, '-', linewidth=2, label="Drone Path")
    if position_x:
        ax.scatter(position_x[0], position_y[0], position_z[0], color='green', s=40, label="Start")
        ax.scatter(position_x[-1], position_y[-1], position_z[-1], color='red', s=40, label="Landing")

    ax.set_xlabel("X Position (m) - forward")
    ax.set_ylabel("Y Position (m) - left +")
    ax.set_zlabel("Z Position (m) - height")
    ax.set_ylim(-BOX_LIMIT, BOX_LIMIT)
    ax.set_zlim(0, MAX_HEIGHT)
    ax.set_title(f"Drone Movement Path | Collisions: {COLLISION_COUNT}")
    ax.legend()
    plt.show()
"---------------------------------------------------------------------------"


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        # Enable position logging
        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)
        logconf.start()

        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as motion_commander:
            with Multiranger(scf) as multiranger:
                horizontal_direction = 0  # Initialize to show that a direction has not been chosen
                start_time = time.time()       # Fixed variable name from 'start'
                try:
                    while keep_flying:
                        elapsed_time = time.time() - start_time  # Calculate elapsed time

                        # **Check if time limit is reached**
                        if elapsed_time >= MAX_RUN_TIME:
                            print("Time limit reached! Landing the drone safely.")
                            motion_commander.land()
                            break

                        velocity_x, velocity_y = VELOCITY, 0.0

                        # **Check if drone has reached the goal**
                        if position_estimate[0] >= GOAL_X:
                            print("Goal reached! Landing.")
                            motion_commander.land()
                            break

                        if is_close(multiranger.right):
                            print("Object detected to the right! Stopping.")
                            motion_commander.stop()
                            COLLISION_COUNT += 1
                            time.sleep(0.5)
                            
                            print("Moving leftward.")
                            motion_commander.start_linear_motion(0, VELOCITY, 0)
                            time.sleep(1)
                            motion_commander.stop()

                        if is_close(multiranger.left):
                            print("Object detected to the left! Stopping.")
                            motion_commander.stop()
                            COLLISION_COUNT += 1
                            time.sleep(0.5)
                            
                            print("Moving rightward.")
                            motion_commander.start_linear_motion(0, -VELOCITY, 0)
                            time.sleep(1)
                            motion_commander.stop()
                            
                        # **Obstacle Avoidance**
                        if is_close(multiranger.front):
                            print("Object detected in front! Stopping.")
                            motion_commander.stop()
                            COLLISION_COUNT += 1
                            time.sleep(0.5)

                            print("Moving backward.")
                            motion_commander.start_linear_motion(-VELOCITY, 0, 0)
                            time.sleep(1)
                            motion_commander.stop()



                            if horizontal_direction == 0:
                                # Randomly choose between right (1) and left (2)
                                horizontal_direction = random.choice([1, 2, 3]) 

                            # Don't go up if that would pass the height limit
                            if horizontal_direction == 3 and position_estimate[2] + VELOCITY > MAX_HEIGHT - BUFFER:
                                print("Too high to go up, choosing a side instead.")
                                horizontal_direction = random.choice([1, 2])

                            #Once a direction is chosen it is kept and the drone moves in the same direction
                            if horizontal_direction == 1:
                                print("Moving right.")
                                motion_commander.start_linear_motion(0, -VELOCITY, 0)
                            elif horizontal_direction == 2:
                                print("Moving left.")       
                                motion_commander.start_linear_motion(0, VELOCITY, 0)
                            elif horizontal_direction == 3:
                                print("Moving up.")
                                motion_commander.start_linear_motion(0, 0, VELOCITY)
                            else:
                                print("invalid value inside of horizontal direction variable")
                                horizontal_direction = 0
                            
                            time.sleep(1)
                            motion_commander.stop()

                        # **Boundary Handling**
                        if position_estimate[1] > BOX_LIMIT - BUFFER:
                            print("Hit left boundary, stopping.")
                            motion_commander.stop()
                            time.sleep(0.5)

                            print("Moving right.")
                            motion_commander.start_linear_motion(0, -VELOCITY, 0)  # move right
                            time.sleep(1)
                            motion_commander.stop()
                            horizontal_direction = 1

                            print("Continuing forward.")

                        elif position_estimate[1] < -BOX_LIMIT + BUFFER:
                            print("Hit right boundary, stopping.")
                            motion_commander.stop()
                            time.sleep(0.5)

                            print("Moving left.")
                            motion_commander.start_linear_motion(0, VELOCITY, 0)  # move left
                            time.sleep(1)
                            motion_commander.stop()
                            horizontal_direction = 2
                            print("Continuing forward.")

                        elif position_estimate[2] > MAX_HEIGHT - BUFFER:
                            print("Hit upper boundary, stopping")
                            motion_commander.stop()
                            time.sleep(.5)

                            print("Moving Down")
                            motion_commander.start_linear_motion(0, 0, -VELOCITY)  # move down
                            time.sleep(1)
                            motion_commander.stop()

                        if is_close(multiranger.up):  # If ceiling is detected
                            print("Ceiling detected! Stopping.")
                            keep_flying = False

                        # **Move if no obstacles detected**
                        if not is_close(multiranger.front):
                            motion_commander.start_linear_motion(velocity_x, velocity_y, 0)

                        time.sleep(0.1)

                except KeyboardInterrupt:
                    print("\nEmergency stop triggered (Ctrl+C). Landing now...")
                    motion_commander.land()
                    time.sleep(1)

        logconf.stop()

    print(f"Total collisions: {COLLISION_COUNT}")
    # Plot recorded position data after flight
    plot_position_data()
