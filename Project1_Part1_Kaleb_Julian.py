import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

import matplotlib.pyplot as plt

# Reference: https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/sbs_motion_commander/

# This script initializes a Crazyflie drone, checks for the necessary sensor decks, 
# and executes a simple takeoff and landing maneuver.

# Define the URI for connecting to the Crazyflie
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')

# Event used to detect if the necessary deck is attached
deck_attached_event = Event()

# Set logging level to suppress detailed debug logs
logging.basicConfig(level=logging.ERROR)

# Default flight height in meters
DEFAULT_HEIGHT = 0.5

# Maximum height in meters
MAX_HEIGHT = 0.5

# Minimum height in meters
MIN_HEIGHT = 0.4

# Defines movement constraints (not currently used in this script)
BOX_LIMIT = 0.5   

# Maximum velocity of the drone
MAX_VEL = 0.15 

# Duration (seconds) the drone stays in the destination before moving to a new destination
MOVE_DURATION = 2 

# The drone runs for 30 (seconds) and then lands. 
MAX_RUN_TIME = 30 

# Define position in three dimensions starting from the origin (0, 0, 0)
# [x, y, z] in meters
position_estimate = [0.000, 0.000, 0.000]  

# Create a 3 dimensional scatter plot drone movement
fig = plt.figure()
ax = plt.axes(projection='3d') 

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(0, 2)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

"---------------------------------------------------------------------------"
"""
Callback function that logs the position estimate of the Crazyflie.


"""
def log_pos_callback(timestamp,data, logcong):
    print(data)
    global position_estimate
    ax.plot([position_estimate[0],data['stateEstimate.x']]
            ,[position_estimate[1],data['stateEstimate.y']]
            ,[position_estimate[2],data['stateEstimate.z']])

    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    position_estimate[2] = data['stateEstimate.z']

"---------------------------------------------------------------------------"
"""
Callback function that checks if a specific sensor deck is attached.

Args:
    name (str): The name of the deck parameter.
    value_str (str): The value as a string, indicating whether the deck is detected.

If the deck is detected, an event is triggered.
"""
def param_deck_flow(name, value_str):
    value = int(value_str)
    if value:
        deck_attached_event.set()
        print(f'Deck {name} is attached!')
    else:
        print(f'Deck {name} is NOT attached!')


"---------------------------------------------------------------------------"
"""
Commands the Crazyflie to take off, hover for 3 seconds, and then land.

Args:
    scf (SyncCrazyflie): Synchronized Crazyflie object for safe communication.
"""
def take_off_simple(scf):
    print("Taking off...")
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)  # Hover in place for 3 seconds
        mc.start_forward(0.3)
        time.sleep(1)
        mc.right(0.4)
        mc.stop()  # Land the drone 

"---------------------------------------------------------------------------"
"""
Commands the Crazyflie to take off and bounce around in a virtual box of which the size is indicated by ‘BOX_LIMIT’.
"""
def move_box_limit(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        body_x_cmd = 0.2
        body_y_cmd = 0.1
        body_z_cmd = -0.05

        start_time = time.time()

        while (time.time() - start_time < MAX_RUN_TIME):
            #if position_estimate[0] > BOX_LIMIT:
            #    mc.start_back()
            #elif position_estimate[0] < -BOX_LIMIT:
            #    mc.start_forward()

            if position_estimate[0] > BOX_LIMIT:
                body_x_cmd = -MAX_VEL
            elif position_estimate[0] < -BOX_LIMIT:
                body_x_cmd = MAX_VEL
            if position_estimate[1] > BOX_LIMIT:
                body_y_cmd = -MAX_VEL
            elif position_estimate[1] < -BOX_LIMIT:
                body_y_cmd = MAX_VEL
            if position_estimate[2] > MAX_HEIGHT:
                body_z_cmd = -MAX_VEL
            elif position_estimate[2] < MIN_HEIGHT:
                body_z_cmd = MAX_VEL

            #time.sleep(MOVE_DURATION)
            ax.plot(position_estimate[0],position_estimate[1],position_estimate[2],'o')

            mc.start_linear_motion(body_x_cmd, body_y_cmd, body_z_cmd)

            time.sleep(0.1)

if __name__ == '__main__':
    print("Initializing drivers...")
    cflib.crtp.init_drivers()

    print("Connecting to Crazyflie...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected!")



        # Add callbacks to update stateEstimate positions as the drone moves
        logconf = LogConfig(name='Position', period_in_ms=1000)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)


        # Add callbacks to check if required decks are attached
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
        scf.cf.param.add_update_callback(group="deck", name="bcMultiranger", cb=param_deck_flow)
        
        time.sleep(1)  # Allow time for the deck check to complete

        # If no flow deck is detected within 5 seconds, exit the script
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected! Exiting...')
            sys.exit(1)

        #Start thr logging process
        logconf.start()

        # Execute the takeoff and landing maneuver
        #take_off_simple(scf)

        move_box_limit(scf)

        #Stop the logging process
        logconf.stop()

        plt.show()
