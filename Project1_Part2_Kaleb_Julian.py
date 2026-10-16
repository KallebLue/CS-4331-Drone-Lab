import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

import matplotlib.pyplot as plt

import math
import random

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
MIN_HEIGHT = 0.3

# Defines movement constraints 
BOX_LIMIT = 0.5   

# Maximum velocity of the drone
MAX_VEL = 0.15 

# Duration (seconds) the drone stays in the destination before moving to a new destination
MOVE_DURATION = 2 

# The drone runs for 30 (seconds) and then lands. 
MAX_RUN_TIME = 30 

# Maximum number of dummies in each trajectory, e.g., from the source to the destination (e.g., d’ or d’’) 
MAX_DUMMY = 2

# Set a boolean variable to tell if to do 3d or 2d navigation
ENABLE_3D = False

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

ax.set_title('Crazyflie Random Dummy-Waypoint Trajectory')

"---------------------------------------------------------------------------"
"""
Callback function that logs the position estimate of the Crazyflie.

"""
def log_pos_callback(timestamp,data, logconf):
    print(data)
    global position_estimate

    new_Position =  [
        data['stateEstimate.x'],
        data['stateEstimate.y'],
        data['stateEstimate.z']
    ]

    ax.plot([position_estimate[0],new_Position[0]],
            [position_estimate[1],new_Position[1]],
            [position_estimate[2],new_Position[2]],
            color = 'blue',
            linewidth = 1
            )

    position_estimate = new_Position
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

def distance_3d(p1, p2):

    return math.sqrt(
        (p1[0] - p2[0]) ** 2
        + (p1[1] - p2[1]) ** 2
        + (p1[2] - p2[2]) ** 2
    )

def clamp(value, low, high):
    return max(low, min(high, value))
 
def random_z():
    # Random height in 3D mode, constant height in 2D mode
    if ENABLE_3D:
        return random.uniform(MIN_HEIGHT, MAX_HEIGHT)
    return DEFAULT_HEIGHT

def plot_waypoint(position, waypoint_type):

    if waypoint_type == 'dummy':
        color = 'red'
    else:
        color = 'green'

    ax.plot(
        position[0],
        position[1],
        position[2],
        'o',
        color = color
    )

"---------------------------------------------------------------------------"
"""
Random Waypoint Mobility: picks a uniformly random real destination inside the
flight box (and at a random height when ENABLE_3D is True).
"""
def generate_random_destination(box):
 
    x_min, x_max, y_min, y_max = box
 
    return [
        random.uniform(x_min, x_max),
        random.uniform(y_min, y_max),
        random_z()
    ]

"---------------------------------------------------------------------------"
"""
Generates dummy locations. x and y are sampled inside the rectangle whose opposite
corners are the start and the destination. In 3D mode, z is sampled over the full
allowed height range (MIN_HEIGHT to MAX_HEIGHT). All dummies are kept inside the
flight box and the allowed height range.
"""
 
def generate_dummy_waypoints(start, destination, num_dummies, box):
 
    x_min = min(start[0], destination[0])
    x_max = max(start[0], destination[0])
 
    y_min = min(start[1], destination[1])
    y_max = max(start[1], destination[1])
 
    box_x_min, box_x_max, box_y_min, box_y_max = box
 
    waypoints = []
 
    for _ in range(num_dummies):
        x = clamp(random.uniform(x_min, x_max), box_x_min, box_x_max)
        y = clamp(random.uniform(y_min, y_max), box_y_min, box_y_max)
 
        if ENABLE_3D:
            # Dummy altitude is random over the full allowed height range,
            # independent of the start/destination heights
            z = random.uniform(MIN_HEIGHT, MAX_HEIGHT)
        else:
            # Keep waypoints at the destination altitude in 2D mode
            z = destination[2]
 
        waypoints.append([x, y, z])
 
    return waypoints

def order_waypoints_nearest_first(start, waypoints):
 
    remaining = list(waypoints)
    ordered = []
    current = list(start)
 
    while remaining:
 
        nearest = min(
            remaining,
            key=lambda point: distance_3d(current, point)
        )
 
        ordered.append(nearest)
        remaining.remove(nearest)
 
        current = nearest
 
    return ordered
 
def random_With_Dummy(scf, x0, y0):
    # x0, y0 = starting position estimate (takeoff point). The flight box is
    # centered on it, so destinations always stay within +/- BOX_LIMIT.
    box = (x0 - BOX_LIMIT, x0 + BOX_LIMIT, y0 - BOX_LIMIT, y0 + BOX_LIMIT)
 
    total_distance = 0.0
    total_waypoints = 0
    total_dummies = 0
    legs = 0
 
    with PositionHlCommander(
        scf,
        x=x0,
        y=y0,
        default_height=DEFAULT_HEIGHT,
        default_velocity=MAX_VEL,
        controller=PositionHlCommander.CONTROLLER_PID
    ) as pc:
 
        print("Taking off and stabilizing...")
        time.sleep(2)
 
        # Last commanded waypoint: used as the start of each leg and for the
        # distance bookkeeping (less noisy than the position estimate).
        last_waypoint = [x0, y0, DEFAULT_HEIGHT]
 
        start_time = time.time()
 
        while time.time() - start_time < MAX_RUN_TIME:
 
            # Select the next real destination (Random Waypoint)
            destination = generate_random_destination(box)
 
            # Generate exactly MAX_DUMMY (2) random dummy waypoints
            dummy_points = generate_dummy_waypoints(
                last_waypoint,
                destination,
                MAX_DUMMY,
                box
            )
 
            # Order dummies by nearest-first distance
            ordered_points = order_waypoints_nearest_first(
                last_waypoint,
                dummy_points
            )
 
            # The actual destination is always last
            route = ordered_points + [destination]
 
            legs += 1
 
            print(f"\nFlight leg: {legs}")
            print("Current position:", [round(v, 3) for v in last_waypoint])
            print("Real destination:", [round(v, 3) for v in destination])
            print("Dummy waypoints:", [[round(v, 3) for v in p] for p in ordered_points])
 
            for i, waypoint in enumerate(route):
 
                # Do not begin another movement after the time limit
                if time.time() - start_time >= MAX_RUN_TIME:
                    break
 
                # Fly to the absolute coordinate using the high-level commander
                pc.go_to(
                    waypoint[0],
                    waypoint[1],
                    waypoint[2],
                    velocity=MAX_VEL
                )
 
                # Approximate commanded route distance
                total_distance += distance_3d(
                    last_waypoint,
                    waypoint
                )
                last_waypoint = list(waypoint)
 
                total_waypoints += 1
 
                # Plot a dot after the movement command returns
                if i == len(route) - 1:
                    plot_waypoint(waypoint, 'real')
                    print("Reached real destination:", [round(v, 3) for v in waypoint])
 
                    # Stay at the real destination before the next leg
                    time.sleep(MOVE_DURATION)
                else:
                    plot_waypoint(waypoint, 'dummy')
                    total_dummies += 1
                    print("Reached dummy waypoint:", [round(v, 3) for v in waypoint])
 
        elapsed = time.time() - start_time
 
        print("\nFlight time limit reached.")
 
    print("\n------ Summary ------")
    print("Flight time (s):", round(elapsed, 1))
    print("Legs:", legs)
    print("Waypoint movements:", total_waypoints)
    print("Dummies visited:", total_dummies)
    print(
        "Approximate commanded route distance:",
        round(total_distance, 3),
        "meters"
    )
 

if __name__ == '__main__':
    print("Initializing drivers...")
    cflib.crtp.init_drivers()

    print("Connecting to Crazyflie...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected!")

        # Add callbacks to update stateEstimate positions as the drone moves
        logconf = LogConfig(name='Position', period_in_ms=200)
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

        # Start the logging process
        initial_estimate = position_estimate
        logconf.start()
    
        try:
            # Wait until the callback has delivered a real position sample
            # (it replaces the position_estimate list with a new one).
            wait_start = time.time()
            while position_estimate is initial_estimate:
                if time.time() - wait_start > 5:
                    print('No position data received! Exiting...')
                    sys.exit(1)
                time.sleep(0.05)
    
            time.sleep(0.5)  # let the estimate settle
    
            start_x, start_y = position_estimate[0], position_estimate[1]
            print(f"Start position: x={start_x:.3f}, y={start_y:.3f}")

            random_With_Dummy(scf, start_x, start_y)

        finally:
            # Stop the logging process (also runs if something goes wrong)
            logconf.stop()

    plt.show()
