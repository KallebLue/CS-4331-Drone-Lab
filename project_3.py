""" 
Example script that allows a user to "push" the Crazyflie 2.1+ around 
using their hands while it's hovering. 
This example uses the Flow and Multi-ranger decks to measure distances 
in all directions and tries to keep away from anything that comes closer 
than 0.2m by setting a velocity in the opposite direction. 
The demo is ended by either pressing Ctrl-C or by holding your hand above the 
Crazyflie. 
""" 
import logging 
import sys 
import time 
import cflib.crtp 
from cflib.crazyflie import Crazyflie 
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie 
from cflib.positioning.motion_commander import MotionCommander 
from cflib.utils import uri_helper 
from cflib.utils.multiranger import Multiranger 
# Define the default URI for communication with the Crazyflie 
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7') 
# Only output errors from the logging framework 
logging.basicConfig(level=logging.ERROR) 
"""---------------------------------------------------------------------------
""" 
""" 
Check if an object is too close to the drone. 
Args: 
 range (float or None): Distance measured by the sensor. 
Returns: 
 bool: True if the object is closer than the minimum distance, otherwise 
False. 
""" 
def is_close(range): 
    MIN_DISTANCE = 0.2 # Minimum allowed distance in meters 
    if range is None: # If no valid distance is detected, return False 
        return False 
    else: 
        return range < MIN_DISTANCE # Return True if the object is within the limit 
"""---------------------------------------------------------------------------
""" 
if __name__ == '__main__': 
 # Initialize the low-level drivers for Crazyflie communication 
    cflib.crtp.init_drivers() 
 # Create a Crazyflie object with a cache for parameters 
    cf = Crazyflie(rw_cache='./cache') 
 # Establish a synchronous connection to the Crazyflie 
    with SyncCrazyflie(URI, cf=cf) as scf: 
 # Send an arming request to enable the drone for flight 
        scf.cf.platform.send_arming_request(True) 
        time.sleep(1.0) # Wait for arming process to complete 
 # Enter motion control mode 
        with MotionCommander(scf) as motion_commander: 
 # Activate the Multi-ranger deck for distance sensing 
            with Multiranger(scf) as multiranger: 
                keep_flying = True # Flag to control the flight loop 
                while keep_flying: 
                    VELOCITY = 0.5 # Movement speed in meters per second 
                    velocity_x = 0.0 # Initial horizontal velocity in X-axis 
                    velocity_y = 0.0 # Initial horizontal velocity in Y-axis 
 # Adjust velocity based on proximity to obstacles 
                    if is_close(multiranger.front): # Object detected in front 
                        velocity_x -= VELOCITY # Move backward 
                    if is_close(multiranger.back): # Object detected behind 
                        velocity_x += VELOCITY # Move forward 
                    if is_close(multiranger.left): # Object detected on the left 
                        velocity_y -= VELOCITY # Move right 
                    if is_close(multiranger.right): # Object detected on the right 
                        velocity_y += VELOCITY # Move left 
                    # If an object is detected above, stop flying 
                    if is_close(multiranger.up): 
                        keep_flying = False 
 # Apply calculated velocity values to move the drone 
                    motion_commander.start_linear_motion( 
                        velocity_x, velocity_y, 0) 
                    time.sleep(0.1) # Short delay before checking sensors again 
            print('Demo terminated!') # Indicate that the program has ended 