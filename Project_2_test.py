import random
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

# Defines movement constraints (not currently used in this script)
BOX_LIMIT = 0.5     


num_win = 0
num_lose = 0
game_log = []
game_rounds = 0
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


"---------------------------------------------------------------------------"
"""
Commands the Crazyflie to take off, hover for 3 seconds, and then land.

Args:
    scf (SyncCrazyflie): Synchronized Crazyflie object for safe communication.
"""
def take_off_simple(scf):
    print("Taking off...")
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
    	time.sleep(1)

if __name__ == '__main__':
    print("Initializing drivers...")
    cflib.crtp.init_drivers()

    print("Connecting to Crazyflie...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print("Connected!")

        # Add callbacks to check if required decks are attached
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=param_deck_flow)
        scf.cf.param.add_update_callback(group="deck", name="bcMultiranger", cb=param_deck_flow)
        
        time.sleep(1)  # Allow time for the deck check to complete

        # If no flow deck is detected within 5 seconds, exit the script
        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected! Exiting...')
            sys.exit(1)

        # Execute the takeoff and landing maneuver
        # take_off_simple(scf)
		cf.param.set_value('ring.effect', 7)
		time.sleep(0.1)
        # Execute the silly takeoff maneuver
        take_off_simple(scf)


		while num_win < 2 and num_lose < 2:
			num_drone = random.randint(0,2)
			num_user = int(input("\nPick Rock (0), Paper (1), or Scissors (2): "))
			while num_user < 0 or num_user > 2:
				print("\nPlease enter a valid number (0, 1, or 2)")
				num_user = int(input())
			if num_drone == 0:
				if num_user == 1:
					print("User wins!")
					num_lose+=1
					game_rounds+=1
					game_log.append("Drone L")
					cf.param.set_value('ring.solidRed', 255)
					mc.down(0.1)
				elif num_user == 2:
					print("Drone wins!")
					num_win+=1
					game_rounds+=1
					game_log.append("Drone W")
					cf.param.set_value('ring.solidGreen', 0)
					mc.up(0.1)
				else:
					print("Tie!")
					game_rounds+=1
					game_log.append("Tie")
					cf.param.set_value('ring.solidBlue', 255)
			elif num_drone == 1:
				if num_user == 0:
					print("User wins!")
					num_lose+=1
					game_rounds+=1
					game_log.append("Drone L")
					cf.param.set_value('ring.solidRed', 255)
					mc.down(0.1)
				elif num_user == 2:
					print("Drone wins!")
					num_win+=1
					game_rounds+=1
					game_log.append("Drone W")
					cf.param.set_value('ring.solidGreen', 0)
					mc.up(0.1)
				else:
					print("Tie!")
					game_rounds+=1
					game_log.append("Tie")
					cf.param.set_value('ring.solidBlue', 255)
			else:
				if num_user == 0:
					print("User wins!")
					num_lose+=1
					game_rounds+=1
					game_log.append("Drone L")
					cf.param.set_value('ring.solidRed', 255)
					mc.down(0.1)
				elif num_user == 1:
					print("Drone wins!")
					num_win+=1
					game_rounds+=1
					game_log.append("Drone W")
					cf.param.set_value('ring.solidGreen', 0)
					mc.up(0.1)
				else:
					print("Tie!")
					game_rounds+=1
					game_log.append("Tie")
					cf.param.set_value('ring.solidBlue', 255)
			

		cf.param.set_value('ring.solidBlue', 255)
		mc.stop()
		print("\nGame Log: ",game_log)
		print("Number of Rounds: ",game_rounds)