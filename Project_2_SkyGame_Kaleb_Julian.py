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


# Reference: https://www.bitcraze.io/documentation/repository/crazyflie-lib-py10020thon/master/user-guides/sbs_motion_commander/

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

MOVE_DISTANCE = 0.1

MAX_WINS = 2


# Game statistics
num_win = 0       # Drone wins
num_lose = 0      # User wins
game_log = []
game_rounds = 0

CHOICES = ["Rock", "Paper", "Scissors"]

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


def get_user_choice():
    """Read and validate the user's choice."""
    while True:
        try:
            choice = int(
                input("\nPick Rock (0), Paper (1), "
                      "or Scissors (2): ")
            )

            if 0 <= choice <= 2:
                return choice

            print("Please enter 0, 1, or 2.")

        except ValueError:
            print("Invalid input. Please enter a number.")


def skygame(scf):
    cf = scf.cf

    print("Starting Sky Game...")

    with MotionCommander(
        scf, default_height=DEFAULT_HEIGHT
    ) as mc:
        try:
            print(f"Taking off to {DEFAULT_HEIGHT} meters...")
            time.sleep(3)


            play_game(cf, mc)

        except (KeyboardInterrupt, EOFError):
            print("\nGame interrupted. Preparing to land.")

        finally:

            mc.stop()

    # Exiting MotionCommander normally initiates landing.
    print("Landed. MotionCommander closed.")


def play_game(cf, mc):
    """Play until either the drone or user wins two rounds."""
    global num_win, num_lose, game_rounds

    print("\n--- Rock, Paper, Scissors ---")
    print("First to 2 wins is the champion!")

    while num_win < MAX_WINS and num_lose < MAX_WINS:
        num_drone = random.randint(0, 2)
        num_user = get_user_choice()

        print(f"You chose: {CHOICES[num_user]}")
        print(f"Drone chose: {CHOICES[num_drone]}")

        game_rounds += 1

        if num_user == num_drone:
            print("Tie!")
            game_log.append("Tie")

        elif (num_user - num_drone) % 3 == 1:
            # User wins this round
            print("User wins!")
            num_lose += 1
            game_log.append("Drone L")

            for i in range(3):
                time.sleep(0.5)
                scf.cf.param.set_value('led.bitmask', 0x82)
                scf.cf.param.set_value('led.bitmask', 0x88)
                time.sleep(0.5)
                scf.cf.param.set_value('led.bitmask', 0x80)

            print("Drone moves down 0.1 meters.")
            mc.down(MOVE_DISTANCE)

        else:
            # Drone wins this round
            print("Drone wins!")
            num_win += 1
            game_log.append("Drone W")

            for i in range(3):
                time.sleep(0.5)
                scf.cf.param.set_value('led.bitmask', 0x84)
                scf.cf.param.set_value('led.bitmask', 0x90)
                time.sleep(0.5)
                scf.cf.param.set_value('led.bitmask', 0x80)

            print("Drone moves up 0.1 meters.")
            mc.up(MOVE_DISTANCE)

        print(f"Score - Drone: {num_win}, User: {num_lose}")

    print("\n--- Game Over ---")

    if num_win >= MAX_WINS:
        print("The drone wins the game!")
    else:
        print("Congratulations! You win the game!")

    print("\nGame Log:", game_log)
    print("Number of Rounds:", game_rounds)
"---------------------------------------------------------------------------"

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

        skygame(scf)