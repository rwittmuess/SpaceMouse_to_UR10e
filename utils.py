# Imports
import rtde_receive
import rtde_control
import pyspacemouse
import time
import numpy as np

# Constants
ROBOT_IP = '192.168.10.0'


def initSpaceMouse():
    if not pyspacemouse.open():
        print("Failed to initialize SpaceMouse.")
        exit(1)
    print("SpaceMouse initialized. Use the x-axis to control the robot.")