import asyncio
import numpy as np
from utils import *

SPACE_MOUSE_SENSITIVITY = 0.8
DEADBAND = 0.02
UPDATE_INTERVAL = 0.01

robot_reader = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
robot_writer = rtde_control.RTDEControlInterface(ROBOT_IP)

initSpaceMouse()

mode = 0
last_button_state = [0, 0]  # Tracks previous button states

def apply_deadband(value, sensitivity):
    return value**3 * sensitivity if np.abs(value) > DEADBAND else 0

try:
    while True:
        state = pyspacemouse.read()

        if state:
            if mode == 0:
                # Standard TCP control
                actual_pose_tcp = robot_reader.getActualTCPPose()

                deltas = [
                    apply_deadband(state.x, SPACE_MOUSE_SENSITIVITY),
                    apply_deadband(state.y, SPACE_MOUSE_SENSITIVITY),
                    apply_deadband(state.z, SPACE_MOUSE_SENSITIVITY),
                    apply_deadband(state.roll, SPACE_MOUSE_SENSITIVITY),
                    apply_deadband(state.pitch, SPACE_MOUSE_SENSITIVITY),
                    apply_deadband(state.yaw, SPACE_MOUSE_SENSITIVITY),
                ]

                acc = max(0.5, np.linalg.norm(robot_reader.getActualTCPSpeed()) * 10)
                if any(np.abs(delta) > DEADBAND for delta in deltas):
                    robot_writer.speedL(deltas, acc, 0.001)
                else:
                    robot_writer.speedL([0, 0, 0, 0, 0, 0], acc+2.5, 0.001)  # Stop TCP movement

            elif 1 <= mode <= 6:
                # Control individual joint based on mode
                joint_index = mode - 1  # Map mode to joint index (0-5)
                joint_speeds = [0] * 6  # Initialize all joint speeds to zero

                # Apply rotation to the selected joint
                joint_speeds[joint_index] = apply_deadband(state.yaw, SPACE_MOUSE_SENSITIVITY)

                joint_position_degrees = np.degrees(robot_reader.getActualQ()[joint_index])
                print(f"Controlling joint {joint_index + 1}, current position: {joint_position_degrees:.2f} degrees")
                
                if np.abs(joint_speeds[joint_index]) > DEADBAND:
                    robot_writer.speedJ(joint_speeds, acceleration=0.5, time=0.1)
                else:
                    robot_writer.speedJ([0, 0, 0, 0, 0, 0], acceleration=5.0, time=0.01)  # Stop joint movement

            # Debounced mode change (button lifted)
            if state.buttons[0] == 0 and last_button_state[0] == 1:
                mode = (mode + 1) % 7  # Cycle through modes 0-6
                print(f"Switched to mode {mode}.")

            # Exit on button 1 release
            if state.buttons[1] == 0 and last_button_state[1] == 1:
                robot_writer.speedJ([0, 0, 0, 0, 0, 0], acceleration=25, time=0.1)  # Stop joint movement
                print("Button pressed. Exiting.")
                break

            # Update button states
            last_button_state = state.buttons[:]

except KeyboardInterrupt:
    print("\nControl interrupted by user.")

finally:
    pyspacemouse.close()
    print("SpaceMouse connection closed.")
