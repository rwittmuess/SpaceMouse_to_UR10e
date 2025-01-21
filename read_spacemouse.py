import pyspacemouse
import time

success = pyspacemouse.open()
if success:
    while 1:
        state = pyspacemouse.read()
        print(state.buttons)
        # print(state.x, state.y, state.z, state.roll, state.pitch, state.yaw)
        time.sleep(0.01)