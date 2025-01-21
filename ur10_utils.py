import rtde_control
import rtde_receive
import numpy as np
import sys


def connect_robot(robot_ip, real_robot = True):
    """Drive the robot in a grid pattern starting at the starting position

    Args:
        robot_ip (str): ipv4 address of the UR robot
        real_robot (bool, optional): specifies if a real or mock UR10 should be used (default True).
    """

    if real_robot:             #should be implemented into a function
        print("connecting to robot")
        try:  
            UR10Controller = rtde_control.RTDEControlInterface(robot_ip)
            UR10Receiver = rtde_receive.RTDEReceiveInterface(robot_ip)
        except Exception:
            print("ERROR: could not connect to the robot, script is terminated")
            print("Robot on, connected and in remote control mode?")
            print("Did you disable all VPNs?")
            sys.exit()
    else:
        print("using mock robot")
        UR10Controller = rtde_control.RTDEControlInterface
        UR10Receiver = rtde_control.RTDEControlInterface

    return UR10Controller, UR10Receiver

def drive_grid(controller: rtde_control.RTDEControlInterface,
               receiver: rtde_receive.RTDEReceiveInterface,
               start_pose_cartesian,
               home_joint_angles = None,
               rows=5,
               cols=5,
               spacing = 0.02,
               speed=0.2,
               real_robot=True,
               pose_reached_event: callable=None):
    """Drive the robot in a grid pattern starting at the starting position

    Args:
        controller (rtde_control.RTDEControlInterface): Connection to control the robot
        receiver (rtde_receive.RTDEReceiveInterface): Connection to receive data from the robot
        start_pos (np_array): Starting pose of the robot in joint angles
        rows (int, optional): Count of rows. Defaults to 5.
        cols (int, optional): Count of columns. Defaults to 5.
        spacing (float, optional): spacing between grid points in meters. Defaults to 0.02 m
        speed (float, optional): Robot speed in m/s and accelleration in m/s^2. Defaults to 0.2.
        real_robot (bool, optional): specifies if the real or a mock robot should be used
        pose_reached_event (callable, optional): Is called when a pose is reached
    """
    ROW_VECTOR = np.array([spacing, 0, 0, 0, 0, 0])    #values in meters
    COL_VECTOR = np.array([0, -spacing, 0, 0, 0, 0])    #values in meters
    #COL_VECTOR = np.array([0, -spacing, spacing, 0, 0.1, 0])    #experimental

    if real_robot:
        controller.moveL(start_pose_cartesian, 0.15, 0.15)
        start_pos = start_pose_cartesian   #receiver.getActualTCPPose()
        #print(start_pos)
    else:
        start_pos = [0,0,0,0,0,0]

    for row_number in range(rows):
        for col_number in range(cols):
            if row_number % 2 == 1:
                # Reverse direction of each 2nd row
                col_number = cols - col_number - 1

            pos = start_pos + row_number * ROW_VECTOR + col_number * COL_VECTOR

            print(f"Go to position {str(row_number)} {str(col_number)}", end='\r')


            if real_robot:
                controller.moveL(pos, speed, speed)
            if pose_reached_event:
                pose_reached_event(pos)
    print("")       #needed for nice output formatting

    if home_joint_angles is not None and home_joint_angles != []:
        if real_robot:
            controller.moveJ(home_joint_angles, 0.15, 0.15)
    

def get_current_joint_angles(receiver: rtde_receive.RTDEReceiveInterface):
    current_joint_angles = receiver.getActualQ()
    print(current_joint_angles)
    return current_joint_angles