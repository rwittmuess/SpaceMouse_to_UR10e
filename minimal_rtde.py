import rtde_receive
import rtde_control
 
# ROBOT_IP = '192.168.1.102'
ROBOT_IP = '192.168.10.0'
robot_read = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
robot = rtde_control.RTDEControlInterface(ROBOT_IP)

actual_pose_tcp = robot_read.getActualTCPPose()
actual_pose_j = robot_read.getActualQ()

actual_pose_tcp[2] += 0.1

robot.moveL(actual_pose_tcp,0.1,0.1)
