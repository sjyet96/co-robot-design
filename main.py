#!/usr/bin/env python
#import rospy
from my_functions import UR_Ctrl
#urx_control.chdir("~/catkin_ws/src/gripper/scripts")
#from talker import Gripper
#talker.chdir("~/catkin_ws/src/gripper/scripts")

def main():
	rospy.init_node('robot_main_node', anonymous=True)
	gripper = Gripper()
	ur_robot = UR_Ctrl(gripper)
	i=0
	print("start")
	ur_robot.start_ctrl()
	while i<1 :
		ur_robot.start()
		ur_robot.movetobottle()
		ur_robot.rotatewirst(90)
		ur_robot.pickupbottle()
		ur_robot.movetobox()
		ur_robot.findBoxPosition()
		ur_robot.putdownbottle()
		i=i+1
	print('end')
	ur_robot.end_ctrl()
if __name__ == '__main__':
	main()