#!/usr/bin/env python

from interbotix_xs_modules.arm import InterbotixManipulatorXS
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool 
import rospy

load_value_pub = rospy.Publisher( 'load_topic', Bool, queue_size = 0 )
#joint_state_pub = rospy.Publisher( 'load_topic', Bool, queue_size = 0 )

global cup_has_water
global first_load_check
global empty_cup_load
global state
global moment

moment = 0
state = "Start"
empty_cup_load = 0
first_load_check = True
cup_has_water = False

def set_wrist_pose():
	print( "Moving arm into position..." )
	neutral_joint_position = [0, 0, 0.506, -0.531, 0]
	bot.arm.set_joint_positions( neutral_joint_position )
	rospy.sleep( 2 )
	

def listener():
	rospy.Subscriber( "/rx150/joint_states", JointState, check_load )

def check_load( joint_states ):
	global state
	global empty_cup_load
	global moment 

	load = joint_states.effort[3] 
	print("Load: " + str(load))
	print("State: " + state)

	if state == "Start":
		empty_cup_load = empty_cup( load, empty_cup_load )	
	elif state == "Waiting":
		moment = filling_cup( load, empty_cup_load )
	elif state == "Filling":
		full_cup( load, moment )
	elif state == "Ready":
		start_pouring()
	return

def stabilise_load_timer():
	

def empty_cup( load, empty_cup_load ):
	global state
	if empty_cup_load == 0:
		empty_cup_load = load
		return empty_cup_load
	else:
		empty_cup_load = load
		state = "Waiting"
		return empty_cup_load
	
def filling_cup( load, empty_cup_load ):
	global state
	global moment

	if load != empty_cup_load:
		moment = load
		state = "Filling"
		return moment

def full_cup( load, m ):
	global state
	if load != m:
		state = "Filling"
	else:
		state = "Ready"

def start_pouring():
	pouring_joint_positions = [0, 0, 0.506, -0.531, -1.57]
	bot.arm.set_joint_positions( pouring_joint_positions )	
	rospy.sleep( 5 )
	print( "Pouring completed" )
	bot.arm.go_to_sleep_pose()


if __name__=='__main__':
	bot = InterbotixManipulatorXS("rx150", "arm", "gripper")
	set_wrist_pose()
	listener()
	while True:
		
		rospy.spin()



