#!/usr/bin/env python

from interbotix_xs_modules.arm import InterbotixManipulatorXS
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool 
import rospy
import time 

load_value_pub = rospy.Publisher( 'load_topic', Bool, queue_size = 0 )

global cup_has_water
global first_load_check
global empty_cup_load
global state
global moment
global jointLoad

jointLoad = 0
moment = 0
state = "Start"
empty_cup_load = 0
first_load_check = True
cup_has_water = False

class State_machine:

	def __init__(self):
		self.state = "Start"
		self.load = 0
		self.empty_cup_load = 0
		self.moment = 0
		self.start_time = 0
		self.timer_running = False
	
	def start_timer(self):
		self.start_time = time.time()
		self.timer_running = True
	
	def stop_timer(self):
		self.timer_running = False
	
	def reset_timer(self):
		self.start_time = time.time()
		self.timer_running = True
		
	def time_elapsed(self):
		elapsed = time.time() - self.start_time
		print( "Time elapsed: " + str(elapsed) )
		return elapsed
		

def set_wrist_pose():
	print( "Moving arm into position..." )
	neutral_joint_position = [0, 0, 0.506, -0.531, 0]
	bot.arm.set_joint_positions( neutral_joint_position )
	rospy.sleep( 1 )
	

def listener():
	rospy.Subscriber( "/rx150/joint_states", JointState, check_load )

def check_load( joint_states ):
	global jointLoad
	
	jointLoad = joint_states.effort[3] 
	#print("Load: " + str(jointLoad))

def process_state( State ):
	print( "Process state load: " + str(State.load) )
	print( "State: " + State.state )

	if State.state == "Start":
		State = empty_cup( State )	
	elif State.state == "Waiting":
		State = is_filling_cup( State )
	elif State.state == "Filling":
		State = cup_being_filled( State )
	elif State.state == "Ready":
		start_pouring()
		State.state = "Finished"
	return State

def empty_cup( robot_arm ):
	if robot_arm.empty_cup_load == 0:
		robot_arm.empty_cup_load = robot_arm.load
	else:
		robot_arm.empty_cup_load = robot_arm.load
		robot_arm.state = "Waiting"
	return robot_arm
	
def is_filling_cup( robot_arm ):
	if robot_arm.load != robot_arm.empty_cup_load:
		robot_arm.moment = robot_arm.load
		robot_arm.state = "Filling"
		
	return robot_arm

def cup_being_filled( robot_arm ):
	if robot_arm.load != robot_arm.moment:
		robot_arm.moment = robot_arm.load
		if robot_arm.timer_running:
			robot_arm.stop_timer()
	else:
		if robot_arm.timer_running:
			if robot_arm.time_elapsed() > 5:
				robot_arm.stop_timer()
				robot_arm.state = "Ready"
		else:
			robot_arm.start_timer()	
		
	return robot_arm

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
	robot_arm = State_machine()
	while not rospy.is_shutdown():
		
		rospy.sleep( 0.05 )
		robot_arm.load = jointLoad
		robot_arm = process_state( robot_arm )



