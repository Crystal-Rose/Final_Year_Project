#!/usr/bin/env python

from interbotix_xs_modules.arm import InterbotixManipulatorXS
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool 
import rospy
import time 

class State_machine:

	def __init__(self):
		self.state = "Start"
		self.load = 0
		self.empty_cup_load = 0
		self.moment = 0
		self.start_time = 0
		self.timer_running = False
		self.amount = 0
	
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

def process_state( State ):
	print( "Load: " + str(State.load) + "	State: " + State.state)

	if State.state == "Start":
		State = check_initial_load( State )	
	elif State.state == "Waiting":
		State = is_filling_cup( State )
	elif State.state == "Filling":
		State = cup_being_filled( State )
	elif State.state == "Ready":
		start_pouring()
		State.state = "Finished"
		bot.arm.go_to_sleep_pose()
	return State

def check_initial_load( robot_arm ):
	if not(robot_arm.load <= -115 and robot_arm.load >= -143):
		print( "Load not initialised correctly" )
		reset_load = [0, -1.57, 0, -0.531, 0]
		bot.arm.set_joint_positions( reset_load )
		rospy.sleep( 1 )
		neutral_joint_position = [0, 0, 0.506, -0.531, 0]
		bot.arm.set_joint_positions( neutral_joint_position )
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
				determine_liquid_amount( robot_arm )
				robot_arm.state = "Ready"
		else:
			robot_arm.start_timer()	
		
	return robot_arm

def start_pouring():
	pouring_joint_positions1 = [0, 0, 0.506, -0.531, -1.67]
	bot.arm.set_joint_positions( pouring_joint_positions1 )
	rospy.sleep( 2 )
	pouring_joint_positions2 = [0.13, -0.08, 0.45, -0.78, -2.1]
	bot.arm.set_joint_positions( pouring_joint_positions2 )
	print( "Pouring completed" )
	rospy.sleep( 4 )
	neutral_joint_position = [0, 0, 0.506, -0.531, 0]
	bot.arm.set_joint_positions( neutral_joint_position )
	rospy.sleep( 1 )

def determine_liquid_amount( robot_arm ):
	end_load = robot_arm.empty_cup_load
	change = robot_arm.load - end_load
	
	if ( end_load <= -161.4 and end_load >= -215.2 ) and ( change <= -13.45 and change >= -40.35 ):
		robot_arm.amount = 50
	if ( end_load <= -234.03 and end_load >= -250.17 ) and ( change <= -43.04 and change >= -94.15 ):
		robot_arm.amount = 100
	if ( end_load <= -263.62 and end_load >= -277.07 ) and ( change <=-61.87 and change >= -123.74 ):
		robot_arm.amount = 150
	if ( end_load <= -293.21 and end_load >= -309.35 ) and ( change <= -72.63 and change >= -169.47 ):
		robot_arm.amount = 200
	if (  end_load <= -328.18 and end_load >= -357.77 ) and ( change <= -104.91 and change >= -215.2 ):
		robot_arm.amount = 250
	else:
		robot_arm.amount = -1
		
	print(robot_arm.amount)

if __name__=='__main__':
	bot = InterbotixManipulatorXS( "rx150", "arm", "gripper" )
	set_wrist_pose()
	listener()
	robot_arm = State_machine()
	while not rospy.is_shutdown():
		
		rospy.sleep( 0.05 )
		robot_arm.load = jointLoad
		robot_arm = process_state( robot_arm )



