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
		start_pouring( State )
		State.state = "Finished"
		bot.arm.go_to_sleep_pose()
	return State

def check_initial_load( robot_arm ):
	reset_load = [0, -1.57, 0.506, -0.531, 0]
	neutral_joint_position = [0, 0, 0.506, -0.531, 0]
	if not(robot_arm.load <= -121 and robot_arm.load >= -226):
		print( "Load not initialised correctly" )
		bot.arm.set_joint_positions( reset_load )
		rospy.sleep( 1 )
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

def start_pouring( robot_arm ):
	volume = robot_arm.amount
	if volume == 50 or volume == 100 or volume == 150:
		standard_pour()
	elif volume == 200 or volume == 250:
		careful_pour()
	else:
		print( "Unable to identify volume, careful pour will occur." )
		careful_pour()

	print( "Pouring completed" )
	rospy.sleep( 4 )
	neutral_joint_position = [0, 0, 0.506, -0.531, 0]
	bot.arm.set_joint_positions( neutral_joint_position )
	rospy.sleep( 1 )

def determine_liquid_amount( robot_arm ):
	end_load = robot_arm.load
	change = robot_arm.load - robot_arm.empty_cup_load
	print( "Empty cup load: " + str( robot_arm.empty_cup_load ) )
	print( "End load: " + str( end_load ) )
	print( "Change: " + str( change ) )	

	if ( end_load <= -134.5 and end_load >= -263.62 ) and ( change <= -13.45 and change >= -37.66 ):
		robot_arm.amount = 50
	elif ( end_load <= -153.33 and end_load >= -317.42 ) and ( change <= -32.28 and change >= -91.46 ):
		robot_arm.amount = 100
	elif ( end_load <= -182.92 and end_load >= -347.01 ) and ( change <=-61.87 and change >= -121.05 ):
		robot_arm.amount = 150
	elif ( end_load <= -196.38 and end_load >= -392.74 ) and ( change <= -76 and change >= -166.78 ):
		robot_arm.amount = 200
	elif (  end_load <= -228.65 and end_load >= -438.47 ) and ( change <= -107.6 and change >= -212.51 ):
		robot_arm.amount = 250
	else:
		robot_arm.amount = -1
		
	print(robot_arm.amount)

def standard_pour():
	standard_pour_joint_positions1 = [0, 0, 0.506, -0.531, -1.67]
	bot.arm.set_joint_positions( standard_pour_joint_positions1 )
	rospy.sleep( 1 )
	empty_the_cup_joint_positions = [0.13, -0.08, 0.45, -0.78, -2.2]
	bot.arm.set_joint_positions( empty_the_cup_joint_positions )

def careful_pour():
	pouring_joint_positions1 = [0, 0, 0.523, -0.61, -0.69]#40
	bot.arm.set_joint_positions( pouring_joint_positions1 )
	rospy.sleep( 0.5 )
	pouring_joint_positions2 = [0, 0, 0.523, -0.61, -1.047]#60
	bot.arm.set_joint_positions( pouring_joint_positions2 )
	rospy.sleep( 2 )
	pouring_joint_positions3 = [0, 0, 0.523, -0.61, -1.67]#95
	bot.arm.set_joint_positions( pouring_joint_positions3 )
	rospy.sleep( 1.5 )
	empty_the_cup_joint_positions = [0.15, -0.08, 0.45, -0.78, -2.35]
	bot.arm.set_joint_positions( empty_the_cup_joint_positions )

if __name__=='__main__':
	bot = InterbotixManipulatorXS( "rx150", "arm", "gripper" )
	set_wrist_pose()
	listener()
	robot_arm = State_machine()
	while not rospy.is_shutdown():
		
		rospy.sleep( 0.05 )
		robot_arm.load = jointLoad
		robot_arm = process_state( robot_arm )
