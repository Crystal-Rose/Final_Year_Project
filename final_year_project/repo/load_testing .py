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
	if not(State.load <= -115 and State.load >= -134.5):
		print( "Load not initialised correctly" )
		reset_load1 = [0, -1.57, 0, -0.531, 0]
		#reset_load2 = [0, 0, 1, -1.5, 0]
		bot.arm.set_joint_positions( reset_load1 )
		#bot.arm.set_joint_positions( reset_load2 )
		rospy.sleep( 1 )
		neutral_joint_position = [0, 0, 0.506, -0.531, 0]
		bot.arm.set_joint_positions( neutral_joint_position )
	else:
		print( "Ready to receive cup" )
	return State


if __name__=='__main__':
	bot = InterbotixManipulatorXS("rx150", "arm", "gripper")
	set_wrist_pose()
	listener()
	robot_arm = State_machine()
	while not rospy.is_shutdown():
		
		rospy.sleep( 0.05 )
		robot_arm.load = jointLoad
		robot_arm = process_state( robot_arm )



