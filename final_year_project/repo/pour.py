from interbotix_xs_modules.arm import InterbotixManipulatorXS
from std_msgs.msg import Bool

import rospy

joint_state_pub = rospy.Publisher( 'load_topic', Bool, queue_size = 0 )

pose_complete = False

def set_wrist_pose():
	neutral_joint_position = [0, 0, 0.506, -0.531, 0]
	bot.arm.set_joint_positions( neutral_joint_position )
	rospy.sleep( 4 )
	load_sub()

def load_sub():
	rospy.Subscriber( "load_topic", Bool, start_pouring )

def start_pouring( pour ):
	pose_complete = True
	if pour:
		pouring_joint_positions = [0, 0, 0.506, -0.531, -1.57]
		bot.arm.set_joint_positions( pouring_joint_positions )	
		rospy.sleep( 5 )
		joint_state_pub.publish( pose_complete )
	else:
		bot.arm.go_to_sleep_pose()
		

if __name__=='__main__':
	bot = InterbotixManipulatorXS("rx150", "arm", "gripper")
	set_wrist_pose()
	rospy.spin()
