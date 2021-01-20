from interbotix_xs_modules.arm import InterbotixManipulatorXS
import rospy

# This script commands some arbitrary positions to the arm joints
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s'
# Then change to this directory and type 'python joint_position_control.py'

def main():
    joint_positions1 = [0, 0, 0.506, -0.531, 0]
    joint_positions2 = [0, 0, 0.506, -0.531, -1.57]
    bot = InterbotixManipulatorXS("rx150", "arm", "gripper")
    bot.arm.go_to_home_pose()
    bot.arm.set_joint_positions(joint_positions1)
    rospy.sleep(5)
    bot.arm.set_joint_positions(joint_positions2)
    rospy.sleep(8)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
