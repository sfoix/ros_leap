#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_teleop_leap')
import rospy
import simple_robot_control
import geometry_msgs

from geometry_msgs.msg import PoseStamped

rospy.init_node('leap_pose_control')
armr = simple_robot_control.Arm('r')
multiplier = 1.0

def callback(msg):
	rospy.loginfo("I received a message! w00t")
	x = (msg.pose.position.x * multiplier) + 0.5
	y = msg.pose.position.y * multiplier
	z = msg.pose.position.z * multiplier
	#armr = simple_robot_control.Arm('r')
	armr.goToPose([x,y,z],[msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w ], 'torso_lift_link', 3.0,False)

def PalmPose():
   #rospy.init_node('leap_pose_control')
   rospy.Subscriber("/leap/palm/pose", PoseStamped, callback)
   rospy.spin()

if __name__ == '__main__':
   PalmPose()
