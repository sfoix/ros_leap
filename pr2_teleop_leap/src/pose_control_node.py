#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_teleop_leap')
import rospy
import simple_robot_control
import geometry_msgs

from visualization_msgs.msg import Marker

from geometry_msgs.msg import PoseStamped

pub_marker = rospy.Publisher('/leap/pr2hand/hand', Marker)
pub_marker2 = rospy.Publisher('/leap/pr2hand/workspace', Marker)

rospy.init_node('leap_pose_control')
armr = simple_robot_control.Arm('r')
multiplier = 1.6
offset = [0.553, 0.013, 1.0]
seed_angles = [-0.97840476860406289, -0.19456832314024575, -2.2066032133465989, -1.0864773957716338, 12.333808797076806, -0.47422239872702321, -16.799966201473772]
orientation = rospy.get_param("~orientation", False)

def callback(msg):
	#ospy.loginfo("I received a message! w00t")
	x = (msg.pose.position.x * multiplier) + 0.55
	y = msg.pose.position.y * multiplier - 0.2
	z = msg.pose.position.z * multiplier + 0.05
	#armr = simple_robot_control.Arm('r')
	if orientation:
		armr.goToPose([x,y,z],[msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w ], 'torso_lift_link', 0.5,False, seed_angles)
	else:
		armr.goToPose([x,y,z],[0.0,0.0,0.0,1.0 ], 'torso_lift_link', 0.5, False, seed_angles)
	marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.pose.position.x = offset[0] + msg.pose.position.x
        marker.pose.position.y = offset[1] + msg.pose.position.y
        marker.pose.position.z = offset[2] + msg.pose.position.z
        marker.pose.orientation.x = msg.pose.orientation.x
        marker.pose.orientation.y = msg.pose.orientation.y
        marker.pose.orientation.z = msg.pose.orientation.z
        marker.pose.orientation.w = msg.pose.orientation.w
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 1.0
        marker.color.r = 0.0
	marker.color.g = 1.0
        marker.color.b = 0.0
	pub_marker.publish( marker )

        marker2 = Marker()
        marker2.header.frame_id = "base_link"
        marker2.header.stamp = rospy.Time.now()
        marker2.ns = "my_namespace"
        marker2.id = 0
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.pose.position.x = offset[0]
        marker2.pose.position.y = offset[1]
        marker2.pose.position.z = offset[2]
        marker2.pose.orientation.x = 0
        marker2.pose.orientation.y = 0
        marker2.pose.orientation.z = 0
        marker2.pose.orientation.w = 1
        marker2.scale.x = 0.4
        marker2.scale.y = 0.4
        marker2.scale.z = 0.4
        marker2.color.a = 0.5
        marker2.color.r = 0.0
        marker2.color.g = 0.0
        marker2.color.b = 1.0
        pub_marker2.publish( marker2 )



def PalmPose():
   #rospy.init_node('leap_pose_control')
   rospy.Subscriber("/leap/palm/pose", PoseStamped, callback, queue_size = 1)
   rospy.spin()

if __name__ == '__main__':
   PalmPose()
