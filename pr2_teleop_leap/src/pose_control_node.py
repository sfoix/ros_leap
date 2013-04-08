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
multiplier = 1.8
offset = [0.553, 0.013, 1.0]

def callback(msg):
	x = (msg.pose.position.x * multiplier) + 0.55
	y = msg.pose.position.y * multiplier - 0.2
	z = msg.pose.position.z * multiplier + 0.05

	dx = x - x_old
	dy = y - y_old
	dz = z - z_old
	distance = sqrt(dx^2 + dy^2 + dz^2)
	trajectory_duration = distance / 0.2 # make it move with a constant velocity dt=ds/v
	trajectory_duration = trajectory_duration + 0.1 # make sure duration is not too small to compute

	armr.goToPose([x,y,z],[msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w ], 'torso_lift_link', trajectory_duration, False)
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

	x_old = x
	y_old = y
	z_old = z

def PalmPose():
   #rospy.init_node('leap_pose_control')
   rospy.Subscriber("/leap/palm/pose", PoseStamped, callback, queue_size = 1)
   rospy.spin()

if __name__ == '__main__':
   PalmPose()
