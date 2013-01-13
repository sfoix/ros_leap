#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_teleop_leap')
import rospy
import geometry_msgs
import tf

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

pub = rospy.Publisher('/leap/pr2hand/twist', Twist)
pub_marker = rospy.Publisher('/leap/pr2hand/hand', Marker)
pub_marker2 = rospy.Publisher('/leap/pr2hand/workspace', Marker)
multiplier = 0.2
offset = [0.553, 0.013, 1.0]

def callback(msg): 
	tw = Twist()
	tw.linear.x =  msg.pose.position.x * multiplier
	tw.linear.y =  msg.pose.position.y * multiplier
	tw.linear.z =  msg.pose.position.z * multiplier
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        tw.angular.x = r * 10
	pub.publish(tw)
	print("Twist", tw)
	#print("Hand Pose", msg)
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
        marker2.type = marker.SPHERE
        marker2.action = marker.ADD
        marker2.pose.position.x = offset[0] 
        marker2.pose.position.y = offset[1] 
        marker2.pose.position.z = offset[2] 
        marker2.pose.orientation.x = 0  
        marker2.pose.orientation.y = 0
        marker2.pose.orientation.z = 0
        marker2.pose.orientation.w = 1
        marker2.scale.x = 0.3
        marker2.scale.y = 0.3
        marker2.scale.z = 0.3
        marker2.color.a = 0.5
        marker2.color.r = 0.0
        marker2.color.g = 0.0
        marker2.color.b = 1.0
        pub_marker2.publish( marker2 )


if __name__ == '__main__':
	rospy.init_node('leap_velocity_control')
	rospy.Subscriber("/leap/palm/pose", PoseStamped, callback)  
	rospy.spin()
