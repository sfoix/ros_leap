#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_teleop_leap')
import rospy
import geometry_msgs
import tf

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

pub = rospy.Publisher('/leap/pr2hand/twist', Twist)
pub_marker = rospy.Publisher('/leap/pr2hand/marker', Marker)
multiplier = 0.5
offset = [0.553, 0.013, 0.702]

def callback(msg): 
	tw = Twist()
	tw.linear.x =  msg.pose.position.x * multiplier
	tw.linear.y =  msg.pose.position.y * multiplier
	tw.linear.z =  msg.pose.position.z * multiplier
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        tw.angular.x = r
	pub.publish(tw)
	print(tw)
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
   

if __name__ == '__main__':
	rospy.init_node('leap_velocity_control')
	rospy.Subscriber("/leap/palm/pose", PoseStamped, callback)  
	rospy.spin()
