#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_teleop_leap')
import rospy
import geometry_msgs
import tf

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/leap/pr2hand/twist', Twist)
multiplier = 0.5;

def callback(msg): 
	tw = Twist()
	tw.linear.x =  msg.pose.position.x * multiplier
	tw.linear.y =  msg.pose.position.y * multiplier
	tw.linear.z =  msg.pose.position.z * multiplier
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        tw.angular.x = r
	pub.publish(tw)
	print(tw)
   

if __name__ == '__main__':
	rospy.init_node('leap_velocity_control')
	rospy.Subscriber("/leap/palm/pose", PoseStamped, callback)  
	rospy.spin()
