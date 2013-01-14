#!/usr/bin/env python
import roslib; roslib.load_manifest('leap_node')
import rospy
from leap_msgs.msg import Leap as Leap_msg
from leap_msgs.msg import Hand, Tool, Finger
from geometry_msgs.msg import Pose, Point
import Leap, sys
from math import cos, sin

def leap2pose(position, direction, palm_normal=None):
    """
    Args: 
        position:  Leap.Vector of leap position
        direction: Leap.Vector of leap direction
        palm_normal: Leap.Vector of leap palm_normal (defaults: None)

    Returns: 
        pose_msg: geometry_msgs/Pose

    """
    if palm_normal is not None:
        roll = - palm_normal.roll
        pitch = - direction.pitch
        yaw = - direction.yaw
    else:
        roll = - direction.roll
        pitch = - direction.pitch
        yaw = - direction.yaw
    pose_msg = Pose()
    pose_msg.position.x = -position.z / 1000
    pose_msg.position.y = -position.x / 1000
    pose_msg.position.z = position.y / 1000
    pose_msg.orientation.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    pose_msg.orientation.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    pose_msg.orientation.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    pose_msg.orientation.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    return pose_msg

def leap2vel(velocity):
    """
    Args: 
        velocity:  Leap.Vector of leap velocity

    Returns: 
        pos_msg: geometry_msgs/Pos

    """
    pos_msg = Point()
    pos_msg.x = -velocity.z / 1000
    pos_msg.y = -velocity.x / 1000
    pos_msg.z = velocity.y / 1000

    return pos_msg

def leap_node():
    """
    Publishes LEAP data to ROS
    """
    rospy.init_node('leap')
    pub = rospy.Publisher('leap/data', Leap_msg)
    controller = Leap.Controller()
    while not rospy.is_shutdown():
        frame = controller.frame()
        msg = Leap_msg()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "leap_base_frame"
        msg.leap_frame_id = frame.id
        msg.leap_time_stamp = frame.timestamp
        msg.hands_count = len(frame.hands)
        msg.fingers_count = len(frame.fingers)
        msg.tools_count = len(frame.tools)
        msg.hands = []
        msg.fingers = []
        msg.tools = []
        for hand_iter, hand in enumerate(frame.hands):
            hand_msg = Hand()
            hand_msg.id = hand.id
            hand_msg.pose = leap2pose(hand.palm_position, hand.direction, hand.palm_normal)
            hand_msg.finger_ids = [finger.id for finger in hand.fingers]
            hand_msg.tool_ids = [tool.id for tool in hand.tools]
            msg.hands.append(hand_msg)
        for finger_iter, finger in enumerate(frame.fingers):
            finger_msg = Finger()
            finger_msg.pose = leap2pose(finger.tip_position, finger.direction)
            finger_msg.velocity = leap2vel(finger.tip_velocity)
            finger_msg.length = finger.length
            finger_msg.width = finger.width
            msg.fingers.append(finger_msg)
        for tool_iter, tool in enumerate(frame.tools):
            tools_msg = Tool()
            tools_msg.pose = leap2pose(tool.tip_position, tool.direction)
            tools_msg.velocity = leap2vel(tool.tip_velocity)
            tools_msg.length = tool.length
            tools_msg.width = tool.width
            msg.tools.append(tools_msg)
        pub.publish(msg)
        rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        leap_node()
    except rospy.ROSInterruptException:
        pass

