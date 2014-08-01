#!/usr/bin/env python

import sys
from math import cos, sin

import rospy
import leap_node_linux.Leap as Leap
from geometry_msgs.msg import Pose, Point
from leap_msgs.msg import Leap as Leap_msg
from leap_msgs.msg import Hand, Tool, Finger, Gesture


def leap2pose(position, direction, palm_normal=None, rpy_scaling=[1, 1, 1]):
    """
    Converts LEAP position in mm and direction into ROS poses in meters.

    Args: 
        position:       Leap.Vector of leap position
        direction:      Leap.Vector of leap direction
        palm_normal:    Leap.Vector of leap palm_normal (defaults: None)
        rpy_scaling:  Vector of 3 floats defining a scaling of the
                        roll pitch and yaw anlge used for PR2 control
                        defaults to [1, 1, 1]

    Returns: 
        pose_msg: geometry_msgs/Pose

    """
    if palm_normal is not None:
        roll  = rpy_scaling[0] * direction.yaw
        pitch = - rpy_scaling[1] * palm_normal.pitch
        yaw   = - rpy_scaling[2] * direction.yaw
    else:
        roll = - direction.yaw
        pitch = - direction.roll
        yaw = - direction.pitch
    pose_msg = Pose()
    pose_msg.position.x = -position.z / 1000
    pose_msg.position.y = -position.x / 1000
    pose_msg.position.z = position.y / 1000
    pose_msg.orientation.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    pose_msg.orientation.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    pose_msg.orientation.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    pose_msg.orientation.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    return pose_msg


def leapvec2point(vector):
    """
    Converts Leap vectors in mm into ros points in m.

    Args: 
        vector: Leap.Vector of e.g. a velocity in [mm/s]

    Returns: 
        point_msg: geometry_msgs/Pos

    """
    point_msg = Point()
    point_msg.x = -vector.z / 1000
    point_msg.y = -vector.x / 1000
    point_msg.z = vector.y / 1000

    return point_msg


def leap_node():
    """
    Publishes LEAP data to ROS
    """
    rospy.init_node('leap')
    roll_scale = rospy.get_param('~roll_scale', 1)
    pitch_scale = rospy.get_param('~pitch_scale', 1)
    yaw_scale = rospy.get_param('~yaw_scale', 1)
    pub = rospy.Publisher('leap/data', Leap_msg)
    controller = Leap.Controller()
    controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
    if(controller.config.set("Gesture.KeyTap.MinDownVelocity", 0.4)
       and controller.config.set("Gesture.KeyTap.HistorySeconds", 0.5)
       and controller.config.set("Gesture.KeyTap.MinDistance", 8)):
        controller.config.save()
    # controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE)
    # if(controller.config.set("Gesture.Circle.MinRadius", 5.0)
    #        and controller.config.set("Gesture.Circle.MinArc", 6)):
    #     controller.config.save()
    # controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP)
    # if(controller.config.set("Gesture.ScreenTap.MinForwardVelocity", 30.0)
    #         and controller.config.set("Gesture.ScreenTap.HistorySeconds", .5)
    #         and controller.config.set("Gesture.ScreenTap.MinDistance", 1.0)):
    #     controller.config.save()

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
        msg.gestures = []
        for hand_iter, hand in enumerate(frame.hands):
            hand_msg = Hand()
            hand_msg.id = hand.id
            hand_msg.pose = leap2pose(
                position=hand.stabilized_palm_position,
                direction=hand.direction,
                palm_normal=hand.palm_normal,
                rpy_scaling=[roll_scale, pitch_scale, yaw_scale]
            )
            hand_msg.finger_ids = [finger.id for finger in hand.fingers]
            hand_msg.tool_ids = [tool.id for tool in hand.tools]
            hand_msg.sphere_radius = hand.sphere_radius
            hand_msg.direction_yaw = -hand.direction.yaw
            msg.hands.append(hand_msg)
        for finger_iter, finger in enumerate(frame.fingers):
            finger_msg = Finger()
            finger_msg.pose = leap2pose(finger.tip_position, finger.direction)
            finger_msg.velocity = leapvec2point(finger.tip_velocity)
            finger_msg.length = finger.length
            finger_msg.width = finger.width
            msg.fingers.append(finger_msg)
        for tool_iter, tool in enumerate(frame.tools):
            tools_msg = Tool()
            tools_msg.pose = leap2pose(tool.tip_position, tool.direction)
            tools_msg.velocity = leapvec2point(tool.tip_velocity)
            tools_msg.length = tool.length
            tools_msg.width = tool.width
            msg.tools.append(tools_msg)
        for gesture_iter, gesture in enumerate(frame.gestures()):
            gesture_msg = Gesture()
            gesture_msg.id = gesture.id
            gesture_msg.type = gesture.type
            gesture_msg.pointables = [pointable.id for pointable in gesture.pointables]
            msg.gestures.append(gesture_msg)
        pub.publish(msg)
        rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        leap_node()
    except rospy.ROSInterruptException:
        pass

