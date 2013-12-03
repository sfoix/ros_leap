#! /usr/bin/env python

import rospy
import simple_robot_control

from leap_msgs.msg import GripperControl

class GripperController():
    def __init__(self):
        self.sub = rospy.Subscriber("/leap/pr2_hand/gripper", GripperControl, self.callback, queue_size = 1)
        self.gripper = simple_robot_control.Gripper('r')

    def callback(self, msg):
        if msg.open:
            self.gripper.openGripper()
        else:
            self.gripper.closeGripper()


if __name__ == "__main__":
    rospy.init_node('leap_gripper_control')
    my_gripper = GripperController()
    rospy.spin()
