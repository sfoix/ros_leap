#include <ros/ros.h>
#include "pr2_teleop_general/pr2_teleop_general_commander.h"
#include "leap_msgs/GripperControl.h"

GeneralCommander *gc;

void gripperCallback(const leap_msgs::GripperControlConstPtr& msg)
{
  gc->sendGripperCommand(gc->ARMS_RIGHT,!msg->open);
  ROS_INFO("Gripper %d", msg->open);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "leap_velocity_driver");
  ros::NodeHandle n;
  gc = new GeneralCommander(false, false, true, false, false);
  ros::Subscriber sub_gripper = n.subscribe("/leap/pr2_hand/gripper",10,gripperCallback);
  ros::spin();
  return 0;
}
