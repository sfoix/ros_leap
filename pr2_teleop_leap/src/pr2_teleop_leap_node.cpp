#include <ros/ros.h>
#include "pr2_teleop_general/pr2_teleop_general_commander.h"
#include "geometry_msgs/Twist.h"
#include "leap_msgs/GripperControl.h"

GeneralCommander *gc;

void chatterCallback(const geometry_msgs::TwistConstPtr& msg)
{
  //ROS_INFO("linear x:%f y:%f z:%f wrist:%f ", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x);
  gc->sendArmVelCommands(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x,0.0,
			 0.0, 0.0, 0.0, 0.0, 0.0, 20);

}

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
  ros::Subscriber sub = n.subscribe("/leap/pr2hand/twist",10,chatterCallback);
  ros::Subscriber sub_gripper = n.subscribe("/leap/pr2_hand/gripper",10,gripperCallback);
  ros::spin();
  return 0;
}
