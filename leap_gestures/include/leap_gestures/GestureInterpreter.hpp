#ifndef LEAP_GESTURES_GESTURE_INTERPRETER_HPP_INCLUDED
#define LEAP_GESTURES_GESTURE_INTERPRETER_HPP_INCLUDED

#include "ros/ros.h"
#include "leap_msgs/Leap.h"
#include "boost/foreach.hpp"
#include "tf/transform_datatypes.h"

namespace leap
{


class GestureInterpreter
{
public:
	GestureInterpreter(ros::NodeHandle nh):
		_nh(nh),
		_workspaceOrigin(0,0,0.3)
	{
		_handEnteredWorkspaceTimestamp = ros::Time(0);
		_subLeap = _nh.subscribe("leap", 10, &GestureInterpreter::leap_cb, this);
	}

	void leap_cb(const leap_msgs::Leap &msg)
	{
		BOOST_FOREACH(const leap_msgs::Hand &hand, msg.hands)
		{
			if(in_workspace(hand))
			{
				// this is the controlling hand

				if(!_handInLastFrame)
				{
					_handInLastFrame = true;

				}

				if()
				{
					// start control with hand
				}
			}
		}

		// no hand found
	}

	bool frameHasHand()
	{

	}

	bool handActive()
	{
		return (!_handEnteredWorkspaceTimestamp.isZero()) &&
				(ros::Time::now() - _handEnteredWorkspaceTimestamp) > ros::Duration(2.0);
	}

	bool in_workspace(const leap_msgs::Hand &hand)
	{
		tf::Point hand_point;
		tf::pointMsgToTF(hand.pose.position, hand_point);
		return (hand_point - _workspaceOrigin).length() < 0.1;
	}

	void run()
	{
		ros::spin();
	}

private:
	ros::NodeHandle _nh;

	ros::Subscriber _subLeap;

	tf::Point _workspaceOrigin;

	// zero if last frame had no hand in workspace
	ros::Time _handEnteredWorkspaceTimestamp;
};


}


#endif
