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
		_nh(nh)
	{
		_handEnteredWorkspaceTimestamp = ros::Time(0);

		double temp;

		ros::NodeHandle nhPriv("~");

		nhPriv.param<double>("wait_before_hand_active", temp, 2.0);
		_waitBeforeHandActive = ros::Duration(temp);

		nhPriv.param<double>("workspace_origin/x", temp, 0);
		_workspaceOrigin.setX(temp);
		nhPriv.param<double>("workspace_origin/y", temp, 0);
		_workspaceOrigin.setX(temp);
		nhPriv.param<double>("workspace_origin/z", temp, 0.3);
		_workspaceOrigin.setX(temp);

		_subLeap = _nh.subscribe("leap", 10, &GestureInterpreter::leapCB, this);
		_pubControl = _nh.advertise<geometry_msgs::PoseStamped>("leap/palm/pose", 10, true);
	}

	void leapCB(const leap_msgs::Leap &msg)
	{
		// use first hand in workspace

		BOOST_FOREACH(const leap_msgs::Hand &hand, msg.hands)
		{
			if(inWorkspace(hand))
			{
				// this is the controlling hand

				frameHasHand();

				if(handActive())
				{
					// do control with hand

					tf::Point hand_point;
					tf::pointMsgToTF(hand.pose.position, hand_point);

					tf::Point relative_hand_point = hand_point - _workspaceOrigin;

					tf::pointTFToMsg(relative_hand_point, _controlPose.position);
					_controlPose.orientation = hand.pose.orientation;
				}
				else
				{
					resetControlPose();
				}
			}
		}

		// no hand found
		frameHasNoHand();
	}

	void resetControlPose()
	{
		_controlPose = geometry_msgs::Pose();
	}

	void frameHasHand()
	{
		if(_handEnteredWorkspaceTimestamp.isZero())
		{
			_handEnteredWorkspaceTimestamp = ros::Time::now();
		}
	}

	void frameHasNoHand()
	{
		_handEnteredWorkspaceTimestamp = ros::Time(0);
		resetControlPose();
	}

	bool handActive()
	{
		return (!_handEnteredWorkspaceTimestamp.isZero()) &&
				(ros::Time::now() - _handEnteredWorkspaceTimestamp) > _waitBeforeHandActive;
	}

	bool inWorkspace(const leap_msgs::Hand &hand)
	{
		tf::Point hand_point;
		tf::pointMsgToTF(hand.pose.position, hand_point);
		return (hand_point - _workspaceOrigin).length() < 0.1;
	}

	void run()
	{
		ros::Rate r(ros::Duration(0.01));
		while(ros::ok())
		{
			ros::spinOnce();

			geometry_msgs::PoseStamped msg;
			msg.header.frame_id = "leap_control_link";
			msg.header.stamp = ros::Time::now();
			msg.pose = _controlPose;
			_pubControl.publish(msg);

			r.sleep();
		}
	}

private:
	ros::NodeHandle _nh;

	ros::Subscriber _subLeap;
	ros::Publisher _pubControl;

	tf::Point _workspaceOrigin;

	// zero if last frame had no hand in workspace
	ros::Time _handEnteredWorkspaceTimestamp;

	geometry_msgs::Pose _controlPose;

	ros::Duration _waitBeforeHandActive;
};


}


#endif
