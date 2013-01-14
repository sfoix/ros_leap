#ifndef LEAP_GESTURES_GESTURE_INTERPRETER_HPP_INCLUDED
#define LEAP_GESTURES_GESTURE_INTERPRETER_HPP_INCLUDED

#include "ros/ros.h"
#include "leap_msgs/Leap.h"
#include "leap_msgs/ControlStatus.h"
#include "leap_msgs/GripperControl.h"
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
		initControl();

		double temp;

		ros::NodeHandle nhPriv("~");

		nhPriv.param<double>("wait_before_hand_active", temp, 1.0);
		_waitBeforeHandActive = ros::Duration(temp);
		ROS_INFO_STREAM("Gesture Interpreter config: wait_before_hand_active = " << temp);

		nhPriv.param<double>("workspace_radius", _workspaceHandRadius, 0.2);
		ROS_INFO_STREAM("Gesture Interpreter config: workspace_radius = " << _workspaceHandRadius);

		nhPriv.param<double>("workspace_inner_radius", _workspaceHandInnerRadius, 0.03);
		ROS_INFO_STREAM("Gesture Interpreter config: workspace_inner_radius = " << _workspaceHandInnerRadius);

		nhPriv.param<bool>("gripper_control_enabled", _gripperControlEnabled, true);
		ROS_INFO_STREAM("Gesture Interpreter config: gripper_control_enabled = " << _gripperControlEnabled);

		nhPriv.param<double>("gripper_control_threshold", _gripperControlThreshold, 0.06);
		ROS_INFO_STREAM("Gesture Interpreter config: gripper_control_threshold = " << _gripperControlThreshold);

		nhPriv.param<bool>("square_workspace", _squareWorkspace, false);
		ROS_INFO_STREAM("Gesture Interpreter config: square_workspace = " << _squareWorkspace);

		nhPriv.param<bool>("reset_to_zero", _resetToZero, true);
		ROS_INFO_STREAM("Gesture Interpreter config: reset_to_zero = " << _resetToZero);


		nhPriv.param<double>("workspace_origin/x", temp, 0);
		_workspaceHand.setX(temp);
		nhPriv.param<double>("workspace_origin/y", temp, 0);
		_workspaceHand.setY(temp);
		nhPriv.param<double>("workspace_origin/z", temp, 0.3);
		_workspaceHand.setZ(temp);
		ROS_INFO("Gesture Interpreter config: workspace_origin = (%f,%f,%f)",
				 _workspaceHand.x(), _workspaceHand.y(), _workspaceHand.z());

		_subLeap = _nh.subscribe("leap/data", 10, &GestureInterpreter::leapCB, this);
		_pubControlHand = _nh.advertise<geometry_msgs::PoseStamped>("leap/palm/pose", 1, true);
		_pubStatus = _nh.advertise<leap_msgs::ControlStatus>("leap/control_status", 10, true);
		_pubControlGripper = _nh.advertise<leap_msgs::GripperControl>("leap/pr2_hand/gripper", 10, true);

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
					ROS_INFO("Frame has hand and active");
					setActive(true);

					///////////////////////////////////
					// Extract Hand Position
					///////////////////////////////////
					tf::Point hand_point;
					tf::pointMsgToTF(hand.pose.position, hand_point);

					tf::Point relative_hand_point = hand_point - _workspaceHand;

					// the inner radius defines a  region where we want 0 movement
					if (relative_hand_point.length() < _workspaceHandInnerRadius)
					{
						relative_hand_point.setValue(0,0,0);
					}

					tf::pointTFToMsg(relative_hand_point, _controlHandPose.position);
					_controlHandPose.orientation = hand.pose.orientation;

					///////////////////////////////////
					// Check fingers for gripper control
					///////////////////////////////////

#ifdef NOT_DEFINED
					BOOST_FOREACH(const int16_t & finger_id, hand.finger_ids)
					{
						leap_msgs::Finger finger;
						BOOST_FOREACH(const leap_msgs::Finger &f, msg.fingers)
						{
							if(f.id == finger_id)
							{
								finger = f;
								break;
							}
						}

						tf::Pose poseFinger; // w_T_f
						tf::poseMsgToTF(finger.pose, poseFinger);
						tf::Pose poseHand; // w_T_h
						tf::poseMsgToTF(hand.pose, poseHand);

						//tf::Pose poseFingerInHandFrame = poseHand.inverse() * poseFinger;
						tf::Pose poseFingerInHandFrame = poseFinger; // FIXME

						ROS_INFO_STREAM(poseFingerInHandFrame.getOrigin().getX() << " " <<
										poseFingerInHandFrame.getOrigin().getY() << " " <<
										poseFingerInHandFrame.getOrigin().getZ() << ", " <<
										_gripperControlThreshold);

						ROS_INFO_STREAM("control value: " << fabs((poseFinger.getOrigin() - poseHand.getOrigin()).getY()));

						if(fabs(poseFingerInHandFrame.getOrigin().getY()) > _gripperControlThreshold)
						//if(fabs((poseFinger.getOrigin() - poseHand.getOrigin()).getY()) > _gripperControlThreshold)
						{
							ROS_INFO("Open");
							_gripperControl.open = true;
						}
						else
						{
							ROS_INFO("Close");
							_gripperControl.open = false;
						}
					}
#endif
					if(hand.finger_ids.size() >= 3)
					{
						ROS_INFO("Open");
						_gripperControl.open = true;
					}
					else
					{
						ROS_INFO("Close");
						_gripperControl.open = false;
					}

				}
				else
				{
					ROS_INFO("Frame has hand but not active");
					setActive(false);
					resetControl();
				}

				return;
			}
		}

		// no hand in workspace found
		frameHasNoHand();
	}

	void setActive(bool active)
	{
		_workspaceHandActive = active;
	}

	void initControl()
	{
		_controlHandPose = geometry_msgs::Pose();
		_controlHandPose.orientation.w = 1;
	}

	void resetControl()
	{
		if(_resetToZero)
		{
			_controlHandPose = geometry_msgs::Pose();
			_controlHandPose.orientation.w = 1;
		}
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
		resetControl();
		setActive(false);
	}

	bool handActive()
	{
		if (!_handEnteredWorkspaceTimestamp.isZero())
		{
			ros::Duration timeSinceHandInWorkspace =
					ros::Time::now() - _handEnteredWorkspaceTimestamp;
			if(timeSinceHandInWorkspace > _waitBeforeHandActive)
			{
				return true;
			}
		}
		return false;
	}

	bool inWorkspace(const leap_msgs::Hand &hand)
	{
		tf::Point hand_point;
		tf::pointMsgToTF(hand.pose.position, hand_point);
		tf::Point relative_hand_point = hand_point - _workspaceHand;
		ROS_DEBUG_STREAM(relative_hand_point.length());
		if(_squareWorkspace)
		{
			// square workspace
			return (fabs(relative_hand_point.x()) < _workspaceHandRadius) &&
					(fabs(relative_hand_point.y()) < _workspaceHandRadius) &&
					(fabs(relative_hand_point.z()) < _workspaceHandRadius);

		}
		else
		{
			// sperical workspace
			return relative_hand_point.length() < _workspaceHandRadius;
		}
	}

	void run()
	{
		ros::Rate r(100);
		while(ros::ok())
		{
			ros::spinOnce();

			{
				geometry_msgs::PoseStamped msg;
				msg.header.frame_id = "leap_control_link";
				msg.header.stamp = ros::Time::now();
				msg.pose = _controlHandPose;
				_pubControlHand.publish(msg);
			}

			{
				leap_msgs::ControlStatus msg;
				msg.control_active = _workspaceHandActive;
				_pubStatus.publish(msg);
			}

			if(_workspaceHandActive && _gripperControlEnabled)
			{
				_pubControlGripper.publish(_gripperControl);
			}

			r.sleep();
		}
	}

private:
	ros::NodeHandle _nh;

	ros::Subscriber _subLeap;
	ros::Publisher _pubControlHand;
	ros::Publisher _pubControlGripper;
	ros::Publisher _pubStatus;

	tf::Point _workspaceHand;
//	tf::Point _workspaceGripper;
	double _workspaceHandRadius;
	double _workspaceHandInnerRadius;
//	double _workspaceGripperRadius;
	bool _workspaceHandActive;
//	bool _workspaceGripperActive;

	geometry_msgs::Pose _controlHandPose; // pose sent to the controller. 0 if not active
	leap_msgs::GripperControl _gripperControl;

	double _gripperControlThreshold;
	bool _squareWorkspace;
	bool _resetToZero;
	bool _gripperControlEnabled;

	// zero if last frame had no hand in workspace
	ros::Time _handEnteredWorkspaceTimestamp;


	ros::Duration _waitBeforeHandActive;
};


}


#endif
