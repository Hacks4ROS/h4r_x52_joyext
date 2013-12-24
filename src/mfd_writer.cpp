/*
 * value2buttonColor Node
 * by Christian Holl (www.rad-lab.net)
 * License: BSD
 *
 * Have Fun! :-)
 */

#include <ros/ros.h>
#include <string>
#include <vector>
#include <set>
#include <limits>
#include <iostream>
#include <sstream>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/Joy.h>
#include <x52_joyext/x52_mfd.h>

enum
{
	INPUT_FLOAT32 = 0,
	INPUT_FLOAT64,
	INPUT_INT8,
	INPUT_INT16,
	INPUT_INT32,
	INPUT_UINT8,
	INPUT_UINT16,
	INPUT_UINT32,
	INPUT_BOOL,
	INPUT_JOY,
};

template<typename T>
class PublishObject
{
	enum
	{
		LEFT, CENTER, RIGHT,
	};

private:
	class PairSortMod: public std::pair<std::string, T>
	{
	public:
		bool operator<(const PairSortMod& other) const
		{
			return this->second < other.second;
		}

	};
	std::set<PairSortMod> ranges;
	ros::NodeHandle *n;	//!<The node handler
	ros::Publisher pub; //!<The publisher
	ros::Subscriber sub; //!<The subscriber

	int line;	//!< The line where the string will be placed
	int pos;	//!<Position from the beginning of the line
	int field_length; //!<Field length
	int align; //!< Align of the text in the field, 0=Left, 1=Center, 2 (or anything else)=Right
	std::string positive_oversize; //!<String to show if the integer part of a value is bigger then the size (positiv)
	std::string negative_oversize; //!<String to show if the integer part of a value is bigger then the size (negative)
	bool axis_or_button; //!<Joy topic addition
	int axis_button; //!< The button or axis number if using Joy topic
	bool stringprint; //!< If this is true, there will be strings instead of values
	std::string stringprint_setup; //!< if stringprint is enabled, this will store the strings to be displayed

public:
	PublishObject(ros::NodeHandle *n) :
			n(n)
	{
		//Get the parameters
		n->param<int>("line", line, 0);
		n->param<int>("pos", pos, 0);
		n->param<int>("field_length", field_length, 16);
		n->param<int>("align", align, 0);
		n->param<std::string>("positive_oversize", positive_oversize, "");
		n->param<std::string>("negative_oversize", negative_oversize, "");
		n->param<bool>("joy_axis_or_button", axis_or_button, 0);
		n->param<int>("joy_axis_button", axis_button, 0);
		n->param<bool>("stringprint", stringprint, "");
		n->param<std::string>("setup_string", stringprint_setup, "");

		if (field_length + pos > 16)
		{
			ROS_ERROR("%s: Field does not fit into display, will be cut!",
					ros::this_node::getName().c_str());
			field_length -= (field_length + pos) - 16;
		}

		if (field_length == 0)
		{
			ROS_ERROR("%s: Field length is zero!",
					ros::this_node::getName().c_str());
		}

		if (stringprint)
		{
			std::vector<std::string> separatedSetup;
			boost::split(separatedSetup, stringprint_setup,
					boost::is_any_of("|"));
			PairSortMod pair;
			unsigned int i;

			for (i = 0; i < separatedSetup.size(); ++i)
			{
				if ((i + 1) % 2)
				{
					if (separatedSetup[i].size() > field_length)
					{
						ROS_ERROR("%s: String: %s too big for field!",
								ros::this_node::getName().c_str(),
								separatedSetup[i].c_str());
						separatedSetup[i].erase(field_length,
								separatedSetup[i].size() - field_length);
					}
				}
				else
				{
					std::istringstream ss(separatedSetup[i]);
					pair.first = separatedSetup[i];
					this->ranges.insert(pair);
				}

				if (!i % 2 || i == 0)
					ROS_ERROR(
							"Error in setup string, missing string for upper end!");
				//Get last one
				pair.second = std::numeric_limits<T>::max();
				this->ranges.insert(pair);
			}

		}
		pub = n->advertise<x52_joyext::x52_mfd>("mfd_text", 1);
	}
	~PublishObject()
	{
	}

	void progressValue(T value)
	{
		x52_joyext::x52_mfd msg;
		msg.clearDisplay = false;
		if (stringprint)
		{
			//Get the right value
			for (typename std::set<PairSortMod>::iterator it = ranges.begin();
					it != ranges.end(); ++it)
			{
				if (it->second > value)
				{

					msg.pos = pos;
					msg.line = line;
					msg.data = it->first;
					break;
				}
			}
		}
		else
		{
			x52_joyext::x52_mfd msg;
			msg.clearDisplay = false;
			msg.pos = pos;
			msg.line = line;
			msg.data;
		}

		if (msg.data.length() < field_length)
		{
			switch (align)
			{
			default:
			case LEFT:
				//No need to do anything!
				break;

			case CENTER: //shift string the size difference devided by two to the right
				for (int s = 0; s < (msg.data.length() - field_length) / 2; ++s)
				{
					msg.data.insert(0, " ");
				}
				;
				break;

			case RIGHT: //shift string the size difference to the right
				for (int s = 0; s < (msg.data.length() - field_length); ++s)
				{
					msg.data.insert(0, " ");
				}
				;
				break;
			}
		}


		pub.publish(msg);
	}

	void JoyCallback(const sensor_msgs::JoyConstPtr &msg)
	{
		if (axis_or_button == false) //Axis
		{
			if (axis_button < (int) msg->axes.size() && axis_button >= 0)
			{
				progressValue((T) msg->axes[axis_button]);
			}
			else
			{
				ROS_ERROR("Axis %i not available for that Joystick!",
						axis_button);
			}
		}
		else
		{
			if (axis_button < (int) msg->buttons.size() && axis_button >= 0)
			{
				progressValue((T) msg->buttons[axis_button]);
			}
			else
			{
				ROS_ERROR("Button %i not available for that Joystick!",
						axis_button);
			}
		}
	}

	template<class MSGPTR>
	void Callback(const MSGPTR& msg)
	{
		progressValue(msg->data);
	}

	template<class MSG, class MSGPTR>
	void start()
	{
		sub = n->subscribe<MSG>("in", 1000, &PublishObject<T>::Callback<MSGPTR>,
				this);
		ros::spin();
	}

	void startJoy()
	{
		sub = n->subscribe<sensor_msgs::Joy>("in", 1000,
				&PublishObject<T>::JoyCallback, this);
		ros::spin();
	}

};

/**
 *	The main function
 * @param argc Size of Arguments supplied
 * @param argv Argument list
 * @return 0
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "mfd_writer");
	ros::NodeHandle n("~");

#define casem(CASE,TYPE,TYPEROS)\
	case CASE:\
	{\
		PublishObject< TYPE > obj(&n);\
		obj.start<std_msgs::TYPEROS, std_msgs:: TYPEROS## ConstPtr>();\
		break;\
	}\

	int type=0;	//!< The input type
	n.param<int>("input_type", type, INPUT_FLOAT64);
	switch (type)
	{
	casem(INPUT_FLOAT64, double, Float64)
	casem(INPUT_FLOAT32, float, Float32)
	casem(INPUT_INT32, int32_t, Int32)
	casem(INPUT_INT16, int16_t, Int16)
	casem(INPUT_INT8, int8_t, Int8)
	casem(INPUT_UINT32, uint32_t, UInt32)
	casem(INPUT_UINT16, uint16_t, UInt16)
	casem(INPUT_UINT8, uint8_t, UInt8)
	casem(INPUT_BOOL, bool, Bool)
	case INPUT_JOY:
	{
		PublishObject<double> obj(&n);
		obj.startJoy();
		break;
	}
	default:
		ROS_ERROR("Unkown Input Type %i!",type);
		break;
	}

	return 0;
}
