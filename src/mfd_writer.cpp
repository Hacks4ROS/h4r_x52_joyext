/*
 * Mfd Writer Node
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
	int char_as_int; //! A char is interpreted as integer if this is 1 or as unsigned integer if this is 2 and as char on any other value

public:
	PublishObject(ros::NodeHandle *n) :
			n(n)
	{
		//Get the parameters
		n->param<int>("line", line, 0);
		n->param<int>("pos", pos, 0);
		n->param<int>("field_length", field_length, 16);
		n->param<int>("align", align, 0);
		n->param<std::string>("positive_oversize", positive_oversize, "#");
		n->param<std::string>("negative_oversize", negative_oversize, "#");
		n->param<bool>("axis_or_button", axis_or_button, 0);
		n->param<int>("axis_button", axis_button, 0);
		n->param<bool>("stringprint", stringprint, 0);
		n->param<std::string>("stringprint_setup", stringprint_setup, "");
		n->param<int>("char_as_int", char_as_int, 0);

		if(char_as_int!=0)
			ROS_WARN("%s: char_as_int = %i (assuming 0 -> interpred as char)",ros::this_node::getName().c_str(), char_as_int);


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
			ROS_INFO("%s",stringprint_setup.c_str());
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
					pair.first = separatedSetup[i];
					ROS_INFO("%s",separatedSetup[i].c_str());
				}
				else
				{
					std::istringstream ss(separatedSetup[i]);
					T value;
					ss>>value;
					if(ss.bad() || ss.fail() || !ss.eof())
					{
						ROS_ERROR("Error in setup string: %s",separatedSetup[i].c_str());
						exit(1);
					}
					pair.second=value;
					this->ranges.insert(pair);
				}
			}

			if (!i % 2 || i == 0)
				ROS_ERROR(
						"Error in setup string, missing string for upper end!");
			//Get last one
			pair.second = std::numeric_limits<T>::max();
			this->ranges.insert(pair);

		}

		pub = n->advertise<x52_joyext::x52_mfd>("mfd_text", 1,1);
		x52_joyext::x52_mfd init_msg;
		init_msg.clearDisplay=false;
		init_msg.line=line;
		init_msg.pos=pos;
		for (int i = 0; i < field_length; ++i) {
		init_msg.data+="-";
		}
		pub.publish(init_msg);
	}
	~PublishObject()
	{
	}


	template <typename V>
	std::string get_value_string(V value)
	{

		x52_joyext::x52_mfd msg;
		std::ostringstream ss;
		std::string value_string;
		ss<<value;


		return ss.str();

	}

	std::string get_value_string(char value)
	{
		x52_joyext::x52_mfd msg;
		std::ostringstream ss;
		std::string value_string;

		//std::cout<<"::"<<value<<"\n";
		{
			ROS_ERROR("%s: Error in creating (char) value string",ros::this_node::getName().c_str());

			ss.clear();

			for(int c = 0; c < field_length; ++c)
			{
				ss<<'E';
			}
		}

		if(char_as_int==1)
		{
			if(value!=0)
			ss<<(signed int)value;
		}
		else if(char_as_int==2)
		{
			ss<<(unsigned int)value;
		}
		else
		{
			ss<<value;
		}

		return ss.str();
	}





	template <typename V>
	void progressValue(V value)
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
			msg.clearDisplay = false;
			msg.pos = pos;
			msg.line = line;
			msg.data=get_value_string(value);

		}




		if (msg.data.length() < field_length)
		{
			switch (align)
			{
			default:
			case LEFT: //filling up the field to overwrite chars not needed
				for (int s = 0; s < (msg.data.length() - field_length); ++s)
				{
					msg.data.push_back(' ');
				}
				break;

			case CENTER: //shift string the size difference devided by two to the right
				for (int s = 0; s < ((msg.data.length() - field_length) / 2); ++s)
				{
					if(s%2)
					msg.data.insert(0, " ");
					else
					msg.data.push_back(' ');
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
		else if(msg.data.length()>field_length)
		{
			//Decimal point position
			int dec_pos=msg.data.find('.');

			std::cout<<msg.data<<std::endl;
			if(dec_pos >= 0 && dec_pos==field_length-1) //decimal is the last what fits into field
			{
				msg.data.erase(dec_pos,field_length-dec_pos);//erase everything after dec_pos including dec_pos
			}
			else if(dec_pos >= 0 && dec_pos<field_length-1)//decimal point is not the last what fits into field
			{
				msg.data.erase(field_length-1,msg.data.length()-field_length);//cut off everything whats longer than field
			}
			else //The string is over size for that field!
			{
				if(value>0)
				{
					msg.data=positive_oversize;
				}
				else
				{
					msg.data=negative_oversize;
				}
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
				progressValue<double>(msg->axes[axis_button]);
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
				progressValue<bool>(msg->buttons[axis_button]);
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
