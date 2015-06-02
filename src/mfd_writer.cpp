/*
 * Mfd Writer Node
 * by Christian Holl (www.rad-lab.net)
 * License: BSD
 *
 * Have Fun! :-)
 */

/**
\page x52_mfd_writer_node x52_mfd_writer_node
Printing values from std_msgs topics to the displays can be easily done with the x52_mfd_writer_node.

For each topic which you want to see information on the display you need a mfd_writer node.

All supported message types can be seen in the following table, in a launch file, they are specified
by the parameter "input_type"

input_type | Description
-----------| ------------
0<sub>\ref mfd_std "a" </sub>  | float64
1          | float32
2		   | int64
3          | int32
4          | int16
5<sub>\ref mfd_chr "b" </sub>       | int8
6          | uint64
7          | uint32
8          | uint16
9<sub>\ref mfd_chr "b"</sub>       | uint8
10         | bool
11<sub>\ref mfd_joy "c"</sub>        | Joy



b) \anchor mfd_chr When using a one byte value (mainly known as char in C/C++), it is normally treated as character.\n
If the number value is wanted instead you have to set the value "char_as_int" to true.

c) \anchor mfd_joy Requires additional setup information, see following table:

Parameter      | Description
---------------|----------
axis_or_button | Defines if the value being used comes from an axis(false) or a button(true)
axis_button    | Defines the axis or button number


To specify the arangement in the display the following parameters are used.

Parameter   | Description
------------|------------
line        | Defines the line, where the field should be placed
pos         | Defines the start of the field inside the line
field_length| Defines the length of the field

To specify what happens to the value alignment when the size of it is smaller the field length,
the parameter align is used.

Align  | Description
-------|------------
0<sub>\ref mfd_std "a" </sub> | Left  - The field is filled with spaces from the right side, so that the value is on the left
1      | Center - The field is filled with spaces from both sides
2      | Right - The field is filled with spaces from the left side, so that the value is on the right



If the integer part of any value exceeds the field size, the user should be notified somehow, that
there is a problem. This is done by specifiying the oversize parameters:

Parameter        | Description
-----------------|------------
positive_oversize| String shown when value is positiv and it's integer value does not fit into the field
negative_oversize| String shown when value is negative and it's integer value does not fit into the field

At least there is the option to print strings instead of values, this uses the same specification method
as known from the value2buttonColor node.

Parameter         | Description
------------------|-------------
strintprint       | if true the stringprint_setup variable specifies the ranges in which a string is printed
stringprint_setup | Specification when to write which string, see following paragraph



The first given string is printed if the value is lower than any given value,
the last one is printed when the value is higher than any given value.

In between the values are printed when the given value is exceeded.

Example:

	Low|-4|One|3|Two|6|High

\n

Example Results:
Value | Result
------|-------
-6	  | Low
-4 	  | Low
 3    | One
 5    | Two
 7    | High

\n

Example Conditions:
String  | Prints when
------- | ------------
Low		| Value <=-4
One		| Value <=3
Two		| Value <=6
High	| Value >6

\n

Copy\&Paste Launchfile Code:

	<node pkg="h4r_x52_joyext" type="x52_mfd_writer_node" name="x52_mfd_writer_string" output="screen">
		<param name="input_type" value="11"/>
		<param name="line" value="1"/>
		<param name="pos" value="0"/>
		<param name="field_length" value="16"/>
		<param name="align" value="1"/>
		<param name="positive_oversize" value="++EE++"/>
		<param name="negative_oversize" value="--EE--"/>
		<param name="axis_or_button" value="false"/>
		<param name="axis_button" value="4"/>
		<param name="stringprint" value="true"/>
		<param name="stringprint_setup" value="Green|-0.5|Yellow|0.5|Red"/>
		<param name="char_as_int" value="0"/>

		<!-- Remap Topics -->
		<remap from="~/in" to="/joy" />
		<remap from="~/mfd_text" to="/x52/mfd_text" />
	</node>


a) \anchor mfd_std Standard value\n
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
#include <std_msgs/Int64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>



#include <std_msgs/Bool.h>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/Joy.h>
#include <h4r_x52_joyext/x52_mfd.h>

enum
{
	INPUT_FLOAT32 = 0,
	INPUT_FLOAT64,
	INPUT_INT8,
	INPUT_INT16,
	INPUT_INT32,
	INPUT_INT64,
	INPUT_UINT8,
	INPUT_UINT16,
	INPUT_UINT32,
	INPUT_UINT64,
	INPUT_BOOL,
	INPUT_JOY,
};

template<typename T>
class PublishMFD
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
	PublishMFD(ros::NodeHandle *n) :
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
			{	
				ROS_ERROR("Error in setup string, missing string for upper end!");
				exit(1);
			}
			//Get last one
			pair.second = std::numeric_limits<T>::max();
			this->ranges.insert(pair);

		}

		pub = n->advertise<h4r_x52_joyext::x52_mfd>("mfd_text", 1,1);
		h4r_x52_joyext::x52_mfd init_msg;
		init_msg.clearDisplay=false;
		init_msg.line=line;
		init_msg.pos=pos;
		for (int i = 0; i < field_length; ++i) {
		init_msg.data+="-";
		}
		pub.publish(init_msg);
	}
	~PublishMFD()
	{
	}


	template <typename V>
	std::string get_value_string(V value)
	{

		h4r_x52_joyext::x52_mfd msg;
		std::ostringstream ss;
		std::string value_string;
		ss<<value;


		return ss.str();

	}

	std::string get_value_string(char value)
	{
		h4r_x52_joyext::x52_mfd msg;
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
		h4r_x52_joyext::x52_mfd msg;
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
		sub = n->subscribe<MSG>("in", 1000, &PublishMFD<T>::Callback<MSGPTR>,
				this);
		ros::spin();
	}

	void startJoy()
	{
		sub = n->subscribe<sensor_msgs::Joy>("in", 1000,
				&PublishMFD<T>::JoyCallback, this);
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
		PublishMFD< TYPE > obj(&n);\
		obj.start<std_msgs::TYPEROS, std_msgs:: TYPEROS## ConstPtr>();\
		break;\
	}\

	int type=0;	//!< The input type
	n.param<int>("input_type", type, INPUT_FLOAT64);
	switch (type)
	{
	casem(INPUT_FLOAT64, double_t, Float64)
	casem(INPUT_FLOAT32, float_t, Float32)
	casem(INPUT_INT64, int64_t, Int64)
	casem(INPUT_INT32, int32_t, Int32)
	casem(INPUT_INT16, int16_t, Int16)
	casem(INPUT_INT8, int8_t, Int8)
	casem(INPUT_UINT64, uint64_t, UInt64)
	casem(INPUT_UINT32, uint32_t, UInt32)
	casem(INPUT_UINT16, uint16_t, UInt16)
	casem(INPUT_UINT8, uint8_t, UInt8)
	casem(INPUT_BOOL, bool, Bool)
	case INPUT_JOY:
	{
		PublishMFD<double> obj(&n);
		obj.startJoy();
		break;
	}
	default:
		ROS_ERROR("Unkown Input Type %i!",type);
		break;
	}

	return 0;
}
