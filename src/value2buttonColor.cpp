/*
 * value2buttonColor Node
 * by Christian Holl (www.rad-lab.net)
 * License: BSD
 *
 * Have Fun! :-)
 */

/**
\page x52_value2buttonColor_node x52_value2buttonColor_node

All supported message types can be seen in the following table, in a launch file, they are specified
by the parameter "input_type"

input_type | Description
-----------| ------------
0<sub>\ref col_std "a" </sub>  | float64
1          | float32
2		   | int64
3          | int32
4          | int16
5          | int8
6          | uint64
7          | uint32
8          | uint16
9          | uint8
10         | bool
11<sub>\ref col_joy "b"</sub>        | Joy

b) \anchor col_joy Requires additional setup information, see following table:

Parameter      | Description
---------------|----------
axis_or_button | Defines if the value being used comes from an axis(false) or a button(true)
axis_button    | Defines the axis or button number


The color LED can be selected with the color_led parameter

color_led | Description
----------|------------
0		  | LED FIRE
1         | LED A
2         | LED B
3         | LED D
4         | LED E
5         | LED T1/2
6         | LED T3/4
7         | LED T5/6
8         | LED POV 2
9         | LED I



Colors available:

Color-Char | Description
-----------| -----------
O          | Off
G		   | Green/On <sub>\ref col_on "c"</sub>
Y          | Yellow
R          | Red


c) \anchor col_on FIRE Button only supports On or Off because the color is controlled by the saftey cover,
when closed it is green and when opened red.

For specifying what color the LED should be at which value the setup_string parameter
must be supplied. It uses a special syntax. The first given color is set if the value is
lower than any given value, the last one is set when the value is higher than any given value.
In between the values are printed when the given value is exceeded.

Example:

	R|-4|Y|3|G|6|Y

	\n

Example Results:
Value | Result
------|-------
-6	  | Red
-4 	  | Red
 3    | Yellow
 5    | Green
 7    | Yellow

\n

Example Conditions:
String  | Prints when
------- | ------------
Red		| Value <=-4
Yellow	| Value <=3
Green	| Value <=6
Yellow	| Value >6

\n

Copy\&Paste Launchfile Code:

	<node pkg="h4r_x52_joyext" type="x52_value2buttonColor_node" name="Color_Button_0" output="screen">
		<param name="input_type" value="11"/>
		<param name="joy_axis_button" value="4"/>
		<param name="joy_axis_or_button" value="false"/>
		<param name="color_led" value="4" />
		<param name="setup_string" value="G|-0.5|Y|0.5|R"/>
		<remap from="/Color_Button_0/in" to="/joy" />
		<remap from="/Color_Button_0/led" to="/x52/leds" />
	</node>


a) \anchor col_std Standard value\n
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
#include <h4r_x52_joyext/x52_led_color.h>

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

template <typename T>
class PublishLED
{
	enum
	{
		OFF='O',
		GREEN='G',
		YELLOW='Y',
		RED='R',
	};

private:
	class PairSortMod : public std::pair<unsigned int,T>
	{
	public:
	    bool operator<(const PairSortMod& other) const
	    {
	        return this->second < other.second;
	    }

	};
	std::set< PairSortMod > ranges;
	ros::NodeHandle *n;
	ros::Publisher pub;
	int led;
	bool axis_or_button;
	int axis_button;
	ros::Subscriber sub;


public:
	PublishLED(ros::NodeHandle *n, std::string setup, int led, int axis, bool axis_or_button)
	:n(n),
	 axis_or_button(axis_or_button),
	 axis_button(axis)
	{
		std::vector<std::string> separatedSetup;
		boost::split(separatedSetup, setup, boost::is_any_of("|"));
		PairSortMod pair;
		unsigned int i;
		for (i = 0; i < separatedSetup.size(); ++i)
		{
			if((i+1)%2 )
			{
				if(separatedSetup[i].size()!=1)
				{
					ROS_ERROR("Faulty setup string part, expected 'O','G','Y' or 'R' got %s",separatedSetup[i].c_str());
					exit(1);
				}
				else
				{
					switch(separatedSetup[i][0])
					{
					case 'O':
						pair.first=1;
						break;

					case 'G':
						pair.first=2;
						break;

					case 'Y':
						pair.first=3;
						break;

					case 'R':
						pair.first=4;
						break;

					default:
						ROS_ERROR("Faulty setup string part, expected 'O','G','Y' or 'R' got %s",separatedSetup[i].c_str());
						exit(1);
					}
				}
			}
			else
			{
				std::istringstream ss(separatedSetup[i]);
				T value;
				ss>>value;
				//std::cout<<"::"<<value<<"\n";
				if(ss.bad() || ss.fail() || !ss.eof())
				{
					ROS_ERROR("Error in setup string: %s",separatedSetup[i].c_str());
					exit(1);
				}
				pair.second=value;
				this->ranges.insert(pair);
			}
		}

		if(!i%2 || i==0)
			ROS_ERROR("Error in setup string, missing led setting for upper end!");
		//Get last one
		pair.second=std::numeric_limits<T>::max();
		this->ranges.insert(pair);

		h4r_x52_joyext::x52_led_color ledmsg;
		if(led<0 || led>(int)ledmsg.color_leds.size())
		{
			ROS_ERROR("LED number out of range!");
			exit(1);
		}
		else
		{
			this->led=led;
		}
		pub=n->advertise< h4r_x52_joyext::x52_led_color >("led",1);
	}
	~PublishLED()
	{}

	void progressValue(T value)
	{
		for (typename std::set<PairSortMod>::iterator it = ranges.begin(); it != ranges.end(); ++it)
		{
			if(it->second>value)
			{
				h4r_x52_joyext::x52_led_color msg;
				msg.color_leds[this->led]=it->first;
				pub.publish(msg);
				break;
			}
		}
	}

	void JoyCallback(const sensor_msgs::JoyConstPtr &msg)
	{
		if(axis_or_button==false)//Axis
		{
			if(axis_button<(int)msg->axes.size() && axis_button>=0)
			{
				progressValue((T)msg->axes[axis_button]);
			}
			else
			{
				ROS_ERROR("Axis %i not available for that Joystick!", axis_button);
			}
		}
		else
		{
			if(axis_button<(int)msg->buttons.size() && axis_button>=0)
			{
				progressValue((T)msg->buttons[axis_button]);
			}
			else
			{
				ROS_ERROR("Button %i not available for that Joystick!", axis_button);
			}
		}
	}

	template <class MSGPTR>
	void Callback(const MSGPTR& msg)
	{
		progressValue(msg->data);
	}



	template <class MSG, class MSGPTR>
	void start()
	{
		sub=n->subscribe<MSG>("in",1000, &PublishLED<T>::Callback< MSGPTR >, this);
		ros::spin();
	}

	void startJoy()
	{
		sub=n->subscribe<sensor_msgs::Joy>("in",1000, &PublishLED<T>::JoyCallback, this);
		ros::spin();
	}

};



//Off, Green, Yellow, Red

int main(int argc, char **argv)
{
	ros::init(argc, argv, "value2buttonColor");
	ros::NodeHandle n("~");

	int type;
	int led;
	bool axis_or_button;
	int axis_button;
	std::string setup;


	n.param<int>("input_type", type, INPUT_FLOAT64);
	n.param<int>("joy_axis_button", axis_button, 0);
	n.param<bool>("joy_axis_or_button", axis_or_button, 0);
	n.param<int>("color_led",led, 0);
	ROS_DEBUG("LED -> %i",led);


	//means OFF till -0.5, GREEN till 0, YELLOW till 0.5, and everything greater is RED
	n.param<std::string>("setup_string", setup, "O|-0.5|G|0|Y|0.5|R");



#define casem(CASE,TYPE,TYPEROS)\
	case CASE:\
	{\
		PublishLED< TYPE > obj(&n,setup,led,axis_button,axis_or_button);\
		obj.start<std_msgs::TYPEROS, std_msgs:: TYPEROS## ConstPtr>();\
		break;\
	}\



	switch(type)
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
			PublishLED< double > obj(&n,setup,led,axis_button, axis_or_button);
			obj.startJoy();
			break;
		}
		default:
			ROS_ERROR("Unkown Input Type!");
			break;
	}
}
