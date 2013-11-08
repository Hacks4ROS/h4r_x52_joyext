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
#include <x52_joyext/x52_led_color.h>

enum
{
	INPUT_FLOAT32=0,
	INPUT_FLOAT64,
	INPUT_INT8,
	INPUT_INT16,
	INPUT_INT32,
	INPUT_UINT8,
	INPUT_UINT16,
	INPUT_UINT32,
	INPUT_BOOL,
	INPUT_JOY,
} ;


template <typename T>
class PublishObject
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
	int axis;
	ros::Subscriber sub;


public:
	PublishObject(ros::NodeHandle *n, std::string setup, int led, int axis)
	:n(n),
	 axis(axis)
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

		x52_joyext::x52_led_color ledmsg;
		if(led<0 || led>(int)ledmsg.color_leds.size())
		{
			ROS_ERROR("LED number out of range!");
			exit(1);
		}
		else
		{
			this->led=led;
		}
		pub=n->advertise< x52_joyext::x52_led_color >("led",1);
	}
	~PublishObject()
	{}

	void progressValue(T value)
	{
		for (typename std::set<PairSortMod>::iterator it = ranges.begin(); it != ranges.end(); ++it)
		{
			if(it->second>value)
			{
				x52_joyext::x52_led_color msg;
				msg.color_leds[this->led]=it->first;
				pub.publish(msg);
				break;
			}
		}
	}

	void JoyCallback(const sensor_msgs::JoyConstPtr &msg)
	{
		if(axis<(int)msg->axes.size() && axis>=0)
		{
			progressValue((T)msg->axes[axis]);
		}
		else
		{
			ROS_ERROR("Axis %i not available for that Joystick!", axis);
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
		sub=n->subscribe<MSG>("in",1000, &PublishObject<T>::Callback< MSGPTR >, this);
		ros::spin();
	}

	void startJoy()
	{
		sub=n->subscribe<sensor_msgs::Joy>("in",1000, &PublishObject<T>::JoyCallback, this);
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
	int axis;
	std::string setup;

	n.param<int>("input_type", type, INPUT_FLOAT64);
	n.param<int>("joy_axis", axis, 0);
	n.param<int>("color_led",led, 0);
	ROS_INFO("LED -> %i",led);

	n.param<std::string>("setup_string", setup, "O|-0.5|G|0|Y|0.5|R");



#define casem(CASE,TYPE,TYPEROS)\
	case CASE:\
	{\
		PublishObject< TYPE > obj(&n,setup,led,axis);\
		obj.start<std_msgs::TYPEROS, std_msgs:: TYPEROS## ConstPtr>();\
		break;\
	}\



	switch(type)
	{
		casem(INPUT_FLOAT64,double,Float64)
		casem(INPUT_FLOAT32,float,Float32)
		casem(INPUT_INT32,int32_t,Int32)
		casem(INPUT_INT16,int16_t,Int16)
		casem(INPUT_INT8,int8_t,Int8)
		casem(INPUT_UINT32,uint32_t,UInt32)
		casem(INPUT_UINT16,uint16_t,UInt16)
		casem(INPUT_UINT8,uint8_t,UInt8)
		casem(INPUT_BOOL,bool,Bool)
		case INPUT_JOY:
		{
			PublishObject< double > obj(&n,setup,led,axis);
			obj.startJoy();
			break;
		}
		default:
			ROS_ERROR("Unkown Input Type!");
			break;
	}
}