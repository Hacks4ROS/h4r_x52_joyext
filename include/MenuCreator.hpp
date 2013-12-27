/*
 * MenuCreator.hpp
 * by Christian Holl (www.rad-lab.net)
 * License: BSD
 *
 * Have Fun! :-)
 */

#ifndef MENUCREATOR_HPP_
#define MENUCREATOR_HPP_

#include <ros/ros.h>
#include <string>
#include <inttypes.h>
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
#include "x52_joyext/x52_mfd.h"


template <typename V, class MSG, class MSGPTR>
class MenuEntry
{
	V value; //!< The value
	ros::NodeHandle *n;	//!<The node handler
	std::string EntryName; //!< The name shown in the display
	ros::Publisher pub; //! The publisher if it is able to publish
	ros::Subscriber sub; //! The subscriber if it is able to subscribe

	enum Type
	{
		Pub,
		Sub,
		PubSub,
		Simple,
	};

private:
	void CallbackFkt()
	{

	}

public:
	MenuEntry(ros::NodeHandle &n, std::string EntryName, MenuEntry::Type type=Simple, std::string topic="", bool latch)
	:EntryName(EntryName)
	{
		if(type!=Simple)
		{
			bool en_pub=false;
			bool en_sub=false;
			if(topic.size()>0)
			{
				switch(type)
				{
				case Pub:
					en_pub=true;
					break;
				case Sub:
					en_sub=true;
					break;
				case PubSub:
					en_sub=true;
					en_pub=true;
					break;
				}

				if(en_pub)
				{
					pub=
				}

				if(en_sub)
				{
					sub = n->subscribe<MSG>("in", 1000, &PublishMFD<T>::Callback<MSGPTR>,
							this);
				}
			}
			else
			{
				ROS_ERROR("MenuEntry: Topic not defined");
			}
		}
	}
	~MenuEntry()
	{

	}
};


class MenuCreator
{
public:
	MenuCreator(ros::NodeHandle &n);
	virtual ~MenuCreator();
};

#endif /* MENUCREATOR_HPP_ */
