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
#include <vector>
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
#include <x52_joyext/x52_mfd.h>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/Joy.h>

class MenuEntryAbstraction
{
public:
	typedef void (*updateFktPtr)(void);
	/**
	 * The type for the list entry
	 */
	typedef enum
	{
		Pub,   //!< Pub Publisher - editable by the user - sends out value after edit
		Sub,   //!< Sub Subscriber - gets value from topic
		PubSub,//!< PubSub Editable by user and gets value from topic
		Simple,//!< Simple Value received by Simple topic
	}EntryType;


protected:
	updateFktPtr update; //!< update function, if there is new stuff to write to display
	bool selected;	//!< True if this entry is currently selected
	std::string EntryName; //!< The name shown in the display

public:
	/**
	 * Constructor
	 */
	MenuEntryAbstraction(std::string EntryName,updateFktPtr update)
	:selected(false),
	 EntryName(EntryName),
	 update(update)
	{}

	/**
	 * Destructor
	 */
	virtual ~MenuEntryAbstraction()
	{}

	/**
	 * When in a value display the routine responsible for showing is getting each line
	 * from in here.
	 *
	 * @param line The line to be returned
	 * @return Line string according to line
	 */
	virtual std::string getFullScreenLine(uint8_t line)=0;

	/**
	 * Scrollwheel
	 * @param up Scrollwheel up
	 * @param down Scrollwheel down
	 * @param enter The scrollwheel button
	 * @return true if user is done with entry
	 */
	virtual bool scrollwheel(bool up, bool down, bool enter)=0;

	/**
	 * Sets the state if the menu entry is selected
	 * @param selected selected state
	 */
	void setSelected(bool selected)
	{
		this->selected=selected;
	}

	/**
	 * When in Menu display the routine responsible for collecting the menu
	 * data will call this to get the then displayed menu line for this entry.
	 * @return EntryName
	 */
	virtual std::string getMenuLine()=0;

	/**
	 * Getter for the EntryName
	 */
	std::string getEntryName()
	{
		return EntryName;
	}



};








template <typename V, class MSG, class MSGPTR>
class MenuEntry : public MenuEntryAbstraction
{
public:

	typedef enum
	{
		USER_INTERACT_BACK,
		USER_INTERACT_PUBLISH,
		USER_INTERACT_PUBVAL,
		USER_INTERACT_SUBVAL,
		USER_INTERACT_PUBVAL_EDIT,
	}user_interact_state_t;



private:
	V value; //!< The value
	V useredit; //!< The variable where the value is stored the user edits
	EntryType type;
	ros::NodeHandle *n;	//!<The node handler
	ros::Publisher pub; //! The publisher if it is able to publish
	ros::Subscriber sub; //! The subscriber if it is able to subscribe


	bool en_pub; //!< True if this entry has a publisher
	bool en_sub; //!< True if this entry has a subscriber

	user_interact_state_t interact; //!<If it is selected this is the interact state.

	bool button_need_release; //!< For the function scrollwheel this stores if the button has been released before

	/**
	 * The callback function for setting the value from a subscriber
	 * @param msg Message data
	 */
	void Callback(const MSGPTR& msg)
	{
		value=msg->data;
	}


	/**
	 * Returns the string for a value
	 * @param value The Value
	 * @return String of value
	 */
	std::string ValueString(V value)
	{
		std::ostringstream ss;
		ss<<value;
		return ss.str();
	}

	/**
	 * Returns the string for a signed char
	 * @param value The Value
	 * @return String of value
	 */
	std::string ValueString(signed char value)
	{
		std::ostringstream ss;
		ss<<(int)value;
		return ss.str();
	}

	/**
	 * Returns the string for a unsigned char
	 * @param value The Value
	 * @return String of value
	 */
	std::string ValueString(unsigned char value)
	{
		std::ostringstream ss;
		ss<<(unsigned int)value;
		return ss.str();
	}

public:
	/**
	 * Constructor
	 * @param n Node handle
	 * @param EntryName The name of the current entry, displayed for the user
	 * @param type The Type, simple for a entry of the simple_in topic, Sub, Pub, PubSub for topics given in launchfile
	 * @param topic The name of the topic when type=Sub,Pub or PubSub
	 * @param latch If a publisher is used, it supplies the latching parameter
	 */
	MenuEntry(ros::NodeHandle &n, std::string EntryName, MenuEntryAbstraction::EntryType type=Simple, std::string topic="", bool latch=0)
	:value(0),
	 useredit(0),
	 type(type),
	 n(n),
	 en_pub(false),
	 en_sub(false),
	 interact(USER_INTERACT_BACK),
	 button_need_release(false)
	{
		this->MenuEntryAbstraction(EntryName);
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
					pub= n.advertise<MSG>(topic,1,latch);
				}

				if(en_sub)
				{
					sub = n.subscribe<MSG>(topic, 1, &MenuEntry<V,MSG,MSGPTR>::Callback,this);
				}
			}
			else
			{
				ROS_ERROR("MenuEntry: Topic not defined");
			}
		}


	}
	/**
	 * The destructor
	 */
	~MenuEntry()
	{}

	std::string getMenuLine()
	{
		return EntryName;
	}


	std::string getFullScreenLine(uint8_t line)
	{

		switch(line)
		{
		case 0:
			return EntryName;
			break;
		case 1:
		{
			return ValueString(value);
		}
		break;
		case 2:

			break;
		default:
			ROS_ERROR("MenuCreator::getFullScreenLine-Error Line not possible: %i",line);
			break;
		}
		return "---LINE-ERROR---";
	}
#define	USER_INTERACT_STATE_ERROR\
		ROS_ERROR("Entry was in wrong interaction state!  %i %i", type, interact);\
		interact=USER_INTERACT_BACK


	bool scrollwheel(bool up, bool down, bool enter)
	{
		//Prevent multiple button press events when it was pressed only once
		if(button_need_release)
		{
			if(!up && !down && !enter)
				button_need_release=false;
			return 0;
		}
		else if(!button_need_release && (up || down || enter))
		{
			button_need_release=true;
		}

		if(enter)//scrollwheel pressed
		{
			switch(interact)
			{
			case USER_INTERACT_BACK:
				return true;
				break;

			case USER_INTERACT_SUBVAL:
				useredit=value;
				break;

			case USER_INTERACT_PUBVAL:
				interact=USER_INTERACT_PUBVAL_EDIT;
				break;

			case USER_INTERACT_PUBLISH:
				{
					MSG msg;
					msg.data=value;
					pub.publish(msg);
				}
				break;
			case USER_INTERACT_PUBVAL_EDIT:
				interact=USER_INTERACT_PUBVAL;
				break;
			}
		}
		else if(up && !down) //up button pressed
		{
			switch(interact)
			{
			case USER_INTERACT_BACK:
					switch(type)
					{
					case Pub:
					case PubSub:
						interact=USER_INTERACT_PUBLISH;
						break;
					default:
						break;
					}
				break;

			case USER_INTERACT_PUBLISH:
				switch(type)
				{
				case Pub:
					interact=USER_INTERACT_PUBVAL;
					break;
				case PubSub:
					interact=USER_INTERACT_SUBVAL;
					break;

				default:
					USER_INTERACT_STATE_ERROR;
					break;
				}
				break;

			case USER_INTERACT_PUBVAL:
				switch(type)
				{
				case Pub:
				case PubSub:
					interact=USER_INTERACT_BACK;
					break;

				default:
					USER_INTERACT_STATE_ERROR;
					break;
				}

				break;

			case USER_INTERACT_SUBVAL:
				switch(type)
				{
				case PubSub:
					interact=USER_INTERACT_PUBVAL;
					break;

				default:
					USER_INTERACT_STATE_ERROR;
					break;
				}
				break;

			case USER_INTERACT_PUBVAL_EDIT:
				useredit++;
				break;
			}
		}
		else if(down && !up)
		{
			switch(interact)
			{
			case USER_INTERACT_BACK:
				switch(type)
				{
				case Pub:
				case PubSub:
					interact=USER_INTERACT_PUBVAL;
					break;
				default:
					break;
				}
				break;

			case USER_INTERACT_SUBVAL:
				switch(type)
				{
				case PubSub:
					interact=USER_INTERACT_PUBLISH;
				break;
				default:
					USER_INTERACT_STATE_ERROR;
					break;
				}
				break;

			case USER_INTERACT_PUBVAL:
				switch(type)
				{
				case Pub:
					interact=USER_INTERACT_PUBLISH;
					break;
				case PubSub:
					interact=USER_INTERACT_SUBVAL;
					break;
				default:
					USER_INTERACT_STATE_ERROR;
					break;
				}
				break;

			case USER_INTERACT_PUBLISH:
				interact=USER_INTERACT_BACK;
				break;

			case USER_INTERACT_PUBVAL_EDIT:
				switch(type)
				{
					case Pub:
					case PubSub:
						useredit--;
						break;
					default:
						USER_INTERACT_STATE_ERROR;
						break;
				}
				break;
			}
		}
		else
		{
			return 0;
		}
		return 0;
	}


	void setSelected(bool selected)
	{
		this->selected=selected;
		this->interact=USER_INTERACT_BACK;
	}

};


class MenuCreator
{
private:
	std::vector<MenuEntryAbstraction *> menu_entry_list;

public:
	MenuCreator(ros::NodeHandle &n);
	virtual ~MenuCreator();

	void run();
};

#endif /* MENUCREATOR_HPP_ */
