/*
 * MenuCreator.cpp
 * by Christian Holl (www.rad-lab.net)
 * License: BSD
 *
 * Have Fun! :-)
 */
#include "MenuCreator.hpp"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <algorithm>
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
};

MenuCreator::MenuCreator(ros::NodeHandle &n)
{
	this->n=&n;
	insideEntry=false;


	std::string setup;
	// TODO Auto-generated constructor stub
	n.param<std::string>("menu_setup", setup, "");

	std::vector<std::string> menu_entries;
	boost::split(menu_entries, setup, boost::is_any_of("|"));

	int num = 0; //Counting statement number
	for (std::vector<std::string>::iterator entry = menu_entries.begin();
			entry != menu_entries.end(); entry++)
	{
		std::vector<std::string> entrySetup;
		boost::split(entrySetup, *entry, boost::is_any_of(";"));

		//Check if we have at least 3 entries
		if (entrySetup.size() < 3)
		{
			ROS_ERROR(
					"ERROR entry number %i: Minimum required arguments: 3! -> not created",
					num);
			continue;
		}

		//Name
		if (entrySetup[0].size() == 0)
		{
			ROS_ERROR(
					"ERROR entry number %i: Name is zero length! -> not created!",
					num);
			continue;
		}

		std::istringstream et(entrySetup[1]);
		unsigned int EntryType;
		et >> EntryType;
		if (et.bad() || et.fail() || !et.eof()
				|| !(EntryType == MenuEntryAbstraction::Pub
						|| EntryType == MenuEntryAbstraction::Sub
						|| EntryType == MenuEntryAbstraction::PubSub
						|| EntryType == MenuEntryAbstraction::Simple))
		{
			ROS_ERROR("ERROR entry type for entry %i: %s  -> Entry not created",
					num, entrySetup[1].c_str());
			continue;
		}

		if (EntryType == MenuEntryAbstraction::Simple)
		{

		}
		else
		{
			if (entrySetup.size() < 4)
			{
				ROS_ERROR(
						"ERROR entry number %i: Minimum required arguments: 4! -> not created",
						num);
				continue;
			}

			std::string topic = entrySetup[3];
			if (EntryType != MenuEntryAbstraction::Simple && topic.size() == 0)
			{
				ROS_ERROR("ERROR topic name is empty for entry %i", num);
			}

			bool latch = false;
			if (entrySetup.size() > 4)
			{
				//to lower case
				std::transform(entrySetup[4].begin(), entrySetup[4].end(),
						entrySetup[4].begin(), ::tolower);
				if (entrySetup[4] == "true" || entrySetup[4] == "1")
				{
					latch = true;
				}
				else if (entrySetup[4] == "false" || entrySetup[4] == "0")
				{
					//already false
				}
				else
				{
					ROS_ERROR("ERROR in latching argument for entry %i", num);
					continue;
				}
			}

#define casePS(CASE,TYPE,TYPEROS)\
	case CASE:\
	{\
		menu_entry_list.push_back(new MenuEntry<TYPE, std_msgs::TYPEROS, std_msgs::TYPEROS##ConstPtr>(&n,entrySetup[0],std::bind(&MenuCreator::update,this),(MenuEntryAbstraction::EntryType)EntryType,topic,latch));\
		break;\
	}\


			int type = 0;	//!< The input type
			n.param<int>("input_type", type, INPUT_FLOAT64);
			switch (type)
			{
			casePS(INPUT_FLOAT64, double_t, Float64)
			casePS(INPUT_FLOAT32, float_t, Float32)
			casePS(INPUT_INT64, int64_t, Int64)
			casePS(INPUT_INT32, int32_t, Int32)
			casePS(INPUT_INT16, int16_t, Int16)
			casePS(INPUT_INT8, int8_t, Int8)
			casePS(INPUT_UINT64, uint64_t, UInt64)
			casePS(INPUT_UINT32, uint32_t, UInt32)
			casePS(INPUT_UINT16, uint16_t, UInt16)
			casePS(INPUT_UINT8, uint8_t, UInt8)
			casePS(INPUT_BOOL, bool, Bool)
			default:
				break;

			}

		}

		num++;
	}

	n.param<int>("up_button", this->up, 36);
	n.param<int>("down_button", this->down, 37);
	n.param<int>("enter_button", this->enter, 38);
	pub_mfd=n.advertise< x52_joyext::x52_mfd >("/mfd_text",1);
	sub_joy=n.subscribe<sensor_msgs::Joy>("/joy",1, &MenuCreator::cb_joy, this);
	current_entry=menu_entry_list.begin();

	for (int i = 0; i < 3; ++i)
	current_display_content.push_back("                ");
}

MenuCreator::~MenuCreator()
{
	// TODO Auto-generated destructor stub

	for (int m = 0; m < this->menu_entry_list.size(); ++m)
	{
		delete this->menu_entry_list[m];
	}
	menu_entry_list.clear();
}

void MenuCreator::run()
{
	//ros::Rate loop_rate(5);

	ros::spin();
}


void MenuCreator::cb_joy(const sensor_msgs::JoyConstPtr &msg)
{

	//Prevent multiple button press events when it was pressed only once
	if(button_need_release)
	{
		if(!msg->buttons[up] && !msg->buttons[down] && !msg->buttons[enter])
		{

			button_need_release=false;
		}
		return;
	}
	else if(!button_need_release && (msg->buttons[up] || msg->buttons[down] || msg->buttons[enter]))
	{
		button_need_release=true;
	}


	if(menu_entry_list.size()!=0)
	{
		if((*current_entry)->getSelected()) //entry control
		{
			(*current_entry)->scrollwheel(msg->buttons[up] ,msg->buttons[down], msg->buttons[enter]);
		}
		else //menu control
		{
			if(msg->buttons[enter])
			{
				(*current_entry)->setSelected(true);
			}
			else if(msg->buttons[up] && !msg->buttons[down] && (current_entry!=menu_entry_list.begin()) )
			{
				current_entry--;
			}
			else if(msg->buttons[down] && !msg->buttons[up] && (current_entry!=menu_entry_list.end()-1))
			{
				current_entry++;
			}
			update();
			ROS_INFO("%s",(*current_entry)->getEntryName().c_str());
		}

	}
}

void MenuCreator::cb_simple(const x52_joyext::x52_simple_menu_entry &msg)
{

}

void MenuCreator::update()
{
	std::vector<std::string>new_content;

	if(menu_entry_list.size()!=0)
	{
		if((*current_entry)->getSelected())
		{
			for (int l = 0; l < 3; ++l)
			{
				new_content.push_back((*current_entry)->getFullScreenLine(l));
			}
		}
		else
		{
			//Previous Line
			if(current_entry==menu_entry_list.begin())
			{
				ROS_INFO("Current List Begin!");
				new_content.push_back("--List---Start--");
			}
			else
			{
 				new_content.push_back((*(current_entry-1))->getEntryName());
			}

			//Current Line
			new_content.push_back(">"+(*current_entry)->getEntryName());

			//Next line
			if(current_entry==menu_entry_list.end()-1)
			{
				ROS_INFO("Current List End!");
				new_content.push_back("---List---End---");
			}
			else
			{
				new_content.push_back((*(current_entry+1))->getEntryName());
			}
		}
	}
	else
	{
		new_content.push_back(" no entries in  ");
		new_content.push_back("    MFD Menu    ");
		new_content.push_back("                ");
	}

	for (int c = 0;  c < 3; ++ c)
	{
		if(new_content[c].size()>16)
			new_content[c].erase(16,new_content.size()-16);

		if(new_content[c].size()<16)
		{
			for (int f = 16-new_content[c].size(); f > 0 ; --f)
			{
				new_content[c].push_back(' ');
			}
		}

			//Publish current line
			x52_joyext::x52_mfd msg;

			msg.clearDisplay=false;
			msg.data=new_content[c];
			msg.line=c;
			msg.pos=0;

			pub_mfd.publish(msg);

	}
}
