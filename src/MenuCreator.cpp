/*
 * MenuCreator.cpp
 * by Christian Holl (www.rad-lab.net)
 * License: BSD
 *
 * Have Fun! :-)
 */
#include "MenuCreator.hpp"

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

#define casem(CASE,TYPE,TYPEROS)\
	case CASE:\
	{\
		MenuEntry<TYPE, std_msgs::TYPEROS, std_msgs:: TYPEROS## ConstPtr>()>a;\
		break;\
	}\



MenuCreator::MenuCreator(ros::NodeHandle &n)
{
	std::string setup;
	// TODO Auto-generated constructor stub
	n.param<std::string>("menu_setup",setup,"");

	std::vector<std::string> menu_entries;
	boost::split(menu_entries, setup,
			boost::is_any_of("|"));

	int num=0; //Counting statement number
	for (std::vector<std::string>::iterator entry= menu_entries.begin(); entry != menu_entries.end(); entry++)
	{
		std::vector<std::string> entrySetup;
		boost::split(entrySetup, *entry,
				boost::is_any_of(";"));

		//Name
		if(entrySetup[0].size()==0)
		{
			ROS_ERROR("ERROR entry number %i: Name is zero length! -> not created!", num);
			continue;
		}

		std::istringstream et(entrySetup[1]);
		unsigned int EntryType;
		et>>EntryType;
		if(et.bad() || et.fail() || !et.eof()
		   || !(EntryType==MenuEntryAbstraction::Pub
				   || EntryType==MenuEntryAbstraction::Sub
				   || EntryType==MenuEntryAbstraction::PubSub
				   || EntryType == MenuEntryAbstraction::Simple)
				   )
		{
			ROS_ERROR("ERROR entry type for entry %i: %s  -> Entry not created", num ,entrySetup[1].c_str());
			continue;
		}

std::string topic;
bool latch;


	int type=0;	//!< The input type
	n.param<int>("input_type", type, INPUT_FLOAT64);
	switch (type)
	{
	case INPUT_FLOAT64:\
		{\
			MenuEntry<double_t, std_msgs::Float64, std_msgs::Float64ConstPtr> a(n,"narf",0,"narf",1);
			break;\
		}\



//	casem(INPUT_FLOAT64, double_t, Float64)
//	casem(INPUT_FLOAT32, float_t, Float32)
//	casem(INPUT_INT64, int64_t, Int64)
//	casem(INPUT_INT32, int32_t, Int32)
//	casem(INPUT_INT16, int16_t, Int16)
//	casem(INPUT_INT8, int8_t, Int8)
//	casem(INPUT_UINT64, uint64_t, UInt64)
//	casem(INPUT_UINT32, uint32_t, UInt32)
//	casem(INPUT_UINT16, uint16_t, UInt16)
//	casem(INPUT_UINT8, uint8_t, UInt8)
//	casem(INPUT_BOOL, bool, Bool)
	default:
		break;

	}



		num++;
	}
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
	while(ros::ok())
	{

	}
}
