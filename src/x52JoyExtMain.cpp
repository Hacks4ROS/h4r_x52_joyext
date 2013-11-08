/*
 * x52_joyext Node
 * by Christian Holl (www.rad-lab.net)
 * License: BSD
 *
 * Have Fun! :-)
 */
#include <ros/ros.h>
#include "X52JoyExt.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "X52JoyExt");
	ros::NodeHandle n("~");
	X52_JoyExt node(n);
}
