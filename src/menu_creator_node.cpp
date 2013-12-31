/*
 *   menu_creation_node.cpp
 *   by Christian Holl (www.rad-lab.net)
 * 	 License: BSD
 *
 *   Have Fun! :-)
 */

#include <ros/ros.h>
#include "MenuCreator.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "X52MenuCreator");
	ros::NodeHandle n("~");
	MenuCreator node(n);
	node.run();
}
