/*
 * x52_joyext Node
 * by Christian Holl (www.rad-lab.net)
 * License: BSD
 *
 * Have Fun! :-)
 */

/*
 * Modified by Murilo FM (muhrix@gmail.com)
 * 12 Dec 2013
 *
 */

#include <ros/ros.h>
#include <h4r_x52_joyext/x52_joyext.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "X52JoyExt");
	ros::NodeHandle n("~");
	X52_JoyExt node(n);
}
