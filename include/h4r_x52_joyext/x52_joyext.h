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

/**
\page x52_joyext_node x52_joyext_node

The x52_joyext_node is responsible for the communication with the joystick extensions.
It sets the LEDs and the MFD according to the information gathered from the input topics.

You can use the following piece of code for your launchfile to start the node

	<node pkg="x52_joyext" type="x52_joyext_node" name="x52" output="screen" />
*/

#include <ros/ros.h>

#include <std_msgs/UInt8.h>
#include <boost/thread/mutex.hpp>
#include <string>
#include <inttypes.h>
#include "h4r_x52_joyext/x52_date.h"
#include "h4r_x52_joyext/x52_time.h"
#include "h4r_x52_joyext/x52_led_color.h"
#include "h4r_x52_joyext/x52_mfd.h"

extern "C" {
#include <x52pro.h>
}

#ifndef X52JOYEXT_HPP_
#define X52JOYEXT_HPP_

class X52_JoyExt
{
	  ros::NodeHandle &nh;
	  ros::Rate *loop_rate;


	  bool updateLED[10];
	  bool updateLED_b;
	  bool updateMFD[3];
	  bool updateMFD_b;
	  bool updateDate;
	  bool updateTime;
	  bool updateOffset[2];
	  bool updateBrightnessMFD;
	  bool updateBrightnessLED;


	  uint8_t LED[19];
	  std::string mfd_content[3];
	  uint8_t Date[3];
	  uint8_t Time_24;
	  uint8_t Time[2];
	  uint8_t Offset[2];
	  uint8_t Offset_24[2];
	  uint8_t Offset_Inv[2];
	  uint8_t brightnessMFD;
	  uint8_t brightnessLED;


	  ros::Subscriber subleds;
	  ros::Subscriber submfd_text;
	  ros::Subscriber subdate;
	  ros::Subscriber subtime;
	  ros::Subscriber subbrightnessMFD;
	  ros::Subscriber subbrightnessLED;

	  void setLEDs(uint8_t inValue, uint8_t *red, uint8_t *green, bool *update)
	  {
		  switch(inValue)
		  {
		  case h4r_x52_joyext::x52_led_color::NO_STATUS_CHANGE:
			  *update=false;
			  break;
		  case h4r_x52_joyext::x52_led_color::OFF:
			  *red=0;
			  *green=0;
			  break;
		  case h4r_x52_joyext::x52_led_color::RED:
			  *red=1;
			  *green=0;
			  break;
		  case h4r_x52_joyext::x52_led_color::GREEN:
			  *red=0;
			  *green=1;
		  	  break;
		  case h4r_x52_joyext::x52_led_color::YELLOW:
			  *red=1;
			  *green=1;
		  	  break;
		  default:
			  ROS_WARN("WRONG VALUE (%i) FOR LED FOUND! Value must be in the range of 0-4",inValue);
			  return;
			  break;
		  }
		  *update=true;
	  }

	void cb_leds(const h4r_x52_joyext::x52_led_colorConstPtr &msg);
	void cb_mfd_text(const h4r_x52_joyext::x52_mfdConstPtr &msg);
	void cb_date(const h4r_x52_joyext::x52_dateConstPtr &msg);
	void cb_time(const h4r_x52_joyext::x52_timeConstPtr &msg);
	void cb_brighnessMFD(const std_msgs::UInt8ConstPtr &msg);
	void cb_brighnessLED(const std_msgs::UInt8ConstPtr &msg);


public:
	X52_JoyExt(ros::NodeHandle &n);
	virtual ~X52_JoyExt();

	void send_to_joystick();
};

#endif /* X52JOYEXT_HPP_ */
