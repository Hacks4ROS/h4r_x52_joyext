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


#include <h4r_x52_joyext/x52_joyext.h>

X52_JoyExt::X52_JoyExt(ros::NodeHandle &n):nh(n)
{
   //bzero(updateLED,10);
   memset(updateLED,1,10);
   updateLED_b=false;
   memset(updateMFD,true,3);

   updateDate=false;
   updateTime=false;
   memset(updateOffset,false,2);
   updateBrightnessMFD=false;
   updateBrightnessLED=false;
   bzero(LED,19);
   //memset(LED,1,19);

   bzero(Date,3);
   bzero(Time,2);
   bzero(Offset,2);
   brightnessMFD=255;
   brightnessLED=255;

   updateBrightnessLED=true;
   updateBrightnessMFD=true;

   //MFD content write
   mfd_content[0]="x52_joyext Node";
   mfd_content[1]=" Joystick under ";
   mfd_content[2]="   ROS Control  ";


   int rate;
   n.param("loop_rate", rate, 100);
   loop_rate=new ros::Rate(rate);

   subleds=nh.subscribe("leds",1000, &X52_JoyExt::cb_leds,this);
   submfd_text=nh.subscribe("mfd_text",1000, &X52_JoyExt::cb_mfd_text,this);
   subdate=nh.subscribe("mfd_date",1000, &X52_JoyExt::cb_date,this);
   subtime=nh.subscribe("mfd_time",1000, &X52_JoyExt::cb_time,this);
   subbrightnessMFD=nh.subscribe("mfd_brightness",1000, &X52_JoyExt::cb_brighnessMFD,this);
   subbrightnessLED=nh.subscribe("leds_brightness",1000, &X52_JoyExt::cb_brighnessLED,this);

   //Move to handling
   send_to_joystick();
}


X52_JoyExt::~X52_JoyExt()
{
	delete loop_rate;
}

void X52_JoyExt::cb_leds(const h4r_x52_joyext::x52_led_colorConstPtr &msg)
{
	if(msg->color_leds[h4r_x52_joyext::x52_led_color::LED_FIRE])
	{
		if( msg->color_leds[ h4r_x52_joyext::x52_led_color::LED_FIRE ] >2)
			ROS_WARN("WRONG VALUE (%i) FOR LED FOUND! Value must be in the range of 0-2",msg->color_leds[h4r_x52_joyext::x52_led_color::LED_FIRE]);
		LED[0]=(msg->color_leds[h4r_x52_joyext::x52_led_color::LED_FIRE]>1);
		updateLED[0]=true;
	}
	//  void setLEDs(uint8_t inValue, uint8_t *red, uint8_t *green, bool *update)
	setLEDs(msg->color_leds[h4r_x52_joyext::x52_led_color::LED_A]	,&LED[X52PRO_LED_ARED-1] ,&LED[X52PRO_LED_AGREEN-1] ,&updateLED[1]);
	setLEDs(msg->color_leds[h4r_x52_joyext::x52_led_color::LED_B]	,&LED[X52PRO_LED_BRED-1] ,&LED[X52PRO_LED_BGREEN-1] ,&updateLED[2]);
	setLEDs(msg->color_leds[h4r_x52_joyext::x52_led_color::LED_D]	,&LED[X52PRO_LED_DRED-1] ,&LED[X52PRO_LED_DGREEN-1] ,&updateLED[3]);
	setLEDs(msg->color_leds[h4r_x52_joyext::x52_led_color::LED_E]	,&LED[X52PRO_LED_ERED-1] ,&LED[X52PRO_LED_EGREEN-1] ,&updateLED[4]);
	setLEDs(msg->color_leds[h4r_x52_joyext::x52_led_color::LED_T12]	,&LED[X52PRO_LED_T1RED-1],&LED[X52PRO_LED_T1GREEN-1],&updateLED[5]);
	setLEDs(msg->color_leds[h4r_x52_joyext::x52_led_color::LED_T34]	,&LED[X52PRO_LED_T2RED-1],&LED[X52PRO_LED_T2GREEN-1],&updateLED[6]);
	setLEDs(msg->color_leds[h4r_x52_joyext::x52_led_color::LED_T56]	,&LED[X52PRO_LED_T3RED-1],&LED[X52PRO_LED_T3GREEN-1],&updateLED[7]);
	setLEDs(msg->color_leds[h4r_x52_joyext::x52_led_color::LED_POV2],&LED[X52PRO_LED_CORED-1],&LED[X52PRO_LED_COGREEN-1],&updateLED[8]);
	setLEDs(msg->color_leds[h4r_x52_joyext::x52_led_color::LED_I]	,&LED[X52PRO_LED_IRED-1] ,&LED[X52PRO_LED_IGREEN-1] ,&updateLED[9]);
}

void  X52_JoyExt::cb_mfd_text(const h4r_x52_joyext::x52_mfdConstPtr &msg)
{
	int curPos=msg->pos+16*msg->line;

	if(msg->clearDisplay)
	{
		mfd_content[0]="                ";
		mfd_content[1]="                ";
		mfd_content[2]="                ";
		memset(updateMFD,true,3);
	}

	for (std::string::const_iterator it=msg->data.begin(); it != msg->data.end(); ++it)
	{
		char curChr=*it;

		if(curPos<3*16)
		{
			uint8_t curLine=curPos/16;
			uint8_t posInLine=curPos-curLine*16;
			if(curChr=='\n')
			{
				//Fill the rest of the line with spaces
				for (curPos = 0; curPos < (curLine+1)*16; ++curPos)
				{
					posInLine=curPos-curLine*16;
					mfd_content[curLine][posInLine]=curChr;
				}
				continue;
			}
			else
			{
				mfd_content[curLine][posInLine]=curChr;
				updateMFD[curLine]=true;
				curPos++;
			}
		}
		else
		{
			ROS_WARN("MFD text exceeds display bounds!");
			break;
		}
	}
	ROS_DEBUG("Line 0: %s", mfd_content[0].c_str());
	ROS_DEBUG("Line 1: %s", mfd_content[1].c_str());
	ROS_DEBUG("Line 2: %s", mfd_content[2].c_str());
}

void  X52_JoyExt::cb_date(const h4r_x52_joyext::x52_dateConstPtr &msg)
{
		Date[2]=msg->date_field_left;
		Date[1]=msg->date_field_center;
		Date[0]=msg->date_field_right;
		updateDate=true;
}

void  X52_JoyExt::cb_time(const h4r_x52_joyext::x52_timeConstPtr &msg)
{
	if(msg->set_time)
	{
		updateTime=true;
		Time_24=msg->time_24;
		Time[0]=msg->time_hours;
		Time[1]=msg->time_minutes;
	}

	if(msg->set_offset_0)
	{
		updateOffset[0]=true;
		Offset_24[0]=msg->offset_0_24;
		Offset_Inv[0]=msg->offset_0_inv;
		Offset[0]=msg->offset_0;
	}

	if(msg->set_offset_1)
	{
		updateOffset[1]=true;
		Offset_24[1]=msg->offset_1_24;
		Offset_Inv[1]=msg->offset_1_inv;
		Offset[1]=msg->offset_1;
	}
}

void  X52_JoyExt::cb_brighnessMFD(const std_msgs::UInt8ConstPtr &msg)
{
		brightnessMFD=msg->data;
		updateBrightnessMFD=true;
}

void  X52_JoyExt::cb_brighnessLED(const std_msgs::UInt8ConstPtr &msg)
{
		brightnessLED=msg->data;
		updateBrightnessLED=true;
}


#define SLEEP_AFTER_COMMAND usleep(5000)

void X52_JoyExt::send_to_joystick()
{
	/*Get Joystick */
	struct x52 *hdl=0;
	while(hdl==0 && ros::ok())
	{
		ROS_INFO("Trying to find joystick...");
		hdl=x52_init();
		loop_rate->sleep();
	}
	ROS_INFO("Joystick found...");
	x52_debug(hdl, 1);


	x52_setled(hdl,0,0);

	while(ros::ok())
	{
		//Update Joystick LED
			for (int lednum = 0; lednum < 10; ++lednum)
			{
				if(updateLED[lednum])
				{
					ROS_DEBUG("Setting LED: %i",lednum);
					if(lednum != 0)
					{
						int led=(lednum)*2;
						x52_setled(hdl,led,LED[led-1]);
						x52_setled(hdl,led+1,LED[led]);
					}
					else
					{
						x52_setled(hdl,1,LED[0]);
					}
					updateLED[lednum]=false;
					usleep(50);
				}
			}

		//Update Brightness MFD
			if(updateBrightnessMFD)
			{
				x52_setbri(hdl,1,brightnessMFD);
				updateBrightnessMFD=false;
				usleep(50);
			}

		//Update Brightness LED

			if(updateBrightnessLED)
			{
				x52_setbri(hdl,0,brightnessLED);
				updateBrightnessLED=false;
				SLEEP_AFTER_COMMAND;
			}

		//Update MFD Text
			for (int line = 0; line < 3; ++line)
			{
				if(updateMFD[line])
				{
					x52_settext(hdl,line,(char*)mfd_content[line].c_str(),mfd_content[line].length());
					updateMFD[line]=false;
				}
				SLEEP_AFTER_COMMAND;
			}

		//Update Date
			if(updateDate)
			{
				x52_setdate(hdl,Date[0],Date[1],Date[2]);
				updateDate=false;
				SLEEP_AFTER_COMMAND;
			}

		//Update  Time
			if(updateTime)
			{
				x52_settime(hdl,Time_24,Time[0], Time[1]);
				updateTime=false;
				SLEEP_AFTER_COMMAND;
			}

		//Offset  Time
			for (int o = 0; o < 2; ++ o)
			{
				if(updateOffset[o])
				{
					x52_setoffs(hdl,o,Offset_24[o],Offset_Inv[o],Offset[o]);
					updateOffset[o]=false;
				}
				SLEEP_AFTER_COMMAND;
			}

		ros::spinOnce();
		loop_rate->sleep();
	}
}
