x52_joyext
===========================================
ROS package for Ubuntu + Hydro to access the MFD  of the Saitek X52 Pro Joystick.
Written by Christian Holl(see https://github.com/cyborg-x1/x52_joyext), 
maintained by Murilo F. M. (see https://github.com/muhrix/x52_joyext/)



Installation
-------------------------------------------
This package was developed and tested for Ubuntu 12.04 and ROS Hydro.

### Install system dependencies ###
    sudo apt-get install libx52pro0 libx52pro-dev

Also make sure you have git installed:
    sudo apt-get install git-core

### Download source ###

Make sure you have a working catkin workspace, as described at:
http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace

Change directory to the source folder of your catkin workspace.
If, for instance, your workspace is `~/catkin_ws`, make sure there is
a `src/` folder within it, then execute:

    cd ~/catkin_ws/src

Download the package source from the github repository:

    git clone -b hydro-devel https://github.com/muhrix/x52_joyext.git

Compile your catkin workspace:

    cd ~/catkin_ws
    catkin_make
