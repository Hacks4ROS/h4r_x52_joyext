x52_joyext
===========================================
ROS package for Ubuntu + Groovy/Hydro to access the MFD  of the Saitek X52 Pro Joystick.



Installation
-------------------------------------------
This package was developed and tested for Ubuntu 12.04 and ROS Groovy/Hydro.

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

    git clone https://github.com/cyborg-x1/x52_joyext.git

Compile your catkin workspace:

    cd ~/catkin_ws
    catkin_make



Documentation
-------------------------------------------
For doxygen documentation, please see:
http://cyborg-x1.github.io/x52_joyext/doc/doxygen/index.html

This package is also documented at:
http://wiki.ros.org/x52_joyext
