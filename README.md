h4r_x52_joyext
===========================================
ROS package to access the MFD  of the Saitek X52 Pro Joystick.


Installation
-------------------------------------------
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

    git clone https://github.com/Hacks4ROS/h4r_x52_joyext

Compile your catkin workspace:

    cd ~/catkin_ws
    catkin_make

Documentation
-------------------------------------------
For doxygen documentation, please see:
http://hacks4ros.github.io/h4r_x52_joyext/doc/doxygen/index.html

This package is also documented at:
http://wiki.ros.org/h4r_x52_joyext
