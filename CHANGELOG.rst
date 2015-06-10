^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package h4r_x52_joyext
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2015-06-10)
------------------
* change doxyfile for logo
* exchange logo
* change css
* fix doxygen source path to relative one
* fix doxygen project name
* css style changes
* change color style hue to red
* add installation for udev and launch files
* add information about udev rules file
* fit readme and package.xml to the new repository
* fix node name
* resurrect udev rule
* remove unnecessary, non-working stuff
* change project name
  move include to subfolder
  add new files to
* add exit for worng setup string (stringprinting)
* Added tags for bugtracker and repo
* Merge pull request `#4 <https://github.com/Hacks4ROS/x52_joyext/issues/4>`_ from muhrix/master
  Updates to package documentation
* Added url tag
* Updated info using rosdoc_lite
* Corrected XML tag
* Merge remote-tracking branch 'origin/hydro-devel'
* Updated package info
* Updated maintainer/author info
* Merge branch 'hotfix/V1.0.1_b' into dev
* Merge branch 'hotfix/V1.0.1_b'
  Fixed wrong refactoring
  Fixed jitter bug with longer sleep after an command (further testing required)
* remove bug notice
* fix two problems, (maybe) including jitter bug
  fix broken refactoring of PublishObject
  add higher waiting time after usb write inside joyext node
* Merge branch 'release/V1.0.0_b' into dev
* Merge branch 'release/V1.0.0_b'
* move x52_joyext_node page to header file
* remove original images
* Merge branch 'feature/doc' into dev
* add documentation
  for value2buttonColor
  for mfd_writer
  for main node
  update doxyfile
  change .gitignore: new doxygen generation directory
* add missing image to img
* add C&P launch code
* rename classes PublishObject inside mfd_writer and value2buttoncolor
* add documentation for mfd_writer
  add int types for 64 bits for color2button and mfd_writer
* add general joystick description
  add several pictures
* Merge branch 'dev' into feature/doc
* update led topic
  now fire button is also an array member of color_led
* Merge branch 'dev' into feature/doc
* Merge branch 'master' into dev
* Merge branch 'hotfix/update_problem'
* add sleep for 50ms after any update to the joystick
  (seems to) prevent flickering Joystick LEDs when there is a high update
  rate
* add svg images for buttons
  move png images to originalimg folder
* add first documentation stuff and pictures
  add style stuff
* Merge branch 'feature/mfd_writer' into dev
* add mfd_writer examples
  add multiple launchfiles for each example
  add one main example launchfile
* added value printing
  add overflow printing
  needs testing
* stringprint working
* added mfd_writer node (not working, yet)
* add setup values
* Start writing mfd writer node
* add comment
* add maintainer and author, better for maintaining it...
* add comment to launchfile
* Add support for joystick buttons in led color
* Updated README.md file
* Files have been renamed
* Header file has been renamed and moved to include/
* Removed LICENSE (only present because x52_pro_lib was packaged with project before)
* Removed udev rule definition (now set up when system-wide libx52pro is installed)
* Removed x52_pro_lib from project (now installed system-wide)
* Removed unnecessary gitignore file
* Overhauled package build specifications
* Updated build and run dependencies
* Moved header file to include/ and added system-wide (installed) x52pro.h
* Including header file within package only
* Including header package within package only
* Change INFO to DEBUG message
* credits to lib dev
* Readme hint for udev rule file
* fix problems with system utilization, fix led debug messages
* Added description to package.xml
* Merge branch 'master' of github.com:cyborg-x1/x52_joyext
* initial commit, most stuff is working, problems with system utilization
* Initial commit
* Contributors: Christian Holl, Murilo FM
