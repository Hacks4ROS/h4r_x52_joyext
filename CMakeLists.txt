cmake_minimum_required(VERSION 2.8.3)
project(h4r_x52_joyext)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_library(LIBX52 libx52pro.so /usr/lib/libx52pro.so)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)




#find_package(PkgConfig REQUIRED)
#pkg_search_module(LIBX52PRO_DEV REQUIRED libx52pro-dev)
#include_directories(SYSTEM ${LIBX52PRO_INCLUDE_DIRS})
#find_package(libx52pro-dev)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   x52_date.msg  
   x52_led_color.msg  
   x52_mfd.msg  
   x52_time.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES x52_joyext
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS libx52pro-dev
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LIBX52PRO_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(x52_joyext src/x52_joyext.cpp)

## Declare a cpp executable
add_executable(x52_joyext_node src/x52_joyext_node.cpp)
add_executable(x52_value2buttonColor_node src/value2buttonColor.cpp)
add_executable(x52_mfd_writer_node src/mfd_writer.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(x52_joyext x52_joyext_generate_messages_cpp)
add_dependencies(x52_joyext_node x52_joyext_generate_messages_cpp)
add_dependencies(x52_value2buttonColor_node x52_joyext_generate_messages_cpp)
add_dependencies(x52_mfd_writer_node x52_joyext_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(x52_joyext
   ${LIBX52}
   ${catkin_LIBRARIES}
   ${LIBX52PRO_LIBRARIES}
)

target_link_libraries(x52_joyext_node
   x52_joyext
   ${LIBX52}
   ${catkin_LIBRARIES}
   ${LIBX52PRO_LIBRARIES}
)
 
target_link_libraries(x52_value2buttonColor_node
   ${LIBX52}
   ${catkin_LIBRARIES}
   ${LIBX52PRO_LIBRARIES}
)

target_link_libraries(x52_mfd_writer_node
   ${LIBX52}
   ${catkin_LIBRARIES}
   ${LIBX52PRO_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS
   x52_joyext
   x52_joyext_node
   x52_value2buttonColor_node
   x52_mfd_writer_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(DIRECTORY launch udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_x52_joyext.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
