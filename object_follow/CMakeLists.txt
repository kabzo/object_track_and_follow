cmake_minimum_required(VERSION 2.8.3)
project(object_follow)

SET(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  sensor_msgs
  tld_msgs
  dynamic_reconfigure
  message_generation
  image_transport

)
#catkin_package(CATKIN_DEPENDS message_runtime std_msgs ardrone_autonomy)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED COMPONENTS
        QtCore
        QtGui)


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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   controller_cmd.srv
 )

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

#dynamic reconfigure
 generate_dynamic_reconfigure_options( cfg/pidParam.cfg)

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
#  INCLUDE_DIRS include
#  LIBRARIES object_follow
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########


# ---------------------------- OBJECT TRACKING GUI --------------------------------------------------
# set header ans source files
set(TRACKING_GUI_SOURCE_FILES
        src/gui/object_track/object_follow_gui.cpp
        src/gui/object_track/baseframegraphicsview.cpp
)

set(TRACKING_GUI_HEADER_FILES
        src/gui/object_track/object_follow_gui.h
        src/gui/object_track/baseframegraphicsview.h
 )

# *.ui
set(TRACKING_GUI_UI_FILES
        src/gui/object_track/object_follow_gui.ui
)

# *.qrc
set(TRACKING_GUI_RESOURCE_FILES
)

# do QT stuff
ADD_DEFINITIONS( -Wall )

include(${QT_USE_FILE})

QT4_ADD_RESOURCES(TRACKING_GUI_RESOURCE_FILES_CPP ${TRACKING_GUI_RESOURCE_FILES})
QT4_WRAP_UI(TRACKING_GUI_UI_FILES_HPP ${TRACKING_GUI_UI_FILES})
QT4_WRAP_CPP(TRACKING_GUI_HEADER_FILES_HPP ${TRACKING_GUI_HEADER_FILES})

include_directories(${CMAKE_CURRENT_BINARY_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/src/gui/object_track)

add_executable(object_track_widget
            src/gui/object_track/main_gui.cpp
            ${TRACKING_GUI_SOURCE_FILES}
            ${TRACKING_GUI_RESOURCE_FILES_CPP}
            ${TRACKING_GUI_UI_FILES_HPP}
            ${TRACKING_GUI_HEADER_FILES_HPP})
target_link_libraries(object_track_widget
                ${QT_LIBRARIES}
                ${catkin_LIBRARIES})
add_dependencies(object_track_widget tld_msgs_generate_messages_cpp)

# ---------------------------- OBJECT TRACKING GUI --------------------------------------------------

# ---------------------------- DRONE CONTROL GUI --------------------------------------------------
# set header ans source files
set(CONTROL_GUI_SOURCE_FILES
        src/gui/drone_control/drone_control.cpp
)

set(CONTROL_GUI_HEADER_FILES
        src/gui/drone_control/drone_control.h
 )

# *.ui
set(CONTROL_GUI_UI_FILES
        src/gui/drone_control/drone_control.ui
)

# *.qrc
set(CONTROL_GUI_RESOURCE_FILES
)

# do QT stuff
QT4_ADD_RESOURCES(CONTROL_GUI_RESOURCE_FILES_CPP ${CONTROL_GUI_RESOURCE_FILES})
QT4_WRAP_UI(CONTROL_GUI_UI_FILES_HPP ${CONTROL_GUI_UI_FILES})
QT4_WRAP_CPP(CONTROL_GUI_HEADER_FILES_HPP ${CONTROL_GUI_HEADER_FILES})

include_directories(${PROJECT_SOURCE_DIR}/src/gui/drone_control)

add_executable(drone_control_widget
            src/gui/drone_control/main_drone_control_node.cpp
            ${CONTROL_GUI_SOURCE_FILES}
            ${CONTROL_GUI_RESOURCE_FILES_CPP}
            ${CONTROL_GUI_UI_FILES_HPP}
            ${CONTROL_GUI_HEADER_FILES_HPP}
)

target_link_libraries(drone_control_widget
                ${QT_LIBRARIES}
                ${catkin_LIBRARIES})


add_dependencies(drone_control_widget tld_msgs_generate_messages_cpp)

# ---------------------------- DRONE CONTROL GUI --------------------------------------------------
# ---------------------------- MAIN GUI --------------------------------------------------

# set header ans source files
set(MAIN_GUI_SOURCE_FILES
        src/gui/main_gui_node.cpp
        src/gui/mainwindow.cpp
        src/gui/dronecontrolgraphicsview.cpp
)

set(MAIN_GUI_HEADER_FILES
        src/gui/mainwindow.h
        src/gui/dronecontrolgraphicsview.h

 )

# *.ui
set(MAIN_GUI_UI_FILES
        src/gui/mainwindow.ui
)
# *.qrc
set(MAIN_GUI_RESOURCE_FILES
)

QT4_ADD_RESOURCES(MAIN_GUI_RESOURCE_FILES_CPP ${MAIN_GUI_RESOURCE_FILES})
QT4_WRAP_UI(MAIN_GUI_UI_FILES_HPP ${MAIN_GUI_UI_FILES})
QT4_WRAP_CPP(MAIN_GUI_HEADER_FILES_HPP ${MAIN_GUI_HEADER_FILES})

include_directories(${PROJECT_SOURCE_DIR}/src/gui)

add_executable(gui_tracking_control
            ${TRACKING_GUI_SOURCE_FILES}
            ${TRACKING_GUI_RESOURCE_FILES_CPP}
            ${TRACKING_GUI_UI_FILES_HPP}
            ${TRACKING_GUI_HEADER_FILES_HPP}
            ${CONTROL_GUI_SOURCE_FILES}
            ${CONTROL_GUI_RESOURCE_FILES_CPP}
            ${CONTROL_GUI_UI_FILES_HPP}
            ${CONTROL_GUI_HEADER_FILES_HPP}
            ${MAIN_GUI_SOURCE_FILES}
            ${MAIN_GUI_RESOURCE_FILES_CPP}
            ${MAIN_GUI_UI_FILES_HPP}
            ${MAIN_GUI_HEADER_FILES_HPP}
            )

target_link_libraries(gui_tracking_control
                ${QT_LIBRARIES}
                ${catkin_LIBRARIES})


add_dependencies(gui_tracking_control tld_msgs_generate_messages_cpp)

# ---------------------------- MAIN GUI --------------------------------------------------


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
 ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(libFollowControll
  src/FollowControll.cpp
  src/FollowControll.h
)
set_target_properties(libFollowControll PROPERTIES COMPILE_FLAGS "-std=c++0x")
target_link_libraries(libFollowControll
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
)

## Declare a cpp library
add_library(libController
  src/Controller.cpp
  src/Controller.h
)
set_target_properties(libController PROPERTIES COMPILE_FLAGS "-std=c++0x")
target_link_libraries(libController
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
)
## Declare a cpp executable
add_executable(object_follow_controller src/main_object_follow_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(object_follow_controller libFollowControll libController ${PROJECT_NAME}_gencfg)



## Specify libraries to link a library or executable target against
 target_link_libraries(object_follow_controller
   ${OpenCV_LIBS}
   libFollowControll
   libController
   ${catkin_LIBRARIES}

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
# install(TARGETS object_follow object_follow_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_object_follow.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
