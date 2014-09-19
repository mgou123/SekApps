cmake_minimum_required(VERSION 2.8.3)
project(sek_drive)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  image_transport
  image_geometry
  cv_bridge
  message_generation
  genmsg
  catkin REQUIRED genmsg actionlib_msgs
)
#add_action_files(DIRECTORY action FILES DoDishes.action)
#generate_messages(DEPENDENCIES actionlib_msgs)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
add_message_files(
    FILES
    ARMarker.msg
)
## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
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
#  INCLUDE_DIRS -std=gnu++0x
#  LIBRARIES sek_drive
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(sek_drive
#   src/${PROJECT_NAME}/sek_drive.cpp
# )
add_library(robodev src/robodev.cpp)
target_link_libraries(robodev ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_library(RoboteqDevice src/RoboteqDevice.cpp)
target_link_libraries(RoboteqDevice ${catkin_LIBRARIES} ${Boost_LIBRARIES})
## Declare a cpp executable
# add_executable(sek_drive_node src/sek_drive_node.cpp)
add_executable(drive_base src/drive_base.cpp)
target_link_libraries(drive_base
  RoboteqDevice
  robodev
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_executable(vel_logger src/vel_logger.cpp)
target_link_libraries(vel_logger
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


add_executable(init_pose src/init_pose.cpp)
target_link_libraries(init_pose
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(record_hok_odom src/record_hok_odom.cpp)
set_target_properties(record_hok_odom PROPERTIES COMPILE_FLAGS  -std=gnu++0x)
target_link_libraries(record_hok_odom
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(stream_merge src/stream_merge.cpp)
set_target_properties(stream_merge PROPERTIES COMPILE_FLAGS  -std=gnu++0x)
target_link_libraries(stream_merge
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(face_detection src/face_detection.cpp)
set_target_properties(face_detection PROPERTIES COMPILE_FLAGS -std=gnu++0x)
target_link_libraries(face_detection
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(hand_rec src/hand_rec.cpp)
set_target_properties(hand_rec PROPERTIES COMPILE_FLAGS -std=gnu++0x)
target_link_libraries(hand_rec
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(marker_detection src/marker_detection.cpp)
set_target_properties(marker_detection PROPERTIES COMPILE_FLAGS -std=gnu++0x)
target_link_libraries(marker_detection
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(align src/align.cpp)
set_target_properties(align PROPERTIES COMPILE_FLAGS -std=gnu++0x)
target_link_libraries(align
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)
#add_executable(robodev src/robodev.cpp)
#add_executable(RoboteqDevice src/RoboteqDevice.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(sek_drive_node sek_drive_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(sek_drive_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS sek_drive sek_drive_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sek_drive.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
