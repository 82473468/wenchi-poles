cmake_minimum_required(VERSION 2.8.3)
project(sensors)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_file(Eigen REQUIRED COMPONENTS)

set(${PROJECT_NAME}_CATKIN_DEPS
        pcl_ros
        roscpp
        roslib
        sensor_msgs
        std_msgs
        genmsg
        tf
        dynamic_reconfigure
        diagnostic_updater
        )


find_package(catkin REQUIRED COMPONENTS
        ${${PROJECT_NAME}_CATKIN_DEPS}
        pcl_conversions
        pcl_ros
        roscpp
        rospy
        std_msgs
        genmsg
        cv_bridge
        message_generation
        )
## Generate messages in the 'msg' folder
add_message_files(
        FILES
        canbus.msg
        gprmc.msg
        gpgga.msg
        rtcm.msg
        gpvtg.msg
        encoders.msg)

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES std_msgs  # Or other packages containing msgs
        )
catkin_package(
        CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
        DEPENDS yaml-cpp python-yaml
        DEPENDENCIES
        CATKIN_DEPENDS message_runtime std_msgs
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
        #  LIBRARIES wheelspeed
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include/sensors
        include
        ${catkin_INCLUDE_DIRS}
)
find_library(SERIAL  serial)

set (SENSOR  src/sensor.cpp include/sensors/sensor.h)



## Declare a C++ library
#add_library(${PROJECT_NAME}   src/wheelspeed_node.cpp ${SERIAL} )

add_executable(gps  src/gps.cpp ${SENSOR} ${SERIAL})
add_dependencies(gps sensors_generate_messages_cpp)
target_link_libraries(gps ${catkin_LIBRARIES} ${SERIAL} )

add_executable(encoder src/encoder.cpp ${SENSOR} ${SERIAL} )
add_dependencies(encoder sensors_generate_messages_cpp)
target_link_libraries(encoder ${catkin_LIBRARIES} ${SERIAL} )

add_executable(nav440 src/nav440.cpp src/memsic_type.cpp include/sensors/memsic_type.h ${SENSOR} ${SERIAL} )
add_dependencies(nav440 sensors_generate_messages_cpp)
target_link_libraries(nav440 ${catkin_LIBRARIES} ${SERIAL} )


add_executable(canbus src/canbus.cpp ${SENSOR} ${SERIAL} )
add_dependencies(canbus sensors_generate_messages_cpp)
target_link_libraries(canbus ${catkin_LIBRARIES} ${SERIAL} )

add_executable(findm src/FindM.cpp  include/sensors/qxwz_rtcm.h ${SENSOR} ${SERIAL} )
target_link_libraries(findm ${catkin_LIBRARIES} ${SERIAL}  ${CMAKE_CURRENT_SOURCE_DIR}/src/librtcm.so )


add_executable(visualization_GPS src/visualize_GPS_path.cpp)
target_link_libraries(visualization_GPS ${catkin_LIBRARIES})

# install runtime files
install(TARGETS gps
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION lib
        )
# install runtime files
install(TARGETS encoder
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION lib
        )

# install runtime files
install(TARGETS nav440
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION lib
        )
# install runtime files
install(TARGETS canbus
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION lib
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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
#catkin_add_gtest(${PROJECT_NAME}-test test/test_wheelspeed.cpp)
#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
#endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
