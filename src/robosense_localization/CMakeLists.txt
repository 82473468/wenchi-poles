cmake_minimum_required(VERSION 2.8)
set(PROJECT_NAME robosense_localization)
set(LIB_NAME rs_localization)
project(${PROJECT_NAME})

set(SOURCE_FILES_main
        include/lidar_localization.h
        src/lidar_localization.cpp
        )

set(SOURCE_FILES_lib
        include/extraction.h
        src/extraction.cpp
        include/localization.h
        src/localization.cpp
        include/canbus.h
        include/encoders.h
        )

find_file(Eigen REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        sensor_msgs
        roscpp
        pcl_conversions
        libpointmatcher_ros
        pcl_ros
        cv_bridge
        genmsg
        tf
        rospy
        )

add_message_files(
            DIRECTORY msg
            FILES
            canbus.msg
            encoders.msg
            )

generate_messages(DEPENDENCIES std_msgs)

include_directories( include ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


find_package(libpointmatcher  REQUIRED)
include_directories("${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 CATKIN_DEPENDS  message_runtime libpointmatcher_ros
)

add_library(${LIB_NAME} SHARED ${SOURCE_FILES_lib})
target_link_libraries(${LIB_NAME}
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(${PROJECT_NAME}
        ${SOURCE_FILES_main}
        ${SOURCE_FILES_lib}
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${libpointmatcher_LIBRARIES}
        ${PCL_LIBRARIES}
        rs_localization
        )

