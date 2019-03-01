# ros
find_package(catkin REQUIRED
        cmake_modules
        geometry_msgs
        nav_msgs
        message_generation
        roscpp
        sensor_msgs
        tf
        visualization_msgs
        )

set(ROS_DIR ${catkin_INCLUDE_DIRS})
include_directories(${ROS_DIR})
set(ROS_LIBRARIES ${catkin_LIBRARIES})

#add_message_files(
#        FILES
#        LineSegment.msg
#        LineSegmentList.msg
#)
#
#generate_messages(
#        DEPENDENCIES
#        sensor_msgs
#)