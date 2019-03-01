//
// Created by waxz on 19-1-16.
//

#include "ros_util.cpp"

namespace ros_util {



    template std::shared_ptr<sensor_msgs::LaserScan>
    Node::createSubscriber(std::string topic_name, unsigned int buffer_size,
                           std::shared_ptr<sensor_msgs::LaserScan> dst);

    //nav_msgs::OccupancyGrid
    template std::shared_ptr<nav_msgs::OccupancyGrid>
    Node::createSubscriber(std::string topic_name, unsigned int buffer_size,
                           std::shared_ptr<nav_msgs::OccupancyGrid> dst);

    //geometry_msgs::PoseWithCovarianceStamped
    template std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped>
    Node::createSubscriber(std::string topic_name, unsigned int buffer_size,
                           std::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> dst);


}
