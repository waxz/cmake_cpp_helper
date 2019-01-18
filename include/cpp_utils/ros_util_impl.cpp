//
// Created by waxz on 19-1-16.
//

#include "ros_util.cpp"

namespace ros_util {


    template void Node::simpleCbk<sensor_msgs::LaserScan>(const sensor_msgs::LaserScan::ConstPtr &msg);

    template std::shared_ptr<sensor_msgs::LaserScan>
    Node::createSubscriber(std::string topic_name, unsigned int buffer_size,
                           std::shared_ptr<sensor_msgs::LaserScan> dst);

}
