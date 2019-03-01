//
// Created by waxz on 18-11-21.
//

#ifndef DEMO_ROS_UTIL_H
#define DEMO_ROS_UTIL_H

#include <ros/ros.h>                // node handler
#include <ros/callback_queue.h>     //  ros::CallbackQueue queue
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>    // tf listenner
#include <tf/transform_broadcaster.h> // tf broadcast

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Eigen>
#include <thread>                   // thread
#include <memory>                   // shared_ptr
#include <functional>               // bind
#include <mutex>                    // mutex, lock
#include <condition_variable>       // condition_variable_any
#include <chrono>                   // time and sleep
#include <map>                      // map
#include <uchar.h>
#include <stdint.h>
#include <vector>
#include <valarray>
#include "signal_util.h"            // msg queue
#include "thread_util.h"  // thread
#include "eigen_util.h"
#include "ublas.h"

namespace ros_util {

    class Node {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        //call callback_queue given specific topic
        // there must be a callback queue for a node, or it will crash
        // a thread run in background to fetch message, when it find no callbackqueue to fill in data, crash!!
        std::map<std::string, std::shared_ptr<ros::CallbackQueue>> m_callbackqueues_;
        std::map<std::string, ros::CallbackQueue &> test_queue;

        std::vector<ros::Subscriber> m_subscribers_;

        bool m_get_msg_;


        template<typename T>
        void simpleCbk(const typename T::ConstPtr &msg);

        template<typename T>
        void bindCbk(const typename T::ConstPtr &msg, std::shared_ptr<T> dst);

    public:
        Node();


        // get msg with timeout
        bool getOneMsg(std::string topic_name, double timeout = 0.01);

        template<typename T>
        std::shared_ptr<T> createSubscriber(std::string topic_name, unsigned int buffer_size,
                                            std::shared_ptr<T> dst = std::make_shared<T>());


        template<typename T>
        void createMq(sig_util::MsgQueue<T> &m);
    };


    // thread publishing tf
    class TfEntry : public thread_util::BaseEntry {
    private:
        std::shared_ptr<tf::TransformBroadcaster> m_tfb;
        tf::StampedTransform m_transform;
        double m_rate;
    public:
        TfEntry(double rate);

        void update(tf::StampedTransform tranform);

        void run();
    };

    // tfentry sig

    template<typename T>
    class TfEntrySig : public thread_util::EntrySig<T> {
    private:
        std::shared_ptr<tf::TransformBroadcaster> m_tfb;
        tf::StampedTransform m_transform;
        double m_rate;


    public:
        TfEntrySig(double rate, T &sig) : thread_util::EntrySig<T>(sig),
                                          m_tfb(std::make_shared<tf::TransformBroadcaster>()),
                                          m_rate(rate) {


        }

        void update(tf::StampedTransform tranform) {

            // get lock
            this->Pause();

            std::unique_lock<std::recursive_mutex> lock(this->m_mutex);

            m_transform = tranform;
            this->Start();
//            lock.unlock();
        }

        void run() {
            std::unique_lock<std::recursive_mutex> lock(this->m_mutex);
            while (ros::ok() && this->isRun()) {

                this->m_cv.wait(lock, std::bind(&TfEntrySig::isStarted, this));

                ros::Duration transform_tolerance;
                transform_tolerance.fromSec(0.1);

                ros::Time tn = ros::Time::now();
                ros::Time transform_expiration = (tn +
                                                  transform_tolerance);

                tf::StampedTransform transformstamped(m_transform,
                                                      transform_expiration,
                                                      m_transform.frame_id_, m_transform.child_frame_id_);


                m_tfb.get()->sendTransform(transformstamped);

                std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 / m_rate)));

            }

        }


    };

    // tf listenner
    class TfListenner {
    private:
        std::shared_ptr<tf::TransformListener> m_tfl;
        tf::StampedTransform m_transform;

    public:
        TfListenner();

        bool getTf(std::string fix_frame_id, std::string target_frame_id, tf::Transform &transform,
                   ros::Time cur_time, double timeout);
    };


    // base type
    class LaserScan {
    public:
        typedef std::vector<float> VectorF;
        typedef std::valarray<float> ValarrayF;

    public:
        VectorF ranges;
        VectorF intensities;
        ValarrayF ranges_val;
        ValarrayF intensities_val;

        ValarrayF cache_angle;
        ValarrayF cache_xs;
        ValarrayF cache_ys;
        float angle_min;
        float angle_max;
        float range_min;
        float range_max;
        float angle_increment;
        std::string frame_id;
        Eigen::MatrixXf m_PositionMatrix;
        ublas::matrix<float> m_ublasMatrix;

//        uint64_t stamp_nsec;
        template<typename T>
        void copy(T a) {
            ranges = a.ranges;
            angle_min = a.angle_min;
            angle_max = a.angle_max;
            intensities = a.intensities;
            range_min = a.range_min;
            range_max = a.range_max;
            angle_increment = a.angle_increment;
            getRangesVal();

//            frame_id = a.header.frame_id;
//            stamp_nsec = a.header.stamp.toNSec();
        }

    public:
        ValarrayF cache_cos;
        ValarrayF cache_sin;

    public:
        LaserScan() {};     // copy constructor

        LaserScan(const LaserScan &a);     // copy constructor
        LaserScan &operator=(const LaserScan &a);     // copy assignment

        LaserScan(const sensor_msgs::LaserScan &a);     // copy constructor
        LaserScan &operator=(const sensor_msgs::LaserScan &a);     // copy assignment

        LaserScan(LaserScan &&a);     // move constructor
        LaserScan &operator=(LaserScan &&a);     // move assignment

        void cache();

        ValarrayF &getRangesVal();

        ValarrayF &RangesVal();

        void getXsYs(ValarrayF &xs, ValarrayF &ys);

        Eigen::MatrixXf &getXsYsMatrix();

        ublas::matrix<float> &getXsYsMatrix_ublas();

        ValarrayF &getIntensitiesVal();

#if 0
        // delete function
        LaserScan(const LaserScan& a) = delete;     // copy constructor
        LaserScan& operator=(const LaserScan& a) = delete;     // copy assignment

#endif

    };


    class GridMap {
    public:
        typedef unsigned char uchar;
//        enum GridType{
//            UCHAR = 0,
//            FLOAT
//        };
    private:
        std::vector<signed char> data;
        size_t height;
        size_t width;
        double resolution;

        eigen_util::TransformationMatrix2d<float> origintransformation;


        uint8_t rr;

    public:
        GridMap();

        GridMap(const nav_msgs::OccupancyGrid &a);

        GridMap(const GridMap &a);

        GridMap &operator()(const nav_msgs::OccupancyGrid &a);

        GridMap &operator()(const GridMap &a);


    };
}


#endif //DEMO_ROS_UTIL_H
