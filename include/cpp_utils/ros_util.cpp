//
// Created by waxz on 19-1-16.
//

#include "ros_util.h"

namespace ros_util {

    Node::Node() : nh_(), nh_private_("~"), m_get_msg_(false) {

    }

    template<typename T>
    void Node::simpleCbk(const typename T::ConstPtr &msg) {

        ROS_ERROR_STREAM(*msg);
        m_get_msg_ = true;

    }

    template<typename T>
    void Node::bindCbk(const typename T::ConstPtr &msg, std::shared_ptr<T> dst) {

        m_get_msg_ = true;
        *dst = *msg;
    }

    bool Node::getOneMsg(std::string topic_name, double timeout) {

        m_get_msg_ = false;

        if (timeout > 0) {

            m_callbackqueues_[topic_name].get()->callAvailable(ros::WallDuration(timeout));
            if (!m_get_msg_) {
                return false;
            }

        } else if (timeout <= 0.0) {

            ros::Rate r(100);
            while (ros::ok() && !m_get_msg_) {
                m_callbackqueues_[topic_name].get()->callAvailable(ros::WallDuration(0.01));
                if (m_get_msg_) {
                    break;
                }
                r.sleep();
            }

        }

        return true;

    }


    template<typename T>
    std::shared_ptr<T> Node::createSubscriber(std::string topic_name, unsigned int buffer_size,
                                              std::shared_ptr<T> dst) {

        // init
        ros::NodeHandle nh;
        std::shared_ptr<ros::CallbackQueue> q(std::make_shared<ros::CallbackQueue>());
        nh.setCallbackQueue(q.get());


        // set callback
        // basic callback
#if 0
        ros::Subscriber sub = nh.subscribe(topic_name, buffer_size,
                                               &Node::simpleCbk<T>, this,
                                               ros::TransportHints().reliable().maxDatagramSize(10000).tcpNoDelay(true));
#endif
        // boost bind callback
#if 1
        ros::Subscriber sub = nh.subscribe<T>(topic_name, buffer_size,
                                              boost::bind(&Node::bindCbk<T>, this, _1, dst),
                                              ros::VoidPtr(),
                                              ros::TransportHints().reliable().maxDatagramSize(10000).tcpNoDelay(
                                                      true));
#endif
        // final process
        // must keep sub and queue alive
        // CallbackQueue has member [mutex, condition_variable]. That is not possible to copy construct;
        m_subscribers_.push_back(sub);
        m_callbackqueues_[topic_name] = q;

        return dst;
    }


    template<typename T>
    void Node::createMq(sig_util::MsgQueue<T> &m) {
        auto dst = createSubscriber<T>(m.Topic(), m.BuffSize(), m.m_msg_ptr);
        // get first arg from upper call entry
        m.connect(boost::bind(&Node::getOneMsg, this, m.Topic(), _1));

    }


    //
    TfEntry::TfEntry(double rate) : thread_util::BaseEntry(),
                                    m_tfb(std::make_shared<tf::TransformBroadcaster>()),
                                    m_rate(rate) {


    }

    void TfEntry::update(tf::StampedTransform tranform) {

        // get lock
        this->Pause();

        std::unique_lock<std::recursive_mutex> lock(m_mutex);

        m_transform = tranform;
//            lock.unlock();
    }

    void TfEntry::run() {
        std::unique_lock<std::recursive_mutex> lock(m_mutex);
        while (ros::ok() && m_running) {

            m_cv.wait(lock, std::bind(&BaseEntry::isStarted, this));

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

    TfListenner::TfListenner() : m_tfl(std::make_shared<tf::TransformListener>()) {

    }

    bool TfListenner::getTf(std::string fix_frame_id, std::string target_frame_id, tf::Transform &transform,
                            ros::Time cur_time, double timeout) {
        tf::StampedTransform stampedTransform;

        if (timeout > 0.0) {
            try {
//                    m_tfl.get()->waitForTransform(fix_frame_id, target_frame_id, cur_time,ros::Duration(0.02));
                m_tfl.get()->lookupTransform(fix_frame_id, target_frame_id, ros::Time(0), stampedTransform);
                transform = tf::Transform(stampedTransform);
                return true;
            } catch (tf::TransformException e) {
                ROS_ERROR_STREAM(e.what());
                return false;
            }


        } else if (timeout < 0.0) {
            ros::Rate rate(20);
            while (ros::ok()) {
                try {
//                        m_tfl.get()->waitForTransform(fix_frame_id, target_frame_id, cur_time,ros::Duration(0.02));

                    m_tfl.get()->lookupTransform(fix_frame_id, target_frame_id, ros::Time(0), stampedTransform);
                    transform = tf::Transform(stampedTransform);

                    return true;
                } catch (tf::TransformException e) {

                    ROS_ERROR_STREAM(e.what());
                    rate.sleep();
                }
            }
        }
        return true;
    }


    LaserScan &LaserScan::operator=(const sensor_msgs::LaserScan &a) {

        copy(a);
        return *this;

    }

    LaserScan &LaserScan::operator=(const LaserScan &a) {
        copy(a);
        return *this;

    }

    LaserScan &LaserScan::operator=(LaserScan &&a) {
        copy(a);
        return *this;

    }

    LaserScan::LaserScan(const LaserScan &a) {
        copy(a);
    }

    LaserScan::LaserScan(const sensor_msgs::LaserScan &a) {
        copy(a);
    }

    LaserScan::LaserScan(LaserScan &&a) {
        copy(a);
    }

    void LaserScan::cache() {
        if (ranges.size() != cache_angle.size() || cache_angle[0] != angle_min) {
            cache_angle = ValarrayF(0.0, ranges.size());
            for (int i = 0; i < ranges.size(); i++) {
                cache_angle[i] = angle_min + i * angle_increment;
            }
            cache_cos = cos(cache_angle);
            cache_sin = sin(cache_angle);

        }

    }

    void LaserScan::getXsYs(ValarrayF &xs, ValarrayF &ys) {

        cache();
        xs = ranges_val * cache_cos;
        ys = ranges_val * cache_sin;
    }

    Eigen::MatrixXf &LaserScan::getXsYsMatrix() {
        ValarrayF xs;
        ValarrayF ys;
        getXsYs(xs, ys);
        // oringin * pixel-matrix
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> PositionMatrix(2, xs.size());

        Eigen::Map<Eigen::MatrixXf> xm(&(xs[0]), 1, xs.size());
        Eigen::Map<Eigen::MatrixXf> ym(&(ys[0]), 1, ys.size());
//
        PositionMatrix.row(0) << xm;
        PositionMatrix.row(1) << ym;
        m_PositionMatrix = PositionMatrix;
        return m_PositionMatrix;
    }

    LaserScan::ValarrayF &LaserScan::getRangesVal() {
        ranges_val = std::valarray<float>(&(ranges[0]), ranges.size());

        return ranges_val;
    }

    LaserScan::ValarrayF &LaserScan::RangesVal() {

        return ranges_val;
    }

    LaserScan::ValarrayF &LaserScan::getIntensitiesVal() {
        intensities_val = std::valarray<float>(&(intensities[0]), intensities.size());

        return intensities_val;
    }



    //

    GridMap::GridMap() {

    }

    GridMap::GridMap(const nav_msgs::OccupancyGrid &a) {
        data = a.data;
        height = a.info.height;
        width = a.info.width;
        resolution = a.info.resolution;
        origintransformation = eigen_util::TransformationMatrix2d<float>(a.info.origin.position.x,
                                                                         a.info.origin.position.y,
                                                                         tf::getYaw(a.info.origin.orientation));
    }

    GridMap::GridMap(const GridMap &a) {
        data = a.data;
        height = a.height;
        width = a.width;
        resolution = a.resolution;
        origintransformation = a.origintransformation;

    }
}