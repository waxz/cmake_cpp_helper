#include <cpp_utils/qt_util.h>
#include <cpp_utils/ros_util.h>
#include <cpp_utils/eigen_util.h>

#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <random>
#include "backend.h"

#include <HMM/Hmm.h>

#include <boost/multiprecision/gmp.hpp>  // Defines the wrappers around the GMP library's types
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

namespace mp = boost::multiprecision;

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

template<typename T>
T clip(const T &n, const T &lower, const T &upper) {
    return std::max(lower, std::min(n, upper));
}


/*
 * ===============
 *     grid map
 *
 *     data:
 *         store cv::Mat
 *         origin transform
 *
 *     member function:
 *         index
 *         position transform
 *
 *
 *
 * */
class GridMap {
public:
    enum OriginType {
        LeftUp = 0,
        LeftBottom
    };

private:
    cv::Mat &m_grid;
    bool isOriginInit;
    int m_height;
    int m_width;
    double m_grid_resolution;

    OriginType originType;
    eigen_util::TransformationMatrix2d<float> LeftUpOrigin;
    /*
     * o----- y
     * |
     * |
     * x
     *
     * */

    /* cv::Mat index
     * o----- x
     * |
     * |
     * y
     *
     * */



public:
    GridMap(cv::Mat grid_, double grid_resolution_) :
            m_grid(grid_),
            isOriginInit(false),
            m_height(m_grid.rows),
            m_width(m_grid.cols), m_grid_resolution(grid_resolution_),
            originType() {


    }

    cv::Mat &grid() {
        return m_grid;
    }

    void setOrigin(double x_, double y_, double yaw_, OriginType type_) {

        eigen_util::TransformationMatrix2d<float> origin(x_, y_, yaw_);
        if (type_ == OriginType::LeftBottom) {
            eigen_util::TransformationMatrix2d<float> bottomToUp(0.0, m_height / m_grid_resolution, -0.5 * M_PI);
            LeftUpOrigin = origin * bottomToUp;

        }

        if (type_ == OriginType::LeftUp) {
            LeftUpOrigin = origin;

        }

        isOriginInit = true;
    }

    template<typename T>
    T &atPixel(int px, int py) {

        if (px < 0 || px > m_height || py < 0 || py > m_width) {
            throw std::out_of_range("px < 0 || px > m_height || py < 0 || py > m_width");
        }
        px = clip(px, 0, m_height);
        py = clip(py, 0, m_width);
        return m_grid.at<T>(px, py);
    }

    template<typename T>
    T &atPosition(float x, float y) {
        if (!isOriginInit) {
            throw std::logic_error("Origin Not Set");
        }
        Eigen::Vector2f position;
        position << x, y;
        position = LeftUpOrigin.inverse() * position;

        int px = static_cast<int>(position(0, 0));
        int py = static_cast<int>(position(1, 0));

        px = clip(px, 0, m_height);
        py = clip(py, 0, m_width);


        return m_grid.at<T>(px, py);
    }

    eigen_util::TransformationMatrix2d<float> &Origin() {
        return LeftUpOrigin;
    }


};


std::vector<std::vector<cv::Point>> createXsYsMatrix() {
    GridMap *gm;
    ros_util::LaserScan ls;


    auto mm = ls.getXsYsMatrix();


}

class SimpleGrid {
public:
    enum OriginType {
        LeftUp = 0,
        LeftBottom
    };


private:
//    ros_util::LaserScan& m_scan;
    float m_range_max;
    float m_grid_resolution;
    size_t m_length;
    size_t m_grid_height;
    std::shared_ptr<cv::Mat> m_grid;
    eigen_util::TransformationMatrix2d<float> LeftUpOrigin;
    bool isOriginSet;


public:

    cv::Mat &getMat() {
        return *m_grid;
    }

    SimpleGrid(float grid_resolution, int grid_height = 0) :
            m_grid_resolution(grid_resolution),
            isOriginSet(false),
            m_grid_height(grid_height) {

    }

    void setOrigin(double x_, double y_, double yaw_, OriginType type_) {

        eigen_util::TransformationMatrix2d<float> origin(x_, y_, yaw_);
        if (type_ == OriginType::LeftBottom) {
            eigen_util::TransformationMatrix2d<float> bottomToUp(0.0, 40.0, -0.5 * M_PI);
            LeftUpOrigin = origin * bottomToUp;

        }

        if (type_ == OriginType::LeftUp) {
            LeftUpOrigin = origin;

        }

        isOriginSet = true;
    }

    eigen_util::TransformationMatrix2d<float> &getOriginMatrix() {
        return LeftUpOrigin;
    }

    Eigen::MatrixXf getFreePoints(ros_util::LaserScan &scan_) {
        // reload
        if (scan_.range_max != m_range_max) {
            m_range_max = scan_.range_max;
            m_length = static_cast<size_t>(round(m_range_max / m_grid_resolution));
            auto m1 = cv::Mat(2 * m_length, 2 * m_length, CV_8SC1, cv::Scalar(0));
            m_grid = std::make_shared<cv::Mat>(m1);
            LeftUpOrigin = eigen_util::TransformationMatrix2d<float>(-m_range_max, -m_range_max, 0.0);
            isOriginSet = true;
        }

        (*m_grid) = cv::Scalar(-100);

        // get laser points in origin frame
        auto laserPoints = scan_.getXsYsMatrix();

        laserPoints = LeftUpOrigin.inverse() * laserPoints;

        std::vector<std::vector<cv::Point>> contours;
        contours.clear();
        std::vector<cv::Point> contour;
        contour.clear();

        cv::Point p(m_length, m_length);
        contour.push_back(p);
        size_t px = 0, py = 0;


        for (int i = 0; i < laserPoints.cols(); i++) {
            px = static_cast<size_t>(laserPoints(1, i) / m_grid_resolution);
            p.x = clip(px, 0 * m_length, 2 * m_length - 1);
//
//  p.y = static_cast<size_t>();

            py = static_cast<size_t>(laserPoints(0, i) / m_grid_resolution);
            p.y = clip(py, 0 * m_length, 2 * m_length - 1);
            contour.push_back(p);
            (*m_grid).at<signed char>(p) = 100;
        }

        contours.push_back(contour);

        // draw cell color lable inside contour
        cv::Scalar free_lable(1);

        cv::drawContours(*m_grid, contours, 0, free_lable, CV_FILLED); // 0: index of contours,
        cv::Rect rect = boundingRect(contours[0]);
        int left = rect.x;
        int top = rect.y;
        int width = rect.width;
        int height = rect.height;
        int x_end = left + width;
        int y_end = top + height;

        // find cell inside contour

        std::vector<float> freePoints;
        freePoints.clear();
        for (size_t x = left; x < x_end; x++) {
            for (size_t y = top; y < y_end; y++) {

                signed char &v = (*m_grid).at<signed char>(y, x);

                if (1 == v) {
                    v = 0;

                    float xf = y * m_grid_resolution;
                    float yf = x * m_grid_resolution;

                    freePoints.push_back(xf);
                    freePoints.push_back(yf);

                }
            }
        }

        auto freePointsMat = eigen_util::createMatrix<float, Eigen::ColMajor>(&(freePoints[0]), 2,
                                                                              freePoints.size() / 2);


        freePointsMat = LeftUpOrigin * freePointsMat;
        return freePointsMat;

    }


};

struct SimpleArray {
    std::vector<double> data;
    size_t m_rows;
    size_t m_cols;

    SimpleArray(int rows_, int cols_) : m_rows(rows_), m_cols(cols_) {

        data.resize(rows_ * cols_);
    }

    double &operator()(int i, int j) {
        return data[i * m_cols + j];
    }

    size_t rows() {
        return m_rows;
    }

    size_t cols() {
        return m_cols;
    }

};


int main(int argc, char *argv[]) {

#if 0
    std::vector<double> x1{1,2,3,4};
    std::vector<double> x2{1.1,2.1,3.1,4.1};

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mm(2,4);
    Eigen::Map<Eigen::MatrixXd> xm1(&(x1[0]), 1, 4);
    Eigen::Map<Eigen::MatrixXd> xm2(&(x2[0]), 1, 4);
    mm.row(0) << xm1;
    mm.row(1) << xm2;

    std::cout << "mm \n" << mm << std::endl;
    return 0;
#endif


// ros
    ros::init(argc, argv, "test");


    // create laser subscriber
    ros_util::Node node;
    auto scan_ptr = node.createSubscriber<sensor_msgs::LaserScan>("/scan", 1);

    auto map_ptr = node.createSubscriber<nav_msgs::OccupancyGrid>("/map", 1);
#if 0
    sensor_msgs::LaserScan scan;
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    int scan_num = 180 * 4;
    scan.angle_increment = (scan.angle_max - scan.angle_min) / (scan_num);
    scan.range_max = 30;
    scan.range_min = 0.0;

    boost::random::mt19937 rng;         // produces randomness out of thin air
    // see pseudo-random number generators
    boost::random::uniform_real_distribution<> six(8.0, 10.0);
    // distribution that maps to 1..6
    // see random number distributions

    for (int i = 0; i < scan_num; i++) {
//        int x = six(rng);

        scan.ranges.push_back(six(rng));
    }

    *scan_ptr = scan;

#endif
    eigen_util::createMatrix<float, Eigen::ColMajor>(&((*scan_ptr).ranges[0]), 1, (*scan_ptr).ranges.size());
    // create tf listenner

#if 0
    ros_util::TfListenner tfListenner;
#endif
    std::string laser_frame_id = (*scan_ptr).header.frame_id;
    tf::Transform map_base_tf;
    tf::Transform base_laser_tf;

#if 1
//    map_base_tf.setIdentity();

    map_base_tf.setOrigin(tf::Vector3(1.3, 2.4, 0.0));
    map_base_tf.setRotation(tf::createQuaternionFromYaw(3.14 / 4));

    base_laser_tf.setIdentity();
//    base_laser_tf.getOrigin().x() = 1.0;
//    base_laser_tf.getOrigin().y() = 1.5;
#endif

    // ranges to xs ys
    // contour offset


//    printf("1 %f, 2 %f, 3 %f, 4 %f", local_x_min, local_y_min, local_x_max, local_y_max);
//    std::cout << "create grid " << "width: " << local_width << "height: " << local_height << std::endl;

    //    std::cout << "occuPointsMat  \n" << occuPointsMat<< std::endl;
//    std::cout << "freePointsMat  \n" << freePointsMat<< std::endl;


    // global grid to map transform

    // ================================
    // HMM
    Eigen::MatrixXf Q(1, STATE_DIM);
    Q << 0.5, 0.5;

    // static occupy
    Eigen::MatrixXf A1(STATE_DIM, STATE_DIM);
    A1 << 0.2, 0.8, 0.05, 0.95;

    // static free
    Eigen::MatrixXf A2(STATE_DIM, STATE_DIM);
    A2 << 0.95, 0.05, 0.8, 0.2;

    // dynamic
    Eigen::MatrixXf A3(STATE_DIM, STATE_DIM);
    A3 << 0.8, 0.2, 0.2, 0.8;

    Eigen::MatrixXf B(STATE_DIM, OBS_DIM);
    B << 0.9, 0.05, 0.05, 0.05, 0.9, 0.05;

    Hmm::TensorX4 Fi(STATE_DIM, STATE_DIM, STATE_DIM, OBS_DIM);
    Hmm::TensorX3 Gama(STATE_DIM, STATE_DIM, OBS_DIM);

    Fi.setZero();
    Gama.setZero();
    Hmm::HmmParams Hmmparams1(Q, A1, B, Fi, Gama);
    Hmm::HmmParams Hmmparams2(Q, A2, B, Fi, Gama);
    Hmm::HmmParams Hmmparams3(Q, A3, B, Fi, Gama);


    // point in map frame to grid frame
    float global_x = 40;
    float global_y = 40;

    // display laser scan data in global grid
    // 0 = free, 255 = occu, 128 = unobserve

    float contour_offset = 0.05;
    float grid_resolution = 0.05;
    int grid_width = global_x / grid_resolution;
    int grid_height = global_x / grid_resolution;
    cv::Mat global_grid(grid_height, grid_width, CV_8UC1, cv::Scalar(0));

    cv::Mat predict_grid(grid_height, grid_width, CV_8S, cv::Scalar(-1));
    cv::Mat prob_grid(grid_height, grid_width, CV_32F, cv::Scalar(0.5));



    ros_util::LaserScan m_scan;
    SimpleGrid m_laser_grid(0.04);
    SimpleGrid m_map_grid(0.05, 40);

    nav_msgs::OccupancyGrid m_map;
    m_map.header.frame_id = "/map";
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan2", 1);
    time_util::Timer t;
    eigen_util::TransformationMatrix2d<float> transMat(0.0, 0.0, 0.0 * 3.14159 / 4);

    Eigen::MatrixXf freePointsMat, occuPointsMat, occuPointsMatInMap, freePointsMatInMap;
    Eigen::MatrixXf occupointsMatInGrid, freepointsMatInGrid;
    Eigen::MatrixXi freepointsMatInGrid_index, occupointsMatInGrid_index;
    Eigen::MatrixXf freepointsMatInGrid_index1, occupointsMatInGrid_index1;

    eigen_util::TransformationMatrix2d<float> originTrans(-20.0, -20.0, 0.0);

    m_map_grid.setOrigin(-20.0, -20.0, 0.0, SimpleGrid::OriginType::LeftBottom);

    auto m_map_grid_origin = m_map_grid.getOriginMatrix();
    double cnt = 0;

    double time_all = 0.0;
    while (1) {
        cnt++;
        node.getOneMsg("/scan", -1);
        t.start();

        m_map.header.stamp = ros::Time::now();
//        (*scan_ptr).header.stamp = ros::Time::now();
//        scan_pub.publish((*scan_ptr));
        m_scan = *scan_ptr;
        occuPointsMat = m_scan.getXsYsMatrix();


        m_scan.RangesVal() -= contour_offset;

        freePointsMat = m_laser_grid.getFreePoints(m_scan);
//        try {
            // map base_link


            // test change occu to free
//             occuPointsMatInMap = transMat * occuPointsMat;
//             freePointsMatInMap = transMat * freePointsMat;

            // map grid origin

#if 1
        auto origin2_to_map = m_map_grid_origin.inverse() * transMat;
        freepointsMatInGrid_index1 = origin2_to_map * freePointsMat;
        occupointsMatInGrid_index1 = origin2_to_map * occuPointsMat;

        occupointsMatInGrid_index1 = occupointsMatInGrid_index1.array() * 20.0;
        freepointsMatInGrid_index1 = freepointsMatInGrid_index1.array() * 20.0;
//
//         freepointsMatInGrid_index = freepointsMatInGrid_index1.cast<int>();
//         occupointsMatInGrid_index = occupointsMatInGrid_index1.cast<int>();

#endif


            // pont index
#if 0
        auto originToMap = originTrans.inverse() * transMat;
             occupointsMatInGrid = originToMap* occuPointsMat;
             freepointsMatInGrid = originToMap * freePointsMat;

        Eigen::MatrixXi occupointsMatInGrid_index(2, occupointsMatInGrid.cols());

        Eigen::MatrixXi freepointsMatInGrid_index(2, freepointsMatInGrid.cols());

        for (int i = 0; i < occupointsMatInGrid_index.cols(); i++) {
            occupointsMatInGrid_index(0, i) = occupointsMatInGrid(0, i) / grid_resolution;
            occupointsMatInGrid_index(1, i) = grid_height - occupointsMatInGrid(1, i) / grid_resolution;

        }

        for (int i = 0; i < freepointsMatInGrid_index.cols(); i++) {
            freepointsMatInGrid_index(0, i) = freepointsMatInGrid(0, i) / grid_resolution;
            freepointsMatInGrid_index(1, i) = grid_height - freepointsMatInGrid(1, i) / grid_resolution;

        }

#endif

//
//        std::cout << "m_map_grid_origin:\n" << m_map_grid_origin.matrix() << std::endl;
//        std::cout << "true occupointsMatInGrid_index\n" << occupointsMatInGrid_index.block(0,0,2,15) << std::endl;
//        std::cout << "false occupointsMatInGrid_index\n" << occupointsMatInGrid_index2.block(0,0,2,15) << std::endl;
//
//        exit(2);
//            std::cout << "================== bug1 \n" <<occupointsMatInGrid_index << std::endl;
//            std::cout << "================== bug2 \n" <<freepointsMatInGrid_index << std::endl;

//            exit(0);
//    cv::warpAffine(local_grid, global_grid, rotateMat, global_grid.size(), cv::INTER_NEAREST);

        // free cell value = 0

        for (int i = 0; i < freepointsMatInGrid_index1.cols(); i++) {
            cv::Point p(static_cast<int>(freepointsMatInGrid_index1(1, i)),
                        static_cast<int>(freepointsMatInGrid_index1(0, i)));


//                global_grid.at<uchar>(p) = 128;


            float &v = prob_grid.at<float>(p);
            Hmmparams1.Q()(0, 1) = v;
            Hmmparams1.Q()(0, 0) = 1.0 - Hmmparams1.Q()(0, 1);


            // predict cell state in predct grid
            int pred_state = 0;

            int obs = 0, state = 0;
            float prob;
            Hmm::predictOne(Hmmparams1, obs, state, prob);
            v = Hmmparams1.Q()(0, 1);

            if (pred_state == 0) {
                predict_grid.at<uchar>(p) = 0;
            } else {
                predict_grid.at<uchar>(p) = 100;

            }
        }

        // occupied cell value = 255
        for (int i = 0; i < occupointsMatInGrid_index1.cols(); i++) {
            cv::Point p(static_cast<int>(occupointsMatInGrid_index1(1, i)),
                        static_cast<int>(occupointsMatInGrid_index1(0, i)));


//                global_grid.at<uchar>(p) = 255;

                // predict cell state in predct grid
            float &v = prob_grid.at<float>(p);
            Hmmparams1.Q()(0, 1) = v;
            Hmmparams1.Q()(0, 0) = 1.0 - Hmmparams1.Q()(0, 1);


            int pred_state = 1;
            int obs = 1, state = 0;
                float prob;
                Hmm::predictOne(Hmmparams1, obs, state, prob);


            v = Hmmparams1.Q()(0, 1);

                if (pred_state == 0) {
                    predict_grid.at<uchar>(p) = 0;
                } else {
                    predict_grid.at<uchar>(p) = 100;

                }

//             cv::circle(global_grid, p, 3, cv::Scalar(255));

            }


            // unobserve cell value = 128
//        for (int i = 0; i < occupointsMatInGrid_index.cols(); i++) {
//            cv::Point p(static_cast<int>(occupointsMatInGrid_index(0,i)),
//                        static_cast<int>(occupointsMatInGrid_index(1,i)));
//
//            global_grid.at<uchar>(p) = 128;
//
//
//        }
//        } catch (...) {
//            std::cerr << "ff3 " << std::endl;
//        }

        t.stop();
        time_all += t.elapsedSeconds();
        std::cout << "time 2 " << time_all / cnt << ", " << t.elapsedSeconds() << std::endl;


        m_map.info.width = predict_grid.cols;
        m_map.info.height = predict_grid.rows;
        m_map.info.resolution = grid_resolution;
        m_map.data.clear();
        m_map.info.origin.position.x = -20;
        m_map.info.origin.position.y = -20;
        m_map.info.origin.orientation.w = 1;
        for (int i = m_map.info.width - 1; i >= 0; i--) {
            for (int j = 0; j < m_map.info.height; j++) {
                signed int d = predict_grid.at<uchar>(i, j);
                m_map.data.emplace_back(d);
            }
        }
        map_pub.publish(m_map);

//        cv::imshow("local_grid", local_grid);
//
//        cv::imshow("global_grid", global_grid);
//
//        cv::imshow("predict_grid", predict_grid);
//
//        cv::imshow("prob_grid", prob_grid);

//        cv::waitKey(1);
//        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // read map data
    // covert to matrix
    // make mask of dynamic area

    // create 3 matrix
    // one for p(o) , represent te probobility to be occupied
    // one for dynamic mask
    // one for update mask

    //1) lable point in laser scan detecting area
    // free = 0, occupy = 1, unobserve = 2
    // three point set
    // free set, occupy set, unobserve = dynamic area - free set - occupy set




    // get laserscan data
    // ranges to xs, ys
    //convert to contour

    // create local obsevation data grid
    // assign points inside contour
    // get observe grid

    // create global obsevation data grid

    // get robot pose
    // rotate to local grid to global grid
    // update contour cell


    // collect observation sequence of each cell

    // training


}