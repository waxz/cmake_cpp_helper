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

int main(int argc, char *argv[]) {




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

    float contour_offset = 0.1;
    float grid_resolution = 0.05;
    int grid_width = global_x / grid_resolution;
    int grid_height = global_x / grid_resolution;
    cv::Mat global_grid(grid_height, grid_width, CV_8UC1, cv::Scalar(0));

    cv::Mat predict_grid(grid_height, grid_width, CV_8UC1, cv::Scalar(0));
    cv::Mat prob_grid(grid_height, grid_width, CV_32F, cv::Scalar(0.5));


    std::valarray<float> xs, ys;
    std::valarray<float> cache_cos, cache_sin;
    std::valarray<float> xs_offset, ys_offset;
    // points to contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Point> contour;
    std::vector<double> occuPoints;

    ros_util::LaserScan m_scan;

    nav_msgs::OccupancyGrid m_map;
    m_map.header.frame_id = "/map";
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);

    while (1) {
        node.getOneMsg("/scan", -1);
        m_map.header.stamp = (*scan_ptr).header.stamp;

        m_scan = *scan_ptr;
        m_scan.getXsYs(xs, ys);


        m_scan.RangesVal() -= contour_offset;
        m_scan.getXsYs(xs_offset, ys_offset);

        contour.clear();
        contours.clear();
        // push contour to contour vector
        // get local grid size
        float local_x_min = xs.min();
        float local_y_min = ys.min();
        float local_x_max = xs.max();
        float local_y_max = ys.max();

        local_x_min = std::fmin(local_x_min, 0.0);


        int local_height = static_cast<int>(round((local_x_max - local_x_min) / grid_resolution));
        int local_width = static_cast<int>(round((local_y_max - local_y_min) / grid_resolution));

        // ====== end preocess scan


        // porcess contour
        cv::Mat local_grid(local_height, local_width, CV_8UC1, cv::Scalar(0));
//        std::cout << "mat " << local_grid.rows << ", " << local_grid.cols << std::endl;
        cv::Point p0(static_cast<int>(round((-local_y_min) / grid_resolution)),
                     static_cast<int>(round((-local_x_min) / grid_resolution)));

        contour.push_back(p0);//your cont
        occuPoints.clear();

        for (int i = 0; i < (*scan_ptr).ranges.size(); i++) {

            size_t px = clip(static_cast<int>(round((ys_offset[i] - local_y_min) / grid_resolution)), 0,
                             local_grid.cols - 1);
            size_t py = clip(static_cast<int>(round((xs_offset[i] - local_x_min) / grid_resolution)), 0,
                             local_grid.rows - 1);

            cv::Point p(px, py);


            int ec = 0;
            try {
                contour.push_back(p);//your cont
                local_grid.at<uchar>(p) = 255;
//                std::cout << "point [ " << p.x << ", " << p.y << " ]\n" ;
//                std::cout << "grid  [ " << local_grid.cols << ", " << local_grid.rows << " ]\n" ;

                if (p.x >= local_grid.cols || p.y >= local_grid.rows) {
                    std::cout << "=====" << std::endl;
                    exit(0);
                }
                occuPoints.push_back(static_cast<double>(xs[i]));
                occuPoints.push_back(static_cast<double>(ys[i]));

            }
            catch (const std::bad_alloc &) {

                ec = 1;
            }
            catch (const std::exception &) {
                ec = 2;
                // You can inspect what() for a particular reason, or even try to dynamic_cast<>
                // to find out which specific exception was thrown.
            }
            catch (...) {
                // You have no more type info here, this should be done for last resort (in the
                // outermost scope) only.
                ec = 3;
            }
        }

//        continue;


        try {
            contours.push_back(contour);

        } catch (std::exception &e) {
            std::cout << "=== exception  " << e.what() << std::endl;
        }


        // fill free lable


        std::vector<double> freePoints;
        freePoints.clear();
        {
            cv::Scalar free_lable(100);

            cv::drawContours(local_grid, contours, 0, free_lable, CV_FILLED); // 0: index of contours,
            cv::Rect rect = boundingRect(contours[0]);
            int left = rect.x;
            int top = rect.y;
            int width = rect.width;
            int height = rect.height;
            int x_end = left + width;
            int y_end = top + height;

//        freePoints.push_back(-1.0);
//        freePoints.push_back(0.0);

            for (size_t x = left; x < x_end; x++) {
                for (size_t y = top; y < y_end; y++) {
                    cv::Point p(x, y);

                    uchar v = 0;//local_grid.at<uchar>(p);
                    try {
                        v = local_grid.at<uchar>(p);

                    } catch (const std::bad_alloc &) {
                        std::cout << "get contour error " << std::endl;
                    }
                    catch (...) {
                        std::cout << "get contour error " << std::endl;
                    }

                    if (100 == v) {
                        local_grid.at<uchar>(p) = 128;

                        /*
                         *
                         *
                         * */

                        double xf = y * grid_resolution + local_x_min;
                        double yf = x * grid_resolution + local_y_min;

                        int ec = 0;
                        try {

                            freePoints.push_back(xf);
                            freePoints.push_back(yf);
                        }
                        catch (const std::bad_alloc &) {
                            ec = 1;
                        }
                        catch (const std::exception &) {
                            ec = 2;
                            // You can inspect what() for a particular reason, or even try to dynamic_cast<>
                            // to find out which specific exception was thrown.
                        }
                        catch (...) {
                            // You have no more type info here, this should be done for last resort (in the
                            // outermost scope) only.
                            ec = 3;
                        }

//                    cv::Point p2(static_cast<int>(round((yf - local_y_min) / grid_resolution)),
//                                static_cast<int>(round((xf - local_x_min) / grid_resolution)));
//                    local_grid.at<uchar>(p2) = 98;
//                    std::cout << "p2: index: " << x << ", " << y << std::endl;
//                    std::cout << "p2: position : " << xf << ", " << yf << std::endl;

//                    std::cout << "p2: local_min: " << local_x_min << ", " << local_y_min << std::endl;


//                    std::cout << "p2: " << p2.x << ", " << p2.y << std::endl;

//                    exit(0);
                    }
                }
            }

        }


        auto occuPointsMat = eigen_util::createMatrix<double, Eigen::ColMajor>(&(occuPoints[0]), 2,
                                                                               occuPoints.size() / 2);

        auto freePointsMat = eigen_util::createMatrix<double, Eigen::ColMajor>(&(freePoints[0]), 2,
                                                                               freePoints.size() / 2);


        // === end process contour



        // === remap to grif frame


        try {
            // map base_link

            eigen_util::TransformationMatrix2d transMat(0.0, 0.0, 0.0 * 3.14159 / 4);

            // test change occu to free
            auto occuPointsMatInMap = transMat * occuPointsMat;
            auto freePointsMatInMap = transMat * freePointsMat;

            // map grid origin
            eigen_util::TransformationMatrix2d originTrans(-20.0, -20.0, 0.0);

            Eigen::MatrixXd occupointsMatInGrid = originTrans.inverse() * occuPointsMatInMap;
            Eigen::MatrixXd freepointsMatInGrid = originTrans.inverse() * freePointsMatInMap;


            // pont index
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

//            std::cout << "================== bug1 \n" <<occupointsMatInGrid_index << std::endl;
//            std::cout << "================== bug2 \n" <<freepointsMatInGrid_index << std::endl;

//            exit(0);
//    cv::warpAffine(local_grid, global_grid, rotateMat, global_grid.size(), cv::INTER_NEAREST);



            // occupied cell value = 255
            for (int i = 0; i < occupointsMatInGrid_index.cols(); i++) {
                cv::Point p(static_cast<int>(occupointsMatInGrid_index(0, i)),
                            static_cast<int>(occupointsMatInGrid_index(1, i)));


                global_grid.at<uchar>(p) = 255;

                // predict cell state in predct grid
                Hmmparams1.Q()(0, 1) = prob_grid.at<float>(p);
                Hmmparams1.Q()(0, 0) = 1.0 - Hmmparams1.Q()(0, 1);

                int pred_state = 1;
                int obs = 1, state = 0;
                float prob;
                Hmm::predictOne(Hmmparams1, obs, state, prob);

                prob_grid.at<float>(p) = Hmmparams1.Q()(0, 1);

                if (pred_state == 0) {
                    predict_grid.at<uchar>(p) = 0;
                } else {
                    predict_grid.at<uchar>(p) = 255;

                }

//             cv::circle(global_grid, p, 3, cv::Scalar(255));

            }
            // free cell value = 0
            for (int i = 0; i < freepointsMatInGrid_index.cols(); i++) {
                cv::Point p(static_cast<int>(freepointsMatInGrid_index(0, i)),
                            static_cast<int>(freepointsMatInGrid_index(1, i)));


                global_grid.at<uchar>(p) = 128;


                Hmmparams1.Q()(0, 1) = prob_grid.at<float>(p);
                Hmmparams1.Q()(0, 0) = 1.0 - Hmmparams1.Q()(0, 1);


                // predict cell state in predct grid
                int pred_state = 0;

                int obs = 0, state = 0;
                float prob;
                Hmm::predictOne(Hmmparams1, obs, state, prob);
                prob_grid.at<float>(p) = Hmmparams1.Q()(0, 1);

                if (pred_state == 0) {
                    predict_grid.at<uchar>(p) = 0;
                } else {
                    predict_grid.at<uchar>(p) = 255;

                }
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
        } catch (...) {
            std::cerr << "ff3 " << std::endl;
        }


        bool get = node.getOneMsg("/map", 1);

        if (get) {
            cv::Mat ros_grid((*map_ptr).info.height, (*map_ptr).info.width, CV_8UC1, &((*map_ptr).data[0]));
            cv::imshow("ros_grid", ros_grid);

        }
        m_map.info.width = predict_grid.cols;
        m_map.info.height = predict_grid.rows;
        m_map.info.resolution = grid_resolution;
        m_map.data.clear();
        m_map.info.origin.position.x = -20;
        m_map.info.origin.position.y = -20;

        for (int i = 0; i < m_map.info.width; i++) {
            for (int j = 0; j < m_map.info.height; j++) {
                signed int d = predict_grid.at<uchar>(i, j);
                m_map.data.emplace_back(d);
            }
        }
        map_pub.publish(m_map);
        cv::imshow("local_grid", local_grid);

        cv::imshow("global_grid", global_grid);

        cv::imshow("predict_grid", predict_grid);

        cv::imshow("prob_grid", prob_grid);

        cv::waitKey(1);
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