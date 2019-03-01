//
// Created by waxz on 19-2-21.
//
#include <iostream>
#include <valarray>
#include <cpp_utils/ublas.h>
#include <cpp_utils/tbb_util.h>
#include <cpp_utils/ros_util.h>
#include <cpp_utils/eigen_util.h>

#include <cpp_utils/time.h>
#include <vector>
#include <boost/array.hpp>
#include <ros/ros.h>
#include <demo/grid.h>
#include <opencv2/opencv.hpp>
#include "test_speed.cpp"
#include <boost/random/geometric_distribution.hpp>
#include <boost/random/normal_distribution.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>

float normal_pdf(float x, float m, float s) {
//    static const float inv_sqrt_2pi = 0.3989422804014327;
//    float a = (x - m) / s;

//    inv_sqrt_2pi / s * std::exp(-0.5f * a * a);

    float pdf_gaussian = (1 / (s * sqrt(2 * M_PI))) * exp(-0.5 * pow((x - m) / s, 2.0));

    return pdf_gaussian;
}

struct GaussLUT {
    std::vector<float> data;
    float m_max_x;
    size_t m_max_num;

    GaussLUT(float m, float s, float max_x, size_t max_num) : m_max_x(max_x), m_max_num(max_num) {
        data = std::vector<float>(max_num, 0.0);
        float b = max_x / max_num;
        for (size_t i = 0; i < max_num; i++) {
            data[i] = normal_pdf(i * b, m, s);
        }
    }

    float operator()(float x) {
        size_t index = m_max_num * x / m_max_x;
        return data[index];
    }
};


struct Partial {

    ublas_util::Transformation2d map_laser_pose;
    ublas::matrix<float> minv_beam;
    float weight;
    mp::mpf_float match_weight;

    cv::Point getPose() {
        cv::Point p(map_laser_pose.matrix()(0, 2), map_laser_pose.matrix()(1, 2));
        return p;
    }

    cv::Point poseInMap;

    Partial() : minv_beam(ublas::matrix<float>(3, 3)) {

    }
};

int main(int argc, char **argv) {

    std::cout << "hello" << std::endl;

//    GaussLUT gaussLUT(0.0, 0.2, 1.0, 20);
//    for (int i = 0; i < 20 ;i++){
//        std::cout << "- " << gaussLUT.data[i] << std::endl;
//    }
//
//
//
//    exit(2);
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan2", 1);
    ros::Publisher scan_pub0 = nh.advertise<sensor_msgs::LaserScan>("scan0", 1);

    ros_util::Node node;
    auto scan_ptr = node.createSubscriber<sensor_msgs::LaserScan>("/scan", 1);
    nav_msgs::OccupancyGrid m_map;
    m_map.header.frame_id = "/map";
    m_map.header.stamp = ros::Time::now();
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);

    geometry_msgs::PoseWithCovarianceStamped initi_pose;
    geometry_msgs::PoseArray partial_cloud;

    auto initialpose_ptr = node.createSubscriber<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    ros::Publisher partial_cloud_pub = nh.advertise<geometry_msgs::PoseArray>("/partial_cloud", 1);
    ros::Publisher final_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);

    geometry_msgs::PoseStamped final_pose;

    partial_cloud.header.frame_id = "base_laser";
    final_pose.header.frame_id = "base_laser";


    test_main();
//    exit(3);

    ublas_util::Transformation2d trans(0.5, 0.2, 0.5 * M_PI);


    // test
    // 1. laserscan
    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "base_laser";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    int scan_num = 180 * 4;
    scan.angle_increment = (scan.angle_max - scan.angle_min) / (scan_num);
    scan.range_max = 30;
    scan.range_min = 0.0;
    boost::random::mt19937 rng;         // produces randomness out of thin air

    boost::random::mt19937 rng_x;         // produces randomness out of thin air
    boost::random::mt19937 rng_y;         // produces randomness out of thin air
    boost::random::mt19937 rng_yaw;         // produces randomness out of thin air

    // see pseudo-random number generators
    boost::random::uniform_real_distribution<> six(6.0, 6.2);


    boost::random::normal_distribution<> gen_x(0.0, 0.35);
    boost::random::normal_distribution<> gen_y(0.0, 0.35);
    boost::random::normal_distribution<> gen_yaw(0.0, 0.13);


    // distribution that maps to 1..6
    // see random number distributions

    for (int i = 0; i < scan_num; i++) {
//        int x = six(rng);

        scan.ranges.push_back(six(rng));
    }




    //===========
    // create map
    grid_util::LaserGrid laser_grid(0.03);

    ros_util::LaserScan m_scan;
    // 3. map grid
    grid_util::MapGrid map_grid(0.05, 20.0);
    map_grid.setOrigin(-10.0, -10.0, 0.0);
    auto map_origin = map_grid.getgetOriginMatrix_ublas();
    ublas_util::Transformation2d map_laser(0, 0, 0.0 * M_PI);
    cv::Mat mat(400, 400, CV_8UC1, cv::Scalar(100));
    laser_grid.setContour_Offset(0.08);
    auto map_origin_inverse = map_origin.inverse();
    ublas::matrix<float> minv = ublas::prod(map_origin_inverse, map_laser.matrix());

    for (int i = 0; i < 40; i++) {

        node.getOneMsg("/scan", -1);
        scan = *(scan_ptr);
        for (auto &reading : scan.ranges) {
            reading = clip(reading, 0.0f, 10.0f);
        }
        // 2. laser grid
        laser_grid.update(scan);
        auto fp = laser_grid.getFreePointsMat_ublas();
        auto op = laser_grid.getOccuPointsMat_ublas();

        auto fp1 = fp;
        auto op1 = op;
        ublas::noalias(fp1) = ublas::prod(minv, fp);
        ublas::noalias(op1) = ublas::prod(minv, op);


        for (size_t j = 0; j < fp1.size2(); j++) {
            cv::Point p(round(fp1(1, j) / 0.05), round(fp1(0, j) / 0.05));
            p.x = clip(p.x, 0, mat.cols);
            p.y = clip(p.y, 0, mat.rows);
            mat.at<uchar>(p) = 0;

        }
        for (size_t j = 0; j < op1.size2(); j++) {
            cv::Point p(round(op1(1, j) / 0.05), round(op1(0, j) / 0.05));

            p.x = clip(p.x, 0, mat.cols);

            p.y = clip(p.y, 0, mat.rows);
            mat.at<uchar>(p) = 255;

        }
        cv::imshow("laser", mat);

        cv::waitKey(1);

    }

#if 1
    std::cout << "dispaly" << std::endl;

    m_map.info.width = mat.cols;
    m_map.info.height = mat.rows;
    m_map.info.resolution = 0.05;
    m_map.data.clear();
    m_map.info.origin.position.x = -10;
    m_map.info.origin.position.y = -10;
    m_map.info.origin.orientation.w = 1;
    for (int i = m_map.info.width - 1; i >= 0; i--) {
        for (size_t j = 0; j < m_map.info.height; j++) {
            signed int d = mat.at<uchar>(i, j);
            m_map.data.emplace_back(d);
        }
    }

#endif

//    std::cout << "get data\n" << m_scan.getXsYsMatrix_ublas() << std::endl;
    node.getOneMsg("/initialpose", -1);

    mp::mpf_float global_best_weight = 10000.0;

//    bool global_enable_match = true;
    time_util::Timer timer;
//    global_enable_match = false;
    while (true) {

#if 0
        if (global_enable_match){
            bool g = node.getOneMsg("/initialpose", 0.1);
            if (g){
                global_enable_match = true;

            }

        }else{
            bool g = node.getOneMsg("/initialpose", 0.1);
            if (g){
                global_enable_match = true;

            }
            continue;
        }
#endif

        bool g = node.getOneMsg("/initialpose", 0.1);
        if (g) {
            global_best_weight = 10000.0;
        }
        auto initialpose = *initialpose_ptr;


        timer.start();
        std::vector<Partial> Partials(1000);


#pragma omp simd
        for (int i = 0; i < 1000; i++) {
            ublas_util::Transformation2d map_laser_beam(gen_x(rng), gen_y(rng), gen_yaw(rng));
            ublas::matrix<float> minv_beam(3, 3);
            geometry_msgs::Pose p;

            p.position.x = initialpose.pose.pose.position.x + map_laser_beam.getX();

            p.position.y = initialpose.pose.pose.position.y + map_laser_beam.getY();

            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(
                    tf::getYaw(initialpose.pose.pose.orientation) + map_laser_beam.getYaw()), p.orientation);

            map_laser_beam.set(p.position.x, p.position.y, tf::getYaw(p.orientation));

            Partials[i].map_laser_pose = map_laser_beam;

            partial_cloud.poses.push_back(p);

            ublas::noalias(minv_beam) = ublas::prod(map_origin_inverse, map_laser_beam.matrix());

            Partials[i].minv_beam = minv_beam;
        }
        timer.stop();
        std::cout << "sample time: " << timer.elapsedSeconds() << std::endl;

        partial_cloud.header.stamp = ros::Time::now();
        partial_cloud_pub.publish(partial_cloud);



//    std::cout << "get data\n" << laser_grid.getFreePointsMat_ublas() << std::endl;
//    std::cout << "get data\n" << laser_grid.getOccuPointsMat_ublas() << std::endl;

        auto fp = laser_grid.getFreePointsMat_ublas();
        auto fp2 = laser_grid.getFreePointsMat();
        auto op = laser_grid.getOccuPointsMat_ublas();
        // check cv mat
        cv::Mat mat_dynamic(400, 400, CV_8UC1, cv::Scalar(0));
//    cv::Mat mat_pritial(400,400, CV_8UC1, cv::Scalar(0));




#if 0
        eigen_util::TransformationMatrix2d<float> map_laser2(10,10,0.6*M_PI);
    fp2 = map_laser2*fp2;
    std::cout << "map laser:\n" << map_laser.matrix() << std::endl;
    std::cout << "map laser2:\n" << map_laser2.matrix() << std::endl;

    auto fp1 = fp;
    auto op1 = op;
//    ublas::inverse(map_laser2,map_laser2);


    std::cout << "map laser inverse:\n" << map_laser.inverse() << std::endl;
    std::cout << "map laser2 inverse:\n" << map_laser2.matrix().inverse() << std::endl;
    std::cout << "minv:\n" << minv << std::endl;
    ublas::noalias(fp1) = ublas::prod(minv, fp);
    ublas::noalias(op1) = ublas::prod(minv, op);





    // check dynamic type
    // create Q matrix for each type [Q0, Q1, Q2...]
    // compute A*Q

    ublas::matrix<float> A1(2,2);

    std::vector<size_t> freePointsXs;
    std::vector<size_t> freePointsYs;
    std::vector<float> freePointsQs;


//    for (size_t i = 0 ; i < op1.size2(); i++){
//        cv::Point p(round(op1(1,i)/0.05), round(op1(0,i)/0.05));
//        mat.at<uchar>(p) = 255;
//
//    }

    // create Q matrix from freePointsQs
    ublas::matrix<float> freeQ(2,100);

    auto freeQ2 = freeQ;
#endif



//    ublas::noalias(freeQ2) = ublas::prod(A, freeQ);

        // update cache


        // create tempral sparase matrix for query
        // update matrix



        // ray trace
        // get beam and index
//    laser_grid.createBeam();
//    auto beams = laser_grid.getBeam();

        std::cout << "================" << std::endl;

        timer.start();





        //==============
        auto scan2 = scan;
        size_t sz = laser_grid.getScan().ranges.size();

//    std::valarray<float> scan3(0.0, sz);

//    std::cout << "op1: \n" << op1 << std::endl;
//    std::cout << "beams: \n" << beams << std::endl;
//
//    exit(2);



        tbb::parallel_for(tbb::blocked_range<int>(0, 1000),
                          [&](tbb::blocked_range<int> r) {
                              for (auto it = r.begin(); it != r.end(); it++) {

                                  std::valarray<float> cos_val, sin_val;
                                  float yaw = Partials[it].map_laser_pose.getYaw();

                                  laser_grid.createBeamBase(0.5 * M_PI + yaw, cos_val, sin_val);

                                  ublas::matrix<float> op_beam = op;
                                  op_beam = ublas::prod(Partials[it].minv_beam, op);
//
                                  float resotion = 0.05;
                                  mp::mpf_float match_weight = 1;
                                  std::vector<float> mask_val(40, 0.0);

                                  for (size_t i = 0; i < sz; i++) {
                                      int s = -laser_grid.getScan().ranges[i] / resotion;
                                      s = clip(s, -20, 20);
                                      int e = (laser_grid.getScan().range_max - laser_grid.getScan().ranges[i]) /
                                              resotion;
                                      e = clip(e, -20, 20);

                                      for (int j = s; j < e; j++) {
                                          mask_val[j + 20] = (2 - 3.4);


#if 1
                                          float px = op_beam(0, i) + j * cos_val[i];
                                          float py = op_beam(1, i) + j * sin_val[i];
                                          cv::Point p(round(py / 0.05), round(px / 0.05));

                                          p.x = clip(p.x, 0, mat_dynamic.cols);
                                          p.y = clip(p.y, 0, mat_dynamic.rows);
                                          //mask_val[j + 20] = (2 - mat.at<uchar>(p)/255 );


                                          if (mat.at<uchar>(p) == 255 || j == e - 1) {

                                              auto diff = fabs(j * laser_grid.getResolution());

                                              diff = clip(static_cast<float>(diff), 0.01f, 0.5f);
                                              match_weight += diff * diff;
                                              break;

                                          } else {

                                          }
#endif


                                      }
//                                  exit(22);


                                      // compute scan

                                  }
                                  Partials[it].match_weight = match_weight;

                              }

                          },
                          tbb::simple_partitioner());
        timer.stop();
        std::cout << "total time: " << timer.elapsedSeconds() << std::endl;
        auto best_weight = Partials[0].match_weight;
        size_t best_id = 0;
        for (size_t i = 0; i < 1000; i++) {
//        std::cout << "------\n id " << i << " ---weight: " << Partials[i].match_weight << std::endl;
//        std::cout << "pose:\nx: " << Partials[i].map_laser_pose.getX() << " y: "<< Partials[i].map_laser_pose.getY()
//                  << " yaw: " << Partials[i].map_laser_pose.getYaw()<< std::endl;
//
//        if (i == 999){
//            std::cout << "map laser :\n" << Partials[i].map_laser_pose.matrix() << std::endl;
//
//            std::cout << "minv:\n" << Partials[i].minv_beam << std::endl;
//            minv;
//            std::cout << "global minv:\n" << minv << std::endl;
//
//        }

            if (Partials[i].match_weight < best_weight) {
                best_weight = Partials[i].match_weight;
                best_id = i;
            }
        }

        std::cout << "best id :" << best_id;
        std::cout << "pose:\nx: " << Partials[best_id].map_laser_pose.getX() << " y: "
                  << Partials[best_id].map_laser_pose.getY()
                  << " yaw: " << Partials[best_id].map_laser_pose.getYaw() << std::endl;

        std::cout << "best weight: " << Partials[best_id].match_weight << std::endl;

        if (Partials[best_id].match_weight < global_best_weight) {
            global_best_weight = Partials[best_id].match_weight;
            //global_enable_match = true;
            final_pose.pose.position.x = Partials[best_id].map_laser_pose.getX();
            final_pose.pose.position.y = Partials[best_id].map_laser_pose.getY();

            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(Partials[best_id].map_laser_pose.getYaw()),
                                  final_pose.pose.orientation);

        } else {
            // global_enable_match = false;

        }


        final_pose.header.stamp = ros::Time::now();
//        cv::imshow("laser", mat);
//        cv::imshow("laser", mat);

//        cv::imshow("mat_dynamic", mat_dynamic);
        final_pose_pub.publish(final_pose);
        (*initialpose_ptr).pose.pose = final_pose.pose;


        scan2.header.stamp = ros::Time::now();
//        scan_pub0.publish(scan);
//        scan_pub.publish(scan2);
        map_pub.publish(m_map);

        cv::waitKey(1);
    }
    // 4. show in cv mat



}
