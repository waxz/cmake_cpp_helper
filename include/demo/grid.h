//
// Created by waxz on 19-2-13.
//

#ifndef DEMO_GRID_H
#define DEMO_GRID_H
//#include <cpp_utils/qt_util.h>
#include <cpp_utils/ros_util.h>
#include <cpp_utils/eigen_util.h>
#include <cpp_utils/ublas.h>

#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <random>
//#include "backend.h"

#include <HMM/Hmm.h>

#include <boost/multiprecision/gmp.hpp>  // Defines the wrappers around the GMP library's types
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <unordered_map>

namespace mp = boost::multiprecision;

#include <opencv2/opencv.hpp>
#include <ros/ros.h>


template<typename T>
T clip(const T &n, const T &lower, const T &upper) {
    return std::max(lower, std::min(n, upper));
}


/*=================
 * hash
 * https://stackoverflow.com/questions/17016175/c-unordered-map-using-a-custom-class-type-as-the-key
 *
 *
 * */
namespace std {
    template<>
    struct hash<cv::Point> {
        size_t operator()(const cv::Point &k) const {
            // Compute individual hash values for first, second and third
            // http://stackoverflow.com/a/1646913/126995
            size_t res = 17;
            res = res * 31 + hash<int>()(k.x);
//            res = res * 31 + hash<int>()( k.y );
            return res;
        }
    };
}


namespace grid_util {
    /*=====================
 * laser scan class
 * provide convention from lasersan to occupy mat and free mat in local grid
 *
 * */

    class LaserGrid {
    public:
        // basic type for matrix

    private:
        ros_util::LaserScan m_scan;

        Eigen::MatrixXf freePointsMat;
        Eigen::MatrixXf occuPointsMat;
        ublas::matrix<float> freePointsMat_ublas;
        ublas::matrix<float> occuPointsMat_ublas;

        float m_range_max;
        float m_grid_resolution;
        float m_contour_offset;
        size_t m_length;
        size_t m_grid_height;
        std::shared_ptr<cv::Mat> m_grid;
        eigen_util::TransformationMatrix2d<float> LeftUpOrigin;
        ublas_util::Transformation2d LeftUpOrigin_ublas;
        bool isOriginSet;
        std::vector<Eigen::MatrixXf> m_beam_base;

        std::vector<Eigen::MatrixXf> m_beam;

        std::vector<ublas::matrix<float>> m_beam_ublas;
        ublas::matrix<float> m_beam_matrix;
        std::vector<size_t> m_beam_index;


    public:

        LaserGrid(float grid_resolution = 0.05);

        //=========
        // update laserscan from ros
        void update(const sensor_msgs::LaserScan &msg);

        //update free mat
        Eigen::MatrixXf &getFreePointsMat();

        ublas::matrix<float> &getFreePointsMat_ublas();

        //update occupy mat
        Eigen::MatrixXf &getOccuPointsMat();

        ublas::matrix<float> &getOccuPointsMat_ublas();

        //
        void setContour_Offset(float v) {
            m_contour_offset = v;
        }

        // =================
        // laser beam model
        void createBeamBase(float yaw, std::valarray<float> &cos_val, std::valarray<float> &sin_val);

        // sample date point on each beam
        void createBeam();


        ublas::matrix<float> &getBeam() {
            return m_beam_matrix;
        }

        std::vector<size_t> &getBeamIndec() {
            return m_beam_index;
        }

        ros_util::LaserScan &getScan() {
            return m_scan;
        }

        float getResolution() {
            return m_grid_resolution;
        }

    };

    // ==============================================
    // grid for map
    class MapGrid {
    public:
        enum OriginType {
            LeftUp = 0,
            LeftBottom
        };

    private:
    private:
        float m_grid_resolution;
        size_t m_length;
        float m_grid_height;
        eigen_util::TransformationMatrix2d<float> LeftUpOrigin;
        ublas_util::Transformation2d LeftUpOrigin_ublas;
        bool isOriginSet;

    public:
        MapGrid(float grid_resolution, float grid_height);

        void setOrigin(double x_, double y_, double yaw_, OriginType type_ = OriginType::LeftBottom);

        eigen_util::TransformationMatrix2d<float> &getOriginMatrix();

        ublas_util::Transformation2d &getgetOriginMatrix_ublas() {
            return LeftUpOrigin_ublas;
        }

        void dispay(const ublas::matrix<float> &m);
    };


}


#endif //DEMO_GRID_H
