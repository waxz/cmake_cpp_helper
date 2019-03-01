//
// Created by waxz on 19-2-13.
//

#ifndef DEMO_PARTIAL_H
#define DEMO_PARTIAL_H

#include "grid.h"
#include <Eigen/Sparse>
/*===========
 * partial
 * contain a map, robot pose
 * a change cells list
 *
 * method
 * given laser scan freePointsMat and occuPointsMat
 * to update map
 * caculate weight : transitioin and match
 *
 * */
namespace partial_util {

    // ===========================
    // a gauss look up table
    // normlise x to index
    // return stored data
    class GaussLUT {
    private:
        float m_stddev;
        float m_max_x;
        float m_resolution;

        size_t xToIndex(float);

        std::vector<float> dataTable;
    public:
        GaussLUT(float, float, float);

        float operator[](float);
    };

    /*=====================
     * a cell model
     * has a position and for side
     * each side has 10 point
     * */
    /* p0******p1 --->x
     * *       *
     * *       *
     * *       *
     * p3******p2
     * |
     * |
     * y
     *
     * */
    class CellModel {
    private:
        int x;
        int y;
        std::vector<Eigen::Matrix2Xf> sidePoints;
        float m_side_len;
        float m_resolution;
        eigen_util::TransformationMatrix2d<float> origin;


    public:
        CellModel(float, float);

        void getPoints();


    };


    /*============================
     * a partial repesent robot pose and map
     * */
    class Partial {
    private:
        // robot pose
        eigen_util::TransformationMatrix2d<float> robot_pose;
        // life long map
        cv::Mat &m_map;

        // cell dynamic type   [0,1,2,3]
        cv::Mat &m_map_type;

        // hmm model for each type
        std::vector<int> m_hmm_model;

        // map grid
        grid_util::MapGrid &m_map_grid;

        // laser grid
        grid_util::LaserGrid &m_laser_grid;

        // changed cell hash map
        std::unordered_map<cv::Point, char> m_changed_cell;

        // store occupied cell
        std::unordered_map<cv::Point, char> m_occupied_cell;

        // store tracking dynamic cell
        std::unordered_map<cv::Point, char> m_track_cell;

        // weight
        float m_weight;


    public:
        Partial(cv::Mat map, cv::Mat map_type, grid_util::MapGrid map_grid, grid_util::LaserGrid laser_grid);

        // get or set robot pose
        eigen_util::TransformationMatrix2d<float> &Pose();

        // update weight using laser scan
        // compute match weight and transition weight
        // maybe run in thread
        void updateWeight();

        // create scan data from map
        // match with real scan
        void matchLaserScan();

        // set weight
        float &Weight();


    };
}

#endif //DEMO_PARTIAL_H
