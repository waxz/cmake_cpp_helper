//
// Created by waxz on 19-2-14.
//
#include <demo/partial.h>

namespace partial_util {

    // define the resolution, sttdev, x range
    GaussLUT::GaussLUT(float stddev, float max_x, float resolution) :
            m_stddev(stddev),
            m_max_x(max_x),
            m_resolution(resolution) {

        size_t sz = static_cast<size_t >(round(m_max_x / m_resolution));
        for (size_t i = 0; i < sz; i++) {

            float w = (1 / (m_stddev * sqrt(2 * M_PI))) * exp(-0.5 * pow(i * m_resolution / m_stddev, 2.0));
            dataTable.push_back(w);

        }
    }

    size_t GaussLUT::xToIndex(float x) {

        x = (fabs(x) > m_max_x) ? m_max_x : fabs(x);
        size_t index = static_cast<size_t >(round(x / m_resolution));
        return index;

    }


    float GaussLUT::operator[](float x) {
        auto i = xToIndex(x);
        return dataTable[i];
    }

    //=========
    CellModel::CellModel(float side_len, float resolution) : m_side_len(side_len), m_resolution(resolution) {

        // fill points on side line
        sidePoints.clear();
        // 0 --> 1
        // 1 --> 2
        // 2 --> 3
        // 3 --> 0
        std::vector<float> x_vec;
        for (int i = 0; i < 10; i++) {
            x_vec.push_back(0.1);
        }

        Eigen::Map<Eigen::Matrix<float, 1, Eigen::Dynamic>> m(x_vec.data(), x_vec.size());


    }


    Partial::Partial(cv::Mat map, cv::Mat map_type, grid_util::MapGrid map_grid, grid_util::LaserGrid laser_grid) :
            m_map(map),
            m_map_type(map_type),
            m_map_grid(map_grid),
            m_laser_grid(laser_grid) {

    }

    eigen_util::TransformationMatrix2d<float> &Partial::Pose() {
        return robot_pose;
    }

    float &Partial::Weight() {
        return m_weight;
    }

    void Partial::matchLaserScan() {
        // each partial should maintence a group of cell
        // first set beam reading from these data

        // if not found , search in grid
        std::vector<cv::Point> occuCells;
        int sz = occuCells.size();

        ros_util::LaserScan scan;

        for (auto i = 0; i < sz; i++) {

            // find 2 point

            // assign to laser ranges

            //1. check relative position
            /* p1******p2
             * *       *
             * *       *
             * *       *
             * p3******p4
             * */
            /* compute each point angle in laser
             * if range < laser.range, update it
             * vector<tuple()>
             * */


        }

        // if not found in cell list
        // search grid
        sz = scan.ranges.size();
        for (int i = 0; i < sz; i++) {
            // if range is invalid from occupied cells

            if (scan.ranges[i] == 0) {
                // go through beam point
                //
                // cell_state = mat.at<char>(x,y)



            }
        }


        // get reference scan


        // compute scan match weight
        mp::mpf_float match_weight = 1;
        for (int i = 0; i < sz; i++) {
            float diff = scan.ranges[i] - scan.ranges[i];
            float weight = 1;
            match_weight *= weight;

        }


        // 1. get fake scan from map
        // go through all points on beam
        // find closest occupied cell on each beam

        // first find on change cell list
        // then on life long map

        //2. compare two scan

    }

    // main entry from upper level
    void Partial::updateWeight() {

        //1. apply movement


        //2. compute scan range reading match score


        //3. apply laser scan to update map
        //3.1 tranform local grid to global grid
        //3.2 update obsevation in each cell
        //3.3 update cell state

        //4. compute cell transition score
        //4.1 compute transition score
        //4.2 update change cell list

        //5. updateweight







        // 1. transform local grid to global grid


        // 2.


    }


}
