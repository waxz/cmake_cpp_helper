//
// Created by waxz on 19-2-13.
//
#include <demo/grid.h>

namespace grid_util {
    template<typename T>
    inline T clip(const T &n, const T &lower, const T &upper) {
        return std::max(lower, std::min(n, upper));
    }

    LaserGrid::LaserGrid(float grid_resolution) :
            m_grid_resolution(grid_resolution),
            isOriginSet(false) {

    }

    void LaserGrid::update(const sensor_msgs::LaserScan &msg) {

        m_scan = msg;

        // reload
        if (msg.range_max != m_range_max) {
            m_range_max = msg.range_max;
            m_length = static_cast<size_t>(round(m_range_max / m_grid_resolution));
            auto m1 = cv::Mat(2 * m_length, 2 * m_length, CV_8SC1, cv::Scalar(0));
            m_grid = std::make_shared<cv::Mat>(m1);
            LeftUpOrigin = eigen_util::TransformationMatrix2d<float>(-m_range_max, -m_range_max, 0.0);
            LeftUpOrigin_ublas.set(-m_range_max, -m_range_max, 0.0);
            isOriginSet = true;
        }

        //clear background
        (*m_grid) = cv::Scalar(-100);

        // get laser points in origin frame
        occuPointsMat = m_scan.getXsYsMatrix();
        occuPointsMat_ublas = m_scan.getXsYsMatrix_ublas();

        m_scan.RangesVal() -= m_contour_offset;

        auto laserPoints = m_scan.getXsYsMatrix();
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
        size_t left = rect.x;
        size_t top = rect.y;
        size_t width = rect.width;
        size_t height = rect.height;
        size_t x_end = left + width;
        size_t y_end = top + height;

        // find cell inside contour

        std::vector<float> freePoints;
        std::vector<float> freePointsX;
        std::vector<float> freePointsY;

        freePoints.clear();
        freePointsX.clear();
        freePointsY.clear();

        for (size_t x = left; x < x_end; x++) {
            for (size_t y = top; y < y_end; y++) {

                signed char &v = (*m_grid).at<signed char>(y, x);

                if (1 == v) {
                    v = 0;

                    float xf = y * m_grid_resolution;
                    float yf = x * m_grid_resolution;

                    freePoints.push_back(xf);
                    freePointsX.push_back(xf);

                    freePoints.push_back(yf);
                    freePointsY.push_back(yf);

                }
            }
        }

#if 1
        // vector to eigen
        freePointsMat = eigen_util::createMatrix<float, Eigen::ColMajor>(&(freePoints[0]), 2,
                                                                         freePoints.size() / 2);


        freePointsMat = LeftUpOrigin * freePointsMat;
#endif

        // vector to ublas array
        freePointsMat_ublas = ublas::scalar_matrix<float>(3, freePointsX.size(), 1);

        std::copy(std::begin(freePointsX), std::end(freePointsX), freePointsMat_ublas.begin2() + 0);

        std::copy(std::begin(freePointsY), std::end(freePointsY), freePointsMat_ublas.begin2() + freePointsX.size());


        ublas::noalias(freePointsMat_ublas) = ublas::prod(LeftUpOrigin_ublas.matrix(), freePointsMat_ublas);


    }

    Eigen::MatrixXf &LaserGrid::getFreePointsMat() {

        return freePointsMat;
    }

    ublas::matrix<float> &LaserGrid::getFreePointsMat_ublas() {

        return freePointsMat_ublas;
    }

    Eigen::MatrixXf &LaserGrid::getOccuPointsMat() {
        return occuPointsMat;

    }

    ublas::matrix<float> &LaserGrid::getOccuPointsMat_ublas() {
        return occuPointsMat_ublas;

    }



    void LaserGrid::createBeam() {

        size_t sz = m_scan.ranges.size();
        float range_offset = 0.5;
        m_beam.clear();
        m_beam_ublas.resize(sz);
        m_beam_index.resize(sz);
        size_t index = 0;
        std::vector<float> pointOnBeamX;
        std::vector<float> pointOnBeamY;
        pointOnBeamX.clear();
        pointOnBeamY.clear();
#if 0
        for (size_t i = 0; i < sz; i++){

            float r_min = (m_scan.ranges[i] - range_offset < 0.0 ) ? m_scan.ranges[i] - range_offset : 0.0;
            float r_max = (m_scan.ranges[i] + range_offset > m_scan.range_max) ? m_scan.ranges[i] + range_offset :  m_scan.range_max;
            std::vector<float> pointOnBeam;


            for (auto r = r_min; r < r_max; r += 0.05){
                pointOnBeamX.push_back(r*m_scan.cache_cos[i]);
                pointOnBeamY.push_back(r*m_scan.cache_sin[i]);

            }
#if 0
            Eigen::Map<Eigen::MatrixXf> pointMat(pointOnBeam.data(),2,pointOnBeam.size()/2);
            m_beam.emplace_back(pointMat);
#endif
            m_beam_index[i] = index;
            index ++;



        }
#endif

        std::valarray<float> cache_cos = m_grid_resolution * m_scan.cache_cos;
        std::valarray<float> cache_sin = m_grid_resolution * m_scan.cache_sin;

        m_beam_matrix = ublas::scalar_matrix<float>(3, sz, 1);;
        std::copy(std::begin(cache_cos), std::end(cache_cos), m_beam_matrix.begin2() + 0);

        std::copy(std::begin(cache_sin), std::end(cache_sin), m_beam_matrix.begin2() + cache_sin.size());


    }


    //==========================
    // MapGrid
    MapGrid::MapGrid(float grid_resolution, float grid_height) :
            m_grid_resolution(grid_resolution),
            m_grid_height(grid_height),
            isOriginSet(false) {
        assert(m_grid_height > 0);

    }

    void MapGrid::setOrigin(double x_, double y_, double yaw_, OriginType type_) {

        eigen_util::TransformationMatrix2d<float> origin(x_, y_, yaw_);
        ublas_util::Transformation2d origin_ublas(x_, y_, yaw_);
        if (type_ == OriginType::LeftBottom) {
            eigen_util::TransformationMatrix2d<float> bottomToUp(0.0, m_grid_height, -0.5 * M_PI);
            ublas_util::Transformation2d bottomToUp_ublas(0.0, m_grid_height, -0.5 * M_PI);

            LeftUpOrigin = origin * bottomToUp;
            ublas::noalias(LeftUpOrigin_ublas.matrix()) = ublas::prod(origin_ublas.matrix(), bottomToUp_ublas.matrix());

        }

        if (type_ == OriginType::LeftUp) {
            LeftUpOrigin = origin;
            LeftUpOrigin_ublas = origin_ublas;
        }

        isOriginSet = true;
    }

    eigen_util::TransformationMatrix2d<float> &MapGrid::getOriginMatrix() {
        return LeftUpOrigin;

    }

    void MapGrid::dispay(const ublas::matrix<float> &m) {
        cv::Mat mat(size_t(round(m_grid_height / m_grid_resolution)), size_t(round(m_grid_height / m_grid_resolution)),
                    CV_8UC1, cv::Scalar(0));


        for (size_t i = 0; i < m.size2(); i++) {
            cv::Point p(round(m(1, i) / 0.05), round(m(0, i) / 0.05));
            mat.at<uchar>(p) = 255;
        }

    }
}

