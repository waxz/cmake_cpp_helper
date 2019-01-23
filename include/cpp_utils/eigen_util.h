//
// Created by waxz on 9/26/18.
//

#ifndef LASER_LINE_EXTRACTION_EIGEN_UTIL_H
#define LASER_LINE_EXTRACTION_EIGEN_UTIL_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eigen_util {
    template<typename T, int M>
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, M> createMatrix(T *p, size_t rows, size_t cols);

    template<typename T>
    inline Eigen::Matrix<T, 2, 2> getRotation2dFromYaw(T yaw, bool cw = true);

//    inline Eigen::Matrix3d getTransform2d(double x, double y, double yaw);

    template<typename T>
    inline Eigen::Matrix<T, 3, 3> getTransform2d(T x, T y, T yaw);

    template<typename T>
    class TransformationMatrix2d {
    private:
        Eigen::Matrix<T, 3, 3> trans_3d;
//        Eigen::MatrixXd trans_4d;

    public:
        TransformationMatrix2d();

        TransformationMatrix2d(T x, T y, T yaw);

        TransformationMatrix2d &operator=(const Eigen::Matrix<T, 3, 3> &matrix);

        TransformationMatrix2d &operator=(const TransformationMatrix2d &matrix);

        explicit TransformationMatrix2d(const Eigen::Matrix<T, 3, 3> &matrix);

        Eigen::Matrix<T, 2, 1> operator*(const Eigen::Matrix<T, 2, 1> &x);

        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>
        operator*(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &x);

        Eigen::Matrix<T, 3, 3> &matrix();

        TransformationMatrix2d operator*(TransformationMatrix2d &rv);

        TransformationMatrix2d inverse();

    };


    // each vector represent a point
    // caculate distance between each point

    inline Eigen::MatrixXd getDistMatrix(const Eigen::MatrixXd &m);
}


#endif //LASER_LINE_EXTRACTION_EIGEN_UTIL_H
