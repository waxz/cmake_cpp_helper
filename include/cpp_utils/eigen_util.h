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


    inline Eigen::Matrix2d getRotation2dFromYaw(double yaw, bool cw = true);

    inline Eigen::Matrix3d getTransform2d(double x, double y, double yaw);

    class TransformationMatrix2d {
    private:
        Eigen::Matrix3d trans_3d;
        Eigen::Matrix4d trans_4d;

    public:

        TransformationMatrix2d(double x, double y, double yaw);

        TransformationMatrix2d &operator=(const Eigen::Matrix3d &matrix);

        TransformationMatrix2d &operator=(const TransformationMatrix2d &matrix);

        explicit TransformationMatrix2d(const Eigen::Matrix3d &matrix);

        Eigen::Vector2d operator*(const Eigen::Vector2d &x);

        Eigen::MatrixXd operator*(const Eigen::MatrixXd &x);

        Eigen::Matrix3d &matrix();

        TransformationMatrix2d operator*(TransformationMatrix2d rv);

        TransformationMatrix2d inverse();

    };


    // each vector represent a point
    // caculate distance between each point

    inline Eigen::MatrixXd getDistMatrix(const Eigen::MatrixXd &m);
}


#endif //LASER_LINE_EXTRACTION_EIGEN_UTIL_H
