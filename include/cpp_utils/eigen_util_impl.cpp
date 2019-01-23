//
// Created by waxz on 19-1-18.
//

#include "eigen_util.cpp"

namespace eigen_util {

    template Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    createMatrix<float, Eigen::RowMajor>(float *p, size_t rows, size_t cols);

    template Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>
    createMatrix<float, Eigen::ColMajor>(float *p, size_t rows, size_t cols);

    template Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    createMatrix<double, Eigen::RowMajor>(double *p, size_t rows, size_t cols);

    template Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>
    createMatrix<double, Eigen::ColMajor>(double *p, size_t rows, size_t cols);

    template Eigen::Matrix<double, 2, 2>
    getRotation2dFromYaw(double yaw, bool cw);

    template Eigen::Matrix<float, 2, 2>
    getRotation2dFromYaw(float yaw, bool cw);

    template Eigen::Matrix<double, 3, 3>
    getTransform2d(double x, double y, double yaw);

    template Eigen::Matrix<float, 3, 3>
    getTransform2d(float x, float y, float yaw);

    template TransformationMatrix2d<double>::TransformationMatrix2d();

    template TransformationMatrix2d<double>::TransformationMatrix2d(double, double, double);

    template TransformationMatrix2d<float>::TransformationMatrix2d();

    template TransformationMatrix2d<float>::TransformationMatrix2d(float, float, float);

    template TransformationMatrix2d<float> TransformationMatrix2d<float>::inverse();

    template TransformationMatrix2d<double> TransformationMatrix2d<double>::inverse();

    template TransformationMatrix2d<double> &
    TransformationMatrix2d<double>::operator=(const TransformationMatrix2d<double> &matrix);

    template TransformationMatrix2d<float> &
    TransformationMatrix2d<float>::operator=(const TransformationMatrix2d<float> &matrix);


    template TransformationMatrix2d<float> &
    TransformationMatrix2d<float>::operator=(const Eigen::Matrix<float, 3, 3> &matrix);


    template TransformationMatrix2d<double> &
    TransformationMatrix2d<double>::operator=(const Eigen::Matrix<double, 3, 3> &matrix);

    template Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
    TransformationMatrix2d<double>::operator*(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &x);

    template Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
    TransformationMatrix2d<float>::operator*(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &x);

    template TransformationMatrix2d<double>
    TransformationMatrix2d<double>::operator*(TransformationMatrix2d<double> &rv);

    template TransformationMatrix2d<float>
    TransformationMatrix2d<float>::operator*(TransformationMatrix2d<float> &rv);


    template Eigen::Matrix<float, 3, 3> &
    TransformationMatrix2d<float>::matrix();


    template Eigen::Matrix<double, 3, 3> &
    TransformationMatrix2d<double>::matrix();

}