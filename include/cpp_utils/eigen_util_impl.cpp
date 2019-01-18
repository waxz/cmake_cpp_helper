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

}