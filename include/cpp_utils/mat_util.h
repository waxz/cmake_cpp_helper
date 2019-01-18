//
// Created by waxz on 19-1-16.
//
/* matrix transform  between cv::Mat and Eigen::Matrix
 *
 * */

#ifndef DEMO_MAT_UTIL_H
#define DEMO_MAT_UTIL_H

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>


namespace mat_util {

    // cv::Mat to Eigen::Matrix
    template<typename T>
    inline Eigen::Matrix <T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    CvToEigen(const cv::Mat &mat) {
        try {
            Eigen::Map <Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> matrix(mat.ptr<T>(),
                                                                                                  mat.rows, mat.cols *
                                                                                                            mat.channels());
        }
        catch (...) {
            throw std::logic_error("CvToEigen error ", mat.type());
        }


        return matrix;
    }


}
#endif //DEMO_MAT_UTIL_H
