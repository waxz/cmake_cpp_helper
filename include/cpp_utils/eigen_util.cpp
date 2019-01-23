//
// Created by waxz on 19-1-18.
//

#include "eigen_util.h"

namespace eigen_util {

    template<typename T, int M>
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, M> createMatrix(T *p, size_t rows, size_t cols) {

//        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> res;
        Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, M>> res(p, rows, cols);
        return res;

    };


    template<typename T>
    inline Eigen::Matrix<T, 2, 2> getRotation2dFromYaw(T yaw, bool cw) {
        //yaw is a counterclockwise rotation of alpha about the zaxis
        Eigen::Matrix<T, 2, 2> rotation;

        rotation << cos(yaw), ((cw) ? -1 : 1) * sin(yaw),
                ((cw) ? 1 : -1) * sin(yaw), cos(yaw);

        return rotation;

        /*
     * http://planning.cs.uiuc.edu/node102.html
     * A yaw is a counterclockwise rotation of alpha about the zaxis. The rotation matrix is given by


     R_z(alpha) = [
                  cos(alpha) -sin(alpha) 0
                  sin(alpha) sin(alpha)  0
                  0          0           1
                  ]


     * */

    }

//    inline Eigen::Matrix3d getTransform2d(double x, double y, double yaw) {
//        //http://f1tenth.org/slides/l3-1.pdf
//        Eigen::Matrix3d trans;
//        trans.setZero();
//        Eigen::Vector3d t;
//        t << x, y, 1.0;
//        trans.block(0, 0, 2, 2) = getRotation2dFromYaw(yaw);
//        trans.block(0, 2, 3, 1) = t;
//
//        return trans;
//
//
//    }

    template<typename T>
    inline Eigen::Matrix<T, 3, 3> getTransform2d(T x, T y, T yaw) {
        //http://f1tenth.org/slides/l3-1.pdf
        Eigen::Matrix<T, 3, 3> trans;
        trans.setZero();
        Eigen::Matrix<T, 3, 1> t;
        t << x, y, 1.0;
        trans.block(0, 0, 2, 2) = getRotation2dFromYaw(yaw);
        trans.block(0, 2, 3, 1) = t;

        return trans;


    }

    template<typename T1>
    TransformationMatrix2d<T1>::TransformationMatrix2d() {
        trans_3d = getTransform2d(static_cast<T1>(0.0), static_cast<T1>(0.0), static_cast<T1>(0.0));
    }

    template<typename T>
    TransformationMatrix2d<T>::TransformationMatrix2d(T x, T y, T yaw) {
        trans_3d = getTransform2d(x, y, yaw);
    }


    template<typename T>
    TransformationMatrix2d<T> &TransformationMatrix2d<T>::operator=(const Eigen::Matrix<T, 3, 3> &matrix) {
        trans_3d = matrix;
        return *this;
    }

    template<typename T>
    TransformationMatrix2d<T>::TransformationMatrix2d(const Eigen::Matrix<T, 3, 3> &matrix) {
        trans_3d = matrix;
    }

    template<typename T>
    TransformationMatrix2d<T> &TransformationMatrix2d<T>::operator=(const TransformationMatrix2d &matrix) {
        trans_3d = matrix.trans_3d;
        return *this;
    }


    ////
    template<typename T>
    Eigen::Matrix<T, 2, 1> TransformationMatrix2d<T>::operator*(const Eigen::Matrix<T, 2, 1> &x) {

        Eigen::Matrix<T, 3, 1> x_;
        x_.setOnes();
        x_.segment(0, 2) = x;


        return (trans_3d * x_).head(2);

    }

    template<typename T>
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>
    TransformationMatrix2d<T>::operator*(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &x) {
        // x : 2d col vector 2*n
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> x_(3, x.cols());
        x_.setOnes();
        x_.block(0, 0, 2, x.cols()) = x;


        return (trans_3d * x_).block(0, 0, 2, x.cols());
    }

    template<typename T>
    Eigen::Matrix<T, 3, 3> &TransformationMatrix2d<T>::matrix() {

        return trans_3d;


    }

    template<typename T>
    TransformationMatrix2d<T> TransformationMatrix2d<T>::operator*(TransformationMatrix2d &rv) {
        TransformationMatrix2d res(this->trans_3d * rv.matrix());
        return res;
    }

    template<typename T>
    TransformationMatrix2d<T> TransformationMatrix2d<T>::inverse() {
        TransformationMatrix2d res(this->trans_3d.inverse());
        return res;
    }


    inline Eigen::MatrixXd getDistMatrix(const Eigen::MatrixXd &m) {

        int sz = m.cols();
        Eigen::MatrixXd M(sz, sz);
        M.setZero();
        for (int i = 0; i < sz; i++) {
            for (int j = i + 1; j < sz; j++) {
                M(i, j) = (m.col(i) - m.col(j)).norm();
            }
        }

        return M;
    }
}