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


    inline Eigen::Matrix2d getRotation2dFromYaw(double yaw, bool cw) {
        //yaw is a counterclockwise rotation of alpha about the zaxis
        Eigen::Matrix2d rotation;

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

    inline Eigen::Matrix3d getTransform2d(double x, double y, double yaw) {
        //http://f1tenth.org/slides/l3-1.pdf
        Eigen::Matrix3d trans;
        trans.setZero();
        Eigen::Vector3d t;
        t << x, y, 1.0;
        trans.block(0, 0, 2, 2) = getRotation2dFromYaw(yaw);
        trans.block(0, 2, 3, 1) = t;

        return trans;


    }


    TransformationMatrix2d::TransformationMatrix2d() {
        trans_3d = getTransform2d(0, 0, 0);
    }

    TransformationMatrix2d::TransformationMatrix2d(double x, double y, double yaw) {
        trans_3d = getTransform2d(x, y, yaw);
    }


    TransformationMatrix2d &TransformationMatrix2d::operator=(const Eigen::MatrixXd &matrix) {
        trans_3d = matrix;
        return *this;
    }

    TransformationMatrix2d::TransformationMatrix2d(const Eigen::MatrixXd &matrix) {
        trans_3d = matrix;
    }

    TransformationMatrix2d &TransformationMatrix2d::operator=(const TransformationMatrix2d &matrix) {
        trans_3d = matrix.trans_3d;
        return *this;
    }


    ////

    Eigen::Vector2d TransformationMatrix2d::operator*(const Eigen::Vector2d &x) {

        Eigen::Vector3d x_;
        x_.setOnes();
        x_.segment(0, 2) = x;


        return (trans_3d * x_).head(2);

    }

    Eigen::MatrixXd TransformationMatrix2d::operator*(const Eigen::MatrixXd &x) {
        // x : 2d col vector 2*n
        Eigen::MatrixXd x_(3, x.cols());
        x_.setOnes();
        x_.block(0, 0, 2, x.cols()) = x;


        return (trans_3d * x_).block(0, 0, 2, x.cols());
    }


    Eigen::MatrixXd &TransformationMatrix2d::matrix() {

        return trans_3d;


    }

    TransformationMatrix2d TransformationMatrix2d::operator*(TransformationMatrix2d rv) {
        TransformationMatrix2d res(this->trans_3d * rv.matrix());
        return res;
    }

    TransformationMatrix2d TransformationMatrix2d::inverse() {
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