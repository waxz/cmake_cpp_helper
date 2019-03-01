//
// Created by waxz on 19-2-22.
//
#include "ublas.h"

namespace ublas_util {


    void Transformation2d::createMat() {

        // rotation
        float cos_yaw = cos(m_yaw);
        float sin_yaw = sin(m_yaw);
        m_mat(0, 0) = cos_yaw;
        m_mat(0, 1) = -sin_yaw;
        m_mat(1, 0) = sin_yaw;
        m_mat(1, 1) = cos_yaw;
        // transmition
        m_mat(0, 2) = m_x;
        m_mat(1, 2) = m_y;
        m_mat(2, 2) = 1.0;

    }

    Transformation2d::Transformation2d() : m_x(0.0), m_y(0.0), m_yaw(0.0), m_mat(ublas::zero_matrix<float>(3, 3)) {

        createMat();
    }

    Transformation2d::Transformation2d(float x, float y, float yaw) : m_x(x), m_y(y), m_yaw(yaw),
                                                                      m_mat(ublas::zero_matrix<float>(3, 3)) {

        createMat();
    }


    void Transformation2d::set(float x, float y, float yaw) {
        m_x = x;
        m_y = y;
        m_yaw = yaw;
        createMat();
    }

    ublas::matrix<float> Transformation2d::inverse() {
        ublas::matrix<float> Ainv = ublas::identity_matrix<float>(m_mat.size1());
        ublas::permutation_matrix<size_t> pm(m_mat.size1());
        ublas::lu_factorize(m_mat, pm);
        ublas::lu_substitute(m_mat, pm, Ainv);
        return Ainv;
    }

//    ublas::matrix<float>& Transformation2d::matrix(){
//        return m_mat;
//    }



}