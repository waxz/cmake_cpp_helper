//
// Created by waxz on 19-2-21.
//

#ifndef DEMO_UBLAS_H
#define DEMO_UBLAS_H

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>

#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/operation_blocked.hpp>

namespace ublas = boost::numeric::ublas;

namespace ublas_util {

    // basic type
    typedef ublas::matrix<float, ublas::column_major> Matrixcf;
    typedef ublas::matrix<float, ublas::row_major> Matrixrf;

    typedef ublas::matrix<double, ublas::column_major> Matrixcd;
    typedef ublas::matrix<double, ublas::row_major> Matrixrd;

    typedef ublas::identity_matrix<float> IdentityMatrixf;
    typedef ublas::identity_matrix<double> IdentityMatrixd;

    typedef ublas::zero_matrix<float> ZeroMatrixf;
    typedef ublas::zero_matrix<double> ZeroMatrixd;

    // scalar_matrix
#if 0
    scalar_matrix<float> m(size1, size2, value);
#endif

    // copy data from vector
    // column_major  matrix
    template<typename T>
    void copydata(T stl_container, Matrixcf &m) {
        std::copy(stl_container.begin(), stl_container.end(), m.begin1());

    }
    // from valarray


    // row_major matrix
    template<typename T>
    void copydata(T stl_container, Matrixrf &m) {
        std::copy(stl_container.begin(), stl_container.end(), m.begin2());

    }
    // from valarray
#if 0
    std::copy(std::begin(mm2), std::end(mm2), m1.begin2() + 0);

    std::copy(std::begin(mm1), std::end(mm1), m1.begin2() + 3);
#endif

    // get pointer
#if 0
    ublas_matrix.data().begin();
#endif

    // operation
    // product
#if 0
    ublas::matrix<float> m3(2,800);

    ublas::noalias(m3) = ublas::prod(m1,m2);
#endif


    // transformation
    class Transformation2d {
    private:
        float m_x;
        float m_y;
        float m_yaw;
        ublas::matrix<float> m_mat;

        void createMat();

    public:
        Transformation2d();

        Transformation2d(float x, float y, float yaw);

        ublas::matrix<float> &matrix() {
            return m_mat;

        };

        void set(float, float, float);

        ublas::matrix<float> inverse();

        float getYaw() {
            return atan2(m_mat(1, 0), m_mat(0, 0));
        }

        float getX() {
            return m_mat(0, 2);
        }

        float getY() {
            return m_mat(1, 2);

        }

    };

}


#endif //DEMO_UBLAS_H
