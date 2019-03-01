//
// Created by waxz on 19-2-21.
//

#ifndef DEMO_MATRIX_H
#define DEMO_MATRIX_H

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <boost/numeric/ublas/operation.hpp>
#include <boost/numeric/ublas/operation_blocked.hpp>

#include <Eigen/Sparse>
#include <Eigen/Dense>


#include <vector>
#include <string>


#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/array.hpp>

namespace ublas = boost::numeric::ublas;

namespace boost {
    namespace serialization {

        /*==============================
         * Eigen
         *
         * */
        template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
        void save(Archive &ar, const Eigen::Matrix <_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &m,
                  const unsigned int version) {
            int rows = m.rows(), cols = m.cols();
            ar & rows;
            ar & cols;
            ar & boost::serialization::make_array(m.data(), rows * cols);
        }

        template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
        void load(Archive &ar, Eigen::Matrix <_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &m,
                  const unsigned int version) {
            int rows, cols;
            ar & rows;
            ar & cols;
            m.resize(rows, cols);
            ar & boost::serialization::make_array(m.data(), rows * cols);
        }

        template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
        void serialize(Archive &ar, Eigen::Matrix <_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &m,
                       const unsigned int version) {
            split_free(ar, m, version);
        }

        /*========================
         * ublas
         * */
        template<class Archive, typename _Scalar, typename _F, typename _A>
        void save(Archive &ar, const ublas::matrix <_Scalar, _F, _A> &m, const unsigned int version) {
            int rows = m.size1(), cols = m.size2();
            ar & rows;
            ar & cols;
            ar & boost::serialization::make_array(m.data().begin(), rows * cols);
        }

        template<class Archive, typename _Scalar, typename _F, typename _A>
        void load(Archive &ar, ublas::matrix <_Scalar, _F, _A> &m, const unsigned int version) {
            int rows, cols;
            ar & rows;
            ar & cols;
            m.resize(rows, cols);
            // ublas storage pointer
            ar & boost::serialization::make_array(m.data().begin(), rows * cols);
        }

        template<class Archive, typename _Scalar, typename _F, typename _A>
        void serialize(Archive &ar, ublas::matrix <_Scalar, _F, _A> &m, const unsigned int version) {
            split_free(ar, m, version);


        }


    }
#if 0
    //==============================
    // how to use

    // save to text file
    std::string save_path = "m.txt";
    std::ofstream ofs(save_path);

    boost::archive::text_oarchive oar(ofs);
    oar << matrix;

    // load from text file
    std::string load_path = "m2.txt";

    std::ifstream ifs(load_path);
    boost::archive::text_iarchive iar(ifs);

    iar >> matrix;

#endif


}


#endif //DEMO_MATRIX_H
