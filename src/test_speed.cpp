//
// Created by waxz on 19-2-22.
//

#include <iostream>
#include <valarray>
#include <cpp_utils/ublas.h>
#include <cpp_utils/tbb_util.h>
#include <cpp_utils/time.h>
#include <vector>
#include <boost/array.hpp>
#include <cpp_utils/ublas.h>


int test_main() {

    std::cout << "hello" << std::endl;

    ublas_util::Transformation2d trans(0.5, 0.2, 0.5 * M_PI);

    std::cout << "m:\n" << trans.matrix() << std::endl;


    unsigned long dim = 3;

    ublas_util::Matrixrf m1 = ublas::scalar_matrix<float>(dim, dim, 1);

    std::valarray<float> mm1{1, 2, 3};
    std::valarray<float> mm2{4, 5, 7};
    std::valarray<float> mm3(6, 6);
    std::copy(std::begin(mm2), std::end(mm2), m1.begin2() + 0);

    std::copy(std::begin(mm1), std::end(mm1), m1.begin2() + 3);
    std::cout << "copy m:\n" << m1 << std::endl;
    m1(0, 0) = 1;
    m1(0, 1) = 1;
    m1(0, 2) = 1;
    m1(1, 0) = 1;
    m1(1, 1) = 1;
    m1(1, 2) = 1;
    m1(2, 0) = 1;
    m1(2, 1) = 1;
    m1(2, 2) = 1;



    //1. run speed test
    //2. choose best compiler flags
    ublas_util::Matrixcf m2 = ublas::identity_matrix<float>(dim, 800 * 20);
    ublas_util::Matrixcf m3 = ublas::identity_matrix<float>(dim, 800);

    std::vector<float> pv(1000 * 800 * 20, 0);


    time_util::Timer timer;
    timer.start();
    tbb::parallel_for(tbb::blocked_range<int>(1, 1000),
                      [&](tbb::blocked_range<int> r) {
                          for (auto it = r.begin(); it != r.end(); it++) {
                              // 1. local grid to global
                              ublas::matrix<float> m21(dim, 800 * 20);
                              ublas::matrix<float> m31(dim, 800);

                              ublas::noalias(m21) = ublas::prod(trans.matrix(), m2);
                              ublas::noalias(m31) = ublas::prod(trans.matrix(), m3);

                              for (unsigned long ss = 0; ss < 800 * 20; ss++) {
                                  pv[ss] = m21(0, ss) + m21(1, ss) + m21(2, ss);
                              }
                              for (unsigned long ss = 0; ss < 800; ss++) {
                                  pv[ss] = m31(0, ss) + m31(1, ss) + m31(2, ss);
                              }

                          }

                      },
                      tbb::simple_partitioner());

    timer.stop();
    std::cout << "test 1 , time: " << timer.elapsedSeconds() << std::endl;




    // test 2
    // 1. compute all matrix
    // 2. compute base and offset for each beam
    timer.start();
    std::vector<float> test_vec(800 * 20, 1);
    tbb::parallel_for(tbb::blocked_range<int>(1, 1000),
                      [&](tbb::blocked_range<int> r) {
                          for (auto it = r.begin(); it != r.end(); it++) {
                              // 1. local grid to global
                              ublas::matrix<float> m21(dim, 800 * 20);

                              ublas::noalias(m21) = ublas::prod(trans.matrix(), m2);

                              for (int i = 0; i < 800 * 20; i++) {
                                  float px = m21(0, i);
                                  float py = m21(1, i);
//                                  test_vec[i] = px+py;
                                  py = px + py;
                              }

                          }

                      },
                      tbb::simple_partitioner());

    timer.stop();
    std::cout << "test 1, compute all array, time2: " << timer.elapsedSeconds() << std::endl;

    timer.start();

    tbb::parallel_for(tbb::blocked_range<int>(1, 1000),
                      [&](tbb::blocked_range<int> r) {
                          for (auto it = r.begin(); it != r.end(); it++) {
                              // 1. local grid to global
                              ublas::matrix<float> m31(dim, 800);

                              ublas::noalias(m31) = ublas::prod(trans.matrix(), m3);

                              // compute offset
                              // find occupied cell in
                              for (int i = 0; i < 800; i++) {
                                  for (int j = 0; j < 20; j++) {
//                                      float px = 0.8 + j * m31(0, i);
//                                      float py = 0.8 + j * m31(1, i);

//                                      test_vec[k] = px+py;
//                                      k++;
                                  }

                              }


                          }

                      },
                      tbb::simple_partitioner());

    timer.stop();
    std::cout << "test 2, compute base array, time2: " << timer.elapsedSeconds() << std::endl;

    timer.start();
    std::vector<float> a(1000 * 40, 1);

    tbb::parallel_for(tbb::blocked_range<int>(1, 1000),
                      [&](tbb::blocked_range<int> r) {
                          for (auto it = r.begin(); it != r.end(); it++) {


                              for (size_t i = 0; i < a.size(); i++) {
                                  a[i] = a[i] * 23.2;

                              }

                          }

                      },
                      tbb::simple_partitioner());

    timer.stop();
    std::cout << "=============\ntest 2, wirte to vector, time3: " << timer.elapsedSeconds() << std::endl;

    return 0;
}