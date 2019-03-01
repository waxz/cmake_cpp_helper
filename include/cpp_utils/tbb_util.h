//
// Created by waxz on 19-2-21.
//

#ifndef DEMO_TBB_UTIL_H
#define DEMO_TBB_UTIL_H

#include <tbb/tbb.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/scalable_allocator.h>

#if 0
// simple useage

    tbb::parallel_for(tbb::blocked_range<int>(1,10),
                      [&](tbb::blocked_range<int> r){
                          for (auto it = r.begin(); it != r.end(); it++){
                              // do some work
                              std::cout << "- " << it << std::endl;

                          }

                      },
                      tbb::simple_partitioner());
#endif

#endif //DEMO_TBB_UTIL_H
