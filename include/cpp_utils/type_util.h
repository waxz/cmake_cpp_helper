//
// Created by waxz on 19-1-18.
//

#ifndef DEMO_TYPE_UTIL_H
#define DEMO_TYPE_UTIL_H

#include <vector>
#include <valarray>
#include <iostream>

namespace type_util {
    template<typename T>
    struct Point {
        T x;
        T y;
        T z;

        Point(T x_, T y_, T z_ = 0) {
            x = x_;
            y = y_;
            z = z_;
        }
    };

}

#endif //DEMO_TYPE_UTIL_H
