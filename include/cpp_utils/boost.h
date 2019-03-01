//
// Created by waxz on 19-2-22.
//

#ifndef DEMO_BOOST_H
#define DEMO_BOOST_H

#include <boost/multiprecision/gmp.hpp>  // Defines the wrappers around the GMP library's types
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>


#if 0
boost::random::mt19937 rng;         // produces randomness out of thin air
// see pseudo-random number generators
boost::random::uniform_real_distribution<> six(8.0, 10.0);
// distribution that maps to 1..6
// see random number distributions

for (int i = 0; i < 122; i++) {
//        int x = six(rng);

    ranges.push_back(six(rng));
}

#endif

#endif //DEMO_BOOST_H
