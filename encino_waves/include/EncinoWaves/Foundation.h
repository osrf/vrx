//-*****************************************************************************
// Copyright 2015 Christopher Jon Horvath
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//-*****************************************************************************

//-*****************************************************************************
// The basic architecture of these Waves is based on the TweakWaves application
// written by Chris Horvath for Tweak Films in 2001.  This, in turn, was based
// on the SIGGRAPH papers and courses by Jerry Tessendorf, and by the paper
// "A Simple Fluid Solver based on the FTT" by Jos Stam.
//
// The TMA, JONSWAP, and Pierson Moskowitz Wave Spectra, as well as the
// directional spreading functions are formulated based on the descriptions
// given in "Ocean Waves: The Stochastic Approach",
// by Michel K. Ochi, published by Cambridge Ocean Technology Series, 1998,2005.
//
// This library is written as a working implementation of the paper:
// Christopher J. Horvath. 2015.
// Empirical directional wave spectra for computer graphics.
// In Proceedings of the 2015 Symposium on Digital Production (DigiPro '15),
// Los Angeles, Aug. 8, 2015, pp. 29-39.
//-*****************************************************************************

#ifndef _EncinoWaves_Foundation_h_
#define _EncinoWaves_Foundation_h_

#include <Util/All.h>

#include <ImathVec.h>
#include <ImathBox.h>
#include <ImathMath.h>
#include <ImathFun.h>

// Note: <fftw3.h> dropped — see ./FftwWrapper.h for the Eigen-backed
// drop-in replacement that lifts EncinoWaves out of FFTW's GPL.

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <complex>
#include <cmath>

#include <memory>
#include <thread>
#include <type_traits>
#include <random>
#include <cstdint>
#include <functional>

#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include <tbb/blocked_range2d.h>
#include <tbb/mutex.h>

#include <boost/format.hpp>

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <sys/types.h>
#include <unistd.h>

namespace EncinoWaves {

using namespace Util;

//-*****************************************************************************
extern tbb::mutex g_printMutex;

#define EWAV_MUTEX_PRINT(TEXT)                  \
  do {                                          \
    tbb::mutex::scoped_lock lock(g_printMutex); \
    std::cout << TEXT;                          \
  } while (0)

//-*****************************************************************************
template <typename T>
struct singular_value_type;

#define SINGULAR_VALUE_TYPE(A, B) \
  template <>                     \
  struct singular_value_type<A> { \
    typedef B type;               \
  };

SINGULAR_VALUE_TYPE(float, float);
SINGULAR_VALUE_TYPE(double, double);
SINGULAR_VALUE_TYPE(std::complex<float>, float);
SINGULAR_VALUE_TYPE(std::complex<double>, float);
SINGULAR_VALUE_TYPE(Imath::Vec2<float>, float);
SINGULAR_VALUE_TYPE(Imath::Vec2<double>, double);
SINGULAR_VALUE_TYPE(Imath::Vec3<float>, float);
SINGULAR_VALUE_TYPE(Imath::Vec3<double>, double);

#undef SINGULAR_VALUE_TYPE

//-*****************************************************************************
static int PowerOfTwo(int i_power) {
  if (i_power <= 0) {
    return 1;
  } else if (i_power >= 30) {
    return 0x1 << 30;
  } else {
    return (0x1 << i_power);
  }
}
#if 0

//-*****************************************************************************
// This wraps the input x into the range [lowerBound,upperBound], with periodic
// repeat. So, for example, for ints, a section of the number line, for the
// lowerbound of 2 and an upper bound of 5:
//
// INPUT X: -3 -2 -1 0 1 2 3 4 5 6 7 8 9 10
//  OUTPUT:  5  2  3 4 5 2 3 4 5 2 3 4 5  2
// template <typename T>
//  T wrap( T i_x, T i_lowerBound, T i_upperBound );

// This version wraps x into the periodic range [0,n). It is the same
// as calling wrap( x, 0, n-1 ), for int-likes,
// and wrap( x, 0, n ) for float-likes
// template <typename T>
//  T wrap( T x, T n );

//-*****************************************************************************
// This assumes a signed integer type for T.
template <typename T>
typename std::enable_if<std::is_integral<T>::value && std::is_signed<T>::value,
                        T>::type
wrap(T i_x, T i_lowerBound, T i_upperBound) {
  const T rangeSize = i_upperBound - i_lowerBound + 1;

  if (i_x < i_lowerBound) {
    i_x += rangeSize * (((i_lowerBound - i_x) / rangeSize) + 1);
  }

  return i_lowerBound + (i_x - i_lowerBound) % rangeSize;
}

//-*****************************************************************************
template <typename T>
typename std::enable_if<std::is_integral<T>::value && std::is_signed<T>::value,
                        T>::type
wrap(T i_x, T N) {
  if (i_x < 0) {
    i_x += N * (((-i_x) / N) + 1);
  }

  return i_x % N;
}

//-*****************************************************************************
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, T>::type wrap(T x,
                                                                        T n) {
  return x - (n * std::floor(x / n));
}

//-*****************************************************************************
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, T>::type wrap(T x,
                                                                        T lb,
                                                                        T ub) {
  return lb + wrap(x - lb, ub - lb);
}

//******************************************************************************
// Smoothstep function
// Goes from 0 to 1.
template <typename T>
T smoothstep(const T& t) {
  if (t <= T(0)) {
    return T(0);
  } else if (t >= T(1)) {
    return T(1);
  } else {
    return t * t * (T(3) - (t * T(2)));
  }
}

//******************************************************************************
template <typename T>
T smoothstep(const T& edge0, const T& edge1, const T& t) {
  return smoothstep((t - edge0) / (edge1 - edge0));
}

//-*****************************************************************************
template <typename T>
const T& linstep(const T& t) {
  return clamp(t, T(0), T(1));
}

//-*****************************************************************************
template <typename T>
T linstep(const T& edge0, const T& edge1, const T& t) {
  return linstep((t - edge0) / (edge1 - edge0));
}

//-*****************************************************************************
template <typename T>
T sqr(const T& i_t) {
  return i_t * i_t;
}

template <typename T>
T cube(const T& i_t) {
  return i_t * i_t * i_t;
}

template <typename T, typename InterpT>
T mix(const T& a, const T& b, InterpT t) {
  return (a * (InterpT(1) - t)) + (b * t);
}
#endif

}  // namespace EncinoWaves

#endif
