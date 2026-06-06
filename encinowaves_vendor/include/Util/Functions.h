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

#ifndef _EncinoWaves_Util_Functions_h_
#define _EncinoWaves_Util_Functions_h_

#include "Foundation.h"

namespace EncinoWaves {
namespace Util {

//-*****************************************************************************
//-*****************************************************************************
// SHOULDER FUNCTION - this implements a curve that's basically just 'x'
// for x <= 0.5, and then asymptotically approaches 1 for any value greater
// than 0.5. This can be used to implement a soft clamp.
template <typename T>
T shoulder(const T &x) {
  static constexpr T Half = 0.5;
  static constexpr T Two  = 2.0;

  if (x <= Half) {
    return x;
  } else {
    return Half + (Half * std::tanh(Two * (x - Half)));
  }
}

//-*****************************************************************************
// Limit value to never get larger than maxVal, using shoulder curve.
template <typename T>
T shoulderLimit(const T &x, const T &maxVal) {
  return maxVal * shoulder(x / maxVal);
}

//-*****************************************************************************
// A collection of simple functions borrowed from various places
// to make this library self-sufficient.
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
const T &clamp(const T &val, const T &lo, const T &hi) {
  return val < lo ? lo : val > hi ? hi : val;
}

//-*****************************************************************************
// You'd think instead of doing the a*(1-t) + b*t, it'd be faster
// and one less multiply to do a + (b-a)*t, right? Bad! Increases floating
// point exception occurances. Same as LERP
template <class T, class T2>
T mix(const T &a, const T &b, const T2 &interp) {
  static constexpr T2 one = ((T2)1);
  return (a * (one - interp)) + (b * interp);
}

//-*****************************************************************************
template <class T>
const T &sign(const T &val) {
  static constexpr T positive = ((T)1);
  static constexpr T negative = ((T)-1);
  static constexpr T zero     = ((T)0);
  return val < zero ? negative : val > zero ? positive : zero;
}

//******************************************************************************
template <class T>
T radians(const T &deg) {
  static constexpr T TPI       = ((T)M_PI);
  static constexpr T ONEEIGHTY = ((T)180);
  return TPI * (deg / ONEEIGHTY);
}

//******************************************************************************
template <class T>
T degrees(const T &rad) {
  static constexpr T TPI       = ((T)M_PI);
  static constexpr T ONEEIGHTY = ((T)180);
  return ONEEIGHTY * (rad / TPI);
}

//-*****************************************************************************
// f(x) = c0 + c1*x + c2*x^2 + c3*x^3
template <typename T1, typename T2>
T1 cubic(const T1 &coeff0, const T1 &coeff1, const T1 &coeff2, const T1 &coeff3,
         const T2 &t) {
  return coeff0 + t * (coeff1 + t * (coeff2 + t * coeff3));
}

//-*****************************************************************************
// Hermite function.
// f(x) = (c0) + (c1)*x + (c2)*x^2 + (c3)*x^3
// f'(x) = (c1) + 2(c2)x + 3(c3)x^2
// f(0) = A        ->        A = (c0)
// f(1) = B        ->        B = (c0) + (c1) + (c2) + (c3)
// f'(0) = sA      ->       sA = (c1)
// f'(1) = sB      ->       sB = (c1) + 2(c2) + 3(c3)
// B - A - sA = c2 + c3 = "M"
// sB - sA = 2(c2) + 3(c3) = "N"
// 2c2 + 2c3 = 2M
// 2c2 + 3c3 = N
// c3 = N - 2M
// c2 = M - c3
template <typename T1, typename T2>
T1 hermite(const T1 &pointA, const T1 &pointB, const T1 &slopeA,
           const T1 &slopeB, const T2 &t) {
  T1 M  = pointB - pointA - slopeA;
  T1 M2 = M * ((T2)2);
  T1 N  = slopeB - slopeA;
  T1 c3 = N - M2;
  return cubic(pointA, slopeA, M - c3, c3, t);
}

//******************************************************************************
// Smoothstep function
// Goes from 0 to 1.
template <class T>
T smoothstep(const T &t) {
  if (t <= (T)0) {
    return (T)0;
  } else if (t >= (T)1) {
    return (T)1;
  } else {
    return t * t * ((T)3 - (t * (2)));
  }
}

//******************************************************************************
template <class T>
T smoothstep(const T &edge0, const T &edge1, const T &t) {
  return smoothstep((t - edge0) / (edge1 - edge0));
}

//-*****************************************************************************
template <class T>
const T &linstep(const T &t) {
  static constexpr T t0 = ((T)0);
  static constexpr T t1 = ((T)1);

  return clamp(t, t0, t1);
}

//-*****************************************************************************
template <class T>
T linstep(const T &edge0, const T &edge1, const T &t) {
  return linstep((t - edge0) / (edge1 - edge0));
}

//-*****************************************************************************
// This wraps the input x into the range [lowerBound,upperBound], with periodic
// repeat. So, for example, for ints, a section of the number line, for the
// lowerbound of 2 and an upper bound of 5:
//
// INPUT X: -3 -2 -1 0 1 2 3 4 5 6 7 8 9 10
//  OUTPUT:  5  2  3 4 5 2 3 4 5 2 3 4 5  2
// template <typename T>
// T wrap( T i_x, T i_lowerBound, T i_upperBound );

// This version wraps x into the periodic range [0,n). It is the same
// as calling wrap( x, 0, n-1 ), for int-likes,
// and wrap( x, 0, n ) for float-likes
// template <typename T>
// T wrap( T x, T n );

//-*****************************************************************************
// This assumes an integer type for T.
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

//-*****************************************************************************
template <class T>
T sqr(const T &a) {
  return a * a;
}

//-*****************************************************************************
template <typename T>
T cube(const T &a) {
  return a * a * a;
}

//-*****************************************************************************
//-*****************************************************************************
// BOX INTERSECTION STUFF
//-*****************************************************************************
//-*****************************************************************************

template <typename T>
Imath::Box<Imath::Vec3<T>> BoxIntersection(
  const Imath::Box<Imath::Vec3<T>> &i_a,
  const Imath::Box<Imath::Vec3<T>> &i_b) {
  return Imath::Box<Imath::Vec3<T>>(
    Imath::Vec3<T>(std::max(i_a.min.x, i_b.min.x),
                   std::max(i_a.min.y, i_b.min.y),
                   std::max(i_a.min.z, i_b.min.z)),

    Imath::Vec3<T>(std::min(i_a.max.x, i_b.max.x),
                   std::min(i_a.max.y, i_b.max.y),
                   std::min(i_a.max.z, i_b.max.z)));
}

template <typename T>
Imath::Box<Imath::Vec3<T>> BoxIntersection(
  const Imath::Box<Imath::Vec3<T>> &i_a, const Imath::Box<Imath::Vec3<T>> &i_b,
  const Imath::Box<Imath::Vec3<T>> &i_c) {
  return BoxIntersection<T>(BoxIntersection<T>(i_a, i_b), i_c);
}

}  // namespace Util
}  // namespace EncinoWaves

#endif
