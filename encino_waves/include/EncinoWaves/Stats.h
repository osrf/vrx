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

#ifndef _EncinoWaves_Stats_h_
#define _EncinoWaves_Stats_h_

#include "Foundation.h"

namespace EncinoWaves {

//-*****************************************************************************
template <typename T>
struct MinMaxSum {
  T min;
  T max;
  T sum;
  MinMaxSum()
    : min(std::numeric_limits<T>::max())
    , max(-std::numeric_limits<T>::max())
    , sum(T(0)) {}

  MinMaxSum(const MinMaxSum<T>& i_s, tbb::split)
    : min(std::numeric_limits<T>::max())
    , max(-std::numeric_limits<T>::max())
    , sum(T(0)) {}

  void operator()(const tbb::blocked_range<const T*>& i_range) {
    T tmin = min;
    T tmax = max;
    T tsum = sum;
    for (const T* a = i_range.begin(); a != i_range.end(); ++a) {
      T ai = *a;
      if (ai < tmin) {
        tmin = ai;
      }
      if (ai > tmax) {
        tmax = ai;
      }
      tsum += ai;
    }
    min = tmin;
    max = tmax;
    sum = tsum;
  }
  void join(const MinMaxSum<T>& i_rhs) {
    min = std::min(min, i_rhs.min);
    max = std::max(max, i_rhs.max);
    sum += i_rhs.sum;
  }
};

//-*****************************************************************************
template <typename T>
void ParallelMinMaxSum(const T* i_values, std::size_t i_size, T& o_min,
                       T& o_max, T& o_sum) {
  MinMaxSum<T> mms;
  tbb::parallel_reduce(
    tbb::blocked_range<const T*>(i_values, i_values + i_size), mms);
  o_min = mms.min;
  o_max = mms.max;
  o_sum = mms.sum;
}

//-*****************************************************************************
template <typename T>
struct Sum {
  T value;
  Sum()
    : value(T(0)) {}
  Sum(const Sum<T>& i_s, tbb::split)
    : value(T(0)) {}
  void operator()(const tbb::blocked_range<const T*>& i_range) {
    T temp = value;
    for (const T* a = i_range.begin(); a != i_range.end(); ++a) {
      temp += *a;
    }
    value = temp;
  }
  void join(const Sum<T>& i_rhs) { value += i_rhs.value; }
};

//-*****************************************************************************
template <typename T>
T ParallelSum(const T* i_values, std::size_t i_size) {
  Sum<T> sum;
  tbb::parallel_reduce(
    tbb::blocked_range<const T*>(i_values, i_values + i_size), sum);
  return sum.value;
}

//-*****************************************************************************
template <typename T>
T ParallelMean(const T* i_values, std::size_t i_size) {
  typedef typename singular_value_type<T>::type denom_type;

  return ParallelSum(i_values, i_size) / denom_type(i_size);
}

//-*****************************************************************************
template <typename T>
struct VarianceSum {
  T value;
  T mean;
  explicit VarianceSum(const T& i_mean)
    : value(T(0))
    , mean(i_mean) {}
  VarianceSum(const VarianceSum<T>& i_v, tbb::split)
    : value(T(0))
    , mean(i_v.mean) {}
  void operator()(const tbb::blocked_range<const T*>& i_range) {
    T temp = value;
    for (const T* a = i_range.begin(); a != i_range.end(); ++a) {
      temp += sqr(*a - mean);
    }
    value = temp;
  }
  void join(const VarianceSum<T>& i_rhs) { value += i_rhs.value; }
};

//-*****************************************************************************
template <typename T>
T ParallelStdDev(T i_mean, const T* i_values, std::size_t i_size) {
  typedef typename singular_value_type<T>::type denom_type;

  VarianceSum<T> vsum(i_mean);
  tbb::parallel_reduce(
    tbb::blocked_range<const T*>(i_values, i_values + i_size), vsum);
  T variance = std::abs(vsum.value / denom_type(i_size));

  return std::sqrt(variance);
}

//-*****************************************************************************
template <typename T>
struct Stats {
  T MinHeight;
  T MaxHeight;
  T MeanHeight;

  T MeanMinE;
  T StdDevMinE;

#if 0
    explicit Stats( const PropagatedState<T>& i_waves )
    {
        ParallelMinMaxSum<T>( i_waves.Height.cdata(), i_waves.Height.size(),
                              MinHeight, MaxHeight, MeanHeight );
        MeanHeight /= T( i_waves.Height.size() );

        MeanMinE = ParallelMean<T>( i_waves.MinE.cdata(), i_waves.MinE.size() );
        StdDevMinE =
            ParallelStdDev<T>( MeanMinE,
                               i_waves.MinE.cdata(), i_waves.MinE.size() );

        std::cout <<
                  ( boost::format( "Height (min, max, mean): (%f, %f, %f)" )
                    % MinHeight % MaxHeight % MeanHeight ) << std::endl <<
                  ( boost::format( "MinE (mean, stddev): (%f, %f)" )
                    % MeanMinE % StdDevMinE ) << std::endl;
    }
#endif

  Stats(const RealSpatialField2D<T>& Height,
        const RealSpatialField2D<T>& MinE) {
    ParallelMinMaxSum<T>(Height.cdata(), Height.size(), MinHeight, MaxHeight,
                         MeanHeight);
    MeanHeight /= T(Height.size());

    MeanMinE   = ParallelMean<T>(MinE.cdata(), MinE.size());
    StdDevMinE = ParallelStdDev<T>(MeanMinE, MinE.cdata(), MinE.size());

    // std::cout <<
    //          ( boost::format( "Height (min, max, mean): (%f, %f, %f)" )
    //            % MinHeight % MaxHeight % MeanHeight ) << std::endl <<
    //          ( boost::format( "MinE (mean, stddev): (%f, %f)" )
    //            % MeanMinE % StdDevMinE ) << std::endl;
  }
};

//-*****************************************************************************
typedef Stats<float> Statsf;
typedef Stats<double> Statsd;

}  // namespace EncinoWaves

#endif
