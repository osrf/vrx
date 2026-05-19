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

#ifndef _EncinoWaves_Basics_h_
#define _EncinoWaves_Basics_h_

#include "Foundation.h"

namespace EncinoWaves {

template <typename T>
constexpr T PI = T(3.1415926535897932385);

template <typename T>
constexpr T PI_2 = PI<T> / 2;

//-*****************************************************************************
//-*****************************************************************************
// TAU
// Wave stuff is easier with Tau (2Pi) than it is with Pi.
//-*****************************************************************************
//-*****************************************************************************
#ifndef M_TAU
// This is 2.0 * M_PI, to same number of significant digits.
#define M_TAU 6.28318530717958647693
#endif

template <typename T>
constexpr T TAU = 2 * PI<T>;

//-*****************************************************************************
//-*****************************************************************************
// WAVE NUMBER (k), WAVE LENGTH (lambda)
// http://en.wikipedia.org/wiki/Wavenumber
// The wave number, which is what we're iterating over in spectal space,
// is simply equal to:  k = 2*PI/Wavelength, or TAU/Wavelength
//-*****************************************************************************
//-*****************************************************************************
template <typename T>
T WavenumberFromWavelength(T i_lambda) {
  return TAU<T> / i_lambda;
}

template <typename T>
T WavelengthFromWavenumber(T i_k) {
  return TAU<T> / i_k;
}

// For brevity
template <typename T>
T k_from_lambda(T i_lambda) {
  return WavenumberFromWavelength(i_lambda);
}

template <typename T>
T lambda_from_k(T i_k) {
  return WavelengthFromWavenumber(i_k);
}

//-*****************************************************************************
//-*****************************************************************************
// ANGULAR FREQUENCY (omega) (and Ordinary Frequency)
// http://en.wikipedia.org/wiki/Angular_frequency
// In Radians per Second:
// omega = 2*PI/T or TAU/T, where T is the period, in seconds.
// omega = 2*PI*f, where f is the ordinary frequency, in hertz (1/seconds)
//-*****************************************************************************
//-*****************************************************************************
template <typename T>
T AngularFrequencyFromPeriod(T i_period) {
  return TAU<T> / i_period;
}

template <typename T>
T PeriodFromAngularFrequency(T i_omega) {
  return TAU<T> / i_omega;
}

template <typename T>
T AngularFrequencyFromOrdinaryFrequency(T i_hertz) {
  return TAU<T> * i_hertz;
}

template <typename T>
T OrdinaryFrequencyFromAngularFrequency(T i_omega) {
  return i_omega / TAU<T>;
}

// Brevity.
template <typename T>
T omega_from_T(T i_T) {
  return AngularFrequencyFromPeriod(i_T);
}

template <typename T>
T T_from_omega(T i_omega) {
  return PeriodFromAngularFrequency(i_omega);
}

template <typename T>
T omega_from_f(T i_f) {
  return AngularFrequencyFromOrdinaryFrequency(i_f);
}

template <typename T>
T f_from_omega(T i_omega) {
  return OrdinaryFrequencyFromAngularFrequency(i_omega);
}

//-*****************************************************************************
//-*****************************************************************************
// PHASE VELOCITY (vPhase) - how fast a the top of the crest moves through
// space.
// http://en.wikipedia.org/wiki/Phase_velocity
// This is interesting, because when the phase velocity is equal to the wind
// velocity, the system is at equilibrium, which gives us a way of computing
// the peak wavelength - it's the one where phase velocity = wind velocity.
// vPhase = Wavelength / Period, or Angular Frequency / Wave Number
//-*****************************************************************************
//-*****************************************************************************
template <typename T>
T PhaseVelocityFromWavelengthAndPeriod(T i_lambda, T i_period) {
  return i_lambda / i_period;
}

template <typename T>
T PhaseVelocityFromAngularFrequencyAndWavenumber(T i_omega, T i_k) {
  return i_omega / i_k;
}

// Brevity.
template <typename T>
T vp_from_omega_and_k(T i_omega, T i_k) {
  return PhaseVelocityFromAngularFrequencyAndWavenumber(i_omega, i_k);
}

template <typename T>
T vp_from_lambda_and_T(T i_lambda, T i_T) {
  return PhaseVelocityFromWavelengthAndPeriod(i_lambda, i_T);
}

//-*****************************************************************************
//-*****************************************************************************
// BASIC SPECTRAL ITERATION FUNCTOR
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename T, typename STATE, typename PROCESSOR>
class SpectralIterationFunctor {
public:
  typedef T real_type;
  typedef std::complex<T> complex_type;
  typedef Imath::Vec2<real_type> vec_type;

protected:
  const STATE* m_state;
  real_type m_domain;
  int N;

  std::size_t m_strideJ;
  real_type m_maxKmag;
  real_type m_dK;

public:
  // Constructor.
  SpectralIterationFunctor(const STATE* i_state, real_type i_domain, int i_N)
      : m_state(i_state)
      , m_domain(i_domain)
      , N(i_N) {
    int width  = (N / 2) + 1;
    int height = N;

    // Constants
    m_strideJ = (N / 2) + 1;
    m_maxKmag = real_type(N / 2) * TAU<T> / m_domain;
    m_dK      = real_type(1) * TAU<T> / m_domain;

    // Execute it!
    int grainSize = std::min(512, N);
    tbb::parallel_for(
      tbb::blocked_range2d<int>(0, height, 1, 0, width, grainSize), *this);
  }

  // Creates a processor from the state.
  void operator()(const tbb::blocked_range2d<int>& i_range) const {
    // Make a processor.
    PROCESSOR proc(*m_state);

    for (int j = i_range.rows().begin(); j != i_range.rows().end(); ++j) {
      // kj is the wave number in the j direction.
      int realJ    = j <= (N / 2) ? j : j - N;
      real_type kj = real_type(realJ) * TAU<T> / m_domain;

      // Compute start index.
      std::size_t index =
        (std::size_t(j) * m_strideJ) + std::size_t(i_range.cols().begin());

      // Iterate over i, updating index as we go.
      for (int i = i_range.cols().begin(); i != i_range.cols().end();
           ++i, ++index) {
        // ki is the wave number in the i direction
        real_type ki   = real_type(i) * TAU<T> / m_domain;
        real_type kMag = std::hypot(ki, kj);

        // By restricting to make sure kMag isn't too big, we
        // avoid diagonal aliasing. It's very very hard to see,
        // aesthetically, but this makes it correct.
        if ((i == 0 && realJ == 0) /*|| ( kMag > m_maxKmag )*/) {
          proc(index);
        } else {
          proc(vec_type(ki, kj), kMag, m_dK, index);
        }
      }
    }
  }
};

}  // namespace EncinoWaves

#endif
