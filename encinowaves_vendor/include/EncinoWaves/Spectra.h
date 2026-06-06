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

#ifndef _EncinoWaves_Spectra_h_
#define _EncinoWaves_Spectra_h_

#include "Foundation.h"
#include "Parameters.h"
#include "Basics.h"
#include "Random.h"

namespace EncinoWaves {

//-*****************************************************************************
// The spectra all have an equation of this form in them.
// A, B, and g are constants. w is omega, and wm is the peak omega,
// which is calculated differently for different spectra.
template <typename T>
T AlphaBetaSpectrum(T A, T B, T g, T w, T wm) {
  return (A * sqr(g) / std::pow(w, T(5.0))) *
         std::exp(-B * std::pow(wm / w, T(4.0)));
}

//-*****************************************************************************
// The Pierson Moskowitz spectrum is just an AlphaBetaSpectrum,
// with A = 8.10e-3, B = 0.74 / 0.87^4 = 1.291, g = gravity,
// and peakOmega is 0.87 * gravity / windSpeed.
template <typename T>
class PiersonMoskowitzSpectrum {
protected:
  T m_gravity   = 9.81;
  T m_windSpeed = 20;
  T m_peakOmega = 0.87 * 9.81 / 20;

public:
  PiersonMoskowitzSpectrum() = default;

  explicit PiersonMoskowitzSpectrum(const Parameters<T>& i_params)
      : m_gravity(i_params.gravity)
      , m_windSpeed(i_params.windSpeed)
      , m_peakOmega(0.87 * m_gravity / m_windSpeed) {}

  T operator()(T i_omega) const {
    return AlphaBetaSpectrum<T>(T(8.1e-3), T(1.291), m_gravity, i_omega,
                                m_peakOmega);
  }
};

//-*****************************************************************************
// The JONSWAP spectrum introduces the fetch parameter. It is an
// AlphaBeta spectrum times a peak sharpening function.  The peak sharpening
// coefficient, gamma, is chosen as a random draw.
// alpha = 0.076 * pow( xbar, -0.22 )
// sigma = 0.07 for w < wm, 0.09 for w > wm
// wm = TAU * 3.5 * ( g / U ) * pow( xbar, -0.33 )
// xbar = g F / U^2
// F = fetch
// U = wind speed
// g = gravity
// y = gaussian( mean=3.30, variance=0.62 ), clamped from 1 to 6
// peakSharpening = pow( y, exp( -(w-wm)^2/2(sigma wm)^2 )
// beta = 1.25
template <typename T>
class JONSWAPSpectrum {
protected:
  T m_gravity;
  T m_windSpeed;
  T m_fetch;
  T m_gamma;

  T m_peakOmega;
  T m_dimensionlessFetch;
  T m_alpha;

  void init(T g, T U, T fkm, T y) {
    m_gravity   = g;
    m_windSpeed = U;
    m_fetch     = fkm * 1000.0;
    m_gamma     = y;

    m_dimensionlessFetch = std::abs(m_gravity * m_fetch / sqr(m_windSpeed));
    m_alpha              = 0.076 * std::pow(m_dimensionlessFetch, T(-0.22));
    m_peakOmega = M_TAU * 3.5 * std::abs(m_gravity / m_windSpeed) *
                  std::pow(m_dimensionlessFetch, T(-0.33));
  }

public:
  JONSWAPSpectrum() { init(9.81, 10.0, 100.0, 3.30); }

  explicit JONSWAPSpectrum(const Parameters<T>& i_params) {
    Rand48_Engine r48;
    std::normal_distribution<T> norm{T(3.30), std::sqrt(T(0.67))};
    r48.seed(i_params.random.seed + 191819);
    T gamma = Imath::clamp(norm(r48), T(1.0), T(6.0));
    // gamma = T( 3.30 );

    init(i_params.gravity, i_params.windSpeed, i_params.fetch, gamma);
  }

  T peakSharpening(T i_omega) const {
    const T sigma = (i_omega <= m_peakOmega) ? T(0.07) : T(0.09);
    return std::pow(
      m_gamma,
      std::exp(-sqr((i_omega - m_peakOmega) / (sigma * m_peakOmega)) / T(2.0)));
  }

  T operator()(T i_omega) const {
    return peakSharpening(i_omega) * AlphaBetaSpectrum<T>(m_alpha, T(1.25),
                                                          m_gravity, i_omega,
                                                          m_peakOmega);
  }
};

//-*****************************************************************************
template <typename T>
class TMASpectrum {
protected:
  JONSWAPSpectrum<T> m_jonswap;
  T m_depth;
  T m_kdGain;

  void init(T h, T g) {
    m_depth  = h;
    m_kdGain = std::sqrt(m_depth / g);
  }

public:
  TMASpectrum()
      : m_jonswap() {
    init(100.0, 9.81);
  }

  explicit TMASpectrum(const Parameters<T>& i_params)
      : m_jonswap(i_params) {
    init(i_params.depth, i_params.gravity);
  }

  T kitaigorodskiiDepth(T i_omega) const {
    const T wh = i_omega * m_kdGain;
    return T(0.5) + (T(0.5) * std::tanh(T(1.8) * (wh - T(1.125))));
  }

  T operator()(T i_omega) const {
    return kitaigorodskiiDepth(i_omega) * m_jonswap(i_omega);
  }
};

}  // namespace EncinoWaves

#endif
