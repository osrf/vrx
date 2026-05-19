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

#ifndef _EncinoWaves_Filter_h_
#define _EncinoWaves_Filter_h_

#include "Foundation.h"
#include "Parameters.h"
#include "Basics.h"

namespace EncinoWaves {

//-*****************************************************************************
// nullptr filter does nothing, always returns 1.0
//-*****************************************************************************
template <typename T>
class NullFilter {
public:
  NullFilter() {}
  explicit NullFilter(const Parameters<T>& i_params) {}

  T operator()(T i_kMag) const { return T(1.0); }
};

//-*****************************************************************************
// We do filtering by wavelength, rather than frequency (which is standard)
// because it's easier for artists to visualize.
// This filter just creates a smooth band that protects the wavelengths
// between smallWavelength and bigWavelength.
//-*****************************************************************************
template <typename T>
class SmoothInvertibleBandPassFilter {
protected:
  T m_edge0;
  T m_edge1;
  T m_edge2;
  T m_edge3;
  T m_min;
  bool m_invert;

public:
  SmoothInvertibleBandPassFilter()
      : m_edge0(0.0)
      , m_edge1(0.0)
      , m_edge2(10000.0)
      , m_edge3(10000.0)
      , m_min(1.0)
      , m_invert(false) {}

  SmoothInvertibleBandPassFilter(const Parameters<T>& i_params)
      : m_edge0(i_params.filter.smallWavelength - i_params.filter.softWidth)
      , m_edge1(i_params.filter.smallWavelength)
      , m_edge2(i_params.filter.bigWavelength)
      , m_edge3(i_params.filter.bigWavelength + i_params.filter.softWidth)
      , m_min(i_params.filter.min)
      , m_invert(i_params.filter.invert) {}

  SmoothInvertibleBandPassFilter(T edge0, T edge1, T edge2, T edge3, T min,
                                 bool invert)
      : m_edge0(edge0)
      , m_edge1(edge1)
      , m_edge2(edge2)
      , m_edge3(edge3)
      , m_min(min)
      , m_invert(invert) {}

  T operator()(T i_kMag) const {
    const T wavelength = WavelengthFromWavenumber(i_kMag);
    const T t = smoothstep(m_edge0, m_edge1, wavelength) -
                smoothstep(m_edge2, m_edge3, wavelength);
    const T f = Imath::clamp(m_min + (T(1.0) - m_min) * t, T(0.0), T(1.0));
    return (m_invert ? T(1.0) - f : f);
  }
};

}  // namespace EncinoWaves

#endif
