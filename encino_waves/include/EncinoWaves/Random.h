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

#ifndef _EncinoWaves_Random_h_
#define _EncinoWaves_Random_h_

#include "Foundation.h"
#include "Basics.h"
#include "Parameters.h"

namespace EncinoWaves {

//-*****************************************************************************
// C++ 11 FOR SURE
//-*****************************************************************************

typedef std::uint_fast32_t seed_type;
typedef std::minstd_rand Rand48_Engine;

//-*****************************************************************************
template <typename T>
uint32_t SeedFromWavenumber(const Imath::Vec2<T> &i_wavenumber,
                            uint32_t i_seed) {
  static constexpr uint32_t p1 = 73856093;
  static constexpr uint32_t p2 = 19349663;
  static constexpr uint32_t p3 = 83492791;

  // Truncate the ks to some precision, and then make them
  // into seeds.

  return (static_cast<uint32_t>(i_wavenumber[0] * 10000) * p1) ^
         (static_cast<uint32_t>(i_wavenumber[1] * 10000) * p2) ^ (i_seed * p3);
}

//-*****************************************************************************
template <typename T>
class BaseRandom {
protected:
  seed_type m_seed;
  Rand48_Engine m_engine;
  std::uniform_real_distribution<T> m_phaseDist;

public:
  BaseRandom()
    : m_seed(0)
    , m_engine(seed_type(0))
    , m_phaseDist(T(0.0), TAU<T>) {
    this->seed(m_seed);
  }

  BaseRandom(const Parameters<T> &i_params)
    : m_seed(i_params.random.seed)
    , m_engine(m_seed)
    , m_phaseDist(T(0.0), TAU<T>) {
    this->seed(m_seed);
  }

  void seed(seed_type i_seed) { m_engine.seed(i_seed + m_seed); }

  void seed(const Imath::Vec2<T> &i_wavenumber) {
    m_engine.seed(SeedFromWavenumber<T>(i_wavenumber, m_seed));
  }

  T nextPhase() { return m_phaseDist(m_engine); }
};

//-*****************************************************************************
// The normal distribution which produces an average value of 0.5 when
// integrated
// from 0 to infinity has a standard deviation of 1.48448
template <typename T>
class NormalRandom : public BaseRandom<T> {
protected:
  std::normal_distribution<T> m_ampDist;

public:
  NormalRandom()
    : BaseRandom<T>()
    , m_ampDist(T(0.0), T(1.0)) {}

  NormalRandom(const Parameters<T> &i_params)
    : BaseRandom<T>(i_params)
    , m_ampDist(T(0.0), T(1.0)) {}

  T nextAmp() { return m_ampDist(this->m_engine); }
};

//------------------------------------------------------------------------------
template <typename T>
class SquaredNormalRandom : public NormalRandom<T> {
public:
  SquaredNormalRandom()
    : NormalRandom<T>() {}

  SquaredNormalRandom(const Parameters<T> &i_params)
    : NormalRandom<T>(i_params) {}

  T nextAmp() { return sqr(NormalRandom<T>::nextAmp()); }
};

//-*****************************************************************************
template <typename T>
class LogNormalRandom : public BaseRandom<T> {
protected:
  std::lognormal_distribution<T> m_ampDist;

public:
  LogNormalRandom()
    : BaseRandom<T>()
    , m_ampDist(T(1.0), T(1.0)) {}

  LogNormalRandom(const Parameters<T> &i_params)
    : BaseRandom<T>(i_params)
    , m_ampDist(T(1.0), T(1.0)) {}

  T nextAmp() { return m_ampDist(this->m_engine); }
};

}  // namespace EncinoWaves

#endif
