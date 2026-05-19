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

#ifndef _EncinoWaves_Dispersion_h_
#define _EncinoWaves_Dispersion_h_

#include "Foundation.h"
#include "Basics.h"
#include "Parameters.h"

namespace EncinoWaves {

//-*****************************************************************************
//-*****************************************************************************
// DISPERSION
// https://en.wikipedia.org/wiki/Dispersion_(water_waves)
// The dispersion relationship relates the angular frequency of waves
// to their wavelengths, based on gravity, and ocean depth, and other physical
// parameters. These are all statistical approximations, and there are
// a ton of them. (Look at the bottom of the wikipedia page)
//
// For some simpler discussion, I referenced:
// http://www.atm.ox.ac.uk/user/read/fluids/fluidsnotes5.pdf
//
// A simple deep water dispersion relationship, in which ocean depth is vastly
// larger than considered wavelengths, is:
// omega^2 = g * k  // g = gravity
// dOmega_dk = g / 2 sqrt( g * k ) = g / ( 2 omega )
//
// A finite-depth water dispersion relationship is:
// omega^2 = g * k * tanh( k * h )  // g = gravity, h = depth
// dOmega_dk = g ( tanh( h k ) + h k sech^2( h k ) ) / 2 sqrt( g k tanh( h k ) )
// dOmega_dk = g ( tanh( h k ) + h k sech^2( h k ) ) / ( 2 omega )
//
// A capillary-wave dispersion relationship is:
// omega^2 = ( g*k + (sigma/rho)k^3 ) tanh( k * h )
// // g = gravity, h = depth, sigma = surface tension, rho = fluid density
// // default sigma = 0.074 N/m, rho = 1000 kg/m^3
// dOmega_dk = ( g + 3 k^2 s )tanh( h k ) + h k ( g + k^2 s )sech^2( h k )
//             / ( 2 omega )
//
// There are others, but they get kinda crazy. We can always come back to
// these.
//
// Note that they're always written in terms of omega^2, which means there
// are two solutions: omega & -omega.  We'll return the positive one.
//
// Note that the capillary formulation given here will turn into
// the finite depth dispersion, for big waves, and the finite depth dispersion
// will turn into the deep dispersion, for deep water. So you can just
// use the capillary dispersion all the time.
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
class DeepDispersion {
protected:
  T m_gravity;

public:
  DeepDispersion()
    : m_gravity(9.81) {}

  explicit DeepDispersion(const Parameters<T>& i_params)
    : m_gravity(i_params.gravity) {}

  T operator()(T i_k) const { return std::sqrt(std::abs(m_gravity * i_k)); }

  void operator()(T i_k, T& o_omega, T& o_dOmegaDk) const {
    o_omega    = std::sqrt(std::abs(m_gravity * i_k));
    o_dOmegaDk = m_gravity / (T(2.0) * o_omega);
  }
};

//-*****************************************************************************
template <typename T>
class FiniteDepthDispersion : public DeepDispersion<T> {
protected:
  T m_depth;

public:
  FiniteDepthDispersion()
    : DeepDispersion<T>()
    , m_depth(1000.0) {}

  explicit FiniteDepthDispersion(const Parameters<T>& i_params)
    : DeepDispersion<T>(i_params)
    , m_depth(i_params.depth) {}

  T operator()(T i_k) const {
    return std::sqrt(
      std::abs(this->m_gravity * i_k * std::tanh(i_k * m_depth)));
  }

  void operator()(T i_k, T& o_omega, T& o_dOmegaDk) const {
    const T hk = i_k * m_depth;
    o_omega    = std::sqrt(std::abs(this->m_gravity * i_k * std::tanh(hk)));

    o_dOmegaDk = (this->m_gravity * (std::tanh(hk) + hk / sqr(std::cosh(hk)))) /
                 (T(2.0) * o_omega);
  }
};

//-*****************************************************************************
template <typename T>
class CapillaryDispersion : public FiniteDepthDispersion<T> {
protected:
  T m_sigmaOverRho;

public:
  CapillaryDispersion()
    : FiniteDepthDispersion<T>()
    , m_sigmaOverRho(0.074 / 1000.0) {}

  explicit CapillaryDispersion(const Parameters<T>& i_params)
    : FiniteDepthDispersion<T>(i_params)
    , m_sigmaOverRho(i_params.surfaceTension / i_params.density) {}

  T operator()(T i_k) const {
    return std::sqrt(
      std::abs(((this->m_gravity * i_k) + (m_sigmaOverRho * cube(i_k))) *
               std::tanh(this->m_depth * i_k)));
  }

  void operator()(T i_k, T& o_omega, T& o_dOmegaDk) const {
    const T hk    = this->m_depth * i_k;
    const T k2s   = sqr(i_k) * m_sigmaOverRho;
    const T gpk2s = this->m_gravity + k2s;

    o_omega = std::sqrt(std::abs(i_k * gpk2s * std::tanh(hk)));

    const T numer =
      ((gpk2s + k2s + k2s) * std::tanh(hk)) + (hk * gpk2s / sqr(std::cosh(hk)));

    o_dOmegaDk = std::abs(numer) / (T(2.0) * o_omega);
  }
};

}  // namespace EncinoWaves

#endif
