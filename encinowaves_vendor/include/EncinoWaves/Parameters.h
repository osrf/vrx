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

#ifndef _EncinoWaves_Parameters_h_
#define _EncinoWaves_Parameters_h_

#include "Foundation.h"
#include "Basics.h"

namespace EncinoWaves {

enum DispersionType {
  kDeepDispersion,
  kFiniteDepthDispersion,
  kCapillaryDispersion
};

enum SpectrumType {
  kPiersonMoskowitzSpectrum,
  kJONSWAPSpectrum,
  kTMASpectrum,

  kNumSpectrumTypes
};

enum DirectionalSpreadingType {
  kPosCosThetaSqrDirectionalSpreading,
  kMitsuyasuDirectionalSpreading,
  kHasselmannDirectionalSpreading,
  kDonelanBannerDirectionalSpreading,
};

enum FilterType {
  kNullFilter,
  kSmoothInvertibleBandPassFilter,
};

enum RandomType {
  kNormalRandom,
  kLogNormalRandom,
};

template <typename T>
struct Parameters {
  // Resolution of the waves.
  int resolutionPowerOfTwo;

  // Domain of the waves. - this is the size of the world space
  // that they occupy.
  T domain;  // in meters

  // Some physical parameters.
  T gravity;         // in meters per second squared.
  T surfaceTension;  // in Newtons per meter
  T density;         // in kilograms per meter cubed
  T depth;           // in meters.

  // Wind stuff. It is assumed that wind travels along the positive
  // X axis, since we assume these fields can be externally transformed.
  // Wind speed is in meters per second.
  T windSpeed;  // in meters per second
  T fetch;      // in KILOMETERS

  T pinch;          // lateral displacement
  T amplitudeGain;  // vertical displacement

  T troughDamping;
  T troughDampingSmallWavelength;
  T troughDampingBigWavelength;
  T troughDampingSoftWidth;

  // Dispersion Stuff - Deep, FiniteDepth, Capillary
  struct Dispersion {
    DispersionType type;
    Dispersion()
      : type(kCapillaryDispersion) {}
  } dispersion;

  // Spectrum Stuff - Phillips, Pierson-Moskowitz, JONSWAP, TMA
  struct Spectrum {
    SpectrumType type;
    Spectrum()
      : type(kTMASpectrum) {}
  } spectrum;

  // Directional Spreading Stuff
  struct DirectionalSpreading {
    DirectionalSpreadingType type;
    T swell;
    DirectionalSpreading()
      : type(kHasselmannDirectionalSpreading)
      , swell(0.0) {}
  } directionalSpreading;

  // Filter
  struct Filter {
    FilterType type;
    T softWidth;
    T smallWavelength;
    T bigWavelength;
    T min;
    bool invert;
    Filter()
      : type(kNullFilter)
      , softWidth(0.0)
      , smallWavelength(0.0)
      , bigWavelength(1000000.0)
      , min(0.0)
      , invert(false) {}
  } filter;

  // Random Stuff
  struct Random {
    RandomType type;
    int seed;
    Random()
      : type(kNormalRandom)
      , seed(54321) {}
  } random;

  // Constructor
  Parameters()
    : resolutionPowerOfTwo(9)
    , domain(100.0)
    , gravity(9.81)
    , surfaceTension(0.074)
    , density(1000.0)
    , depth(100.0)
    , windSpeed(17.0)
    , fetch(300.0)
    , pinch(0.75)
    , amplitudeGain(1.0)
    , troughDamping(0.0)
    , troughDampingSmallWavelength(1.0)
    , troughDampingBigWavelength(4.0)
    , troughDampingSoftWidth(2.0) {}

  int resolution() const { return 1 << resolutionPowerOfTwo; }
};

//-*****************************************************************************
typedef Parameters<float> Parametersf;
typedef Parameters<double> Parametersd;

}  // namespace EncinoWaves

#endif
