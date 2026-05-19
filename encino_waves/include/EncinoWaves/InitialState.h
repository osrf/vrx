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

#ifndef _EncinoWaves_InitialState_h_
#define _EncinoWaves_InitialState_h_

#include "Foundation.h"
#include "SpectralSpatialField.h"
#include "Basics.h"
#include "Parameters.h"
#include "Dispersion.h"
#include "Spectra.h"
#include "Filter.h"
#include "DirectionalSpreading.h"
#include "Random.h"

namespace EncinoWaves {

//-*****************************************************************************
template <typename T>
struct InitialState {
  ComplexSpectralField2D<T> HSpectralPos;
  ComplexSpectralField2D<T> HSpectralNeg;
  RealSpectralField2D<T> Omega;

  InitialState(const Parameters<T>& i_params);

  int resolution() const { return HSpectralPos.height(); }
};

typedef InitialState<float> InitialStatef;
typedef InitialState<double> InitialStated;

//-*****************************************************************************
template <typename DISPERSION, typename SPECTRUM,
          typename DIRECTIONAL_SPREADING, typename FILTER, typename RANDOM,
          typename T>
struct InitialStateHelper {
  typedef T real_type;
  typedef std::complex<T> complex_type;
  typedef Imath::Vec2<T> vec_type;

  typedef InitialStateHelper<DISPERSION, SPECTRUM, DIRECTIONAL_SPREADING,
                             FILTER, RANDOM, T> this_type;

  // Functional configuration
  const DISPERSION* Dispersion;
  const SPECTRUM* Spectrum;
  const DIRECTIONAL_SPREADING* DirectionalSpreading;
  const FILTER* Filter;
  const RANDOM* Random;

  // Arrays
  complex_type* HSpectralPos;
  complex_type* HSpectralNeg;
  real_type* Omega;

  // Info
  real_type RhoG;
  real_type Domain;

  // Little processor object that is run at each point.
  struct Processor {
    const DISPERSION& Dispersion;
    const SPECTRUM& Spectrum;
    const DIRECTIONAL_SPREADING& DirectionalSpreading;
    const FILTER& Filter;
    RANDOM Random;

    complex_type* HSpectralPos;
    complex_type* HSpectralNeg;
    real_type* Omega;

    real_type RhoG;
    real_type Domain;

    Processor(const this_type& i_func)
      : Dispersion(*(i_func.Dispersion))
      , Spectrum(*(i_func.Spectrum))
      , DirectionalSpreading(*(i_func.DirectionalSpreading))
      , Filter(*(i_func.Filter))
      , Random(*(i_func.Random))
      , HSpectralPos(i_func.HSpectralPos)
      , HSpectralNeg(i_func.HSpectralNeg)
      , Omega(i_func.Omega)
      , RhoG(i_func.RhoG)
      , Domain(i_func.Domain) {}

    void operator()(std::size_t i_index) {
      HSpectralPos[i_index] = complex_type(0.0, 0.0);
      HSpectralNeg[i_index] = complex_type(0.0, 0.0);
      Omega[i_index]        = 0.0;
    }

    void operator()(const vec_type& k, real_type kMag, real_type dK,
                    std::size_t index);
  };
};

//-*****************************************************************************
// CJH HACK - hacked all over
template <typename _D, typename _S, typename _DS, typename _F, typename _R,
          typename T>
void InitialStateHelper<_D, _S, _DS, _F, _R, T>::Processor::operator()(
  const vec_type& k, real_type kMag, real_type dK, std::size_t index) {
  // Seed the random number generator from the wave numbers.
  Random.seed(k);

  // get thetaPos and thetaNeg from k.
  const real_type thetaPos = std::atan2(-k[1], k[0]);
  const real_type thetaNeg = std::atan2(k[1], -k[0]);
  EWAV_ASSERT(std::isfinite(thetaPos) && std::isfinite(thetaNeg),
                   "Broken thetas : " << thetaPos << ", " << thetaNeg
                                      << " at index: " << index);

  // Get omega and dOmegaDk from k. Dispersion functors provide both, by
  // reference.
  real_type omega, dOmegaDk;
  Dispersion(kMag, omega, dOmegaDk);
  assert(omega >= T(0.0));
  assert(dOmegaDk >= T(0.0));
  EWAV_ASSERT(std::isfinite(omega) && std::isfinite(dOmegaDk),
                   "Broken omegas : " << omega << ", " << dOmegaDk
                                      << " at index: " << index);

  // The area of each point being integrated is dki * dkj.  However,
  // We're evaluating the function in omega & theta space.  We need to
  // use the theory of change of variables to convert dki * dkj to
  // dtheta * domega.  We actually do two change of variables. The
  // first is to kMag and theta, where kMag is sqrt( ki^2 + kj^2 ), and
  // theta is atan( kj / ki ).  The second is from kMag and theta to
  // omega and theta.  Both changes involve multiplying the result by the
  // absolute value of the determinant of the Jacobian of the variable
  // matrix. Using the finite-depth form of the dispersion relationship
  // for: omega = sqrt( g kmag tanh( h kmag ) ), we get the following
  // relationship.
  // (1/2)amp^2 = Spectrum( omega, theta ) deltaOmega deltaTheta
  // (1/2)amp^2 = S(omega,theta) *
  //              abs( ( dOmegaDkmag / kMag ) deltaKi deltaKj )
  // Where dOmegaDkmag is computed by the Dispersion relationship.

  // Get spectrum.
  real_type DeltaSPos = Spectrum(omega);
  real_type DeltaSNeg = DeltaSPos;
  EWAV_ASSERT(std::isfinite(DeltaSPos) && std::isfinite(DeltaSNeg),
                   "Broken deltaSs : " << DeltaSPos << ", " << DeltaSNeg
                                       << " at index: " << index);

  // Attenuate by directional spreading
  const real_type dTheta = std::abs(std::atan2(dK, kMag));
  DeltaSPos *= DirectionalSpreading(omega, thetaPos, kMag, dTheta);
  DeltaSNeg *= DirectionalSpreading(omega, thetaNeg, kMag, dTheta);

  // Multiply DeltaSPos by dOmegaDk / kMag, which completes the
  // change of variables, and then multiply by dK^2.
  DeltaSPos *= (dK * dK) * dOmegaDk / (kMag);
  DeltaSNeg *= (dK * dK) * dOmegaDk / (kMag);

  // Amp is equal to sqrt( 2 DeltaS );
  real_type ampPos = Random.nextAmp() * std::sqrt(std::abs(DeltaSPos * T(2.0)));
  real_type ampNeg = Random.nextAmp() * std::sqrt(std::abs(DeltaSNeg * T(2.0)));
  EWAV_ASSERT(std::isfinite(ampPos) && std::isfinite(ampNeg),
                   "Broken amps : " << ampPos << ", " << ampNeg
                                    << " at index: " << index);

  // Filter amplitudes. Filter has to be outside the sqrt so that it
  // is properly invertible.
  const real_type filt = Filter(kMag);
  ampPos *= filt;
  ampNeg *= filt;
  EWAV_ASSERT(std::isfinite(ampPos) && std::isfinite(ampNeg),
                   "Broken filtered amps : " << ampPos << ", " << ampNeg
                                             << " at index: " << index);

  // Store results!
  // Get random draws for phase.
  const real_type phasePos = Random.nextPhase();
  const real_type phaseNeg = Random.nextPhase();
  HSpectralPos[index] =
    ampPos * complex_type(std::cos(phasePos), -std::sin(phasePos));
  HSpectralNeg[index] =
    ampNeg * complex_type(std::cos(phaseNeg), -std::sin(phaseNeg));

  // Assuming, for now, that angular velocity is the same for positive
  // and negative waves, which is not always true - some dispersion
  // relationships have faster travel for bigger waves.
  Omega[index] = omega;
}

//-*****************************************************************************
//-*****************************************************************************
// CASCADING SELECTION OF FUNCTORS FROM PARAMETERS.
// This is how you do an N-dimensional code switch at run time.
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
// This execution is called after all the selection has been done below.
template <typename DISPERSION, typename SPECTRUM,
          typename DIRECTIONAL_SPREADING, typename FILTER, typename RANDOM,
          typename T>
void ExecuteRange(const DISPERSION& i_dispersion, const SPECTRUM& i_spectrum,
                  const DIRECTIONAL_SPREADING& i_directionalSpreading,
                  const FILTER& i_filter, const RANDOM& i_random,

                  InitialState<T>& o_state, T i_domain, T i_rhoG) {
  typedef InitialStateHelper<DISPERSION, SPECTRUM, DIRECTIONAL_SPREADING,
                             FILTER, RANDOM, T> F_type;
  typedef typename F_type::Processor P_type;

  // Make a state.
  F_type F;

  // Components
  F.Dispersion           = &i_dispersion;
  F.Spectrum             = &i_spectrum;
  F.DirectionalSpreading = &i_directionalSpreading;
  F.Filter               = &i_filter;
  F.Random               = &i_random;

  // Output Arrays
  F.HSpectralPos = o_state.HSpectralPos.data();
  F.HSpectralNeg = o_state.HSpectralNeg.data();
  F.Omega        = o_state.Omega.data();

  // Info.
  F.RhoG   = i_rhoG;
  F.Domain = i_domain;

  // Spectral Iterate.
  int N = o_state.HSpectralPos.height();
  { SpectralIterationFunctor<T, F_type, P_type> iter(&F, i_domain, N); }
};

//-*****************************************************************************
template <typename DISPERSION, typename SPECTRUM,
          typename DIRECTIONAL_SPREADING, typename FILTER, typename T>
void ConfigRandom_CascadeExec(
  const Parameters<T>& i_params, const DISPERSION& i_dispersion,
  const SPECTRUM& i_spectrum,
  const DIRECTIONAL_SPREADING& i_directionalSpreading, const FILTER& i_filter,

  InitialState<T>& o_state) {
  NormalRandom<T> Fnorm(i_params);
  LogNormalRandom<T> FlogNorm(i_params);
  const T rhoG = i_params.gravity;

  switch (i_params.random.type) {
  default:
  case kNormalRandom:
    std::cout << "Normal Random Distribution" << std::endl;
    ExecuteRange<DISPERSION, SPECTRUM, DIRECTIONAL_SPREADING, FILTER,
                 NormalRandom<T>, T>(i_dispersion, i_spectrum,
                                     i_directionalSpreading, i_filter, Fnorm,
                                     o_state, i_params.domain, rhoG);
    break;
  case kLogNormalRandom:
    std::cout << "Log-Normal Random Distribution" << std::endl;
    ExecuteRange<DISPERSION, SPECTRUM, DIRECTIONAL_SPREADING, FILTER,
                 LogNormalRandom<T>, T>(
      i_dispersion, i_spectrum, i_directionalSpreading, i_filter, FlogNorm,
      o_state, i_params.domain, rhoG);
    break;
  };
}

//-*****************************************************************************
template <typename DISPERSION, typename SPECTRUM,
          typename DIRECTIONAL_SPREADING, typename T>
void ConfigFilter_CascadeExec(
  const Parameters<T>& i_params, const DISPERSION& i_dispersion,
  const SPECTRUM& i_spectrum,
  const DIRECTIONAL_SPREADING& i_directionalSpreading,

  InitialState<T>& o_state) {
  SmoothInvertibleBandPassFilter<T> Fsibp(i_params);
  NullFilter<T> Fnull(i_params);

  switch (i_params.filter.type) {
  case kSmoothInvertibleBandPassFilter:
    std::cout << "Smooth Invertible Band-Pass Filter." << std::endl;
    ConfigRandom_CascadeExec<DISPERSION, SPECTRUM, DIRECTIONAL_SPREADING,
                             SmoothInvertibleBandPassFilter<T>, T>(
      i_params, i_dispersion, i_spectrum, i_directionalSpreading, Fsibp,
      o_state);
    break;

  default:
  case kNullFilter:
    std::cout << "Null Filter.h" << std::endl;
    ConfigRandom_CascadeExec<DISPERSION, SPECTRUM, DIRECTIONAL_SPREADING,
                             NullFilter<T>, T>(
      i_params, i_dispersion, i_spectrum, i_directionalSpreading, Fnull,
      o_state);
    break;
  };
}

//-*****************************************************************************
template <typename DISPERSION, typename SPECTRUM, typename T>
void ConfigDirectionalSpreading_CascadeExec(const Parameters<T>& i_params,
                                            const DISPERSION& i_dispersion,
                                            const SPECTRUM& i_spectrum,

                                            InitialState<T>& o_state) {
  PosCosSquaredDirectionalSpreading<T> FCosSqr(i_params);
  MitsuyasuDirectionalSpreading<T> FMitsuyasu(i_params);
  HasselmannDirectionalSpreading<T> FHasselmann(i_params);
  DonelanBannerDirectionalSpreading<T> FDonelanBanner(i_params);

  switch (i_params.directionalSpreading.type) {
  default:
  case kDonelanBannerDirectionalSpreading:
    std::cout << "Donelan Banner Directional Spreading." << std::endl;
    ConfigFilter_CascadeExec<DISPERSION, SPECTRUM,
                             DonelanBannerDirectionalSpreading<T>, T>(
      i_params, i_dispersion, i_spectrum, FDonelanBanner, o_state);
    break;
  case kHasselmannDirectionalSpreading:
    std::cout << "Hasselmann Directional Spreading." << std::endl;
    ConfigFilter_CascadeExec<DISPERSION, SPECTRUM,
                             HasselmannDirectionalSpreading<T>, T>(
      i_params, i_dispersion, i_spectrum, FHasselmann, o_state);
    break;
  case kMitsuyasuDirectionalSpreading:
    std::cout << "Mitsuyasu Directional Spreading." << std::endl;
    ConfigFilter_CascadeExec<DISPERSION, SPECTRUM,
                             MitsuyasuDirectionalSpreading<T>, T>(
      i_params, i_dispersion, i_spectrum, FMitsuyasu, o_state);
    break;

  case kPosCosThetaSqrDirectionalSpreading:
    std::cout << "Pos Cos Theta Squared Directional Spreading." << std::endl;
    ConfigFilter_CascadeExec<DISPERSION, SPECTRUM,
                             PosCosSquaredDirectionalSpreading<T>, T>(
      i_params, i_dispersion, i_spectrum, FCosSqr, o_state);
    break;
  };
}

//-*****************************************************************************
template <typename DISPERSION, typename T>
void ConfigSpectrum_CascadeExec(const Parameters<T>& i_params,
                                const DISPERSION& i_dispersion,

                                InitialState<T>& o_state) {
  PiersonMoskowitzSpectrum<T> FPiersonMoskowitz(i_params);
  JONSWAPSpectrum<T> FJONSWAP(i_params);
  TMASpectrum<T> FTMA(i_params);

  switch (i_params.spectrum.type) {
  case kPiersonMoskowitzSpectrum:
    std::cout << "Pierson Moskowitz Spectrum." << std::endl;
    ConfigDirectionalSpreading_CascadeExec<DISPERSION,
                                           PiersonMoskowitzSpectrum<T>, T>(
      i_params, i_dispersion, FPiersonMoskowitz, o_state);
    break;

  case kJONSWAPSpectrum:
    std::cout << "JONSWAP Spectrum." << std::endl;
    ConfigDirectionalSpreading_CascadeExec<DISPERSION, JONSWAPSpectrum<T>, T>(
      i_params, i_dispersion, FJONSWAP, o_state);
    break;

  default:
  case kTMASpectrum:
    std::cout << "Texel Marsen Arsloe (TMA) Spectrum." << std::endl;
    ConfigDirectionalSpreading_CascadeExec<DISPERSION, TMASpectrum<T>, T>(
      i_params, i_dispersion, FTMA, o_state);
    break;
  };
}

//-*****************************************************************************
template <typename T>
void ConfigDispersion_CascadeExec(const Parameters<T>& i_params,

                                  InitialState<T>& o_state) {
  DeepDispersion<T> FDeep(i_params);
  FiniteDepthDispersion<T> FFiniteDepth(i_params);
  CapillaryDispersion<T> FCapillary(i_params);

  switch (i_params.dispersion.type) {
  case kDeepDispersion:
    std::cout << "Deep Dispersion." << std::endl;
    ConfigSpectrum_CascadeExec<DeepDispersion<T>, T>(i_params, FDeep, o_state);
    break;

  case kFiniteDepthDispersion:
    std::cout << "Finite Depth Dispersion." << std::endl;
    ConfigSpectrum_CascadeExec<FiniteDepthDispersion<T>, T>(
      i_params, FFiniteDepth, o_state);
    break;

  default:
  case kCapillaryDispersion:
    std::cout << "Capillary Dispersion." << std::endl;
    ConfigSpectrum_CascadeExec<CapillaryDispersion<T>, T>(i_params, FCapillary,
                                                          o_state);
    break;
  };
}

//-*****************************************************************************
//-*****************************************************************************
// DO IT
//-*****************************************************************************
//-*****************************************************************************
template <typename T>
InitialState<T>::InitialState(const Parameters<T>& i_params)
  : HSpectralPos(i_params.resolutionPowerOfTwo)
  , HSpectralNeg(i_params.resolutionPowerOfTwo)
  , Omega(i_params.resolutionPowerOfTwo) {
  ConfigDispersion_CascadeExec<T>(i_params, *this);
}

}  // namespace EncinoWaves

#endif
