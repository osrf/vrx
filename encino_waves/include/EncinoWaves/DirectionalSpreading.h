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

#ifndef _EncinoWaves_DirectionalSpreading_h_
#define _EncinoWaves_DirectionalSpreading_h_

#include "Foundation.h"
#include "Basics.h"
#include "Parameters.h"

namespace EncinoWaves {

//------------------------------------------------------------------------------
template <typename T, typename FUNC>
T numericallyIntegrate(FUNC f, T a, T b, int n) {
  T nf  = static_cast<T>(n);
  T sum = 0;
  for (int k = 1; k < n; ++k) {
    sum += f(a + (k * (b - a) / nf));
  }
  return ((b - a) / nf) * ((f(a) / 2) + (f(b) / 2) + sum);
}

//------------------------------------------------------------------------------
template <typename T>
T swellShape(T omega, T modal_omega, T swell_amount) {
  return 16.1 * std::tanh(modal_omega / omega) * sqr(swell_amount);
}

//------------------------------------------------------------------------------
template <typename T>
T swell(T theta, T omega, T modal_omega, T swell_amount) {
  T shape = swellShape(omega, modal_omega, swell_amount);
  return std::pow(std::abs(std::cos(theta / 2.0)), 2.0 * shape);
}

//------------------------------------------------------------------------------
template <typename T, typename FUNCA, typename FUNCB>
T normalizedSwellDirectionalProduct(T theta, FUNCA A, FUNCB B) {
  auto product = [A, B](T x) -> T { return A(x) * B(x); };

  T denom = numericallyIntegrate(product, -PI<T> / 2, PI<T> / 2, 36);
  return product(theta) / denom;
}

//------------------------------------------------------------------------------
template <typename T>
T modalAngularFrequencyJONSWAP(T gravity, T meanWindSpeed, T fetchLength) {
  T dimensionlessFetch = gravity * fetchLength / sqr(meanWindSpeed);
  return TAU<T> * 3.5 * (gravity / meanWindSpeed) *
         std::pow(dimensionlessFetch, -0.33);
}

//------------------------------------------------------------------------------
template <typename T>
class DonelanBannerDirectionalSpreading {
public:
  DonelanBannerDirectionalSpreading(const Parameters<T>& params)
      : m_modalAngularFrequency(modalAngularFrequencyJONSWAP(
          params.gravity, params.windSpeed, params.fetch))
      , m_swell(params.directionalSpreading.swell) {}

  T operator()(T i_omega, T i_theta, T i_kMag, T i_dTheta) const {
    T omega_over_modal_omega = i_omega / m_modalAngularFrequency;
    T beta_s;
    if (omega_over_modal_omega < 0.95) {
      beta_s = 2.61 * std::pow(omega_over_modal_omega, 1.3);
    } else if (omega_over_modal_omega < 1.6) {
      beta_s = 2.28 * std::pow(omega_over_modal_omega, -1.3);
    } else {
      T expo =
        -0.4 +
        0.8393 * std::exp(-0.567 * std::log(sqr(omega_over_modal_omega)));
      beta_s = std::pow(10, expo);
    }

    // Make a hyperbolic secant function
    static auto sech = [](T x) { return 1.0 / std::cosh(x); };

    // We need to do a numerical integration to determine the
    // normalization factor for the product of the original function (B)
    // with the swell elongation (A).
    auto A = [this, i_omega](T x) -> T {
      return swell(x, i_omega, m_modalAngularFrequency, m_swell);
    };
    auto B = [beta_s](T x) -> T { return sqr(sech(beta_s * x)); };

    if (m_swell >= 0.0) {
      return normalizedSwellDirectionalProduct(i_theta, A, B);
    } else {
      T integral =
        (std::tanh(beta_s * PI<T>) - std::tanh(-beta_s * PI<T>)) / beta_s;
      T d = B(i_theta) / integral;
      return Imath::lerp(d, static_cast<T>(-1.0 / (2.0 * PI<T>)),
                         Imath::clamp(-m_swell, T(0), T(1)));
    }
  }

protected:
  T m_modalAngularFrequency;
  T m_swell;
};

//------------------------------------------------------------------------------
template <typename T>
class MitsuyasuDirectionalSpreading {
public:
  MitsuyasuDirectionalSpreading(const Parameters<T>& params)
      : m_modalAngularFrequency(modalAngularFrequencyJONSWAP(
          params.gravity, params.windSpeed, params.fetch))
      , m_modalShape(11.5 * std::pow(m_modalAngularFrequency *
                                       params.windSpeed / params.gravity,
                                     -2.5))
      , m_modalCelerity(params.gravity / m_modalAngularFrequency)
      , m_windSpeedOverCelerity(params.windSpeed / m_modalCelerity)
      , m_swell(params.directionalSpreading.swell) {}

  T operator()(T i_omega, T i_theta, T i_kMag, T i_dTheta) const {
    T shape_bias = 0.0;
    if (m_swell >= 0.0) {
      shape_bias = swellShape(i_omega, m_modalAngularFrequency, m_swell);
    }

    T shape_exp = i_omega <= m_modalAngularFrequency ? 5.0 : -2.5;
    T shape =
      m_modalShape * std::pow(i_omega / m_modalAngularFrequency, shape_exp);

    shape += shape_bias;

    T factor_A = std::pow(2.0, (2.0 * shape) - 1.0) / PI<T>;
    T factor_B =
      sqr(std::tgamma(shape + 1.0)) / std::tgamma((2.0 * shape) + 1.0);
    T factor_C = std::pow(std::abs(std::cos(i_theta / 2.0)), 2.0 * shape);
    if (m_swell < 0) {
      return Imath::lerp(factor_A * factor_B * factor_C, T(1) / T(TAU<T>),
                         Imath::clamp(-m_swell, T(0), T(1)));
    } else {
      return factor_A * factor_B * factor_C;
    }
  }

protected:
  T m_modalAngularFrequency;
  T m_modalShape;
  T m_modalCelerity;
  T m_windSpeedOverCelerity;
  T m_swell;
};

//------------------------------------------------------------------------------
template <typename T>
class HasselmannDirectionalSpreading {
public:
  HasselmannDirectionalSpreading(const Parameters<T>& params)
      : m_modalAngularFrequency(modalAngularFrequencyJONSWAP(
          params.gravity, params.windSpeed, params.fetch))
      , m_modalShape(11.5 * std::pow(m_modalAngularFrequency *
                                       params.windSpeed / params.gravity,
                                     -2.5))
      , m_modalCelerity(params.gravity / m_modalAngularFrequency)
      , m_windSpeedOverCelerity(params.windSpeed / m_modalCelerity)
      , m_swell(params.directionalSpreading.swell) {}

  T operator()(T i_omega, T i_theta, T i_kMag, T i_dTheta) const {
    T shape_bias = 0.0;
    if (m_swell >= 0.0) {
      shape_bias = swellShape(i_omega, m_modalAngularFrequency, m_swell);
    }

    T shape;
    if (i_omega > m_modalAngularFrequency) {
      shape =
        9.77 * std::pow(i_omega / m_modalAngularFrequency,
                        -2.33 - (1.45 * (m_windSpeedOverCelerity - 1.17)));
    } else {
      shape = 6.97 * std::pow(i_omega / m_modalAngularFrequency, 4.06);
    }
    shape += shape_bias;

    T factor_A = std::pow(2.0, (2.0 * shape) - 1.0) / PI<T>;
    T factor_B =
      sqr(std::tgamma(shape + 1.0)) / std::tgamma((2.0 * shape) + 1.0);
    T factor_C = std::pow(std::abs(std::cos(i_theta / 2.0)), 2.0 * shape);
    if (m_swell < 0) {
      return Imath::lerp(factor_A * factor_B * factor_C, T(1) / T(TAU<T>),
                         Imath::clamp(-m_swell, T(0), T(1)));
    } else {
      return factor_A * factor_B * factor_C;
    }
  }

protected:
  T m_modalAngularFrequency;
  T m_modalShape;
  T m_modalCelerity;
  T m_windSpeedOverCelerity;
  T m_swell;
};

//------------------------------------------------------------------------------
template <typename T>
class PosCosSquaredDirectionalSpreading {
protected:
  static T modalAngularFrequencyJONSWAP(T gravity, T meanWindSpeed,
                                        T fetchLength) {
    T dimensionlessFetch = gravity * fetchLength / sqr(meanWindSpeed);
    return TAU<T> * 3.5 * (gravity / meanWindSpeed) *
           std::pow(dimensionlessFetch, -0.33);
  }

public:
  PosCosSquaredDirectionalSpreading(const Parameters<T>& params)
      : m_modalAngularFrequency(modalAngularFrequencyJONSWAP(
          params.gravity, params.windSpeed, params.fetch))
      , m_swell(params.directionalSpreading.swell) {}

  T operator()(T i_omega, T i_theta, T i_kMag, T i_dTheta) const {
    auto A = [this, i_omega](T x) -> T {
      return swell(x, i_omega, m_modalAngularFrequency, m_swell);
    };
    auto B = [](T x) -> T {
      if (x < -PI_2<T> || x > PI_2<T>) {
        return T{0};
      } else {
        return sqr(std::cos(x));
      }
    };

    return normalizedSwellDirectionalProduct(i_theta, A, B);
  }

protected:
  T m_modalAngularFrequency;
  T m_swell;
};

}  // namespace EncinoWaves

#endif
