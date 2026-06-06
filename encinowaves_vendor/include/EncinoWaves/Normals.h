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

#ifndef _EncinoWaves_Normals_h_
#define _EncinoWaves_Normals_h_

#include "Foundation.h"
#include "SpectralSpatialField.h"
#include "Propagation.h"

namespace EncinoWaves {

template <typename T>
struct ComputeNormalsWithPinching {
  using V3T = Imath::Vec3<T>;

  const T* H   = nullptr;
  const T* DX  = nullptr;
  const T* DY  = nullptr;
  V3T* Normals = nullptr;

  int N = 0;
  T Spacing;
  T AmpGain;
  T Pinch;

  std::size_t index(std::size_t x, std::size_t y) const {
    return (y * std::size_t(N + 1)) + x;
  }

  V3T pointAtIndex(T i_xMult, T i_yMult, std::size_t i_index) const {
    return V3T((i_xMult * Spacing) - (Pinch * DX[i_index]),
               (i_yMult * Spacing) - (Pinch * DY[i_index]),
               AmpGain * H[i_index]);
  }

  void operator()(const tbb::blocked_range2d<int>& range) const {
    for (auto y = range.rows().begin(); y != range.rows().end(); ++y) {
      auto downY = wrap(y - 1, N);
      auto cenY  = wrap(y, N);
      auto upY   = wrap(y + 1, N);

      for (auto x = range.cols().begin(); x != range.cols().end(); ++x) {
        auto leftX  = wrap(x - 1, N);
        auto cenX   = wrap(x, N);
        auto rightX = wrap(x + 1, N);

        auto leftIndex  = index(leftX, cenY);
        auto rightIndex = index(rightX, cenY);
        auto downIndex  = index(cenX, downY);
        auto upIndex    = index(cenX, upY);

        auto downPoint  = pointAtIndex(T(0.0), T(-1.0), downIndex);
        auto leftPoint  = pointAtIndex(T(-1.0), T(0.0), leftIndex);
        auto rightPoint = pointAtIndex(T(1.0), T(0.0), rightIndex);
        auto upPoint    = pointAtIndex(T(0.0), T(1.0), upIndex);

        auto dPdU = rightPoint - leftPoint;
        auto dPdV = upPoint - downPoint;

        Normals[index(x, y)] = dPdU.cross(dPdV).normalized();
      }
    }
  }
};

//-*****************************************************************************
template <typename T>
void ComputeNormals(const Parameters<T>& i_params,
                    const PropagatedState<T>& i_waves,
                    Imath::Vec3<T>* o_normals) {
  const int N     = i_waves.Height.unpaddedWidth();
  const T spacing = i_params.domain / T(N);

  ComputeNormalsWithPinching<T> F;
  F.H       = i_waves.Height.cdata();
  F.DX      = i_waves.Dx.cdata();
  F.DY      = i_waves.Dy.cdata();
  F.Normals = o_normals;

  F.N       = N;
  F.Spacing = spacing;
  F.AmpGain = i_params.amplitudeGain;
  F.Pinch   = i_params.pinch;

  tbb::parallel_for(tbb::blocked_range2d<int>{0, N + 1, 1, 0, N + 1, 512}, F);
}

}  // namespace EncinoWaves

#endif
