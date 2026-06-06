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

#ifndef _EncinoWaves_EncinoWaves_MipMap_h_
#define _EncinoWaves_EncinoWaves_MipMap_h_

#include "Foundation.h"
#include "Propagation.h"

namespace EncinoWaves {

//-*****************************************************************************
// This could not be more bloated if my life depended on it. Blech.
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
struct EdgeKernel {
  static constexpr T edge4x4   = 0.029797;
  static constexpr T corner4x4 = 0.004783;
  T operator()(const T& a, const T& b, const T& c, const T& d) const {
    return ((a + d) * corner4x4) + ((b + c) * edge4x4);
  }
  T operator()(const T* v) const {
    return ((v[0] + v[3]) * corner4x4) + ((v[1] + v[2]) * edge4x4);
  }
};

//-*****************************************************************************
template <typename T>
struct CenterKernel {
  static constexpr T center4x4 = 0.185622;
  static constexpr T edge4x4 = 0.029797;
  T operator()(const T& a, const T& b, const T& c, const T& d) const {
    return ((a + d) * edge4x4) + ((b + c) * center4x4);
  }
  T operator()(const T* v) const {
    return ((v[0] + v[3]) * edge4x4) + ((v[1] + v[2]) * center4x4);
  }
};

//-*****************************************************************************
template <typename T, typename TRANSFER_OP, typename KERNEL>
void DownsampleTransferFunc(const T* i_src, int i_srcN, T* o_dst, int i_dstN) {
  static TRANSFER_OP top;
  static KERNEL k;

  // Do the first pixel.
  top(o_dst[0], k(i_src[i_srcN - 1], i_src[0], i_src[1], i_src[2]));

  // Loop over the non-edge ones.
  const T* srcPtr = i_src + 1;
  T* dstPtr       = o_dst + 1;
  T* dstPtrEnd = o_dst + (i_dstN - 1);
  for (; dstPtr < dstPtrEnd; ++dstPtr, srcPtr += 2) {
    top(*dstPtr, k(srcPtr));
  }

  // Do the last pixel.
  top(*dstPtrEnd,
      k(i_src[i_srcN - 3], i_src[i_srcN - 2], i_src[i_srcN - 1], i_src[0]));
}

//-*****************************************************************************
template <typename T>
struct AssignTransferOp {
  void operator()(T& o_dst, const T& i_src) const { o_dst = i_src; }
};

//-*****************************************************************************
template <typename T>
struct PlusEqualsTransferOp {
  void operator()(T& o_dst, const T& i_src) const { o_dst += i_src; }
};

//-*****************************************************************************
template <typename T>
struct DownsampleFunc {
  static constexpr T center4x4 = 0.185622;
  static constexpr T edge4x4   = 0.029797;
  static constexpr T corner4x4 = 0.004783;

  typedef T value_type;
  typedef DownsampleFunc<T> this_type;

  const T* Src;
  int SrcN;
  int SrcStrideJ;

  T* Dst;
  int DstN;
  int DstStrideJ;

  T& dstPixel(int i, int j) {
    return Dst[wrap(i, DstN) + (wrap(j, DstN) * DstStrideJ)];
  }

  T srcPixel(int i, int j) const {
    return Src[wrap(i, SrcN) + (wrap(j, SrcN) * SrcStrideJ)];
  }

  void processDstLine(int j) const {
    int srcJ = j * 2;

    const T* SrcA = Src + (SrcStrideJ * wrap((srcJ - 1), SrcN));
    const T* SrcB = Src + (SrcStrideJ * wrap((srcJ + 0), SrcN));
    const T* SrcC = Src + (SrcStrideJ * wrap((srcJ + 1), SrcN));
    const T* SrcD = Src + (SrcStrideJ * wrap((srcJ + 2), SrcN));

    T* dst = Dst + (DstStrideJ * j);

    DownsampleTransferFunc<T, AssignTransferOp<T>, EdgeKernel<T> >(SrcA, SrcN,
                                                                   dst, DstN);

    DownsampleTransferFunc<T, PlusEqualsTransferOp<T>, CenterKernel<T> >(
      SrcB, SrcN, dst, DstN);

    DownsampleTransferFunc<T, PlusEqualsTransferOp<T>, CenterKernel<T> >(
      SrcC, SrcN, dst, DstN);

    DownsampleTransferFunc<T, PlusEqualsTransferOp<T>, EdgeKernel<T> >(
      SrcD, SrcN, dst, DstN);
  }

  void operator()(const tbb::blocked_range<int>& i_range) const {
    for (int j = i_range.begin(); j < i_range.end(); ++j) {
      processDstLine(j);
    }
  }
};

//-*****************************************************************************
// Apply Mip Map functor to downsample one Spatial Field to another.
template <typename T>
void Downsample(const RealSpatialField2D<T>& i_src,
                RealSpatialField2D<T>& o_dst) {
  EWAV_ASSERT(i_src.unpaddedWidth() == (o_dst.unpaddedWidth() * 2) &&
                i_src.unpaddedHeight() == (o_dst.unpaddedHeight() * 2) &&
                i_src.unpaddedWidth() == i_src.unpaddedHeight(),
              "Mip-map sizes are wrong");
  // Downsample
  {
    DownsampleFunc<T> F;

    F.Src        = i_src.cdata();
    F.SrcN       = i_src.unpaddedWidth();
    F.SrcStrideJ = i_src.stride();

    F.Dst        = o_dst.data();
    F.DstN       = o_dst.unpaddedWidth();
    F.DstStrideJ = o_dst.stride();

    tbb::parallel_for(tbb::blocked_range<int>{0, (int)o_dst.unpaddedHeight()},
                      F);
  }

  // Fill in the repeated border.
  {
    CopyWrappedBorder<T> F;
    F.Data = o_dst.data();
    F.N = o_dst.unpaddedWidth();
    tbb::parallel_for(tbb::blocked_range<int>{0, (int)o_dst.height()}, F);
  }
}

//-*****************************************************************************
// Apply a Mip Map functor to downsample one Propagated State to another.
template <typename T>
void DownsampleState(const PropagatedState<T>& i_src,
                     PropagatedState<T>& o_dst) {
  Downsample<T>(i_src.Height, o_dst.Height);
  Downsample<T>(i_src.Dx, o_dst.Dx);
  Downsample<T>(i_src.Dy, o_dst.Dy);
  Downsample<T>(i_src.MinE, o_dst.MinE);
}

}  // namespace EncinoWaves

#endif
