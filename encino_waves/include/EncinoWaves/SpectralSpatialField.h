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

#ifndef _EncinoWaves_SpectralSpatialField_h_
#define _EncinoWaves_SpectralSpatialField_h_

#include "Foundation.h"
#include "FftwWrapper.h"

namespace EncinoWaves {

//-*****************************************************************************
template <typename T>
class BaseField2D {
public:
  typedef T value_type;
  typedef value_type* pointer;
  typedef const value_type* const_pointer;
  typedef value_type& reference;
  typedef const value_type& const_reference;

  typedef value_type* iterator;
  typedef const value_type* const_iterator;

protected:
  BaseField2D(int i_width, int i_height)
      : m_width(i_width)
      , m_height(i_height)
      , m_dataSize(static_cast<std::size_t>(i_width) *
                   static_cast<std::size_t>(i_height))
      , m_data(nullptr) {
    // Nothing
  }

public:
  ~BaseField2D() { m_data = nullptr; }

  iterator begin() { return m_data; }
  const_iterator begin() const { return m_data; }
  const_iterator cbegin() const { return m_data; }

  iterator end() { return m_data + m_dataSize; }
  const_iterator end() const { return m_data + m_dataSize; }
  const_iterator cend() const { return m_data + m_dataSize; }

  pointer data() { return m_data; }
  const_pointer data() const { return m_data; }
  const_pointer cdata() const { return m_data; }

  std::size_t size() const { return m_dataSize; }
  int width() const { return m_width; }
  int height() const { return m_height; }
  std::size_t stride() const { return std::size_t(m_width); }

  // Return a row, with wrapping.
  pointer row(int i_y) { return m_data + (wrap(i_y, m_height) * m_width); }

  const_pointer row(int i_y) const {
    return m_data + (wrap(i_y, m_height) * m_width);
  }

  const_pointer crow(int i_y) const {
    return m_data + (wrap(i_y, m_height) * m_width);
  }

  // It's a 2D field, so the index operator returns a row.
  pointer operator[](int i_y) { return row(i_y); }
  const_pointer operator[](int i_y) const { return crow(i_y); }

  // Direct access with two indices - DIFFERS FROM IMF!!!
  reference operator()(int i_x, int i_y) {
    return row(i_y)[wrap(i_x, m_width)];
  }

  const_reference operator()(int i_x, int i_y) const {
    return crow(i_y)[wrap(i_x, m_width)];
  }

  // Direct access with an Imath::Vec2i.
  reference operator()(const Imath::V2i& i_xy) {
    return row(i_xy[1])[wrap(i_xy[0], m_width)];
  }

  const_reference operator()(const Imath::V2i& i_xy) const {
    return row(i_xy[1])[wrap(i_xy[0], m_width)];
  }

protected:
  int m_width;
  int m_height;
  std::size_t m_dataSize;
  pointer m_data;
};

//-*****************************************************************************
// Spatial Field is easy when using FFTW out-of-place, which is what we'll do.
template <typename T>
class SpatialField2D : public BaseField2D<T> {
protected:
  int m_pad;

public:
  typedef typename singular_value_type<T>::type base_value_type;
  typedef FftwWrapperT<base_value_type> FFT;
  typedef BaseField2D<T> super_type;

  SpatialField2D()
      : super_type(0, 0)
      , m_pad(0) {
    this->m_data = nullptr;
  }

  explicit SpatialField2D(int i_powerOfTwo, int i_pad = 0)
      : super_type(i_pad + PowerOfTwo(Imath::clamp(i_powerOfTwo, 0, 30)),
                   i_pad + PowerOfTwo(Imath::clamp(i_powerOfTwo, 0, 30)))
      , m_pad(i_pad) {
    this->m_data =
      reinterpret_cast<T*>(FFT::Malloc(this->m_dataSize * sizeof(T)));
    std::fill(this->m_data, this->m_data + this->m_dataSize, T(0.0));
  }

  ~SpatialField2D() {
    if (this->m_data) {
      FFT::Free(this->m_data);
      this->m_data = nullptr;
    }
  }

  int unpaddedWidth() const { return this->width() - m_pad; }
  int unpaddedHeight() const { return this->height() - m_pad; }
  int padding() const { return m_pad; }
};

//-*****************************************************************************
template <typename T>
class RealSpatialField2D : public SpatialField2D<T> {
public:
  typedef SpatialField2D<T> super_type;
  RealSpatialField2D()
      : super_type() {}
  explicit RealSpatialField2D(int i_powerOfTwo, int i_pad = 0)
      : super_type(i_powerOfTwo, i_pad) {}
};

//-*****************************************************************************
template <typename T>
class ComplexSpatialField2D : public SpatialField2D<std::complex<T> > {
public:
  typedef SpatialField2D<std::complex<T> > super_type;
  ComplexSpatialField2D()
      : super_type() {}
  explicit ComplexSpatialField2D(int i_powerOfTwo, int i_pad = 0)
      : super_type(i_powerOfTwo, i_pad) {}
};

//-*****************************************************************************
//-*****************************************************************************
// SPECTRAL FIELDS
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
// Spectral Field is easy when using FFTW out-of-place, which is what we'll do.
template <typename T>
class SpectralField2D : public BaseField2D<T> {
public:
  typedef typename singular_value_type<T>::type base_value_type;
  typedef FftwWrapperT<base_value_type> FFT;
  typedef BaseField2D<T> super_type;

  SpectralField2D()
      : super_type(0, 0) {
    this->m_data = nullptr;
  }

  explicit SpectralField2D(int i_powerOfTwo)
      : super_type((PowerOfTwo(Imath::clamp(i_powerOfTwo, 0, 30)) / 2) + 1,
                   PowerOfTwo(Imath::clamp(i_powerOfTwo, 0, 30))) {
    this->m_data =
      reinterpret_cast<T*>(FFT::Malloc(this->m_dataSize * sizeof(T)));

    std::fill(this->m_data, this->m_data + this->m_dataSize, T(0.0));
  }

  ~SpectralField2D() {
    if (this->m_data) {
      FFT::Free(this->m_data);
      this->m_data = nullptr;
    }
  }
};

//-*****************************************************************************
template <typename T>
class RealSpectralField2D : public SpectralField2D<T> {
public:
  typedef SpectralField2D<T> super_type;
  RealSpectralField2D()
      : super_type() {}
  explicit RealSpectralField2D(int i_powerOfTwo)
      : super_type(i_powerOfTwo) {}
};

//-*****************************************************************************
template <typename T>
class ComplexSpectralField2D : public SpectralField2D<std::complex<T> > {
public:
  typedef SpectralField2D<std::complex<T> > super_type;
  ComplexSpectralField2D()
      : super_type() {}
  explicit ComplexSpectralField2D(int i_powerOfTwo)
      : super_type(i_powerOfTwo) {}
};

//-*****************************************************************************
//-*****************************************************************************
// FAST FOURIER TRANSFORM
//-*****************************************************************************
//-*****************************************************************************

//-*****************************************************************************
template <typename T>
class SpectralToSpatial2D {
public:
  typedef FftwWrapperT<T> FFT;
  typedef typename FFT::plan_type plan_type;

  SpectralToSpatial2D(ComplexSpectralField2D<T>& i_spectral,
                      RealSpatialField2D<T>& o_spatial, int i_numThreads = -1)
      : m_widthHeight(o_spatial.width()) {
    EWAV_ASSERT((o_spatial.height() == m_widthHeight) &&
                       (i_spectral.width() == ((m_widthHeight / 2) + 1)) &&
                       (i_spectral.height() == m_widthHeight),
                     "Mismatched spectral and spatial sizes");

    if (i_numThreads <= 0) {
      i_numThreads = std::thread::hardware_concurrency();
    }

    if (i_numThreads > 1) {
      FftwInitThreadsT<T>();
      // std::cout << "initialized threads" << std::endl;
      FFT::plan_with_nthreads(i_numThreads);
      // std::cout << "Using: " << i_numThreads << " threads" << std::endl;
    }

// We're creating an out-of-place transform that destroys input.
#if 0
        m_plan = FFT::plan_dft_c2r_2d( m_widthHeight, m_widthHeight,
                                       i_spectral.data(),
                                       o_spatial.data(),
                                       FFTW_ESTIMATE | FFTW_DESTROY_INPUT );
#else
    m_plan = FFT::plan_guru_dft_c2r(m_widthHeight, m_widthHeight,
                                    i_spectral.data(), o_spatial.data(),
                                    FFTW_ESTIMATE | FFTW_DESTROY_INPUT);

#endif
  }

  ~SpectralToSpatial2D() {
    if (m_plan) {
      FFT::destroy_plan(m_plan);
      m_plan = nullptr;
    }
  }

  void execute(ComplexSpectralField2D<T>& i_spectral,
               RealSpatialField2D<T>& o_spatial) {
    EWAV_ASSERT((i_spectral.width() == ((m_widthHeight / 2) + 1)) &&
                       (i_spectral.height() == m_widthHeight) &&
                       (o_spatial.width() == m_widthHeight) &&
                       (o_spatial.height() == m_widthHeight),
                     "Mismatched spectral and spatial sizes");

    FFT::execute_dft_c2r(m_plan, i_spectral.data(), o_spatial.data());
  }

protected:
  int m_widthHeight;
  plan_type m_plan;
};

//-*****************************************************************************
template <typename T>
struct CopyWrappedBorder {
  T* Data;
  int N;

  void operator()(const tbb::blocked_range<int>& i_rows) const {
    std::size_t stride = N + 1;
    for (int y = i_rows.begin(); y != i_rows.end(); ++y) {
      if (y == N) {
        std::copy(Data,                             // input begin
                  Data + stride,                    // input end
                  Data + (stride * std::size_t(N))  // output begin
                  );
      } else {
        std::size_t rowBeginIndex = y * stride;
        Data[rowBeginIndex + N]   = Data[rowBeginIndex];
      }
    }
  }
};

//-*****************************************************************************
template <typename T>
class SpectralToPaddedSpatial2D {
public:
  typedef FftwWrapperT<T> FFT;
  typedef typename FFT::plan_type plan_type;

  SpectralToPaddedSpatial2D(ComplexSpectralField2D<T>& i_spectral,
                            RealSpatialField2D<T>& o_spatial,
                            int i_numThreads = -1)
      : m_widthHeight(i_spectral.height()) {
    EWAV_ASSERT((o_spatial.width() == (m_widthHeight + 1)) &&
                       (o_spatial.height() == (m_widthHeight + 1)) &&
                       (i_spectral.width() == ((m_widthHeight / 2) + 1)),
                     "Mismatched spectral and spatial sizes");

    if (i_numThreads <= 0) {
      i_numThreads = std::thread::hardware_concurrency();
    }

    if (i_numThreads > 1) {
      FftwInitThreadsT<T>();
      // std::cout << "initialized threads" << std::endl;
      FFT::plan_with_nthreads(i_numThreads);
      // std::cout << "Using: " << i_numThreads << " threads" << std::endl;
    }

    // We're creating an out-of-place transform that destroys input.
    m_plan = FFT::plan_guru_dft_c2r_output_padded(
      m_widthHeight, m_widthHeight, 1, 1, i_spectral.data(), o_spatial.data(),
      FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
  }

  ~SpectralToPaddedSpatial2D() {
    if (m_plan) {
      FFT::destroy_plan(m_plan);
      m_plan = nullptr;
    }
  }

  void execute(ComplexSpectralField2D<T>& i_spectral,
               RealSpatialField2D<T>& o_spatial) {
    EWAV_ASSERT((i_spectral.width() == ((m_widthHeight / 2) + 1)) &&
                       (i_spectral.height() == m_widthHeight) &&
                       (o_spatial.width() == m_widthHeight + 1) &&
                       (o_spatial.height() == m_widthHeight + 1),
                     "Mismatched spectral and spatial sizes");

    FFT::execute_dft_c2r(m_plan, i_spectral.data(), o_spatial.data());

    // Fill in the repeated border.
    {
      CopyWrappedBorder<T> F;
      F.Data = o_spatial.data();
      F.N = m_widthHeight;
      tbb::parallel_for(tbb::blocked_range<int>(0, m_widthHeight + 1), F);
    }
  }

protected:
  int m_widthHeight;
  plan_type m_plan;
};

//-*****************************************************************************
//-*****************************************************************************
// TYPEDEFS
//-*****************************************************************************
//-*****************************************************************************

typedef RealSpatialField2D<float> RSpatialField2Df;
typedef RealSpatialField2D<double> RSpatialField2Dd;

typedef ComplexSpatialField2D<float> CSpatialField2Df;
typedef ComplexSpatialField2D<double> CSpatialField2Dd;

typedef RealSpectralField2D<float> RSpectralField2Df;
typedef RealSpectralField2D<double> RSpectralField2Dd;

typedef ComplexSpectralField2D<float> CSpectralField2Df;
typedef ComplexSpectralField2D<double> CSpectralField2Dd;

typedef SpectralToSpatial2D<float> SpectralToSpatial2Df;
typedef SpectralToSpatial2D<double> SpectralToSpatial2Dd;

}  // namespace EncinoWaves

#endif
