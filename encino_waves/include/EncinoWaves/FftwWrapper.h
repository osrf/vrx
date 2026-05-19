/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * Source-compatible replacement for `EncinoWaves/FftwWrapper.h` (Apache-2.0,
 * Copyright 2015 Christopher Jon Horvath) that removes the GPL-licensed FFTW
 * dependency in favour of Eigen::FFT (BSD-3 via KissFFT or MPL-2 via PocketFFT,
 * whichever Eigen wraps in the host build).
 *
 * Why: VRX is Apache-2.0 and cannot incorporate GPL code in its dependency
 * chain. EncinoWaves itself is Apache-2.0, but its only FFTW touchpoint is
 * this header (plus a tiny .cpp), so replacing the wrapper lifts the whole
 * library cleanly into our license envelope.
 *
 * What's the same:
 *   - The `EncinoWaves::FftwWrapperT<float>` / `FftwWrapperT<double>` static
 *     interface (init_threads, plan_dft_c2r_2d, plan_guru_dft_c2r,
 *     plan_guru_dft_c2r_output_padded, Malloc, Free, execute,
 *     execute_dft_c2r, destroy_plan, cleanup_threads, cleanup).
 *   - The unnormalized inverse convention. FFTW's c2r IFFT computes
 *     out[j] = Σ_k in[k]·exp(+i·2π·jk/N) with no 1/N factor; Eigen::FFT
 *     normalizes by default, so we set Eigen::FFT::Unscaled to match.
 *   - The hermitian-packed input layout (slow×(fast/2+1) complex → slow×fast
 *     real, both row-major, with optional padding on the output row stride).
 *
 * What's different (no behavioural impact):
 *   - `plan_type` is now an opaque pointer to our own `Plan` struct rather
 *     than `fftwf_plan`/`fftw_plan`. Callers only ever store and forward it,
 *     so the change is invisible.
 *   - `iodim_type` is a dummy struct; only used inside `plan_guru_*` to mirror
 *     the FFTW shape declarations, never escapes.
 *   - Threading entry points (`init_threads`, `plan_with_nthreads`,
 *     `cleanup_threads`) are no-ops. Eigen::FFT is single-threaded; we leave
 *     the higher-level TBB parallelism in Propagation.h to do the work.
 *   - `Malloc` / `Free` use `std::aligned_alloc` / `std::free` rather than
 *     `fftw_malloc` / `fftw_free`. Eigen::FFT has no special-alignment
 *     requirement, so this is a safe substitution.
 *
 * The actual 2D complex-to-real inverse FFT is decomposed into two 1D
 * passes:
 *   1. Column pass — for each of the `(fast/2+1)` columns of the
 *      hermitian-packed input, run an N=slow complex-to-complex inverse FFT
 *      along the slow direction.
 *   2. Row pass — for each of the `slow` rows of the intermediate matrix,
 *      run an N=fast hermitian-to-real inverse FFT along the fast direction.
 *
 * Eigen::FFT supports both flavours natively, so the implementation reduces
 * to two for-loops with no manual butterfly math.
 *
 * To switch back to FFTW for performance experiments, replace the contents
 * of this header with the original GPL-licensed wrapper — the rest of
 * EncinoWaves doesn't care.
 */

#ifndef _EncinoWaves_FftwWrapper_h_
#define _EncinoWaves_FftwWrapper_h_

#include <cassert>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <unsupported/Eigen/FFT>

namespace EncinoWaves
{

namespace detail
{
  /// Plan handle returned from `plan_*` and consumed by `execute*` /
  /// `destroy_plan`. Holds the geometry of the 2D c2r IFFT plus optional
  /// default in/out pointers (so the parameterless `execute()` overload
  /// works against pointers captured at plan time).
  template <typename T>
  struct Plan
  {
    int width{0};              ///< fast (inner) dim, hermitian-packed in input
    int height{0};             ///< slow (outer) dim
    int outputRowStride{0};    ///< row stride of the real output (= width
                               ///<     unless `plan_guru_dft_c2r_output_padded`
                               ///<     bumped it by `widthPad`)
    std::complex<T> *defaultIn{nullptr};
    T *defaultOut{nullptr};
  };

  /// One unused-but-present struct so `FftwWrapperT::iodim_type` typedef has
  /// something to point at, even though Encino never reaches in.
  struct IodimDummy
  {
    int n{0};
    int is{0};
    int os{0};
  };

  /// Shared implementation of the 2D c2r inverse FFT. Templated on the real
  /// scalar so float/double specializations of `FftwWrapperT` can both call
  /// it without code duplication.
  ///
  /// Input:  `slow × (fast/2+1)` complex, row-major. Hermitian symmetry is
  ///         along the fast (inner) dimension.
  /// Output: `slow × fast` real, row-major. Row r begins at
  ///         `out[r * outputRowStride]`.
  template <typename T>
  inline void Execute2dC2r(int slow, int fast, int outputRowStride,
                           const std::complex<T> *in, T *out)
  {
    using Complex = std::complex<T>;
    using VecC = Eigen::Matrix<Complex, Eigen::Dynamic, 1>;
    using VecR = Eigen::Matrix<T, Eigen::Dynamic, 1>;

    const int halfFast = (fast / 2) + 1;
    if (slow <= 0 || fast <= 0 || halfFast <= 0 || in == nullptr || out == nullptr)
      return;

    // Two FFT engines: the column pass is full-size complex-to-complex,
    // the row pass is hermitian (half-spectrum) to real. Eigen::FFT's
    // `HalfSpectrum` flag is what tells `inv()` that the complex input
    // is N/2+1 hermitian-packed and to produce N real outputs; without
    // it Eigen treats the input as full-size and resizes the destination
    // to (N/2+1), which trips an internal assertion when we then try to
    // read indices N/2+1..N-1.
    //
    // Both engines set `Unscaled` to match FFTW's unnormalized inverse
    // convention (no 1/N factor). Encino's Propagation.h was written
    // against FFTW and would mis-scale otherwise.
    Eigen::FFT<T> fftC2C;
    fftC2C.SetFlag(Eigen::FFT<T>::Unscaled);
    Eigen::FFT<T> fftC2R;
    fftC2R.SetFlag(Eigen::FFT<T>::Unscaled);
    fftC2R.SetFlag(Eigen::FFT<T>::HalfSpectrum);

    // Pass 1: column pass along the slow dimension.
    // For each of the halfFast columns, gather `slow` complex values from
    // `in`, run a complex-to-complex inverse FFT of length `slow`, then
    // scatter the result back into the same shape (the intermediate buffer
    // is `slow × halfFast` complex).
    std::vector<Complex> intermediate(
        static_cast<std::size_t>(slow) * halfFast);

    VecC colIn(slow);
    VecC colOut(slow);
    for (int j = 0; j < halfFast; ++j)
    {
      for (int i = 0; i < slow; ++i)
        colIn(i) = in[static_cast<std::size_t>(i) * halfFast + j];
      fftC2C.inv(colOut, colIn);
      for (int i = 0; i < slow; ++i)
        intermediate[static_cast<std::size_t>(i) * halfFast + j] = colOut(i);
    }

    // Pass 2: row pass along the fast dimension.
    // Each row of the intermediate is `halfFast` complex values arranged
    // as the standard hermitian half-spectrum. With HalfSpectrum set,
    // Eigen::FFT::inv reads halfFast complex inputs and writes `fast`
    // real outputs.
    VecC rowIn(halfFast);
    VecR rowOut(fast);
    for (int i = 0; i < slow; ++i)
    {
      for (int j = 0; j < halfFast; ++j)
        rowIn(j) = intermediate[static_cast<std::size_t>(i) * halfFast + j];
      fftC2R.inv(rowOut, rowIn);
      // Scatter into the output, honouring the (possibly padded) row stride.
      T *outRow = out + static_cast<std::size_t>(i) * outputRowStride;
      for (int j = 0; j < fast; ++j)
        outRow[j] = rowOut(j);
    }
  }
}  // namespace detail

/// Template base — never instantiated directly. The float and double
/// specializations below provide the static methods Encino calls.
template <typename T>
struct FftwWrapperT;

//-----------------------------------------------------------------------------
// SINGLE PRECISION
//-----------------------------------------------------------------------------
template <>
struct FftwWrapperT<float>
{
  using real_type = float;
  using complex_type = std::complex<float>;
  using plan_type = detail::Plan<float> *;
  using iodim_type = detail::IodimDummy;

  // ----------- Threading (no-op stubs; Eigen::FFT is single-threaded) -----

  /// Returns 1 to mimic FFTW's "init OK" return value. Encino's
  /// `FftwInitThreadsT_InitHelper` checks `err != 0` and aborts otherwise.
  static int init_threads()
  {
    return 1;
  }

  static void plan_with_nthreads(int /*i_nthreads*/) {}

  // ----------- Plan creation ----------------------------------------------

  /// Build a plan for an out-of-place 2D complex-to-real inverse FFT.
  /// Mirrors `fftwf_plan_dft_c2r_2d(i_width, i_height, in, out, flags)`.
  ///
  /// Recall the FFTW naming: `i_width` is the SLOW (outer/row) dimension
  /// and `i_height` is the FAST (inner/column) dimension. The hermitian
  /// packing is along the fast dim.
  static plan_type plan_dft_c2r_2d(int i_width, int i_height,
                                   complex_type *i_in, real_type *o_out,
                                   unsigned int /*i_flags*/)
  {
    auto *p = new detail::Plan<float>();
    p->width = i_width;
    p->height = i_height;
    p->outputRowStride = i_height;
    p->defaultIn = i_in;
    p->defaultOut = o_out;
    return p;
  }

  /// Guru variant of the above. We don't need the additional iodim machinery
  /// at all — the (width, height) sizes carry all the information our shim
  /// uses, so we just delegate.
  static plan_type plan_guru_dft_c2r(int i_width, int i_height,
                                     complex_type *i_in, real_type *o_out,
                                     unsigned int i_flags)
  {
    return plan_dft_c2r_2d(i_width, i_height, i_in, o_out, i_flags);
  }

  /// Guru variant with output padding. The `i_widthPad` extends the output
  /// row stride (the only padding kind that affects single-batch layout);
  /// `i_heightPad` only matters across batches, which Encino never uses
  /// (howmany == 1), so we ignore it.
  static plan_type plan_guru_dft_c2r_output_padded(
      int i_width, int i_height, int i_widthPad, int /*i_heightPad*/,
      complex_type *i_in, real_type *o_out, unsigned int /*i_flags*/)
  {
    auto *p = new detail::Plan<float>();
    p->width = i_width;
    p->height = i_height;
    p->outputRowStride = i_height + i_widthPad;
    p->defaultIn = i_in;
    p->defaultOut = o_out;
    return p;
  }

  // ----------- Allocation -------------------------------------------------

  /// Encino capitalises Malloc/Free deliberately to make sure they aren't
  /// confused with system malloc/free. We deliberately use `std::malloc`
  /// (not `std::aligned_alloc`) — Eigen::FFT has no alignment requirement,
  /// and `aligned_alloc` would require sizes rounded up to the alignment.
  static void *Malloc(std::size_t i_size)
  {
    return std::malloc(i_size);
  }

  static void Free(void *i_data)
  {
    std::free(i_data);
  }

  // ----------- Execution --------------------------------------------------

  /// Execute the plan against its captured default buffers.
  static void execute(const plan_type i_plan)
  {
    if (!i_plan) return;
    detail::Execute2dC2r<float>(i_plan->width, i_plan->height,
                                 i_plan->outputRowStride,
                                 i_plan->defaultIn, i_plan->defaultOut);
  }

  /// Execute the plan against caller-supplied buffers (FFTW's
  /// `execute_dft_c2r` API). Encino's SpectralSpatialField uses this to
  /// reuse a plan across multiple in/out pairs.
  static void execute_dft_c2r(const plan_type i_plan,
                              complex_type *i_in, real_type *o_out)
  {
    if (!i_plan) return;
    detail::Execute2dC2r<float>(i_plan->width, i_plan->height,
                                 i_plan->outputRowStride, i_in, o_out);
  }

  // ----------- Teardown ---------------------------------------------------

  static void destroy_plan(const plan_type i_plan)
  {
    delete i_plan;
  }

  static void cleanup_threads() {}
  static void cleanup() {}
};

//-----------------------------------------------------------------------------
// DOUBLE PRECISION
//-----------------------------------------------------------------------------
template <>
struct FftwWrapperT<double>
{
  using real_type = double;
  using complex_type = std::complex<double>;
  using plan_type = detail::Plan<double> *;
  using iodim_type = detail::IodimDummy;

  static int init_threads() { return 1; }
  static void plan_with_nthreads(int /*i_nthreads*/) {}

  static plan_type plan_dft_c2r_2d(int i_width, int i_height,
                                   complex_type *i_in, real_type *o_out,
                                   unsigned int /*i_flags*/)
  {
    auto *p = new detail::Plan<double>();
    p->width = i_width;
    p->height = i_height;
    p->outputRowStride = i_height;
    p->defaultIn = i_in;
    p->defaultOut = o_out;
    return p;
  }

  static plan_type plan_guru_dft_c2r(int i_width, int i_height,
                                     complex_type *i_in, real_type *o_out,
                                     unsigned int i_flags)
  {
    return plan_dft_c2r_2d(i_width, i_height, i_in, o_out, i_flags);
  }

  static plan_type plan_guru_dft_c2r_output_padded(
      int i_width, int i_height, int i_widthPad, int /*i_heightPad*/,
      complex_type *i_in, real_type *o_out, unsigned int /*i_flags*/)
  {
    auto *p = new detail::Plan<double>();
    p->width = i_width;
    p->height = i_height;
    p->outputRowStride = i_height + i_widthPad;
    p->defaultIn = i_in;
    p->defaultOut = o_out;
    return p;
  }

  static void *Malloc(std::size_t i_size) { return std::malloc(i_size); }
  static void Free(void *i_data) { std::free(i_data); }

  static void execute(const plan_type i_plan)
  {
    if (!i_plan) return;
    detail::Execute2dC2r<double>(i_plan->width, i_plan->height,
                                  i_plan->outputRowStride,
                                  i_plan->defaultIn, i_plan->defaultOut);
  }

  static void execute_dft_c2r(const plan_type i_plan,
                              complex_type *i_in, real_type *o_out)
  {
    if (!i_plan) return;
    detail::Execute2dC2r<double>(i_plan->width, i_plan->height,
                                  i_plan->outputRowStride, i_in, o_out);
  }

  static void destroy_plan(const plan_type i_plan) { delete i_plan; }
  static void cleanup_threads() {}
  static void cleanup() {}
};

//-----------------------------------------------------------------------------
// GLOBAL THREAD INIT HELPER
// Kept source-compatible with the FFTW wrapper Encino ships with. The Init
// helper instance is a process-wide singleton that the upstream code calls
// `FftwInitThreadsT<T>()` to set up; since our threading entry points are
// no-ops the helper does nothing useful, but having the symbols satisfies
// any translation unit that still calls into them.
//-----------------------------------------------------------------------------
template <typename T>
struct FftwInitThreadsT_InitHelper
{
  using FFT = FftwWrapperT<T>;

  FftwInitThreadsT_InitHelper() { (void)FFT::init_threads(); }
  ~FftwInitThreadsT_InitHelper() { FFT::cleanup_threads(); }
};

template <typename T> struct __BaseFftwInitThreadsT;

template <>
struct __BaseFftwInitThreadsT<float>
{
  using Init = FftwInitThreadsT_InitHelper<float>;
  static std::unique_ptr<Init> sm_init;
};

template <>
struct __BaseFftwInitThreadsT<double>
{
  using Init = FftwInitThreadsT_InitHelper<double>;
  static std::unique_ptr<Init> sm_init;
};

template <typename T>
inline void FftwInitThreadsT()
{
  using Base = __BaseFftwInitThreadsT<T>;
  if (!Base::sm_init)
    Base::sm_init.reset(new typename Base::Init);
}

}  // namespace EncinoWaves

#endif  // _EncinoWaves_FftwWrapper_h_
