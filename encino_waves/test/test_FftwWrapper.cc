/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *
 * Smoke tests for the Eigen-backed FftwWrapper shim. Three sanity checks:
 *
 *   1. Zero spectrum → zero output (no spurious DC).
 *   2. Pure DC spectrum (only X[0,0] non-zero) → constant output equal to
 *      that DC value. Catches normalization mistakes — if Eigen's default
 *      1/N scaling slipped through, output would be DC/(N1*N2).
 *   3. Pure single-mode spectrum (one sinusoid in the slow direction) →
 *      a single-mode cosine in the spatial domain with amplitude consistent
 *      with FFTW's unnormalized convention.
 *
 * The float and double specializations are exercised independently.
 */

#include "EncinoWaves/FftwWrapper.h"

#include <cmath>
#include <complex>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace
{
  /// One-stop checker: prints a banner, runs `fn`, increments
  /// `failures` if `fn` returns false. Cheap test harness so we don't pull
  /// in gtest for a five-test file.
  template <typename Fn>
  void Check(const char *name, int &failures, Fn fn)
  {
    std::cout << "  test: " << name << " ... ";
    const bool ok = fn();
    std::cout << (ok ? "PASS" : "FAIL") << "\n";
    if (!ok) ++failures;
  }

  /// Hermitian-packed input layout: slow rows × (fast/2+1) cols complex,
  /// stored row-major. Helper to index into it cleanly.
  template <typename T>
  std::complex<T> &At(std::vector<std::complex<T>> &spec,
                       int slow, int fast, int i, int j)
  {
    (void)slow;
    const int halfFast = (fast / 2) + 1;
    return spec[static_cast<std::size_t>(i) * halfFast + j];
  }

  /// Output layout: slow × fast real, row-major.
  template <typename T>
  T &OutAt(std::vector<T> &out, int slow, int fast, int i, int j)
  {
    (void)slow;
    return out[static_cast<std::size_t>(i) * fast + j];
  }

  template <typename T>
  bool MaxAbsDiff(const std::vector<T> &actual, T expected, T tol)
  {
    T maxDiff = T(0);
    for (T v : actual)
    {
      const T d = std::abs(v - expected);
      if (d > maxDiff) maxDiff = d;
    }
    const bool ok = maxDiff <= tol;
    if (!ok)
      std::cout << "(max|v - " << expected << "| = " << maxDiff
                << ", tol = " << tol << ") ";
    return ok;
  }
}  // namespace

template <typename T>
int RunForType(const char *typeName)
{
  std::cout << "== FftwWrapperT<" << typeName << "> ==\n";
  using FFT = EncinoWaves::FftwWrapperT<T>;
  using Complex = std::complex<T>;

  constexpr int Slow = 8;   // = i_width in FFTW guru naming (outer dim)
  constexpr int Fast = 8;   // = i_height (inner dim, hermitian-packed)
  const int halfFast = (Fast / 2) + 1;

  std::vector<Complex> spec(static_cast<std::size_t>(Slow) * halfFast);
  std::vector<T> out(static_cast<std::size_t>(Slow) * Fast);

  int failures = 0;

  // --- Test 1: zero spectrum produces zero output -----------------------
  Check("zero spectrum → zero output", failures, [&]() {
    std::fill(spec.begin(), spec.end(), Complex(0, 0));
    std::fill(out.begin(), out.end(), T(7));  // poison the buffer first
    auto plan = FFT::plan_dft_c2r_2d(Slow, Fast, spec.data(), out.data(), 0u);
    FFT::execute(plan);
    FFT::destroy_plan(plan);
    return MaxAbsDiff(out, T(0), static_cast<T>(1e-5));
  });

  // --- Test 2: DC spectrum → constant output ----------------------------
  // FFTW unnormalized convention: IFFT([X[0,0]=c]) = constant c.
  Check("DC spectrum → constant output (unnormalized)", failures, [&]() {
    std::fill(spec.begin(), spec.end(), Complex(0, 0));
    const T dc = T(3.5);
    At<T>(spec, Slow, Fast, 0, 0) = Complex(dc, 0);
    std::fill(out.begin(), out.end(), T(0));
    auto plan = FFT::plan_dft_c2r_2d(Slow, Fast, spec.data(), out.data(), 0u);
    FFT::execute(plan);
    FFT::destroy_plan(plan);
    return MaxAbsDiff(out, dc, static_cast<T>(1e-5));
  });

  // --- Test 3: single slow-direction mode → real cosine ------------------
  // For real spatial pattern x[i, j] = cos(2π·i / Slow), the hermitian
  // spectrum has only two non-zero entries: X[1, 0] and X[Slow-1, 0],
  // each = (Slow * Fast) / 2 under FFTW's unnormalized forward.
  // IFFT of that spectrum gives back (Slow * Fast) * cos(2π·i / Slow).
  Check("slow-direction cosine → cos with unnormalized amplitude",
        failures, [&]() {
    std::fill(spec.begin(), spec.end(), Complex(0, 0));
    const T amp = T(Slow) * T(Fast) * T(0.5);
    At<T>(spec, Slow, Fast, 1, 0) = Complex(amp, 0);
    At<T>(spec, Slow, Fast, Slow - 1, 0) = Complex(amp, 0);
    std::fill(out.begin(), out.end(), T(0));
    auto plan = FFT::plan_dft_c2r_2d(Slow, Fast, spec.data(), out.data(), 0u);
    FFT::execute(plan);
    FFT::destroy_plan(plan);

    // Reference: every cell should be (Slow * Fast) * cos(2π·i / Slow).
    const T scale = T(Slow) * T(Fast);
    const T tol = static_cast<T>(1e-4);
    T maxDiff = T(0);
    for (int i = 0; i < Slow; ++i)
    {
      const T expected = scale *
          static_cast<T>(std::cos(2.0 * M_PI *
                                  static_cast<double>(i) / Slow));
      for (int j = 0; j < Fast; ++j)
      {
        const T d = std::abs(OutAt<T>(out, Slow, Fast, i, j) - expected);
        if (d > maxDiff) maxDiff = d;
      }
    }
    if (maxDiff > tol)
      std::cout << "(max diff = " << maxDiff << ", tol = " << tol << ") ";
    return maxDiff <= tol;
  });

  // --- Test 4: execute_dft_c2r against a different buffer pair ----------
  // The plan should be reusable on caller-supplied in/out. We feed a DC
  // spectrum but to a NEW output buffer, and verify that the result lands
  // in that buffer (not the plan's captured default).
  Check("execute_dft_c2r with caller-supplied buffers", failures, [&]() {
    std::vector<Complex> spec2(spec.size(), Complex(0, 0));
    std::vector<T> out2(out.size(), T(0));
    const T dc = T(-1.25);
    At<T>(spec2, Slow, Fast, 0, 0) = Complex(dc, 0);

    // Plan captures `spec` / `out` as defaults — neither should be touched.
    std::fill(out.begin(), out.end(), T(99));
    std::fill(spec.begin(), spec.end(), Complex(0, 0));
    auto plan = FFT::plan_dft_c2r_2d(Slow, Fast, spec.data(), out.data(), 0u);

    FFT::execute_dft_c2r(plan, spec2.data(), out2.data());
    FFT::destroy_plan(plan);

    // The defaults must be untouched, the caller buffer holds the result.
    if (!MaxAbsDiff(out2, dc, static_cast<T>(1e-5))) return false;
    for (T v : out)
    {
      if (v != T(99))
      {
        std::cout << "(default buffer was clobbered) ";
        return false;
      }
    }
    return true;
  });

  // --- Test 5: padded-output plan honours the wider row stride -----------
  // A `widthPad` of 2 should make every output row 2 elements wider than
  // `Fast`. The valid `Fast` columns hold the IFFT result, and the trailing
  // 2 columns must be untouched (still the poison value 0xCD).
  Check("plan_guru_dft_c2r_output_padded honours pad", failures, [&]() {
    const int widthPad = 2;
    std::vector<Complex> spec3(static_cast<std::size_t>(Slow) * halfFast,
                                Complex(0, 0));
    std::vector<T> out3(
        static_cast<std::size_t>(Slow) * (Fast + widthPad), T(-77));
    const T dc = T(2);
    At<T>(spec3, Slow, Fast, 0, 0) = Complex(dc, 0);

    auto plan = FFT::plan_guru_dft_c2r_output_padded(
        Slow, Fast, widthPad, 0 /*heightPad*/,
        spec3.data(), out3.data(), 0u);
    FFT::execute(plan);
    FFT::destroy_plan(plan);

    const T tol = static_cast<T>(1e-5);
    for (int i = 0; i < Slow; ++i)
    {
      // valid cols 0..Fast-1 must equal `dc`
      for (int j = 0; j < Fast; ++j)
      {
        const T v = out3[i * (Fast + widthPad) + j];
        if (std::abs(v - dc) > tol)
        {
          std::cout << "(valid col mismatch at (" << i << "," << j << ")) ";
          return false;
        }
      }
      // pad cols Fast..Fast+widthPad-1 must still be the poison value
      for (int j = Fast; j < Fast + widthPad; ++j)
      {
        const T v = out3[i * (Fast + widthPad) + j];
        if (v != T(-77))
        {
          std::cout << "(pad col was written at (" << i << "," << j << ")) ";
          return false;
        }
      }
    }
    return true;
  });

  if (failures == 0)
    std::cout << "  OK\n\n";
  else
    std::cout << "  FAILURES: " << failures << "\n\n";
  return failures;
}

int main()
{
  std::cout << "FftwWrapper Eigen-backed shim smoke tests\n\n";

  int total = 0;
  total += RunForType<float>("float");
  total += RunForType<double>("double");

  std::cout << "Summary: " << (total == 0 ? "all tests passed"
                                          : "FAILURES present")
            << " (" << total << " failed)\n";
  return total == 0 ? 0 : 1;
}
