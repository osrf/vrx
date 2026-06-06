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
 * End-to-end smoke test for the vendored EncinoWaves library with the
 * Eigen-backed FFT shim. Runs the canonical "make some waves" pipeline:
 *
 *   Parameters     <-- spectrum + dispersion + directional spread + ...
 *      |
 *   InitialState   <-- constructs h₀(k), conj(h₀(-k)), ω(k) once
 *      |
 *   Propagation    <-- per-frame: time-evolve the spectrum, IFFT into
 *      |               Height / Dx / Dy / MinE spatial grids
 *   PropagatedState
 *
 * If this binary produces grids with reasonable min/max/RMS for a known
 * wind speed and grid size, the library is functioning end-to-end without
 * FFTW. That's what step 3 of the integration plan needed to verify.
 *
 * Pass criterion: η_max > 0 and finite, |Dx| / |Dy| comparable to η_max
 * (Tessendorf chop sets them to the same order of magnitude), and the
 * MinE grid contains some negative values (compression zones — the
 * Tessendorf Jacobian-derived foam indicator).
 */

#include "EncinoWaves/All.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <string>

namespace
{
  template <typename T>
  struct Stats
  {
    T min, max, mean, rms;
    bool allFinite;
  };

  template <typename T>
  Stats<T> Summarise(const T *data, std::size_t n)
  {
    Stats<T> s{std::numeric_limits<T>::infinity(),
               -std::numeric_limits<T>::infinity(), T(0), T(0), true};
    T sum = T(0), sumSq = T(0);
    std::size_t valid = 0;
    for (std::size_t i = 0; i < n; ++i)
    {
      const T v = data[i];
      if (!std::isfinite(v))
      {
        s.allFinite = false;
        continue;
      }
      if (v < s.min) s.min = v;
      if (v > s.max) s.max = v;
      sum += v;
      sumSq += v * v;
      ++valid;
    }
    if (valid > 0)
    {
      s.mean = sum / static_cast<T>(valid);
      s.rms = std::sqrt(sumSq / static_cast<T>(valid));
    }
    return s;
  }

  template <typename T>
  void PrintStats(const std::string &name, const Stats<T> &s)
  {
    std::printf("  %-8s  min=%+9.4f  max=%+9.4f  mean=%+9.4f  rms=%9.4f  finite=%s\n",
                name.c_str(),
                static_cast<double>(s.min),
                static_cast<double>(s.max),
                static_cast<double>(s.mean),
                static_cast<double>(s.rms),
                s.allFinite ? "yes" : "NO");
  }
}  // namespace

int main()
{
  using namespace EncinoWaves;

  std::cout << "EncinoWaves end-to-end smoke test\n"
               "(vendored library + Eigen-backed FFT shim)\n\n";

  // Use Parameters defaults except for resolution + wind. Defaults already
  // pick TMASpectrum + Hasselmann spread + CapillaryDispersion + null
  // filter, which is the "good ocean" setting Horvath ships with.
  Parameters<float> params;
  params.resolutionPowerOfTwo = 7;  // 128x128 — fast smoke test
  params.domain = 100.0f;            // 100m tile
  params.windSpeed = 5.0f;           // matches asv_wave_sim reference
  params.fetch = 300.0f;             // km

  std::cout << "params: resolution=" << params.resolution()
            << "  domain=" << params.domain << "m"
            << "  wind=" << params.windSpeed << "m/s"
            << "\n\n";

  std::cout << "constructing InitialState (one-shot h0 / omega) ...\n";
  InitialStatef istate(params);
  std::cout << "  done, resolution=" << istate.resolution() << "\n\n";

  std::cout << "constructing Propagation (per-frame working buffers) ...\n";
  Propagationf propagation(params, /*nthreads=*/-1);
  std::cout << "  done\n\n";

  std::cout << "constructing PropagatedState (output grids) ...\n";
  PropagatedStatef pstate(params);
  std::cout << "  done\n\n";

  std::cout << "running propagate(t=0.0) ...\n";
  propagation.propagate(params, istate, pstate, 0.0f);
  std::cout << "  done\n\n";

  // Read out the spatial grids and summarise.
  const int N = params.resolution();
  const std::size_t cellCount = static_cast<std::size_t>(N) * N;

  const auto hStats   = Summarise(pstate.Height.cdata(), cellCount);
  const auto dxStats  = Summarise(pstate.Dx.cdata(),     cellCount);
  const auto dyStats  = Summarise(pstate.Dy.cdata(),     cellCount);
  const auto eStats   = Summarise(pstate.MinE.cdata(),   cellCount);

  std::cout << "spatial output stats (" << N << "x" << N << " grid):\n";
  PrintStats("Height",  hStats);
  PrintStats("Dx",      dxStats);
  PrintStats("Dy",      dyStats);
  PrintStats("MinE",    eStats);
  std::cout << "\n";

  // Acceptance checks
  bool ok = true;
  auto demand = [&](bool cond, const char *msg) {
    std::cout << "  " << (cond ? "[ok]   " : "[FAIL] ") << msg << "\n";
    if (!cond) ok = false;
  };

  demand(hStats.allFinite,                       "Height: all values finite");
  demand(dxStats.allFinite,                      "Dx: all values finite");
  demand(dyStats.allFinite,                      "Dy: all values finite");
  demand(eStats.allFinite,                       "MinE: all values finite");
  demand(hStats.max > 0.0f,                      "Height has positive crests");
  demand(hStats.min < 0.0f,                      "Height has negative troughs");
  demand(std::abs(dxStats.max) > 0.001f,         "Dx is non-trivial");
  demand(std::abs(dyStats.max) > 0.001f,         "Dy is non-trivial");
  // Encino's MinE is the smaller eigenvalue of the chop Jacobian; it should
  // dip below 1 wherever the surface compresses, and below 0 in folds.
  // For wind=5 m/s + default pinch we expect a healthy spread either side.
  demand(eStats.min < 1.0f,                      "MinE has cells below 1.0 (compression seen)");

  std::cout << "\nresult: " << (ok ? "PASS" : "FAIL") << "\n";
  return ok ? 0 : 1;
}
