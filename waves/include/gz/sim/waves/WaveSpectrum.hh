/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_WAVES_WAVESPECTRUM_HH_
#define GZ_SIM_WAVES_WAVESPECTRUM_HH_

#include "gz/sim/waves/Wavefield.hh"

namespace gz::sim::waves
{

/// \brief Compute deep-water angular frequency from wavenumber: ω = √(g·k).
double DeepWaterDispersionToOmega(double wavenumber);

/// \brief Compute deep-water wavenumber from angular frequency: k = ω²/g.
double DeepWaterDispersionToWavenumber(double omega);

/// \brief Pierson-Moskowitz spectral density at `omega` for peak `omegaP`.
/// Returns S(ω) in m²·s.
double PiersonMoskowitz(double omega, double omegaP);

/// \brief Populate the derived component arrays in `data` from `data.params`.
///
/// Reads `params` (model, period, gain, direction, etc.) and writes the
/// `amplitudes`, `wavenumbers`, `angularFrequencies`, `steepnesses`, and
/// `directions` arrays. Resizes each to `params.number` entries.
///
/// Supports `model == "PMS"` (Pierson-Moskowitz sampling) and
/// `model == "CWR"` (constant wavelength ratio, parametric). Unknown models
/// leave the derived arrays empty and log a warning.
///
/// Bumps `data.generation` on success.
void SampleSpectrum(WavefieldData &data);

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_WAVESPECTRUM_HH_
