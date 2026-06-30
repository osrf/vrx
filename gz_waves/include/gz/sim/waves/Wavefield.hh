/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef GZ_SIM_WAVES_WAVEFIELD_HH_
#define GZ_SIM_WAVES_WAVEFIELD_HH_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <istream>
#include <limits>
#include <memory>
#include <ostream>
#include <string>

#include <gz/math/Helpers.hh>  // GZ_PI

namespace gz::sim::waves
{

/// \brief The polymorphic wave-field engine, forward-declared. Consumers reach
/// it only through the `Wavefield` component (which holds a `shared_ptr`) and
/// the `Eval.hh` free functions, so they never need the full interface — an
/// incomplete type suffices for the `shared_ptr` member in `WavefieldData`.
/// Producers and the `Eval` implementations include `WaveSimulation.hh` for
/// the full definition.
class IWaveField;

/// \brief Human-facing wave parameters parsed from SDF or set programmatically.
struct WaveParameters
{
  /// \brief Spectrum / sampling model: "PMS" or "CWR".
  std::string model{"PMS"};

  /// \brief Number of component waves (for Gerstner) or grid samples (for
  /// future FFT layouts).
  std::size_t number{3};

  /// \brief Mean wave period [s].
  double period{5.0};

  /// \brief Mean wave amplitude [m] (CWR model only).
  double amplitude{0.0};

  /// \brief Mean wave direction [rad] from +X.
  double direction{0.0};

  /// \brief Angle between mean direction and largest/smallest component [rad].
  double angle{0.4};

  /// \brief Scale between mean and largest/smallest component waves.
  double scale{1.1};

  /// \brief Wave steepness in [0, 1].
  double steepness{0.0};

  /// \brief Phase [rad] (common to all components).
  double phase{0.0};

  /// \brief Time constant for the startup ramp `(1 - exp(-t/tau))` [s].
  double tau{2.0};

  /// \brief PMS amplitude multiplier (unitless).
  double gain{1.0};

  // ---- FFT-only parameters (ignored by the Gerstner backend) -------------

  /// \brief Physical tile extent [m] along each axis for the periodic
  /// FFT wave field.
  double tileSize{200.0};

  /// \brief Number of grid samples per axis for the FFT. Must be a power of
  /// two (FFT/IFFT requirement). 64/128/256 typical.
  std::size_t gridSize{128};

  /// \brief RNG seed used to generate the stochastic spectral amplitudes.
  /// Same seed → bit-for-bit identical wave field.
  std::uint32_t seed{0};

  /// \brief Tessendorf "choppiness" multiplier applied to the horizontal
  /// displacement field in the visual shader. Negative values bunch
  /// particles toward wave crests (the canonical choice). Typical range
  /// [-2, 0]; 0 disables choppy displacement.
  double choppiness{-1.0};

  /// \brief FFT spectrum model (EncinoWaves): "pms", "jonswap", or "tma".
  std::string spectrum{"tma"};

  /// \brief FFT directional spreading (EncinoWaves): "poscos2", "mitsuyasu",
  /// "hasselmann", or "donelanbanner".
  std::string spreading{"hasselmann"};

  /// \brief FFT dispersion relation (EncinoWaves): "deep", "finite", or
  /// "capillary".
  std::string dispersion{"capillary"};

  /// \brief FFT water depth [m] (EncinoWaves dispersion input).
  double depth{100.0};

  /// \brief FFT wind fetch [km] (EncinoWaves spectrum input).
  double fetch{300.0};

  /// \brief FFT swell elongation (EncinoWaves directional spreading); 0 = none.
  double swell{0.0};

  /// \brief FFT breaking-wave trough damping in [0, 1] (EncinoWaves); 0 = none.
  double troughDamping{0.0};

  /// \brief FFT band-pass filter lower edge [m]. > 0 enables the band-pass,
  /// keeping wavelengths within [filterMinWavelength, filterMaxWavelength].
  double filterMinWavelength{0.0};

  /// \brief FFT band-pass filter upper edge [m] (0 = no upper bound).
  double filterMaxWavelength{0.0};

  /// \brief FFT band-pass transition width [m] (0 = auto from the lower edge).
  double filterSoftWidth{0.0};

  /// \brief FFT band-pass suppression floor in [0, 1] (0 = full cut outside).
  double filterMin{0.0};

  /// \brief FFT band-pass invert: band-stop (notch) instead of band-pass.
  bool filterInvert{false};

  /// \brief WMO sea state code (0-9). A convenience that, when set, makes the
  /// engine reproduce that sea state's significant wave height and peak period
  /// (see WithSeaState / SeaStateFromCode). -1 (default) means "unset": use the
  /// explicit period/gain above instead.
  int seaState{-1};

  /// \brief Gravitational acceleration magnitude [m/s²] driving the wave
  /// physics (dispersion, spectrum, sea-state period). Read from the world via
  /// the `gz::sim::World::Gravity` API in `WavesSystemBase::Configure` (not
  /// parsed from the plugin SDF) so the waves stay consistent with buoyancy and
  /// rigid-body dynamics; defaults to gz-sim's world default (9.8) when the
  /// world exposes no gravity.
  double gravity{9.8};
};

/// \brief Single source of truth for the `WaveParameters` fields shared by
/// serialization, SDF parsing (`<wave>` tags), and the `set_parameters`
/// service. Each row is `X(member, "name", KIND)` where KIND is one of:
///   DBL  → double          SIZE → std::size_t     U32 → std::uint32_t
///   INT  → int             STR  → std::string     BOOL → bool
/// Listed in struct-declaration order so `operator<<` / `operator>>` stream the
/// fields in this exact order. `gravity` is intentionally absent: it is sourced
/// from the world (not SDF or the service) and is streamed explicitly by the
/// operators. Add a parameter once here and it is picked up by all four sites.
#define GZ_WAVES_PARAM_TABLE(X)                  \
  X(model,               "model",          STR)  \
  X(number,              "number",         SIZE) \
  X(period,              "period",         DBL)  \
  X(amplitude,           "amplitude",      DBL)  \
  X(direction,           "direction",      DBL)  \
  X(angle,               "angle",          DBL)  \
  X(scale,               "scale",          DBL)  \
  X(steepness,           "steepness",      DBL)  \
  X(phase,               "phase",          DBL)  \
  X(tau,                 "tau",            DBL)  \
  X(gain,                "gain",           DBL)  \
  X(tileSize,            "tile_size",      DBL)  \
  X(gridSize,            "grid_size",      SIZE) \
  X(seed,                "seed",           U32)  \
  X(choppiness,          "choppiness",     DBL)  \
  X(spectrum,            "spectrum",       STR)  \
  X(spreading,           "spreading",      STR)  \
  X(dispersion,          "dispersion",     STR)  \
  X(depth,               "depth",          DBL)  \
  X(fetch,               "fetch",          DBL)  \
  X(swell,               "swell",          DBL)  \
  X(troughDamping,       "trough_damping", DBL)  \
  X(filterMinWavelength, "filter_min_wl",  DBL)  \
  X(filterMaxWavelength, "filter_max_wl",  DBL)  \
  X(filterSoftWidth,     "filter_soft",    DBL)  \
  X(filterMin,           "filter_min",     DBL)  \
  X(filterInvert,        "filter_invert",  BOOL) \
  X(seaState,            "sea_state",      INT)

/// \brief Canonical sea-state descriptor: significant wave height, peak period,
/// and the fully-developed wind speed that produces them.
struct SeaStateSpec
{
  /// \brief Significant wave height Hs [m].
  double significantWaveHeight{0.0};

  /// \brief Peak period Tp [s].
  double peakPeriod{0.0};

  /// \brief Fully-developed 19.5 m wind speed [m/s].
  double windSpeed{0.0};
};

/// \brief Map a WMO Sea State code (0-9) to representative wave parameters.
///
/// The code's defining quantity is the significant-wave-height band; we take
/// the band midpoint as the representative Hs, then derive the wind speed and
/// peak period for a fully-developed Pierson-Moskowitz sea:
///   Hs = 0.21 * V19.5^2 / g,  omega_p = 0.879 * g / V19.5,  Tp = 2*pi/omega_p
///
/// Canonical source: WMO Sea State code (code table 3700), WMO Manual on Codes
/// (WMO-No. 306). Accessible table: https://en.wikipedia.org/wiki/Sea_state
///
/// \param[in]  _code WMO sea state code (0-9).
/// \param[out] _out  Filled with the representative spec on success.
/// \param[in]  _g    Gravity magnitude [m/s²] used in the PM relations above.
/// \return true on success; false (leaving _out untouched) for an out-of-range
/// code.
inline bool SeaStateFromCode(int _code, SeaStateSpec &_out, double _g = 9.8)
{
  // Representative Hs [m] per WMO code: 0 calm/glassy, 1 calm/rippled,
  // 2 smooth, 3 slight, 4 moderate, 5 rough, 6 very rough, 7 high,
  // 8 very high, 9 phenomenal (band midpoints; 9 is open-ended).
  static constexpr double kHs[10] =
      {0.0, 0.05, 0.3, 0.875, 1.875, 3.25, 5.0, 7.5, 11.5, 16.0};
  if (_code < 0 || _code > 9)
    return false;
  const double g = _g;
  const double hs = kHs[_code];
  const double v  = std::sqrt(hs * g / 0.21);
  const double tp = (v > 0.0) ? (2.0 * GZ_PI * v / (0.879 * g)) : 0.0;
  _out = SeaStateSpec{hs, tp, v};
  return true;
}

/// \brief Return a copy of _p with the sea state (if set) applied: the peak
/// period drives <period>, <gain> is normalised to 1 so the field shows the
/// physical significant wave height (code 0 -> gain 0, i.e. flat), and for the
/// CWR model the regular-wave <amplitude> is set to Hs/2. An unset (-1) or
/// out-of-range seaState returns _p unchanged. Each provider calls this at the
/// top of SetParameters, so <sea_state> works for every backend.
/// \param[in] _p Wave parameters to derive from.
/// \return A copy of _p with the sea state applied (or _p unchanged if unset).
inline WaveParameters WithSeaState(const WaveParameters &_p)
{
  WaveParameters p = _p;
  SeaStateSpec s;
  if (p.seaState < 0 || !SeaStateFromCode(p.seaState, s, p.gravity))
    return p;
  if (s.significantWaveHeight <= 0.0)
  {
    p.gain = 0.0;            // sea state 0: calm / glassy -> flat
    return p;
  }
  p.period = s.peakPeriod;
  p.gain   = 1.0;            // physical Hs (no artistic boost)
  if (p.model == "CWR")
    p.amplitude = 0.5 * s.significantWaveHeight;
  return p;
}

/// \brief Soft-start ramp factor `1 - exp(-t/tau)` in [0, 1] (`tau <= 0`
/// disables it, returning 1). Shared by the wave engines so the startup
/// transient is identical across backends.
/// \param[in] _t   Elapsed simulation time [s].
/// \param[in] _tau Ramp time constant [s] (`<= 0` disables the ramp).
/// \return Ramp factor in [0, 1].
inline double StartupRamp(double _t, double _tau)
{
  if (_tau <= 0.0)
    return 1.0;
  return 1.0 - std::exp(-_t / _tau);
}

/// \brief State held by the `Wavefield` ECM component. Wraps a polymorphic
/// `IWaveField`. Consumers use the free functions in `Eval.hh` to query the
/// wave field; they don't see the backend directly.
struct WavefieldData
{
  /// \brief Backend identifier — "gerstner" or "fft". Serialized so the GUI
  /// process can reconstruct the right backend on deserialization.
  std::string algorithm{"gerstner"};

  /// \brief Inputs to the backend.
  WaveParameters params;

  /// \brief The polymorphic wave-field implementation. Constructed by the wave
  /// source system from `algorithm` + `params`. Consumers query via
  /// `Eval::*` free functions.
  std::shared_ptr<IWaveField> simulation;

  /// \brief Monotonically increasing counter, bumped whenever the data is
  /// rewritten. Consumers (visual upload, etc.) compare against their last
  /// seen value to cheaply detect changes.
  std::uint64_t generation{0};

  /// \brief Server-side wave Update rate [Hz]. Mirrored from
  /// `<update_rate>` so the GUI visual can throttle its own per-frame
  /// Update to match the server's cadence — keeping them in step lets
  /// the user retune a single SDF knob without the visual silently
  /// drifting ahead.
  double updateRate{30.0};
};

/// \brief Stream-out for ECM serialization. Writes the algorithm + the
/// parameter struct. The simulation pointer is not serialized — it's
/// reconstructed on the receiving side via the factory in WaveSimulation.hh.
/// \param[in,out] _os Output stream to write to.
/// \param[in]     _d  Wave-field state to serialize.
/// \return The output stream `_os`.
inline std::ostream &operator<<(std::ostream &_os, const WavefieldData &_d)
{
  // Emit doubles at full round-trip precision so the replicated copy rebuilds a
  // bit-for-bit identical engine (the seed-reproducibility contract relies on
  // this). Restore the caller's precision afterwards.
  const auto oldPrecision =
    _os.precision(std::numeric_limits<double>::max_digits10);
  // Strings go through std::quoted so an empty value or one containing spaces
  // (e.g. set via the runtime service) can't desync the positional field
  // stream on read-back. Field order/coverage comes from GZ_WAVES_PARAM_TABLE.
#define GZ_WAVES_WR_STR(m)  _os << std::quoted(_d.params.m) << ' ';
#define GZ_WAVES_WR_DBL(m)  _os << _d.params.m << ' ';
#define GZ_WAVES_WR_SIZE(m) _os << _d.params.m << ' ';
#define GZ_WAVES_WR_U32(m)  _os << _d.params.m << ' ';
#define GZ_WAVES_WR_INT(m)  _os << _d.params.m << ' ';
#define GZ_WAVES_WR_BOOL(m) _os << _d.params.m << ' ';
#define GZ_WAVES_WR(member, name, kind) GZ_WAVES_WR_##kind(member)
  _os << std::quoted(_d.algorithm) << ' ' << _d.generation << ' ';
  GZ_WAVES_PARAM_TABLE(GZ_WAVES_WR)
  _os << _d.params.gravity << ' ' << _d.updateRate << ' ';
#undef GZ_WAVES_WR
#undef GZ_WAVES_WR_STR
#undef GZ_WAVES_WR_DBL
#undef GZ_WAVES_WR_SIZE
#undef GZ_WAVES_WR_U32
#undef GZ_WAVES_WR_INT
#undef GZ_WAVES_WR_BOOL
  _os.precision(oldPrecision);
  return _os;
}

/// \brief Stream-in for ECM deserialization. Reads the *recipe* (algorithm +
/// parameters) only; `simulation` is left null. Consumers that need a live wave
/// field build their own engine from the deserialized params via
/// `CreateWaveSimulation` (e.g. WaterVisual) — so each consumer owns a private
/// instance and there is no shared, process-global engine.
/// \param[in,out] _is Input stream to read from.
/// \param[out]    _d  Wave-field state to populate (recipe only; engine null).
/// \return The input stream `_is`.
inline std::istream &operator>>(std::istream &_is, WavefieldData &_d)
{
  // Field order/coverage mirrors operator<< via GZ_WAVES_PARAM_TABLE.
#define GZ_WAVES_RD_STR(m)  _is >> std::quoted(_d.params.m);
#define GZ_WAVES_RD_DBL(m)  _is >> _d.params.m;
#define GZ_WAVES_RD_SIZE(m) _is >> _d.params.m;
#define GZ_WAVES_RD_U32(m)  _is >> _d.params.m;
#define GZ_WAVES_RD_INT(m)  _is >> _d.params.m;
#define GZ_WAVES_RD_BOOL(m) _is >> _d.params.m;
#define GZ_WAVES_RD(member, name, kind) GZ_WAVES_RD_##kind(member)
  _is >> std::quoted(_d.algorithm) >> _d.generation;
  GZ_WAVES_PARAM_TABLE(GZ_WAVES_RD)
  _is >> _d.params.gravity >> _d.updateRate;
#undef GZ_WAVES_RD
#undef GZ_WAVES_RD_STR
#undef GZ_WAVES_RD_DBL
#undef GZ_WAVES_RD_SIZE
#undef GZ_WAVES_RD_U32
#undef GZ_WAVES_RD_INT
#undef GZ_WAVES_RD_BOOL
  // The recipe is restored; build no engine. `simulation` stays null so no two
  // consumers ever share a process-global instance.
  _d.simulation.reset();
  return _is;
}

}  // namespace gz::sim::waves

#endif  // GZ_SIM_WAVES_WAVEFIELD_HH_
