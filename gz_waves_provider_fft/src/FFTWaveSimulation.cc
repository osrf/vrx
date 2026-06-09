/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include "gz/sim/waves/FFTWaveSimulation.hh"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "gz/sim/waves/Wavefield.hh"

#include "EncinoWaves/All.h"

namespace gz::sim::waves
{

namespace
{
constexpr double kGravity = 9.80665;
constexpr double k2Pi    = 6.28318530717958647692;

//////////////////////////////////////////////////
/// Returns the integer log2 of n if n is a positive power of two; -1 otherwise.
int Log2Pow2(std::size_t n)
{
  if (n == 0 || (n & (n - 1)) != 0) return -1;
  int r = 0;
  while ((static_cast<std::size_t>(1u) << r) < n) ++r;
  return r;
}

//////////////////////////////////////////////////
/// Smallest power of two >= n (>= 1).
std::size_t CeilPow2(std::size_t n)
{
  std::size_t p = 1;
  while (p < n) p <<= 1;
  return p;
}

//////////////////////////////////////////////////
// ---- Human-readable names for the EncinoWaves model enums (logging) --------
const char *SpectrumName(EncinoWaves::SpectrumType t)
{
  switch (t)
  {
    case EncinoWaves::kPiersonMoskowitzSpectrum: return "pms";
    case EncinoWaves::kJONSWAPSpectrum:          return "jonswap";
    case EncinoWaves::kTMASpectrum:              return "tma";
    default:                                     return "?";
  }
}

//////////////////////////////////////////////////
const char *DispersionName(EncinoWaves::DispersionType t)
{
  switch (t)
  {
    case EncinoWaves::kDeepDispersion:        return "deep";
    case EncinoWaves::kFiniteDepthDispersion: return "finite";
    case EncinoWaves::kCapillaryDispersion:   return "capillary";
    default:                                  return "?";
  }
}

//////////////////////////////////////////////////
const char *SpreadingName(EncinoWaves::DirectionalSpreadingType t)
{
  switch (t)
  {
    case EncinoWaves::kPosCosThetaSqrDirectionalSpreading: return "poscos2";
    case EncinoWaves::kMitsuyasuDirectionalSpreading:      return "mitsuyasu";
    case EncinoWaves::kHasselmannDirectionalSpreading:     return "hasselmann";
    case EncinoWaves::kDonelanBannerDirectionalSpreading:  return "donelanbanner";
    default:                                               return "?";
  }
}

//////////////////////////////////////////////////
const char *FilterName(EncinoWaves::FilterType t)
{
  switch (t)
  {
    case EncinoWaves::kNullFilter:                   return "none";
    case EncinoWaves::kSmoothInvertibleBandPassFilter: return "bandpass";
    default:                                         return "?";
  }
}

//////////////////////////////////////////////////
// ---- <spectrum>/<spreading>/<dispersion> SDF strings -> EncinoWaves enums ---
// Names match the *Name() helpers above and the SDF tag values. Return false on
// an unrecognised value so the caller can warn and keep the Encino default.
bool SpectrumFromString(const std::string &s, EncinoWaves::SpectrumType &out)
{
  if (s == "pms" || s == "pm") out = EncinoWaves::kPiersonMoskowitzSpectrum;
  else if (s == "jonswap")     out = EncinoWaves::kJONSWAPSpectrum;
  else if (s == "tma")         out = EncinoWaves::kTMASpectrum;
  else return false;
  return true;
}

//////////////////////////////////////////////////
bool DispersionFromString(const std::string &s,
                          EncinoWaves::DispersionType &out)
{
  if (s == "deep")           out = EncinoWaves::kDeepDispersion;
  else if (s == "finite" || s == "finite_depth")
                             out = EncinoWaves::kFiniteDepthDispersion;
  else if (s == "capillary") out = EncinoWaves::kCapillaryDispersion;
  else return false;
  return true;
}

//////////////////////////////////////////////////
bool SpreadingFromString(const std::string &s,
                         EncinoWaves::DirectionalSpreadingType &out)
{
  if (s == "poscos2" || s == "poscossqr")
    out = EncinoWaves::kPosCosThetaSqrDirectionalSpreading;
  else if (s == "mitsuyasu")
    out = EncinoWaves::kMitsuyasuDirectionalSpreading;
  else if (s == "hasselmann")
    out = EncinoWaves::kHasselmannDirectionalSpreading;
  else if (s == "donelanbanner" || s == "donelan")
    out = EncinoWaves::kDonelanBannerDirectionalSpreading;
  else return false;
  return true;
}

//////////////////////////////////////////////////
// Map the SDF spectrum selectors and numeric knobs onto `ep`. Unknown selector
// values warn and leave the Encino default in place. The numeric knobs default
// (via WaveParameters) to Encino's own defaults, so an SDF that sets none of
// them reproduces the stock Horvath "good ocean" config.
void ApplyEncinoParams(EncinoWaves::Parametersf &ep, const WaveParameters &p)
{
  if (!SpectrumFromString(p.spectrum, ep.spectrum.type))
    std::cerr << "[FFTWaveSimulation] ignoring unknown <spectrum>='"
              << p.spectrum << "' (want pms|jonswap|tma)" << std::endl;
  if (!DispersionFromString(p.dispersion, ep.dispersion.type))
    std::cerr << "[FFTWaveSimulation] ignoring unknown <dispersion>='"
              << p.dispersion << "' (want deep|finite|capillary)" << std::endl;
  if (!SpreadingFromString(p.spreading, ep.directionalSpreading.type))
    std::cerr << "[FFTWaveSimulation] ignoring unknown <spreading>='"
              << p.spreading
              << "' (want poscos2|mitsuyasu|hasselmann|donelanbanner)"
              << std::endl;

  ep.depth         = static_cast<float>(p.depth);
  ep.fetch         = static_cast<float>(p.fetch);
  ep.directionalSpreading.swell = static_cast<float>(p.swell);
  ep.troughDamping = static_cast<float>(p.troughDamping);

  // Optional spectral band-pass over wavelength: keep wavelengths within
  // [filter_min_wl, filter_max_wl] m and roll off outside over filter_soft.
  // filter_min raises the suppression floor (0 = full cut); filter_invert turns
  // the band into a notch. Enabled when a band edge is given. The amplitude
  // calibration renormalises total energy, so the filter reshapes *which*
  // wavelengths survive rather than the overall sea height.
  if (p.filterMinWavelength > 0.0 || p.filterMaxWavelength > 0.0)
  {
    ep.filter.type = EncinoWaves::kSmoothInvertibleBandPassFilter;
    ep.filter.smallWavelength = static_cast<float>(p.filterMinWavelength);
    if (p.filterMaxWavelength > 0.0)
      ep.filter.bigWavelength = static_cast<float>(p.filterMaxWavelength);
    ep.filter.min    = static_cast<float>(p.filterMin);
    ep.filter.invert = p.filterInvert;
    // A zero soft width collapses the smoothstep edges (NaN spectrum); default
    // a positive transition from the lower cutoff when none was supplied.
    ep.filter.softWidth = (p.filterSoftWidth > 0.0)
        ? static_cast<float>(p.filterSoftWidth)
        : std::max(0.25f * ep.filter.smallWavelength, 1.0f);
  }
}

}  // namespace

//-----------------------------------------------------------------------------
// EncinoState — pimpl that holds the vendored Horvath-spectrum library's
// per-instance state. Defined here (not in the header) so EncinoWaves headers
// stay out of the public include surface.
//-----------------------------------------------------------------------------
struct FFTWaveSimulation::EncinoState
{
  EncinoWaves::Parametersf params;
  std::unique_ptr<EncinoWaves::InitialStatef> initial;
  std::unique_ptr<EncinoWaves::Propagationf>   propagation;
  std::unique_ptr<EncinoWaves::PropagatedStatef> state;
};

FFTWaveSimulation::~FFTWaveSimulation() = default;

FFTWaveSimulation::FFTWaveSimulation() = default;

//////////////////////////////////////////////////
FFTWaveSimulation::FFTWaveSimulation(const WaveParameters &p,
                                     double tile,
                                     std::size_t grid,
                                     std::uint32_t seed)
{
  // The constructor's tile/grid/seed arguments override whatever the params
  // struct carries (callers and tests rely on this). Fold them into a params
  // copy and route through the single SetParameters setup path.
  WaveParameters q = p;
  q.tileSize = tile;
  q.gridSize = grid;
  q.seed     = seed;
  this->SetParameters(q);
}

//////////////////////////////////////////////////
void FFTWaveSimulation::SetParameters(const WaveParameters &_params)
{
  // Resolve <sea_state> (if set) into period/gain before configuring.
  const WaveParameters p = WithSeaState(_params);
  this->tileSize_ = p.tileSize;
  this->gridSize_ = p.gridSize;
  this->gain_     = p.gain;
  this->tau_      = p.tau;
  const std::uint32_t seed = p.seed;

  // EncinoWaves requires a power-of-two grid; round up if the SDF asks for
  // something else.
  if (Log2Pow2(this->gridSize_) < 0)
  {
    const std::size_t adjusted = CeilPow2(this->gridSize_);
    std::cerr << "[FFTWaveSimulation] grid_size=" << this->gridSize_
              << " is not a power of two (required by the FFT spectrum); using "
              << adjusted << std::endl;
    this->gridSize_ = adjusted;
  }

  // PMS relation: peak omega <-> wind speed at 19.5 m. period -> omegaP -> V19.
  const double omegaP = k2Pi / p.period;
  this->windSpeed_ = 0.879 * kGravity / omegaP;

  const int N = static_cast<int>(this->gridSize_);
  this->heightGrid_ = Eigen::MatrixXd::Zero(N, N);
  this->dispXGrid_  = Eigen::MatrixXd::Zero(N, N);
  this->dispYGrid_  = Eigen::MatrixXd::Zero(N, N);
  this->minEGrid_   = Eigen::MatrixXd::Constant(N, N, 1.0);  // 1 = no foam

  // Build the EncinoWaves spectral state -- the spectral engine the fft system
  // is built on. The <spectrum>/<spreading>/<dispersion> SDF selectors choose
  // the spectral models (default TMA + Hasselmann + capillary).
  this->encino_ = std::make_unique<EncinoState>();
  auto &ep = this->encino_->params;
  ep.resolutionPowerOfTwo = Log2Pow2(this->gridSize_);
  ep.domain        = static_cast<float>(this->tileSize_);
  ep.windSpeed     = static_cast<float>(this->windSpeed_);
  ep.amplitudeGain = static_cast<float>(this->gain_);
  ep.random.seed   = static_cast<int>(seed);

  // Map the SDF spectrum selectors + numeric/filter knobs onto Encino.
  ApplyEncinoParams(ep, p);

  this->encino_->initial =
      std::make_unique<EncinoWaves::InitialStatef>(ep);
  this->encino_->propagation =
      std::make_unique<EncinoWaves::Propagationf>(ep, /*nthreads=*/-1);
  this->encino_->state =
      std::make_unique<EncinoWaves::PropagatedStatef>(ep);

  // --- Physics-based amplitude calibration -------------------------------
  // EncinoWaves' amplitudeGain only feeds its (here unused) normal computation,
  // not the height field, and its intrinsic variance does not correspond to a
  // physical sea state at our wind speeds. Measure the intrinsic RMS once (one
  // propagation past the ramp) and rescale so the significant wave height
  // follows the standard fully-developed Pierson-Moskowitz relation
  // Hs = 0.21 * V19.5^2 / g, i.e. sigma = Hs/4. The selected spectrum still
  // sets the spectral *shape*; this only fixes the overall energy.
  {
    using RowMatF = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,
                                  Eigen::RowMajor>;
    this->encino_->propagation->propagate(
        this->encino_->params, *this->encino_->initial,
        *this->encino_->state, 10.0f);
    const int M = ep.resolution();
    const double sigmaEncino = std::sqrt(
        Eigen::Map<const RowMatF>(this->encino_->state->Height.cdata(),
                                  M, M).cast<double>().array()
            .square().mean());
    const double sigmaTarget =
        0.21 / (4.0 * kGravity) * this->windSpeed_ * this->windSpeed_;
    this->encinoScale_ =
        (sigmaEncino > 1e-9) ? (sigmaTarget / sigmaEncino) : 1.0;
  }

  std::cout << "[FFTWaveSimulation] EncinoWaves spectrum library active "
            << "(res=" << ep.resolution() << " domain=" << ep.domain
            << "m wind=" << ep.windSpeed << "m/s seed=" << ep.random.seed
            << " spectrum=" << SpectrumName(ep.spectrum.type)
            << " dispersion=" << DispersionName(ep.dispersion.type)
            << " spreading=" << SpreadingName(ep.directionalSpreading.type)
            << " depth=" << ep.depth << "m fetch=" << ep.fetch << "km"
            << " swell=" << ep.directionalSpreading.swell
            << " troughDamp=" << ep.troughDamping
            << " filter=" << FilterName(ep.filter.type)
            << " ampCalib=" << this->encinoScale_
            << " targetHs=" << (4.0 * 0.21 / (4.0 * kGravity) *
                                this->windSpeed_ * this->windSpeed_)
            << "m)" << std::endl;

  this->Update(0.0);
}

//////////////////////////////////////////////////
double FFTWaveSimulation::Ramp(double t) const
{
  if (this->tau_ <= 0.0)
    return 1.0;
  return 1.0 - std::exp(-t / this->tau_);
}

//////////////////////////////////////////////////
void FFTWaveSimulation::Update(double t)
{
  // Idempotent: the field at time t is deterministic, so a repeat call for the
  // same t is a no-op. Lets the Waves system and any number of WaveBuoyancy
  // consumers that advance the same instance to the same tick share a single
  // recompute instead of each paying for a full propagation.
  if (t == this->lastUpdateT_)
    return;
  this->lastUpdateT_ = t;

  if (!this->encino_ || !this->encino_->propagation)
    return;  // default-constructed, not yet configured

  const int N = static_cast<int>(this->gridSize_);

  this->encino_->propagation->propagate(
      this->encino_->params,
      *this->encino_->initial,
      *this->encino_->state,
      static_cast<float>(t));

  // Combined output scale per Update:
  //  * ramp         — fade the field in over `tau`;
  //  * encinoScale_ — physics-based amplitude calibration to a PM sea state
  //                   (Encino's amplitudeGain doesn't scale the height);
  //  * gain_        — the SDF <gain> user multiplier (a no-op via Encino's
  //                   amplitudeGain, so we apply it here to make it work).
  const double scale = this->Ramp(t) * this->encinoScale_ * this->gain_;

  // Encino stores its spatial fields row-major in float; our grids are
  // column-major in double. Map+cast assignment lets Eigen vectorize the
  // conversion + layout swap; the scale folds into the same expression.
  using RowMatF = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,
                                Eigen::RowMajor>;
  this->heightGrid_ = Eigen::Map<const RowMatF>(
      this->encino_->state->Height.cdata(), N, N).cast<double>() * scale;
  this->dispXGrid_ = Eigen::Map<const RowMatF>(
      this->encino_->state->Dx.cdata(), N, N).cast<double>() * scale;
  this->dispYGrid_ = Eigen::Map<const RowMatF>(
      this->encino_->state->Dy.cdata(), N, N).cast<double>() * scale;

  // Foam: Encino computes MinE = -(min eigenvalue of the displacement
  // Jacobian) at its internal amplitude. At our calibrated amplitude the
  // minimum eigenvalue is 1 - scale*(MinE + 1). Jacobian() bilinear-samples
  // this; Eval::FoamMask turns values below its threshold into whitecaps. At
  // t=0 (scale=0) the surface is flat -> eigenvalue 1 -> no foam.
  this->minEGrid_ = (1.0 - scale *
      (Eigen::Map<const RowMatF>(this->encino_->state->MinE.cdata(), N, N)
          .cast<double>().array() + 1.0)).matrix();
}

//////////////////////////////////////////////////
double FFTWaveSimulation::BilinearSample(const Eigen::MatrixXd &grid,
                                         double x, double y) const
{
  const int N = static_cast<int>(this->gridSize_);
  const double L = this->tileSize_;

  // Wrap query into [0, L). Grid cell (i, j) corresponds to world position
  // (i·L/N, j·L/N) — the IFFT output's natural layout, no centring shift.
  auto wrap = [L](double v) {
    double r = std::fmod(v, L);
    if (r < 0.0) r += L;
    return r;
  };
  const double wx = wrap(x);
  const double wy = wrap(y);

  // Grid index in [0, N).
  const double fi = wx / L * N;
  const double fj = wy / L * N;
  int i0 = static_cast<int>(std::floor(fi)) % N;
  int j0 = static_cast<int>(std::floor(fj)) % N;
  if (i0 < 0) i0 += N;
  if (j0 < 0) j0 += N;
  const int i1 = (i0 + 1) % N;
  const int j1 = (j0 + 1) % N;
  const double fx = fi - std::floor(fi);
  const double fy = fj - std::floor(fj);

  return grid(i0, j0) * (1 - fx) * (1 - fy)
       + grid(i1, j0) * fx       * (1 - fy)
       + grid(i0, j1) * (1 - fx) * fy
       + grid(i1, j1) * fx       * fy;
}

//////////////////////////////////////////////////
double FFTWaveSimulation::Elevation(double x, double y, double /*t*/) const
{
  // Caller is expected to have called Update(t) ≤ this tick (the Waves
  // system does this in PreUpdate). The bilinear sample is time-free.
  return this->BilinearSample(this->heightGrid_, x, y);
}

//////////////////////////////////////////////////
gz::math::Vector3d FFTWaveSimulation::ParticleVelocity(
  double /*x*/, double /*y*/, double /*t*/) const
{
  // Stage 2: stub. Stage 4+ will compute via additional FFTs of i·k·h(k,t)
  // for the horizontal components and ∂η/∂t for the vertical. Returning
  // zero here means drag against still water — not physically accurate yet.
  return gz::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
gz::math::Vector3d FFTWaveSimulation::Normal(
  double x, double y, double /*t*/) const
{
  // Finite-difference normal from the height grid. Good enough for unit
  // testing; Stage 4 will produce derivative grids via FFT.
  const double L = this->tileSize_;
  const double h = L / static_cast<double>(this->gridSize_);
  const double dx =
    (this->BilinearSample(this->heightGrid_, x + h, y) -
     this->BilinearSample(this->heightGrid_, x - h, y)) / (2.0 * h);
  const double dy =
    (this->BilinearSample(this->heightGrid_, x, y + h) -
     this->BilinearSample(this->heightGrid_, x, y - h)) / (2.0 * h);
  gz::math::Vector3d n{-dx, -dy, 1.0};
  n.Normalize();
  return n;
}

//////////////////////////////////////////////////
double FFTWaveSimulation::Jacobian(double x, double y, double /*t*/) const
{
  // Bilinear-sample the per-cell minimum eigenvalue of the displacement
  // Jacobian (1 = flat, < 1 → folding). Eval::FoamMask turns values below its
  // threshold into whitecap intensity, so foam appears on the pinched crests.
  return this->BilinearSample(this->minEGrid_, x, y);
}

//////////////////////////////////////////////////
const WaveField2D *FFTWaveSimulation::Field() const
{
  // Repopulate the view from the current grids each call: Update reallocates
  // them, so cached data() pointers would dangle. Eigen is column-major,
  // matching WaveField2D's documented (i + j*N) layout. The renderer
  // finite-diffs η for normals; foam is the Encino MinE folding metric.
  this->field_.n    = this->gridSize_;
  this->field_.tile = this->tileSize_;
  this->field_.dz   = this->heightGrid_.data();
  this->field_.dx   = this->dispXGrid_.data();
  this->field_.dy   = this->dispYGrid_.data();
  this->field_.foam = this->minEGrid_.data();
  return &this->field_;
}

//////////////////////////////////////////////////
/// \brief Factory used to register the FFT engine under the "fft" token (see
/// RegisterWaveEngineFactory). Returns a default-constructed engine; the caller
/// applies SetParameters.
std::shared_ptr<IWaveField> MakeFFTWaveField()
{
  return std::make_shared<FFTWaveSimulation>();
}

}  // namespace gz::sim::waves
