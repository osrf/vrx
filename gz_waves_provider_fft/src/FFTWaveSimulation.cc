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
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>

#include <unsupported/Eigen/FFT>

#include <gz/plugin/Register.hh>

#include "gz/sim/waves/Wavefield.hh"

#ifdef GZ_WAVES_WITH_ENCINO
#include "EncinoWaves/All.h"
#endif

namespace gz::sim::waves
{

namespace
{
constexpr double kGravity = 9.80665;
constexpr double k2Pi    = 6.28318530717958647692;

/// Returns the integer log2 of n if n is a positive power of two; -1 otherwise.
int Log2Pow2(std::size_t n)
{
  if (n == 0 || (n & (n - 1)) != 0) return -1;
  int r = 0;
  while ((static_cast<std::size_t>(1u) << r) < n) ++r;
  return r;
}

#ifdef GZ_WAVES_WITH_ENCINO
/// True when GZ_WAVES_USE_ENCINO=1 is set at construction time. Read once
/// per FFTWaveSimulation instance — toggling the env var mid-run won't
/// take effect until the wave field gets rebuilt.
bool EncinoEnabledByEnv()
{
  const char *v = std::getenv("GZ_WAVES_USE_ENCINO");
  return v && std::string(v) == "1";
}

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

const char *FilterName(EncinoWaves::FilterType t)
{
  switch (t)
  {
    case EncinoWaves::kNullFilter:                   return "none";
    case EncinoWaves::kSmoothInvertibleBandPassFilter: return "bandpass";
    default:                                         return "?";
  }
}

/// Apply optional environment-variable overrides for EncinoWaves' distinctive
/// controls onto `ep`. Like GZ_WAVES_USE_ENCINO itself, these are experiment
/// knobs deliberately kept out of the stable SDF surface. They all feed the
/// spectrum at construction (InitialState), so the field stays seamlessly
/// periodic and identical between the server and GUI processes. Unrecognized
/// values are ignored with a warning, leaving the Horvath default in place.
void ApplyEncinoEnvOverrides(EncinoWaves::Parametersf &ep)
{
  if (const char *v = std::getenv("GZ_WAVES_ENCINO_SPECTRUM"))
  {
    const std::string s(v);
    if (s == "pms" || s == "pm")
      ep.spectrum.type = EncinoWaves::kPiersonMoskowitzSpectrum;
    else if (s == "jonswap")
      ep.spectrum.type = EncinoWaves::kJONSWAPSpectrum;
    else if (s == "tma")
      ep.spectrum.type = EncinoWaves::kTMASpectrum;
    else
      std::cerr << "[FFTWaveSimulation] ignoring unknown "
                << "GZ_WAVES_ENCINO_SPECTRUM='" << s
                << "' (want pms|jonswap|tma)" << std::endl;
  }
  if (const char *v = std::getenv("GZ_WAVES_ENCINO_DISPERSION"))
  {
    const std::string s(v);
    if (s == "deep")
      ep.dispersion.type = EncinoWaves::kDeepDispersion;
    else if (s == "finite" || s == "finite_depth")
      ep.dispersion.type = EncinoWaves::kFiniteDepthDispersion;
    else if (s == "capillary")
      ep.dispersion.type = EncinoWaves::kCapillaryDispersion;
    else
      std::cerr << "[FFTWaveSimulation] ignoring unknown "
                << "GZ_WAVES_ENCINO_DISPERSION='" << s
                << "' (want deep|finite|capillary)" << std::endl;
  }
  if (const char *v = std::getenv("GZ_WAVES_ENCINO_SPREADING"))
  {
    const std::string s(v);
    if (s == "poscos2" || s == "poscossqr")
      ep.directionalSpreading.type =
        EncinoWaves::kPosCosThetaSqrDirectionalSpreading;
    else if (s == "mitsuyasu")
      ep.directionalSpreading.type =
        EncinoWaves::kMitsuyasuDirectionalSpreading;
    else if (s == "hasselmann")
      ep.directionalSpreading.type =
        EncinoWaves::kHasselmannDirectionalSpreading;
    else if (s == "donelanbanner" || s == "donelan")
      ep.directionalSpreading.type =
        EncinoWaves::kDonelanBannerDirectionalSpreading;
    else
      std::cerr << "[FFTWaveSimulation] ignoring unknown "
                << "GZ_WAVES_ENCINO_SPREADING='" << s
                << "' (want poscos2|mitsuyasu|hasselmann|donelanbanner)"
                << std::endl;
  }

  // Numeric overrides. Each only affects the Height/Dx/Dy fields VRX copies.
  auto envFloat = [](const char *_name, float &_io)
  {
    if (const char *v = std::getenv(_name))
    {
      try
      {
        _io = std::stof(v);
      }
      catch (...)
      {
        std::cerr << "[FFTWaveSimulation] ignoring non-numeric " << _name
                  << "='" << v << "'" << std::endl;
      }
    }
  };
  envFloat("GZ_WAVES_ENCINO_DEPTH",          ep.depth);
  envFloat("GZ_WAVES_ENCINO_FETCH",          ep.fetch);
  envFloat("GZ_WAVES_ENCINO_SWELL",          ep.directionalSpreading.swell);
  envFloat("GZ_WAVES_ENCINO_TROUGH_DAMPING", ep.troughDamping);

  // Optional spectral band-pass over wavelength (Encino's
  // SmoothInvertibleBandPassFilter). Setting either band edge enables it: the
  // spectrum keeps wavelengths within [MIN_WL, MAX_WL] metres and rolls off
  // outside over a soft transition (_SOFT). _MIN raises the suppression floor
  // (0 = full cut); _INVERT turns the band into a notch. Useful to drop the
  // small ripples that shimmer at grid resolution, isolate a swell band, or
  // remove components near/above the tile size. Note: the amplitude
  // calibration renormalises total energy, so the filter reshapes *which*
  // wavelengths survive rather than the overall sea height.
  if (std::getenv("GZ_WAVES_ENCINO_FILTER_MIN_WL") ||
      std::getenv("GZ_WAVES_ENCINO_FILTER_MAX_WL"))
  {
    ep.filter.type = EncinoWaves::kSmoothInvertibleBandPassFilter;
    envFloat("GZ_WAVES_ENCINO_FILTER_MIN_WL", ep.filter.smallWavelength);
    envFloat("GZ_WAVES_ENCINO_FILTER_MAX_WL", ep.filter.bigWavelength);
    envFloat("GZ_WAVES_ENCINO_FILTER_SOFT",   ep.filter.softWidth);
    envFloat("GZ_WAVES_ENCINO_FILTER_MIN",    ep.filter.min);
    if (const char *v = std::getenv("GZ_WAVES_ENCINO_FILTER_INVERT"))
      ep.filter.invert = (std::string(v) == "1");
    // The smoothstep band edges are [smallWL - soft, smallWL] and
    // [bigWL, bigWL + soft]; a zero soft width collapses them and Encino's
    // smoothstep divides by zero (NaN spectrum). Default a positive transition
    // from the lower cutoff when the user didn't supply one.
    if (ep.filter.softWidth <= 0.0f)
      ep.filter.softWidth = std::max(0.25f * ep.filter.smallWavelength, 1.0f);
  }
}
#endif  // GZ_WAVES_WITH_ENCINO
}  // namespace

//-----------------------------------------------------------------------------
// EncinoState — pimpl that holds the vendored Horvath-spectrum library's
// per-instance state. Defined here (not in the header) so EncinoWaves headers
// stay out of the public include surface.
//-----------------------------------------------------------------------------
#ifdef GZ_WAVES_WITH_ENCINO
struct FFTWaveSimulation::EncinoState
{
  EncinoWaves::Parametersf params;
  std::unique_ptr<EncinoWaves::InitialStatef> initial;
  std::unique_ptr<EncinoWaves::Propagationf>   propagation;
  std::unique_ptr<EncinoWaves::PropagatedStatef> state;
};
#else
// Empty when EncinoWaves isn't compiled in. The encino_ member stays present in
// the header (so the class layout is identical with or without Encino — the
// white-box tests rely on that); it is simply never constructed on this build,
// and this complete (empty) type lets the unique_ptr destructor compile.
struct FFTWaveSimulation::EncinoState {};
#endif  // GZ_WAVES_WITH_ENCINO

FFTWaveSimulation::~FFTWaveSimulation() = default;

FFTWaveSimulation::FFTWaveSimulation() = default;

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

void FFTWaveSimulation::SetParameters(const WaveParameters &p)
{
  this->tileSize_ = p.tileSize;
  this->gridSize_ = p.gridSize;
  this->gain_     = p.gain;
  this->tau_      = p.tau;
  const std::uint32_t seed = p.seed;

  // PMS relation: peak omega <-> wind speed at 19.5 m. We have period →
  // omegaP → V19. (deep-water dispersion places the spectral peak at
  // omegaP ≈ 0.879 * g / V19.)
  const double omegaP = k2Pi / p.period;
  this->windSpeed_ = 0.879 * kGravity / omegaP;
  this->windDirX_ = std::cos(p.direction);
  this->windDirY_ = std::sin(p.direction);

  const int N = static_cast<int>(this->gridSize_);
  const double L = this->tileSize_;

  // Wavenumber grid in standard FFT layout: k=0 at index 0, positive k for
  // i < N/2, negative k for i ≥ N/2. (Not fftshift-centered — Eigen::FFT
  // expects this layout natively.)
  this->kxRow_.resize(N);
  this->kyCol_.resize(N);
  for (int i = 0; i < N; ++i)
  {
    const double mi = (i < N / 2) ? static_cast<double>(i)
                                  : static_cast<double>(i - N);
    this->kxRow_[i] = k2Pi * mi / L;
    this->kyCol_[i] = k2Pi * mi / L;
  }

  // omega(k) = √(g·|k|) cached per cell.
  this->omegaGrid_.resize(N, N);
  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < N; ++j)
    {
      const double kmag =
        std::hypot(this->kxRow_[i], this->kyCol_[j]);
      this->omegaGrid_(i, j) = std::sqrt(kGravity * kmag);
    }
  }

  // Generate h0(k) from the Phillips spectrum. Tessendorf eq. 24:
  //   h0(k) = (xi_r + i·xi_i) · sqrt(P_h(k) / 2)
  // with xi_r, xi_i ~ N(0, 1). The conjugate copy h0_conj(-k) is stored so
  // Update() can evolve the spectrum without re-indexing per cell.
  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, 1.0);

  this->h0_.resize(N, N);
  this->h0Conj_.resize(N, N);
  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < N; ++j)
    {
      const double Ph = this->Phillips(this->kxRow_[i], this->kyCol_[j]);
      const double sigma = std::sqrt(std::max(0.0, Ph) * 0.5);
      const std::complex<double> xi(gauss(rng), gauss(rng));
      this->h0_(i, j) = xi * sigma;
    }
  }
  // h0_conj(k) = conj(h0(-k))
  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < N; ++j)
    {
      const int ic = (N - i) % N;
      const int jc = (N - j) % N;
      this->h0Conj_(i, j) = std::conj(this->h0_(ic, jc));
    }
  }

  // Continuous-spectrum-to-discrete-IFFT amplitude correction.
  // Variance argument: we want σ²_η = ∫ P(k) dk_x dk_y. The discrete sum
  // gives Σ P(k_n) · (2π/L)², and Eigen's inverse FFT additionally
  // normalises by 1/N². Pre-scaling h0 by `2π·N²/L` makes the per-cell
  // variance recover the continuous integral; the √2 compensates for the
  // (1/√2) factor already baked into the random h0 amplitudes above.
  // Without this fix the resulting η is ~200× too small for typical
  // tile/grid choices (e.g. L=200 m, N=128).
  const double specScale =
      std::sqrt(2.0) * k2Pi * static_cast<double>(N) *
      static_cast<double>(N) / L;
  this->h0_     *= specScale;
  this->h0Conj_ *= specScale;

  this->heightGrid_ = Eigen::MatrixXd::Zero(N, N);
  this->dispXGrid_  = Eigen::MatrixXd::Zero(N, N);
  this->dispYGrid_  = Eigen::MatrixXd::Zero(N, N);
  this->minEGrid_   = Eigen::MatrixXd::Constant(N, N, 1.0);  // 1 = no foam
  this->slopeXGrid_   = Eigen::MatrixXd::Zero(N, N);
  this->slopeYGrid_   = Eigen::MatrixXd::Zero(N, N);
  this->dispDxDxGrid_ = Eigen::MatrixXd::Zero(N, N);
  this->dispDyDyGrid_ = Eigen::MatrixXd::Zero(N, N);
  this->dispDxDyGrid_ = Eigen::MatrixXd::Zero(N, N);

  // Optional: bring up the Apache-2.0 EncinoWaves spectrum library. The
  // toggle is intentionally an env var rather than an SDF parameter for
  // now — it's an experiment knob, not a stable interface. The Encino
  // path leaves slope/chop-derivative grids zeroed; consumers must check
  // UseEncino() and skip their slope/chop-deriv upload.
#ifdef GZ_WAVES_WITH_ENCINO
  this->useEncino_ = EncinoEnabledByEnv();
  if (this->useEncino_)
  {
    const int log2N = Log2Pow2(this->gridSize_);
    if (log2N < 0)
    {
      std::cerr << "[FFTWaveSimulation] GZ_WAVES_USE_ENCINO=1 requested but "
                << "gridSize=" << this->gridSize_ << " is not a power of two; "
                << "falling back to Phillips path." << std::endl;
      this->useEncino_ = false;
    }
    else
    {
      this->encino_ = std::make_unique<EncinoState>();
      // Match upstream Horvath defaults except for the four knobs we
      // share with the in-tree Phillips path. EncinoWaves's defaults pick
      // TMA spectrum + Hasselmann directional spread + Capillary
      // dispersion + Normal random distribution, which is the "good
      // ocean" config the Horvath 2015 paper validates against.
      auto &ep = this->encino_->params;
      ep.resolutionPowerOfTwo = log2N;
      ep.domain        = static_cast<float>(this->tileSize_);
      ep.windSpeed     = static_cast<float>(this->windSpeed_);
      ep.amplitudeGain = static_cast<float>(this->gain_);
      ep.random.seed   = static_cast<int>(seed);

      // Optional experiment knobs to exercise Encino's distinctive spectra,
      // dispersion and directional spreading without touching the SDF schema.
      ApplyEncinoEnvOverrides(ep);

      this->encino_->initial =
          std::make_unique<EncinoWaves::InitialStatef>(ep);
      this->encino_->propagation =
          std::make_unique<EncinoWaves::Propagationf>(ep, /*nthreads=*/-1);
      this->encino_->state =
          std::make_unique<EncinoWaves::PropagatedStatef>(ep);

      // --- Physics-based amplitude calibration -----------------------------
      // EncinoWaves' amplitudeGain only feeds its (here unused) normal
      // computation, not the height field, and its intrinsic variance does
      // not correspond to a physical sea state at our wind speeds. Measure
      // the intrinsic RMS once (one propagation past the ramp) and rescale so
      // the significant wave height follows the standard fully-developed
      // Pierson-Moskowitz relation Hs = 0.21 * V19.5^2 / g, i.e.
      // sigma = Hs/4 = 0.21/(4 g) * V^2. The selected spectrum still sets the
      // spectral *shape*; this only fixes the overall energy.
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
    }
  }
#else
  this->useEncino_ = false;
#endif  // GZ_WAVES_WITH_ENCINO

  this->Update(0.0);
}

double FFTWaveSimulation::Phillips(double kx, double ky) const
{
  const double k2 = kx * kx + ky * ky;
  if (k2 < 1e-12)
    return 0.0;

  const double kmag = std::sqrt(k2);
  const double V = this->windSpeed_;
  const double L = V * V / kGravity;            // wind-fetch length scale
  const double kL2 = (kmag * L) * (kmag * L);

  // Damping at short wavelengths (Tessendorf 2001 §4.3).
  const double l = L * 1e-3;
  const double damp = std::exp(-(k2) * (l * l));

  // Directional weight |k̂ · ŵ|²
  const double khatDotW = (kx * this->windDirX_ + ky * this->windDirY_) / kmag;
  const double directional = khatDotW * khatDotW;

  constexpr double kA = 0.0081;
  return this->gain_ * kA *
         std::exp(-1.0 / std::max(kL2, 1e-12)) /
         (k2 * k2) * directional * damp;
}

double FFTWaveSimulation::Ramp(double t) const
{
  if (this->tau_ <= 0.0)
    return 1.0;
  return 1.0 - std::exp(-t / this->tau_);
}

namespace
{
  // 2D inverse FFT via row-then-column passes (Eigen::FFT is 1D). Returns
  // the real part of the result, multiplied by `scale`.
  Eigen::MatrixXd Ifft2DReal(const Eigen::MatrixXcd &spectrum, double scale)
  {
    const int N = static_cast<int>(spectrum.rows());
    Eigen::FFT<double> fft;
    Eigen::MatrixXcd rowOut(N, N);
    for (int i = 0; i < N; ++i)
    {
      Eigen::VectorXcd freq = spectrum.row(i);
      Eigen::VectorXcd time(N);
      fft.inv(time, freq);
      rowOut.row(i) = time;
    }
    Eigen::MatrixXcd colOut(N, N);
    for (int j = 0; j < N; ++j)
    {
      Eigen::VectorXcd freq = rowOut.col(j);
      Eigen::VectorXcd time(N);
      fft.inv(time, freq);
      colOut.col(j) = time;
    }
    return colOut.real() * scale;
  }
}

void FFTWaveSimulation::Update(double t)
{
  // Idempotent: the field at time t is deterministic, so a repeat call for the
  // same t is a no-op. Lets the Waves system and any number of WaveBuoyancy
  // consumers that advance the same instance to the same tick share a single
  // recompute instead of each paying for a full set of IFFTs.
  if (t == this->lastUpdateT_)
    return;
  this->lastUpdateT_ = t;

  const int N = static_cast<int>(this->gridSize_);

#ifdef GZ_WAVES_WITH_ENCINO
  // ENCINO-BACKED PATH.
  // Drive the heightGrid_/dispXGrid_/dispYGrid_ outputs through the
  // vendored Horvath spectrum library instead of our Phillips path.
  // Slope and chop-derivative grids stay zeroed — the visual layer
  // checks UseEncino() and falls back to finite-diff normals.
  if (this->useEncino_ && this->encino_ && this->encino_->propagation)
  {
    this->encino_->propagation->propagate(
        this->encino_->params,
        *this->encino_->initial,
        *this->encino_->state,
        static_cast<float>(t));

    // Combined output scale per Update:
    //  * ramp        — fade the field in over `tau`, matching the Phillips
    //                  path so switching backends doesn't pop the surface;
    //  * encinoScale_ — physics-based amplitude calibration to a PM sea state
    //                  (Encino's amplitudeGain doesn't scale the height);
    //  * gain_       — the SDF <gain> user multiplier (a no-op via Encino's
    //                  amplitudeGain, so we apply it here to make it work).
    const double scale = this->Ramp(t) * this->encinoScale_ * this->gain_;

    // Encino stores its spatial fields row-major in float; our grids
    // are column-major in double. The old scalar copy loop showed up
    // as ~7% of the server's hot-thread self-time (16k iterations
    // per grid × 3 grids per call). Map+cast assignment lets Eigen
    // vectorize the conversion + layout swap; the scale folds into the
    // same vectorized expression.
    using RowMatF = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,
                                  Eigen::RowMajor>;
    this->heightGrid_ = Eigen::Map<const RowMatF>(
        this->encino_->state->Height.cdata(), N, N).cast<double>() * scale;
    this->dispXGrid_ = Eigen::Map<const RowMatF>(
        this->encino_->state->Dx.cdata(), N, N).cast<double>() * scale;
    this->dispYGrid_ = Eigen::Map<const RowMatF>(
        this->encino_->state->Dy.cdata(), N, N).cast<double>() * scale;

    // Foam: Encino computes MinE = -(min eigenvalue of the displacement
    // Jacobian) at its internal amplitude. The folding metric scales with the
    // displacement, so at our calibrated amplitude the minimum eigenvalue is
    // 1 - scale·(MinE + 1). Jacobian() bilinear-samples this; Eval::FoamMask
    // turns values below its threshold into whitecap intensity. At t=0
    // (scale=0) the surface is flat → eigenvalue 1 → no foam.
    this->minEGrid_ = (1.0 - scale *
        (Eigen::Map<const RowMatF>(this->encino_->state->MinE.cdata(), N, N)
            .cast<double>().array() + 1.0)).matrix();
    return;
  }
#endif  // GZ_WAVES_WITH_ENCINO

  const double ramp = this->Ramp(t);

  // Evolve the height spectrum:
  //   h(k, t) = h0(k)·exp(i·ω·t) + h0_conj(k)·exp(-i·ω·t).
  // Then derive the horizontal-displacement spectra by Tessendorf eq. 29:
  //   Dx(k, t) = -i · (kx / |k|) · h(k, t)
  //   Dy(k, t) = -i · (ky / |k|) · h(k, t)
  // (the unit-vector k̂ projects the displacement into the wave's
  // propagation direction). All three IFFTs share a single ifft2 pass.
  Eigen::MatrixXcd hkt(N, N);
  Eigen::MatrixXcd dxkt(N, N);
  Eigen::MatrixXcd dykt(N, N);
  // Slope / chop-derivative spectra are only filled when a downstream
  // consumer asks for them (visual VS with useSlopeMap=1). Five IFFTs
  // worth of work that's wasted on the GPU-FFT visual path and on
  // anything that finite-differences for normals.
  const bool derivs = this->computeDerivatives_;
  Eigen::MatrixXcd sxkt, sykt, dxdxkt, dydykt, dxdykt;
  if (derivs)
  {
    sxkt.resize(N, N);
    sykt.resize(N, N);
    dxdxkt.resize(N, N);
    dydykt.resize(N, N);
    dxdykt.resize(N, N);
  }
  const std::complex<double> kMinusI(0.0, -1.0);
  const std::complex<double> kPlusI(0.0, 1.0);
  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < N; ++j)
    {
      const double w = this->omegaGrid_(i, j);
      const std::complex<double> e_plus(std::cos(w * t), std::sin(w * t));
      const std::complex<double> e_minus = std::conj(e_plus);
      const std::complex<double> h = this->h0_(i, j) * e_plus +
                                     this->h0Conj_(i, j) * e_minus;
      hkt(i, j) = h;

      const double kx = this->kxRow_[i];
      const double ky = this->kyCol_[j];
      const double kmag = std::hypot(kx, ky);
      if (kmag < 1e-12)
      {
        dxkt(i, j) = 0.0;
        dykt(i, j) = 0.0;
        if (derivs)
        {
          dxdxkt(i, j) = 0.0;
          dydykt(i, j) = 0.0;
          dxdykt(i, j) = 0.0;
        }
      }
      else
      {
        const std::complex<double> factor = kMinusI / kmag;
        dxkt(i, j) = factor * kx * h;
        dykt(i, j) = factor * ky * h;
        if (derivs)
        {
          dxdxkt(i, j) = (kx * kx / kmag) * h;
          dydykt(i, j) = (ky * ky / kmag) * h;
          dxdykt(i, j) = (kx * ky / kmag) * h;
        }
      }
      if (derivs)
      {
        sxkt(i, j) = kPlusI * kx * h;
        sykt(i, j) = kPlusI * ky * h;
      }
    }
  }

  this->heightGrid_   = Ifft2DReal(hkt, ramp);
  this->dispXGrid_    = Ifft2DReal(dxkt, ramp);
  this->dispYGrid_    = Ifft2DReal(dykt, ramp);
  if (derivs)
  {
    this->slopeXGrid_   = Ifft2DReal(sxkt, ramp);
    this->slopeYGrid_   = Ifft2DReal(sykt, ramp);
    this->dispDxDxGrid_ = Ifft2DReal(dxdxkt, ramp);
    this->dispDyDyGrid_ = Ifft2DReal(dydykt, ramp);
    this->dispDxDyGrid_ = Ifft2DReal(dxdykt, ramp);
  }
}

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

double FFTWaveSimulation::Elevation(double x, double y, double /*t*/) const
{
  // Caller is expected to have called Update(t) ≤ this tick (the Waves
  // system does this in PreUpdate). The bilinear sample is time-free.
  return this->BilinearSample(this->heightGrid_, x, y);
}

gz::math::Vector3d FFTWaveSimulation::ParticleVelocity(
  double /*x*/, double /*y*/, double /*t*/) const
{
  // Stage 2: stub. Stage 4+ will compute via additional FFTs of i·k·h(k,t)
  // for the horizontal components and ∂η/∂t for the vertical. Returning
  // zero here means drag against still water — not physically accurate yet.
  return gz::math::Vector3d::Zero;
}

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

double FFTWaveSimulation::Jacobian(double x, double y, double /*t*/) const
{
  // Encino path: bilinear-sample the per-cell minimum eigenvalue of the
  // displacement Jacobian (1 = flat, < 1 → folding). Eval::FoamMask turns
  // values below its threshold into whitecap intensity, so foam appears on the
  // pinched crests. The in-tree Phillips path doesn't compute a CPU folding
  // field (its foam is derived in the shader from the chop-derivative grids),
  // so it stays identically 1 → no CPU foam.
  if (this->useEncino_)
    return this->BilinearSample(this->minEGrid_, x, y);
  return 1.0;
}

const WaveField2D *FFTWaveSimulation::Field() const
{
  // Repopulate the view from the current grids each call: Update may
  // reallocate them (the Phillips path reassigns heightGrid_), so cached
  // data() pointers would dangle. Eigen is column-major, matching
  // WaveField2D's documented (i + j*N) layout. The renderer finite-diffs η
  // for normals (slopeX/slopeY left null). foam is the folding metric — the
  // Encino MinE grid; the Phillips path leaves it at 1 (flat = no whitecaps).
  this->field_.n    = this->gridSize_;
  this->field_.tile = this->tileSize_;
  this->field_.dz   = this->heightGrid_.data();
  this->field_.dx   = this->dispXGrid_.data();
  this->field_.dy   = this->dispYGrid_.data();
  this->field_.foam = this->minEGrid_.data();
  return &this->field_;
}

}  // namespace gz::sim::waves

// Register the stochastic FFT backend (Phillips, or EncinoWaves spectra when
// GZ_WAVES_USE_ENCINO=1) as a gz-plugin wave-field provider, discovered by the
// core through the convention "gz-waves-provider-fft".
GZ_ADD_PLUGIN(
  gz::sim::waves::FFTWaveSimulation,
  gz::sim::waves::IWaveField)
