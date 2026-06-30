/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>

#include <gtest/gtest.h>

#include "gz/sim/waves/Eval.hh"
#include "gz/sim/waves/FFTWaveSimulation.hh"
#include "gz/sim/waves/WaveSimulation.hh"
#include "gz/sim/waves/Wavefield.hh"

namespace gsw = gz::sim::waves;

namespace
{
// The fft engine is no longer a dlopen'd gz-plugin; this binary links it
// directly, so register its factory for the CreateWaveSimulation tests.
const bool kFftRegistered = [] {
  gsw::RegisterWaveEngineFactory("fft", &gsw::MakeFFTWaveField);
  return true;
}();

/// \brief Default FFT wave parameters (PMS model, V19 ≈ 8 m/s).
/// \return The default WaveParameters.
gsw::WaveParameters DefaultParams()
{
  gsw::WaveParameters p;
  p.model = "PMS";
  p.number = 3;          // ignored by FFT engine (uses gridSize)
  p.period = 6.0;        // ω_P ≈ 1.05 rad/s, V19 ≈ 8 m/s
  p.gain = 1.0;
  p.direction = 0.0;
  p.tau = 2.0;
  return p;
}

/// \brief Build a 64² / 100 m FFT simulation from DefaultParams().
/// \param[in] _seed RNG seed for the spectrum amplitudes.
/// \return The constructed FFTWaveSimulation.
gsw::FFTWaveSimulation MakeSim(std::uint32_t _seed = 1)
{
  return gsw::FFTWaveSimulation(
    DefaultParams(), /*tile=*/100.0, /*grid=*/64, _seed);
}

/// \brief Root-mean-square of a grid's values.
/// \param[in] _g The grid to measure.
/// \return The RMS value.
double RmsHeight(const Eigen::MatrixXd &_g)
{
  return std::sqrt(_g.squaredNorm() / static_cast<double>(_g.size()));
}

/// \brief Largest |elevation| over a few scattered sample points.
/// \param[in] _d Wave-field state to query.
/// \param[in] _t Simulation time [s].
/// \return The maximum absolute elevation [m].
double MaxAbsElevation(const gsw::WavefieldData &_d, double _t)
{
  const double xs[] = {10.0, 33.0, 5.0, 61.0};
  const double ys[] = {12.0, 47.0, 80.0, 23.0};
  double m = 0.0;
  for (int i = 0; i < 4; ++i)
    m = std::max(m, std::abs(gsw::SurfaceElevation(_d, xs[i], ys[i], _t)));
  return m;
}
}  // namespace

//////////////////////////////////////////////////
TEST(FFTWaveSimulation, FactoryRoute)
{
  auto sim = gsw::CreateWaveSimulation("fft", DefaultParams());
  ASSERT_NE(sim, nullptr);
  EXPECT_EQ(sim->Kind(), "fft");
  ASSERT_TRUE(sim->Bounds().has_value());
  EXPECT_GT(sim->Bounds()->x, 0.0);
}

// The Wavefield component serializes only the *recipe* (algorithm + params),
// not a live engine: operator>> restores the params and leaves `simulation`
// null. A consumer rebuilds its own engine from the recipe (what WaterVisual
// does), so each consumer owns a private field instance — no shared
// process-global engine. Same seed → the rebuilt field matches the original.
//////////////////////////////////////////////////
TEST(Wavefield, FftComponentSerializesRecipeNotEngine)
{
  gsw::WavefieldData live;
  live.algorithm = "fft";
  live.generation = 918273;
  live.params.model = "PMS";
  live.params.period = 3.2;
  live.params.gain = 1.0;
  live.params.tileSize = 100.0;
  live.params.gridSize = 64;
  live.params.seed = 7;
  live.simulation = gsw::CreateWaveSimulation(live.algorithm, live.params);
  ASSERT_NE(live.simulation, nullptr);
  live.simulation->Update(20.0);
  EXPECT_GT(MaxAbsElevation(live, 20.0), 1e-3)
    << "a live, time-advanced FFT field must be non-flat";

  // Round-trip through the component serialization (what replication does).
  std::stringstream ss;
  ss << live;
  gsw::WavefieldData round;
  ss >> round;

  // Deserialization restores the recipe but builds no engine.
  EXPECT_EQ(round.simulation, nullptr)
    << "operator>> must not build a (process-global) engine";
  EXPECT_EQ(round.algorithm, "fft");
  EXPECT_EQ(round.generation, 918273u);
  EXPECT_EQ(round.params.seed, 7u);
  EXPECT_NEAR(round.params.period, 3.2, 1e-9);

  // A consumer rebuilds its own engine from the recipe; same seed → same field.
  round.simulation = gsw::CreateWaveSimulation(round.algorithm, round.params);
  ASSERT_NE(round.simulation, nullptr);
  round.simulation->Update(20.0);
  EXPECT_NEAR(MaxAbsElevation(round, 20.0), MaxAbsElevation(live, 20.0), 1e-9)
    << "a privately rebuilt field matches the original (deterministic by seed)";
}

//////////////////////////////////////////////////
TEST(FFTWaveSimulation, HeightsAreFinite)
{
  auto sim = MakeSim();
  sim.Update(5.0);
  const auto &g = sim.HeightGrid();
  for (int i = 0; i < g.rows(); ++i)
    for (int j = 0; j < g.cols(); ++j)
      EXPECT_TRUE(std::isfinite(g(i, j))) << "(" << i << "," << j << ")";
}

//////////////////////////////////////////////////
TEST(FFTWaveSimulation, HeightsAreBounded)
{
  auto sim = MakeSim();
  sim.Update(20.0);          // past startup ramp
  const auto &g = sim.HeightGrid();
  const double sigma = RmsHeight(g);
  EXPECT_GT(sigma, 0.0);     // not all zero
  // 99.99% of a Gaussian is within ±4σ; allow a margin.
  EXPECT_LE(g.cwiseAbs().maxCoeff(), 6.0 * sigma);
}

//////////////////////////////////////////////////
TEST(FFTWaveSimulation, RampUpFromZero)
{
  auto sim = MakeSim();
  sim.Update(0.0);
  const double rms0 = RmsHeight(sim.HeightGrid());
  sim.Update(20.0);
  const double rmsLong = RmsHeight(sim.HeightGrid());
  EXPECT_NEAR(rms0, 0.0, 1e-9);
  // Phillips amplitudes at 100 m tile / V≈8 m/s wind give an RMS height in
  // the low-cm range; bound generously to avoid spectrum-tuning regressions.
  EXPECT_GT(rmsLong, 1e-4);
}

// Encino's MinE folding metric is hooked through Jacobian() → Eval::FoamMask:
// pinched crests report Jacobian < 1 and produce whitecaps.
//////////////////////////////////////////////////
TEST(FFTWaveSimulation, FoamFromJacobian)
{
  gsw::WaveParameters p;
  p.model = "PMS";
  p.period = 3.2;
  p.gain = 20.0;           // large amplitude so folding is unambiguous
  p.tileSize = 100.0;
  p.gridSize = 64;
  p.seed = 7;
  gsw::FFTWaveSimulation sim(p, 100.0, 64, 7);
  const double L = sim.TileSizeMeters();

  // Flat surface at t=0 (ramp 0): no folding → Jacobian 1, no foam, either way.
  sim.Update(0.0);
  for (int i = 0; i < 32; ++i)
    for (int j = 0; j < 32; ++j)
      EXPECT_NEAR(sim.Jacobian(i * L / 32.0, j * L / 32.0, 0.0), 1.0, 1e-9);

  // Past the ramp: sample the folding metric and the derived foam.
  sim.Update(20.0);
  double minJac = 1e9, maxFoam = 0.0;
  for (int i = 0; i < 32; ++i)
    for (int j = 0; j < 32; ++j)
    {
      const double jac = sim.Jacobian(i * L / 32.0, j * L / 32.0, 20.0);
      minJac = std::min(minJac, jac);
      const double foam =
        jac < 0.6 ? std::clamp(1.0 - jac / 0.6, 0.0, 1.0) : 0.0;
      maxFoam = std::max(maxFoam, foam);
    }

  EXPECT_LT(minJac, 0.6) << "Encino crests should fold below the foam threshold";
  EXPECT_GT(maxFoam, 0.0) << "Encino path should produce whitecaps";
}

// The <filter_*> band-pass reshapes which wavelengths survive in the spectrum.
// Same seed → without the filter the field is one thing; with a narrow band it
// differs. (The amplitude calibration renormalises total energy, so we compare
// the field shape, not its RMS.)
//////////////////////////////////////////////////
TEST(FFTWaveSimulation, BandPassFilterReshapesField)
{
  gsw::WaveParameters p;
  p.model = "PMS";
  p.period = 3.2;
  p.gain = 1.0;
  p.tileSize = 200.0;
  p.gridSize = 64;
  p.seed = 11;

  gsw::FFTWaveSimulation plain(p, 200.0, 64, 11);
  plain.Update(20.0);
  const Eigen::MatrixXd a = plain.HeightGrid();

  // Keep only 30–100 m waves (suppresses the ~16 m peak and the ripples).
  p.filterMinWavelength = 30.0;
  p.filterMaxWavelength = 100.0;
  gsw::FFTWaveSimulation filtered(p, 200.0, 64, 11);
  filtered.Update(20.0);
  const Eigen::MatrixXd b = filtered.HeightGrid();

  EXPECT_GT((a - b).cwiseAbs().maxCoeff(), 1e-3)
    << "band-pass filter should reshape the field";
}

// The <spectrum>/<spreading>/<dispersion> SDF selectors feed EncinoWaves. Two
// different spectra with the same seed produce different fields.
//////////////////////////////////////////////////
TEST(FFTWaveSimulation, SpectrumSelectorChangesField)
{
  gsw::WaveParameters p;
  p.model = "PMS";
  p.period = 3.2;
  p.tileSize = 200.0;
  p.gridSize = 64;
  p.seed = 11;

  p.spectrum = "tma";
  gsw::FFTWaveSimulation tma;
  tma.SetParameters(p);
  tma.Update(20.0);
  const Eigen::MatrixXd a = tma.HeightGrid();

  p.spectrum = "pms";
  gsw::FFTWaveSimulation pms;
  pms.SetParameters(p);
  pms.Update(20.0);
  const Eigen::MatrixXd b = pms.HeightGrid();

  EXPECT_GT((a - b).cwiseAbs().maxCoeff(), 1e-3)
    << "different <spectrum> selectors should produce different fields";
}

//////////////////////////////////////////////////
TEST(FFTWaveSimulation, TimeEvolutionChangesField)
{
  auto sim = MakeSim();
  sim.Update(10.0);
  const Eigen::MatrixXd snapshotA = sim.HeightGrid();
  sim.Update(11.0);
  const Eigen::MatrixXd snapshotB = sim.HeightGrid();
  EXPECT_GT((snapshotB - snapshotA).cwiseAbs().maxCoeff(), 1e-3);
}

// The Field() rendering view must alias the engine grids with the documented
// column-major layout (element (i,j) at i + j*N), so the renderer can upload
// it without knowing the concrete engine.
//////////////////////////////////////////////////
TEST(FFTWaveSimulation, FieldExposesGridViews)
{
  auto sim = MakeSim();
  sim.Update(8.0);
  const auto *f = sim.Field();
  ASSERT_NE(f, nullptr);
  EXPECT_EQ(f->n, sim.GridSize());
  EXPECT_NEAR(f->tile, sim.TileSizeMeters(), 1e-12);
  ASSERT_NE(f->dz, nullptr);
  ASSERT_NE(f->dx, nullptr);
  ASSERT_NE(f->dy, nullptr);
  ASSERT_NE(f->foam, nullptr);

  const int N = static_cast<int>(f->n);
  const auto &eta = sim.HeightGrid();
  const auto &dxG = sim.DispXGrid();
  const auto &dyG = sim.DispYGrid();
  for (int j = 0; j < N; j += N / 4)
  {
    for (int i = 0; i < N; i += N / 4)
    {
      EXPECT_DOUBLE_EQ(f->dz[i + j * N], eta(i, j)) << "i=" << i << " j=" << j;
      EXPECT_DOUBLE_EQ(f->dx[i + j * N], dxG(i, j));
      EXPECT_DOUBLE_EQ(f->dy[i + j * N], dyG(i, j));
      EXPECT_TRUE(std::isfinite(f->foam[i + j * N]));
    }
  }
}

//////////////////////////////////////////////////
TEST(FFTWaveSimulation, DeterministicAcrossRunsWithSameSeed)
{
  auto a = MakeSim(/*seed=*/42);
  auto b = MakeSim(/*seed=*/42);
  a.Update(7.0);
  b.Update(7.0);
  EXPECT_NEAR(
    (a.HeightGrid() - b.HeightGrid()).cwiseAbs().maxCoeff(),
    0.0, 1e-12);
}

//////////////////////////////////////////////////
TEST(FFTWaveSimulation, DifferentSeedsProduceDifferentFields)
{
  auto a = MakeSim(/*seed=*/1);
  auto b = MakeSim(/*seed=*/2);
  a.Update(7.0);
  b.Update(7.0);
  EXPECT_GT(
    (a.HeightGrid() - b.HeightGrid()).cwiseAbs().maxCoeff(),
    1e-3);
}

//////////////////////////////////////////////////
TEST(FFTWaveSimulation, BilinearSampleMatchesGrid)
{
  auto sim = MakeSim();
  sim.Update(8.0);
  const auto &g = sim.HeightGrid();
  const double L = sim.TileSizeMeters();
  const std::size_t N = sim.GridSize();

  // Bilinear at the exact grid corners should return the grid value.
  // World coordinates of grid cell (i,j): x = i*L/N - L/2 + 0,
  // y = j*L/N - L/2 + 0  (we wrap into [0, L) internally, so the offset
  // doesn't matter as long as it's consistent).
  for (std::size_t i = 0; i < N; i += N / 4)
  {
    for (std::size_t j = 0; j < N; j += N / 4)
    {
      const double x = static_cast<double>(i) * L / N;
      const double y = static_cast<double>(j) * L / N;
      EXPECT_NEAR(sim.Elevation(x, y, 8.0), g(i, j), 1e-9)
        << "i=" << i << " j=" << j;
    }
  }
}

//////////////////////////////////////////////////
TEST(FFTWaveSimulation, ElevationIsPeriodic)
{
  auto sim = MakeSim();
  sim.Update(5.0);
  const double L = sim.TileSizeMeters();
  // Sampling at (x + L, y) should give the same height as (x, y).
  for (double x = -20.0; x <= 20.0; x += 3.7)
  {
    for (double y = -20.0; y <= 20.0; y += 4.1)
    {
      EXPECT_NEAR(sim.Elevation(x, y, 5.0),
                  sim.Elevation(x + L, y, 5.0), 1e-9);
      EXPECT_NEAR(sim.Elevation(x, y, 5.0),
                  sim.Elevation(x, y + L, 5.0), 1e-9);
    }
  }
}

//////////////////////////////////////////////////
TEST(FFTWaveSimulation, NormalIsUnitAndPointsUp)
{
  auto sim = MakeSim();
  sim.Update(8.0);
  for (double t = 1.0; t < 5.0; t += 0.7)
  {
    const auto n = sim.Normal(3.0 * t, 1.0 * t, 8.0);
    EXPECT_NEAR(n.Length(), 1.0, 1e-9);
    EXPECT_GT(n.Z(), 0.0);
  }
}

//////////////////////////////////////////////////
// ParticleVelocity is the time-derivative of the displacement field. On a
// moving sea it must be finite, physically bounded, and non-zero somewhere.
TEST(FFTWaveSimulation, ParticleVelocityIsFiniteBoundedAndNonzero)
{
  auto sim = MakeSim();
  sim.Update(8.0);
  double maxSpeed = 0.0;
  for (double x = 0.0; x < 100.0; x += 9.0)
  {
    for (double y = 0.0; y < 100.0; y += 9.0)
    {
      const auto v = sim.ParticleVelocity(x, y, 8.0);
      ASSERT_TRUE(std::isfinite(v.X()) && std::isfinite(v.Y()) &&
                  std::isfinite(v.Z()));
      maxSpeed = std::max(maxSpeed, v.Length());
    }
  }
  EXPECT_GT(maxSpeed, 1e-3) << "velocity should be non-zero on a moving sea";
  EXPECT_LT(maxSpeed, 25.0) << "velocity should be physically bounded";
}

//////////////////////////////////////////////////
// At t = 0 the startup ramp is zero, so the field is flat and still.
TEST(FFTWaveSimulation, ParticleVelocityZeroAtRest)
{
  auto sim = MakeSim();
  sim.Update(0.0);
  const auto v = sim.ParticleVelocity(12.0, 34.0, 0.0);
  EXPECT_NEAR(v.Length(), 0.0, 1e-9);
}

//////////////////////////////////////////////////
TEST(FFTWaveSimulation, WindDirectionBiasesAmplitude)
{
  // Encino's directional spreading concentrates energy along the wind axis
  // (assumed +x). We expect a higher RMS along x than perpendicular when
  // sampled over a long enough strip.
  gsw::WaveParameters p = DefaultParams();
  p.direction = 0.0;  // wind along +x
  gsw::FFTWaveSimulation sim(p, 100.0, 64, 7);
  sim.Update(20.0);

  double sxx = 0, syy = 0;
  int    cxx = 0, cyy = 0;
  for (double s = -40.0; s <= 40.0; s += 1.0)
  {
    sxx += std::abs(sim.Elevation(s, 0.0, 20.0)); ++cxx;
    syy += std::abs(sim.Elevation(0.0, s, 20.0)); ++cyy;
  }
  EXPECT_GT(sxx / cxx, syy / cyy);
}

// WMO sea state code -> representative wave parameters (the canonical table).
//////////////////////////////////////////////////
TEST(SeaState, CodeMapsToCanonicalHeights)
{
  gsw::SeaStateSpec prev{};
  for (int c = 1; c <= 9; ++c)
  {
    gsw::SeaStateSpec s;
    ASSERT_TRUE(gsw::SeaStateFromCode(c, s));
    EXPECT_GT(s.significantWaveHeight, prev.significantWaveHeight);  // increasing
    EXPECT_GT(s.peakPeriod, 0.0);
    EXPECT_GT(s.windSpeed, 0.0);
    prev = s;
  }
  gsw::SeaStateSpec dummy;
  EXPECT_FALSE(gsw::SeaStateFromCode(-1, dummy));
  EXPECT_FALSE(gsw::SeaStateFromCode(10, dummy));
  gsw::SeaStateSpec calm;
  ASSERT_TRUE(gsw::SeaStateFromCode(0, calm));
  EXPECT_NEAR(calm.significantWaveHeight, 0.0, 1e-9);  // sea state 0 = flat
}

// <sea_state> drives the field to that WMO sea state's significant wave height.
//////////////////////////////////////////////////
TEST(FFTWaveSimulation, SeaStateSetsSignificantWaveHeight)
{
  gsw::WaveParameters p;
  p.model = "PMS";
  p.gridSize = 64;
  p.tileSize = 256.0;
  p.seed = 11;
  p.seaState = 5;  // "rough", representative Hs ~3.25 m
  gsw::FFTWaveSimulation sim;
  sim.SetParameters(p);
  sim.Update(50.0);  // past the startup ramp

  // Hs = 4 * RMS(eta). Encino calibrates to the PM Hs derived from the
  // sea-state wind/period, so it should land near the canonical value.
  const double hs = 4.0 * RmsHeight(sim.HeightGrid());
  gsw::SeaStateSpec s;
  ASSERT_TRUE(gsw::SeaStateFromCode(5, s));
  EXPECT_NEAR(hs, s.significantWaveHeight, 0.30 * s.significantWaveHeight)
    << "sea state 5 should give Hs ~= " << s.significantWaveHeight
    << " m, got " << hs;
}
