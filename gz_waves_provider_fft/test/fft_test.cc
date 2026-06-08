/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>
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

gsw::WaveParameters DefaultParams()
{
  gsw::WaveParameters p;
  p.model = "PMS";
  p.number = 3;          // ignored by FFT backend (uses gridSize)
  p.period = 6.0;        // ω_P ≈ 1.05 rad/s, V19 ≈ 8 m/s
  p.gain = 1.0;
  p.direction = 0.0;
  p.tau = 2.0;
  return p;
}

gsw::FFTWaveSimulation MakeSim(std::uint32_t seed = 1)
{
  return gsw::FFTWaveSimulation(
    DefaultParams(), /*tile=*/100.0, /*grid=*/64, seed);
}

double RmsHeight(const Eigen::MatrixXd &g)
{
  return std::sqrt(g.squaredNorm() / static_cast<double>(g.size()));
}

// Largest |elevation| over a few scattered sample points.
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

TEST(FFTWaveSimulation, FactoryRoute)
{
  auto sim = gsw::CreateWaveSimulation("fft", DefaultParams());
  ASSERT_NE(sim, nullptr);
  EXPECT_EQ(sim->Kind(), "fft");
  ASSERT_TRUE(sim->Bounds().has_value());
  EXPECT_GT(sim->Bounds()->x, 0.0);
}

// Reproduces, at the unit level, the cross-process bug behind a "flat" FFT
// wave field: the Wavefield component serializes only the *recipe*, so the
// deserialization path mints a FRESH simulation that has only ever run
// Update(0) (ramp=0 → identically zero). A consumer that reads the
// deserialized instance therefore sees a flat field unless it advances the
// instance it actually holds. (Gerstner is analytic/stateless and immune;
// this only bites the FFT/Encino grid-based backend.)
TEST(Wavefield, FftSimDecouplesAcrossSerialization)
{
  gsw::WavefieldData live;
  live.algorithm = "fft";
  live.generation = 918273;  // unique → fresh entry in operator>>'s static cache
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
  ASSERT_NE(round.simulation, nullptr);

  // BUG: the deserialized instance was only ever Update(0)'d → identically flat.
  EXPECT_NEAR(MaxAbsElevation(round, 20.0), 0.0, 1e-9)
    << "deserialized FFT sim is never advanced → flat (the reported symptom)";

  // FIX: advancing the held instance on read recovers the field, regardless of
  // which instance a consumer ended up with. Same seed → matches the live field.
  round.simulation->Update(20.0);
  EXPECT_GT(MaxAbsElevation(round, 20.0), 1e-3)
    << "advancing the held sim makes the read correct";
}

TEST(FFTWaveSimulation, HeightsAreFinite)
{
  auto sim = MakeSim();
  sim.Update(5.0);
  const auto &g = sim.HeightGrid();
  for (int i = 0; i < g.rows(); ++i)
    for (int j = 0; j < g.cols(); ++j)
      EXPECT_TRUE(std::isfinite(g(i, j))) << "(" << i << "," << j << ")";
}

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
// pinched crests report Jacobian < 1 and produce whitecaps. The in-tree
// Phillips CPU path has no folding field, so it stays identically 1 (no foam).
// The test binary picks up whatever the build provides — Encino when compiled
// with EncinoWaves (the default), Phillips otherwise — and branches on
// UseEncino().
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
  double minJac = 1e9, maxJac = -1e9, maxFoam = 0.0;
  for (int i = 0; i < 32; ++i)
    for (int j = 0; j < 32; ++j)
    {
      const double jac = sim.Jacobian(i * L / 32.0, j * L / 32.0, 20.0);
      minJac = std::min(minJac, jac);
      maxJac = std::max(maxJac, jac);
      const double foam =
        jac < 0.6 ? std::clamp(1.0 - jac / 0.6, 0.0, 1.0) : 0.0;
      maxFoam = std::max(maxFoam, foam);
    }

  if (sim.UseEncino())
  {
    EXPECT_LT(minJac, 0.6) << "Encino crests should fold below the foam threshold";
    EXPECT_GT(maxFoam, 0.0) << "Encino path should produce whitecaps";
  }
  else
  {
    EXPECT_NEAR(minJac, 1.0, 1e-9) << "Phillips CPU path has no folding field";
    EXPECT_NEAR(maxJac, 1.0, 1e-9);
    EXPECT_NEAR(maxFoam, 0.0, 1e-12);
  }
}

// The Encino band-pass filter (GZ_WAVES_ENCINO_FILTER_*) reshapes which
// wavelengths survive in the spectrum. Same seed → without the filter the
// field is one thing; with a narrow band it differs. (The amplitude
// calibration renormalises total energy, so we compare the field shape, not
// its RMS.) Only meaningful on the Encino backend; no-op early-out otherwise.
TEST(FFTWaveSimulation, EncinoBandPassFilterReshapesField)
{
  gsw::WaveParameters p;
  p.model = "PMS";
  p.period = 3.2;
  p.gain = 1.0;
  p.tileSize = 200.0;
  p.gridSize = 64;
  p.seed = 11;

  unsetenv("GZ_WAVES_ENCINO_FILTER_MIN_WL");
  unsetenv("GZ_WAVES_ENCINO_FILTER_MAX_WL");
  gsw::FFTWaveSimulation plain(p, 200.0, 64, 11);
  if (!plain.UseEncino())
    return;  // band-pass filter only applies to the Encino spectrum
  plain.Update(20.0);
  const Eigen::MatrixXd a = plain.HeightGrid();

  // Keep only 30–100 m waves (suppresses the ~16 m peak and the ripples).
  setenv("GZ_WAVES_ENCINO_FILTER_MIN_WL", "30", 1);
  setenv("GZ_WAVES_ENCINO_FILTER_MAX_WL", "100", 1);
  gsw::FFTWaveSimulation filtered(p, 200.0, 64, 11);
  filtered.Update(20.0);
  const Eigen::MatrixXd b = filtered.HeightGrid();
  unsetenv("GZ_WAVES_ENCINO_FILTER_MIN_WL");
  unsetenv("GZ_WAVES_ENCINO_FILTER_MAX_WL");

  EXPECT_GT((a - b).cwiseAbs().maxCoeff(), 1e-3)
    << "band-pass filter should reshape the Encino field";
}

TEST(FFTWaveSimulation, TimeEvolutionChangesField)
{
  auto sim = MakeSim();
  sim.Update(10.0);
  const Eigen::MatrixXd snapshotA = sim.HeightGrid();
  sim.Update(11.0);
  const Eigen::MatrixXd snapshotB = sim.HeightGrid();
  EXPECT_GT((snapshotB - snapshotA).cwiseAbs().maxCoeff(), 1e-3);
}

// The Field() rendering view must alias the backend grids with the documented
// column-major layout (element (i,j) at i + j*N), so the renderer can upload
// it without knowing the concrete backend.
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

TEST(FFTWaveSimulation, WindDirectionBiasesAmplitude)
{
  // Spectra peak amplitude along the wind direction (Phillips/Tessendorf).
  // We expect a higher RMS along the wind axis than perpendicular when
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
