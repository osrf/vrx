/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

#include <cmath>
#include <limits>
#include <sstream>

#include <gtest/gtest.h>

#include "gz/sim/waves/Eval.hh"
#include "gz/sim/waves/GerstnerWaveSimulation.hh"
#include "gz/sim/waves/WaveSimulation.hh"
#include "gz/sim/waves/Wavefield.hh"

namespace gsw = gz::sim::waves;

namespace
{
gsw::WavefieldData MakePmsField()
{
  gsw::WavefieldData data;
  data.algorithm = "gerstner";
  data.params.model = "PMS";
  data.params.number = 3;
  data.params.period = 5.0;
  data.params.gain = 0.3;
  data.params.direction = 0.0;
  data.params.angle = 0.4;
  data.params.scale = 1.1;
  data.params.steepness = 0.0;
  data.params.tau = 2.0;
  data.simulation = gsw::CreateWaveSimulation(data.algorithm, data.params);
  return data;
}
}  // namespace

TEST(Factory, GerstnerInstantiates)
{
  auto wf = MakePmsField();
  EXPECT_NE(wf.simulation, nullptr);
  EXPECT_EQ(wf.simulation->Kind(), "gerstner");
}

TEST(Factory, UnknownAlgorithmReturnsNullptr)
{
  gsw::WaveParameters p;
  auto sim = gsw::CreateWaveSimulation("bogus", p);
  EXPECT_EQ(sim, nullptr);
}

namespace
{
// Largest |elevation| over a few scattered sample points.
double MaxAbsElevation(const gsw::WavefieldData &_d, double _t)
{
  double m = 0.0;
  for (const auto &xy : {std::pair{10.0, 12.0}, std::pair{33.0, 47.0},
                         std::pair{5.0, 80.0}, std::pair{61.0, 23.0}})
    m = std::max(m, std::abs(gsw::SurfaceElevation(_d, xy.first, xy.second, _t)));
  return m;
}
}  // namespace

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

TEST(Gerstner, AccessorsArePopulated)
{
  auto wf = MakePmsField();
  const auto *g =
    dynamic_cast<const gsw::GerstnerWaveSimulation *>(wf.simulation.get());
  ASSERT_NE(g, nullptr);
  EXPECT_EQ(g->Amplitudes().size(), 3u);
  EXPECT_EQ(g->Wavenumbers().size(), 3u);
  EXPECT_EQ(g->AngularFrequencies().size(), 3u);
  EXPECT_EQ(g->Steepnesses().size(), 3u);
  EXPECT_EQ(g->Directions().size(), 3u);
}

TEST(Gerstner, DispersionRelationHolds)
{
  auto wf = MakePmsField();
  const auto *g =
    dynamic_cast<const gsw::GerstnerWaveSimulation *>(wf.simulation.get());
  ASSERT_NE(g, nullptr);
  for (std::size_t i = 0; i < g->Amplitudes().size(); ++i)
  {
    const double omega = g->AngularFrequencies()[i];
    const double k = g->Wavenumbers()[i];
    EXPECT_NEAR(omega * omega, 9.80665 * k, 1e-6 * omega * omega);
  }
}

TEST(SurfaceElevation, ZeroAmplitudeProducesZeroElevation)
{
  gsw::WavefieldData data;
  data.algorithm = "gerstner";
  data.params.model = "CWR";
  data.params.number = 3;
  data.params.period = 5.0;
  data.params.amplitude = 0.0;
  data.simulation = gsw::CreateWaveSimulation(data.algorithm, data.params);

  EXPECT_NEAR(gsw::SurfaceElevation(data, 0.0, 0.0, 100.0), 0.0, 1e-12);
  EXPECT_NEAR(gsw::SurfaceElevation(data, 10.0, -3.0, 50.0), 0.0, 1e-12);
}

TEST(SurfaceElevation, IsBounded)
{
  auto wf = MakePmsField();
  const auto *g =
    dynamic_cast<const gsw::GerstnerWaveSimulation *>(wf.simulation.get());
  double bound = 0.0;
  for (double a : g->Amplitudes())
    bound += std::abs(a);

  for (double t = 5.0; t < 30.0; t += 1.7)
  {
    for (double x = -50.0; x <= 50.0; x += 7.3)
    {
      for (double y = -50.0; y <= 50.0; y += 6.1)
      {
        const double eta = gsw::SurfaceElevation(wf, x, y, t);
        EXPECT_LE(std::abs(eta), bound + 1e-9);
      }
    }
  }
}

TEST(SurfaceElevation, RampUpFromZero)
{
  auto wf = MakePmsField();
  EXPECT_NEAR(gsw::SurfaceElevation(wf, 1.0, 2.0, 0.0), 0.0, 1e-12);
  const double eta_long = gsw::SurfaceElevation(wf, 1.0, 2.0, 100.0);
  const double eta_short = gsw::SurfaceElevation(wf, 1.0, 2.0, 0.5);
  EXPECT_LT(std::abs(eta_short), std::abs(eta_long) * 0.5);
}

TEST(ParticleVelocity, FiniteOnPmsField)
{
  auto wf = MakePmsField();
  for (double t = 5.0; t < 20.0; t += 1.0)
  {
    const auto v = gsw::ParticleVelocity(wf, 0.0, 0.0, t);
    EXPECT_TRUE(std::isfinite(v.X()));
    EXPECT_TRUE(std::isfinite(v.Y()));
    EXPECT_TRUE(std::isfinite(v.Z()));
    EXPECT_LT(v.Length(), 10.0);
  }
}

TEST(Normal, IsUnitVector)
{
  auto wf = MakePmsField();
  for (double t = 5.0; t < 20.0; t += 1.0)
  {
    const auto n = gsw::Normal(wf, 1.0, 2.0, t);
    EXPECT_NEAR(n.Length(), 1.0, 1e-9);
    EXPECT_GT(n.Z(), 0.0);
  }
}

TEST(Jacobian, OneAtZeroSteepness)
{
  auto wf = MakePmsField();
  EXPECT_NEAR(gsw::Jacobian(wf, 0.0, 0.0, 10.0), 1.0, 1e-12);
  EXPECT_NEAR(gsw::Jacobian(wf, 5.0, -3.0, 7.0), 1.0, 1e-12);
}

TEST(FoamMask, ZeroAtZeroSteepness)
{
  auto wf = MakePmsField();
  EXPECT_EQ(gsw::FoamMask(wf, 0.0, 0.0, 10.0), 0.0);
}

TEST(Serialization, RoundTripRebuildsSimulation)
{
  auto wf = MakePmsField();
  std::stringstream ss;
  ss << wf;
  gsw::WavefieldData rebuilt;
  ss >> rebuilt;
  EXPECT_NE(rebuilt.simulation, nullptr);
  EXPECT_EQ(rebuilt.algorithm, "gerstner");
  EXPECT_NEAR(
    gsw::SurfaceElevation(wf, 10.0, 5.0, 7.0),
    gsw::SurfaceElevation(rebuilt, 10.0, 5.0, 7.0),
    1e-9);
}

// The analytic backend samples itself onto a grid for the unified rendering
// contract. The sampled values must match the closed-form queries at the same
// world points, so the renderer draws the same field the buoyancy sees.
TEST(Gerstner, FieldSamplesAnalyticGrid)
{
  auto wf = MakePmsField();
  auto *g = dynamic_cast<gsw::GerstnerWaveSimulation *>(wf.simulation.get());
  ASSERT_NE(g, nullptr);
  g->Update(3.0);
  const auto *f = g->Field();
  ASSERT_NE(f, nullptr);
  EXPECT_GT(f->n, 0u);
  EXPECT_GT(f->tile, 0.0);
  ASSERT_NE(f->dz, nullptr);
  ASSERT_NE(f->dx, nullptr);
  ASSERT_NE(f->dy, nullptr);
  ASSERT_NE(f->foam, nullptr);

  const int N = static_cast<int>(f->n);
  const double T = f->tile;
  double maxAbs = 0.0;
  for (int j = 0; j < N; j += N / 4)
  {
    for (int i = 0; i < N; i += N / 4)
    {
      const double x = static_cast<double>(i) * T / N;
      const double y = static_cast<double>(j) * T / N;
      EXPECT_NEAR(f->dz[i + j * N], g->Elevation(x, y, 3.0), 1e-9);
      EXPECT_NEAR(f->foam[i + j * N], g->Jacobian(x, y, 3.0), 1e-9);
      EXPECT_TRUE(std::isfinite(f->dx[i + j * N]));
      EXPECT_TRUE(std::isfinite(f->dy[i + j * N]));
      const double a = std::abs(f->dz[i + j * N]);
      if (a > maxAbs) maxAbs = a;
    }
  }
  EXPECT_GT(maxAbs, 0.0) << "sampled field is flat — Update didn't fill it";
}
