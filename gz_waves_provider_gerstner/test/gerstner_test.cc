/*
 * Copyright (C) 2026 Honu Robotics
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
// The gerstner engine is no longer a dlopen'd gz-plugin; this binary links it
// directly, so register its factory for the CreateWaveSimulation tests.
const bool kGerstnerRegistered = [] {
  gsw::RegisterWaveEngineFactory("gerstner", &gsw::MakeGerstnerWaveField);
  return true;
}();

/// \brief Build a PMS-model Gerstner WavefieldData with a configured engine.
/// \return The populated WavefieldData (algorithm "gerstner").
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

//////////////////////////////////////////////////
TEST(Factory, GerstnerInstantiates)
{
  auto wf = MakePmsField();
  EXPECT_NE(wf.simulation, nullptr);
  EXPECT_EQ(wf.simulation->Kind(), "gerstner");
}

//////////////////////////////////////////////////
TEST(Factory, UnknownAlgorithmReturnsNullptr)
{
  gsw::WaveParameters p;
  auto sim = gsw::CreateWaveSimulation("bogus", p);
  EXPECT_EQ(sim, nullptr);
}

// NOTE: the FFT serialization/decoupling test lives in fft_test.cc, which
// links the fft engine and registers its factory. Engines are plain libraries
// now (no GzPluginHook), so a binary links + registers whichever engine it
// exercises; this one covers gerstner.

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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
    EXPECT_NEAR(omega * omega, gsw::WaveParameters{}.gravity * k,
                1e-6 * omega * omega);
  }
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
TEST(SurfaceElevation, RampUpFromZero)
{
  auto wf = MakePmsField();
  // The startup ramp (1 - exp(-t/tau)) multiplies the whole field: exactly zero
  // at t = 0, growing with time. Probe it via the spatial RMS rather than a
  // single point — the RMS is phase-independent (a single point can sit on a
  // node at one instant and a crest at another), so RMS(t) tracks the ramp
  // directly. With tau = 2 s, ramp(0.5)/ramp(100) ~ 0.22, well under 0.5.
  EXPECT_NEAR(gsw::SurfaceElevation(wf, 1.0, 2.0, 0.0), 0.0, 1e-12);
  const auto spatialRms = [&](double t)
  {
    double sumSq = 0.0;
    int count = 0;
    for (double x = 0.0; x < 150.0; x += 7.0)
      for (double y = 0.0; y < 150.0; y += 7.0)
      {
        const double e = gsw::SurfaceElevation(wf, x, y, t);
        sumSq += e * e;
        ++count;
      }
    return std::sqrt(sumSq / count);
  };
  EXPECT_LT(spatialRms(0.5), spatialRms(100.0) * 0.5);
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
TEST(Jacobian, OneAtZeroSteepness)
{
  auto wf = MakePmsField();
  EXPECT_NEAR(gsw::Jacobian(wf, 0.0, 0.0, 10.0), 1.0, 1e-12);
  EXPECT_NEAR(gsw::Jacobian(wf, 5.0, -3.0, 7.0), 1.0, 1e-12);
}

//////////////////////////////////////////////////
TEST(FoamMask, ZeroAtZeroSteepness)
{
  auto wf = MakePmsField();
  EXPECT_EQ(gsw::FoamMask(wf, 0.0, 0.0, 10.0), 0.0);
}

// The component serializes only the recipe; operator>> restores the params and
// leaves `simulation` null. A consumer rebuilds its own engine from the recipe.
//////////////////////////////////////////////////
TEST(Serialization, RoundTripPreservesRecipe)
{
  auto wf = MakePmsField();
  std::stringstream ss;
  ss << wf;
  gsw::WavefieldData rebuilt;
  ss >> rebuilt;

  EXPECT_EQ(rebuilt.simulation, nullptr)
    << "operator>> must not build a (process-global) engine";
  EXPECT_EQ(rebuilt.algorithm, "gerstner");

  // Rebuild from the recipe; same params → same analytic field.
  rebuilt.simulation =
    gsw::CreateWaveSimulation(rebuilt.algorithm, rebuilt.params);
  ASSERT_NE(rebuilt.simulation, nullptr);
  EXPECT_NEAR(
    gsw::SurfaceElevation(wf, 10.0, 5.0, 7.0),
    gsw::SurfaceElevation(rebuilt, 10.0, 5.0, 7.0),
    1e-9);
}

// The analytic engine samples itself onto a grid for the unified rendering
// contract. The sampled values must match the closed-form queries at the same
// world points, so the renderer draws the same field the buoyancy sees.
//////////////////////////////////////////////////
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

// <sea_state> scales the Gerstner field: a rougher sea -> bigger waves.
//////////////////////////////////////////////////
TEST(Gerstner, SeaStateScalesWaveHeight)
{
  auto sigma = [](int code)
  {
    gsw::WaveParameters p;
    p.model = "PMS";
    p.number = 3;
    p.seaState = code;
    gsw::GerstnerWaveSimulation sim;
    sim.SetParameters(p);
    double s2 = 0.0;
    for (double a : sim.Amplitudes())
      s2 += a * a;
    return std::sqrt(0.5 * s2);  // surface RMS = sqrt(sum a_i^2 / 2)
  };
  EXPECT_GT(sigma(2), 0.0);
  EXPECT_GT(sigma(6), sigma(2)) << "rougher sea state should produce bigger waves";
}

// A non-positive <period> would make omega = 2*pi/period infinite and the PMS
// spectrum (∝ 1/omega^5) blow up. SetParameters must reject it and leave the
// field flat rather than emit NaNs into elevation/normals.
//////////////////////////////////////////////////
TEST(Gerstner, NonPositivePeriodLeavesFieldFlat)
{
  for (double badPeriod : {0.0, -1.0})
  {
    gsw::WaveParameters p;
    p.model = "PMS";
    p.number = 3;
    p.period = badPeriod;
    gsw::GerstnerWaveSimulation sim;
    sim.SetParameters(p);  // must not crash
    EXPECT_EQ(sim.Amplitudes().size(), 0u) << "period=" << badPeriod;
    const double eta = sim.Elevation(1.0, 2.0, 3.0);
    EXPECT_TRUE(std::isfinite(eta)) << "period=" << badPeriod;
    EXPECT_DOUBLE_EQ(eta, 0.0) << "period=" << badPeriod;
  }
}
