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

#include <gtest/gtest.h>

#include "gz/sim/waves/Eval.hh"
#include "gz/sim/waves/WaveSpectrum.hh"
#include "gz/sim/waves/Wavefield.hh"

namespace gsw = gz::sim::waves;

namespace
{

// Default PMS scenario reminiscent of typical VRX moderate sea-state input.
gsw::WavefieldData MakePmsField()
{
  gsw::WavefieldData data;
  data.params.model = "PMS";
  data.params.number = 3;
  data.params.period = 5.0;
  data.params.gain = 0.3;
  data.params.direction = 0.0;
  data.params.angle = 0.4;
  data.params.scale = 1.1;
  data.params.steepness = 0.0;
  data.params.tau = 2.0;
  gsw::SampleSpectrum(data);
  return data;
}

}  // namespace

TEST(SampleSpectrum, PmsPopulatesAllArrays)
{
  const auto wf = MakePmsField();
  EXPECT_EQ(wf.amplitudes.size(), 3u);
  EXPECT_EQ(wf.wavenumbers.size(), 3u);
  EXPECT_EQ(wf.angularFrequencies.size(), 3u);
  EXPECT_EQ(wf.steepnesses.size(), 3u);
  EXPECT_EQ(wf.directions.size(), 3u);
  EXPECT_EQ(wf.generation, 1u);

  for (std::size_t i = 0; i < wf.amplitudes.size(); ++i)
  {
    EXPECT_GT(wf.amplitudes[i], 0.0) << "amplitude[" << i << "]";
    EXPECT_GT(wf.wavenumbers[i], 0.0) << "wavenumber[" << i << "]";
    EXPECT_GT(wf.angularFrequencies[i], 0.0) << "omega[" << i << "]";
    EXPECT_GE(wf.steepnesses[i], 0.0) << "steepness[" << i << "]";
    EXPECT_LE(wf.steepnesses[i], 1.0) << "steepness[" << i << "]";
    EXPECT_NEAR(wf.directions[i].Length(), 1.0, 1e-12)
      << "direction is unit length";
  }
}

TEST(SampleSpectrum, DispersionRelationHolds)
{
  // ω² = g·k must hold for each component.
  const auto wf = MakePmsField();
  for (std::size_t i = 0; i < wf.amplitudes.size(); ++i)
  {
    const double omega = wf.angularFrequencies[i];
    const double k = wf.wavenumbers[i];
    EXPECT_NEAR(omega * omega, 9.80665 * k, 1e-6 * omega * omega)
      << "deep-water dispersion at component " << i;
  }
}

TEST(SampleSpectrum, UnknownModelClearsArrays)
{
  gsw::WavefieldData data;
  data.params.model = "BOGUS";
  data.params.number = 3;
  gsw::SampleSpectrum(data);

  EXPECT_TRUE(data.amplitudes.empty());
  EXPECT_TRUE(data.wavenumbers.empty());
  EXPECT_EQ(data.generation, 0u);
}

TEST(SampleSpectrum, GenerationBumpsOnEachCall)
{
  gsw::WavefieldData data;
  data.params.model = "PMS";
  data.params.number = 3;
  data.params.period = 5.0;
  data.params.gain = 0.5;

  gsw::SampleSpectrum(data);
  const auto gen1 = data.generation;
  gsw::SampleSpectrum(data);
  const auto gen2 = data.generation;

  EXPECT_GT(gen2, gen1);
}

TEST(SurfaceElevation, ZeroAmplitudeProducesZeroElevation)
{
  // CWR with amplitude=0 → all components have amplitude 0.
  gsw::WavefieldData data;
  data.params.model = "CWR";
  data.params.number = 3;
  data.params.period = 5.0;
  data.params.amplitude = 0.0;
  gsw::SampleSpectrum(data);

  // After the tau ramp settles, elevation should be (numerically) zero.
  EXPECT_NEAR(gsw::SurfaceElevation(data, 0.0, 0.0, 100.0), 0.0, 1e-12);
  EXPECT_NEAR(gsw::SurfaceElevation(data, 10.0, -3.0, 50.0), 0.0, 1e-12);
}

TEST(SurfaceElevation, IsBounded)
{
  // Sum of |amplitudes| is the analytic upper bound on |η|.
  const auto wf = MakePmsField();
  double bound = 0.0;
  for (double a : wf.amplitudes)
    bound += std::abs(a);

  // Sweep a coarse grid of (x, y, t) and check the bound holds within
  // a tolerance for the post-ramp regime.
  for (double t = 5.0; t < 30.0; t += 1.7)
  {
    for (double x = -50.0; x <= 50.0; x += 7.3)
    {
      for (double y = -50.0; y <= 50.0; y += 6.1)
      {
        const double eta = gsw::SurfaceElevation(wf, x, y, t);
        EXPECT_LE(std::abs(eta), bound + 1e-9)
          << "x=" << x << " y=" << y << " t=" << t;
      }
    }
  }
}

TEST(SurfaceElevation, RampUpFromZero)
{
  // At t=0 the ramp factor is exactly 0, so elevation is zero.
  const auto wf = MakePmsField();
  EXPECT_NEAR(gsw::SurfaceElevation(wf, 1.0, 2.0, 0.0), 0.0, 1e-12);

  // After many tau, elevation should be essentially un-attenuated.
  const double eta_long = gsw::SurfaceElevation(wf, 1.0, 2.0, 100.0);
  const double eta_short = gsw::SurfaceElevation(wf, 1.0, 2.0, 0.5);
  // Short-time elevation is heavily damped relative to long-time.
  EXPECT_LT(std::abs(eta_short), std::abs(eta_long) * 0.5);
}

TEST(ParticleVelocity, ZeroAtZeroAmplitude)
{
  gsw::WavefieldData data;
  data.params.model = "CWR";
  data.params.number = 3;
  data.params.amplitude = 0.0;
  gsw::SampleSpectrum(data);

  const auto v = gsw::ParticleVelocity(data, 5.0, 5.0, 50.0);
  EXPECT_NEAR(v.X(), 0.0, 1e-12);
  EXPECT_NEAR(v.Y(), 0.0, 1e-12);
  EXPECT_NEAR(v.Z(), 0.0, 1e-12);
}

TEST(ParticleVelocity, FiniteOnPmsField)
{
  const auto wf = MakePmsField();
  for (double t = 5.0; t < 20.0; t += 1.0)
  {
    const auto v = gsw::ParticleVelocity(wf, 0.0, 0.0, t);
    EXPECT_TRUE(std::isfinite(v.X()));
    EXPECT_TRUE(std::isfinite(v.Y()));
    EXPECT_TRUE(std::isfinite(v.Z()));
    EXPECT_LT(v.Length(), 10.0) << "orbital velocity is reasonable";
  }
}

TEST(Normal, IsUnitVector)
{
  const auto wf = MakePmsField();
  for (double t = 5.0; t < 20.0; t += 1.0)
  {
    const auto n = gsw::Normal(wf, 1.0, 2.0, t);
    EXPECT_NEAR(n.Length(), 1.0, 1e-9);
    EXPECT_GT(n.Z(), 0.0) << "normal points up";
  }
}

TEST(Jacobian, OneAtZeroSteepness)
{
  // With steepness 0, Gerstner reduces to Airy: J should be exactly 1.
  const auto wf = MakePmsField();  // steepness=0
  EXPECT_NEAR(gsw::Jacobian(wf, 0.0, 0.0, 10.0), 1.0, 1e-12);
  EXPECT_NEAR(gsw::Jacobian(wf, 5.0, -3.0, 7.0), 1.0, 1e-12);
}

TEST(Jacobian, DipsBelowOneAtHighSteepness)
{
  gsw::WavefieldData data;
  data.params.model = "CWR";
  data.params.number = 3;
  data.params.period = 5.0;
  data.params.amplitude = 1.0;
  data.params.steepness = 1.0;  // max Gerstner steepness
  gsw::SampleSpectrum(data);

  // Sweep and check that the Jacobian goes below 1 somewhere.
  double j_min = std::numeric_limits<double>::infinity();
  for (double t = 5.0; t < 30.0; t += 0.5)
  {
    for (double x = -10.0; x <= 10.0; x += 0.5)
    {
      const double j = gsw::Jacobian(data, x, 0.0, t);
      j_min = std::min(j_min, j);
    }
  }
  EXPECT_LT(j_min, 1.0) << "max-steepness Gerstner should fold (J<1) somewhere";
}

TEST(FoamMask, ZeroAtZeroSteepness)
{
  const auto wf = MakePmsField();
  EXPECT_EQ(gsw::FoamMask(wf, 0.0, 0.0, 10.0), 0.0);
  EXPECT_EQ(gsw::FoamMask(wf, 5.0, -3.0, 7.0), 0.0);
}
