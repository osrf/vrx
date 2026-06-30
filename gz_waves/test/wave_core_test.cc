/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// Unit tests for the gz_waves *core* — the engine-agnostic contract. The core
// can't link a real engine (that would be a dependency cycle), so these tests
// supply a tiny stub IWaveField and a stub WavesSystemBase subclass and drive
// everything — Eval, the factory registry, Wavefield serialization, the
// sea-state helpers, and the source-system plumbing (Configure / PreUpdate /
// Reset / the set_parameters service) — through a real EntityComponentManager.

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include <sdf/Root.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/World.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Wavefield.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/systems/WavesSystemBase.hh"
#include "gz/sim/waves/Eval.hh"
#include "gz/sim/waves/WaveSimulation.hh"
#include "gz/sim/waves/Wavefield.hh"

namespace gsw = gz::sim::waves;

namespace
{
/// \brief A minimal `IWaveField` stub: deterministic and instrumented. Returns
/// values built from its arguments so tests can confirm `Eval` forwards
/// (x, y, t) faithfully, and counts `Update`/`SetParameters` calls.
class StubWaveField : public gsw::IWaveField
{
  /// \brief Elevation = x + y + t, so callers can verify Eval forwards args.
  public: double Elevation(double _x, double _y, double _t) const override
  {
    return _x + _y + _t;
  }
  /// \brief Particle velocity = (x, y, t).
  public: gz::math::Vector3d ParticleVelocity(
    double _x, double _y, double _t) const override
  {
    return {_x, _y, _t};
  }
  /// \brief A fixed +Z unit normal.
  public: gz::math::Vector3d Normal(
    double /*_x*/, double /*_y*/, double /*_t*/) const override
  {
    return gz::math::Vector3d(0.0, 0.0, 2.0).Normalized();
  }
  /// \brief Returns the settable `jac` test value.
  public: double Jacobian(
    double /*_x*/, double /*_y*/, double /*_t*/) const override
  {
    return this->jac;
  }
  /// \brief Record the params and count the call.
  public: void SetParameters(const gsw::WaveParameters &_p) override
  {
    this->lastParams = _p;
    ++this->setParamsCount;
  }
  /// \brief Record the time and count the call.
  public: void Update(double _t) override
  {
    this->lastUpdate = _t;
    ++this->updateCount;
  }
  /// \brief Backend token.
  public: std::string_view Kind() const override { return "stub"; }

  /// \brief Value returned by Jacobian() (drives FoamMask in the tests).
  public: double jac{1.0};
  /// \brief Params seen by the last SetParameters() call.
  public: gsw::WaveParameters lastParams;
  /// \brief Number of SetParameters() calls.
  public: int setParamsCount{0};
  /// \brief Time seen by the last Update() call.
  public: double lastUpdate{-1.0};
  /// \brief Number of Update() calls.
  public: int updateCount{0};
};

/// \brief A stub source system: selects the "stub" engine and builds one.
class StubWavesSystem : public gz::sim::systems::WavesSystemBase
{
  /// \brief The engine token recorded in the component.
  protected: std::string EngineToken() const override { return "stub"; }
  /// \brief Build and configure a StubWaveField from `_p`.
  protected: std::shared_ptr<gsw::IWaveField> MakeEngine(
    const gsw::WaveParameters &_p) const override
  {
    auto engine = std::make_shared<StubWaveField>();
    engine->SetParameters(_p);
    return engine;
  }
};

// Register factories used by the registry tests (one good, one that returns
// null). Done once at static init.
const bool kRegistered = []
{
  gsw::RegisterWaveEngineFactory("stub",
    [] { return std::static_pointer_cast<gsw::IWaveField>(
           std::make_shared<StubWaveField>()); });
  gsw::RegisterWaveEngineFactory("null-factory",
    [] { return std::shared_ptr<gsw::IWaveField>(nullptr); });
  return true;
}();
}  // namespace

// ===========================================================================
// Eval — the consumer query facade (Eval.cc)
// ===========================================================================
//////////////////////////////////////////////////
TEST(Eval, ForwardsToEngine)
{
  gsw::WavefieldData wf;
  wf.simulation = std::make_shared<StubWaveField>();

  EXPECT_DOUBLE_EQ(gsw::SurfaceElevation(wf, 1.0, 2.0, 3.0), 6.0);
  const auto v = gsw::ParticleVelocity(wf, 1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(v.X(), 1.0);
  EXPECT_DOUBLE_EQ(v.Z(), 3.0);
  const auto n = gsw::Normal(wf, 0.0, 0.0, 0.0);
  EXPECT_NEAR(n.Length(), 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(gsw::Jacobian(wf, 0.0, 0.0, 0.0), 1.0);

  gsw::Advance(wf, 7.5);
  EXPECT_EQ(
    static_cast<StubWaveField *>(wf.simulation.get())->updateCount, 1);
  EXPECT_DOUBLE_EQ(
    static_cast<StubWaveField *>(wf.simulation.get())->lastUpdate, 7.5);
}

//////////////////////////////////////////////////
TEST(Eval, NullEngineFallsBackToStillWater)
{
  gsw::WavefieldData wf;  // no simulation
  ASSERT_EQ(wf.simulation, nullptr);

  EXPECT_DOUBLE_EQ(gsw::SurfaceElevation(wf, 1.0, 2.0, 3.0), 0.0);
  EXPECT_EQ(gsw::ParticleVelocity(wf, 1.0, 2.0, 3.0), gz::math::Vector3d::Zero);
  EXPECT_EQ(gsw::Normal(wf, 1.0, 2.0, 3.0), gz::math::Vector3d::UnitZ);
  EXPECT_DOUBLE_EQ(gsw::Jacobian(wf, 1.0, 2.0, 3.0), 1.0);
  EXPECT_DOUBLE_EQ(gsw::FoamMask(wf, 1.0, 2.0, 3.0), 0.0);
  gsw::Advance(wf, 1.0);  // no-op, must not crash
}

//////////////////////////////////////////////////
TEST(Eval, FoamMaskFromJacobian)
{
  gsw::WavefieldData wf;
  auto stub = std::make_shared<StubWaveField>();
  wf.simulation = stub;

  stub->jac = 1.0;  // unfolded -> no foam
  EXPECT_DOUBLE_EQ(gsw::FoamMask(wf, 0, 0, 0, 0.6), 0.0);
  stub->jac = 0.6;  // exactly at threshold -> no foam
  EXPECT_DOUBLE_EQ(gsw::FoamMask(wf, 0, 0, 0, 0.6), 0.0);
  stub->jac = 0.3;  // half-folded
  EXPECT_NEAR(gsw::FoamMask(wf, 0, 0, 0, 0.6), 0.5, 1e-12);
  stub->jac = 0.0;  // fully folded
  EXPECT_DOUBLE_EQ(gsw::FoamMask(wf, 0, 0, 0, 0.6), 1.0);
  stub->jac = -1.0;  // beyond folded -> clamped to 1
  EXPECT_DOUBLE_EQ(gsw::FoamMask(wf, 0, 0, 0, 0.6), 1.0);
}

// ===========================================================================
// Registry — RegisterWaveEngineFactory / CreateWaveSimulation (WaveSimulation.cc)
// ===========================================================================
//////////////////////////////////////////////////
TEST(Registry, BuildsAndConfigures)
{
  gsw::WaveParameters p;
  p.period = 4.2;
  auto sim = gsw::CreateWaveSimulation("stub", p);
  ASSERT_NE(sim, nullptr);
  EXPECT_EQ(sim->Kind(), "stub");
  // CreateWaveSimulation must call SetParameters with our params.
  EXPECT_DOUBLE_EQ(
    static_cast<StubWaveField *>(sim.get())->lastParams.period, 4.2);
  EXPECT_EQ(static_cast<StubWaveField *>(sim.get())->setParamsCount, 1);
}

//////////////////////////////////////////////////
TEST(Registry, UnknownTokenReturnsNull)
{
  EXPECT_EQ(gsw::CreateWaveSimulation("nope", gsw::WaveParameters{}), nullptr);
}

//////////////////////////////////////////////////
TEST(Registry, NullFactoryReturnsNull)
{
  EXPECT_EQ(gsw::CreateWaveSimulation("null-factory", gsw::WaveParameters{}),
            nullptr);
}

// ===========================================================================
// Wavefield serialization — operator<< / operator>> (Wavefield.hh)
// ===========================================================================
//////////////////////////////////////////////////
TEST(Serialization, RoundTripsEveryFieldAndDropsEngine)
{
  // Set every field to a distinct non-default value, with doubles chosen to
  // need full precision, so a field forgotten in operator<< / operator>> (or a
  // precision-losing serializer) is caught. Keep this in sync with the struct.
  gsw::WavefieldData in;
  in.algorithm = "stub";
  in.generation = 9;
  in.updateRate = 17.5;
  auto &p = in.params;
  p.model = "CWR";
  p.number = 7;
  p.period = 3.141592653589793;
  p.amplitude = 0.123456789012345;
  p.direction = 1.234567890123456;
  p.angle = 0.456789012345678;
  p.scale = 1.098765432109876;
  p.steepness = 0.246802468024680;
  p.phase = 2.718281828459045;
  p.tau = 1.414213562373095;
  p.gain = 0.876543210987654;
  p.tileSize = 222.222222222222;
  p.gridSize = 256;
  p.seed = 4242424242u;
  p.choppiness = -1.732050807568877;
  p.spectrum = "jonswap";
  p.spreading = "mitsuyasu";
  p.dispersion = "finite";
  p.depth = 88.8888888888888;
  p.fetch = 321.987654321098;
  p.swell = 0.314159265358979;
  p.troughDamping = 0.271828182845904;
  p.filterMinWavelength = 12.3456789012345;
  p.filterMaxWavelength = 98.7654321098765;
  p.filterSoftWidth = 3.33333333333333;
  p.filterMin = 0.135792468013579;
  p.filterInvert = true;
  p.seaState = 6;
  p.gravity = 9.806649999999999;
  in.simulation = std::make_shared<StubWaveField>();  // must NOT serialize

  std::stringstream ss;
  ss << in;
  gsw::WavefieldData out;
  ss >> out;

  EXPECT_EQ(out.algorithm, in.algorithm);
  EXPECT_EQ(out.generation, in.generation);
  EXPECT_DOUBLE_EQ(out.updateRate, in.updateRate);
  const auto &q = out.params;
  EXPECT_EQ(q.model, p.model);
  EXPECT_EQ(q.number, p.number);
  EXPECT_DOUBLE_EQ(q.period, p.period);
  EXPECT_DOUBLE_EQ(q.amplitude, p.amplitude);
  EXPECT_DOUBLE_EQ(q.direction, p.direction);
  EXPECT_DOUBLE_EQ(q.angle, p.angle);
  EXPECT_DOUBLE_EQ(q.scale, p.scale);
  EXPECT_DOUBLE_EQ(q.steepness, p.steepness);
  EXPECT_DOUBLE_EQ(q.phase, p.phase);
  EXPECT_DOUBLE_EQ(q.tau, p.tau);
  EXPECT_DOUBLE_EQ(q.gain, p.gain);
  EXPECT_DOUBLE_EQ(q.tileSize, p.tileSize);
  EXPECT_EQ(q.gridSize, p.gridSize);
  EXPECT_EQ(q.seed, p.seed);
  EXPECT_DOUBLE_EQ(q.choppiness, p.choppiness);
  EXPECT_EQ(q.spectrum, p.spectrum);
  EXPECT_EQ(q.spreading, p.spreading);
  EXPECT_EQ(q.dispersion, p.dispersion);
  EXPECT_DOUBLE_EQ(q.depth, p.depth);
  EXPECT_DOUBLE_EQ(q.fetch, p.fetch);
  EXPECT_DOUBLE_EQ(q.swell, p.swell);
  EXPECT_DOUBLE_EQ(q.troughDamping, p.troughDamping);
  EXPECT_DOUBLE_EQ(q.filterMinWavelength, p.filterMinWavelength);
  EXPECT_DOUBLE_EQ(q.filterMaxWavelength, p.filterMaxWavelength);
  EXPECT_DOUBLE_EQ(q.filterSoftWidth, p.filterSoftWidth);
  EXPECT_DOUBLE_EQ(q.filterMin, p.filterMin);
  EXPECT_TRUE(q.filterInvert);
  EXPECT_EQ(q.seaState, p.seaState);
  EXPECT_DOUBLE_EQ(q.gravity, p.gravity);
  EXPECT_EQ(out.simulation, nullptr);  // recipe-only: engine is rebuilt later
}

//////////////////////////////////////////////////
// String fields go through std::quoted, so an empty value or one containing a
// space (both reachable via the set_parameters service) round-trips without
// desyncing the positional field stream and corrupting later fields.
TEST(Serialization, QuotedStringsSurviveSpacesAndEmpty)
{
  gsw::WavefieldData in;
  in.algorithm = "stub";
  in.params.model = "";                 // empty
  in.params.spectrum = "two words";     // embedded space
  in.params.spreading = "";             // empty
  in.params.dispersion = "a b c";       // embedded spaces
  in.params.seaState = 5;               // a field *after* the strings
  in.params.gravity = 9.81;

  std::stringstream ss;
  ss << in;
  gsw::WavefieldData out;
  ss >> out;

  EXPECT_EQ(out.params.model, "");
  EXPECT_EQ(out.params.spectrum, "two words");
  EXPECT_EQ(out.params.spreading, "");
  EXPECT_EQ(out.params.dispersion, "a b c");
  // The trailing fields are still aligned despite the awkward strings.
  EXPECT_EQ(out.params.seaState, 5);
  EXPECT_DOUBLE_EQ(out.params.gravity, 9.81);
}

// ===========================================================================
// Helpers — StartupRamp / SeaStateFromCode / WithSeaState (Wavefield.hh)
// ===========================================================================
//////////////////////////////////////////////////
TEST(Helpers, StartupRamp)
{
  EXPECT_DOUBLE_EQ(gsw::StartupRamp(5.0, 0.0), 1.0);   // disabled
  EXPECT_DOUBLE_EQ(gsw::StartupRamp(5.0, -1.0), 1.0);  // disabled
  EXPECT_DOUBLE_EQ(gsw::StartupRamp(0.0, 2.0), 0.0);   // t=0
  EXPECT_NEAR(gsw::StartupRamp(2.0, 2.0), 1.0 - std::exp(-1.0), 1e-12);
  EXPECT_GT(gsw::StartupRamp(4.0, 2.0), gsw::StartupRamp(2.0, 2.0));
}

//////////////////////////////////////////////////
TEST(Helpers, SeaStateFromCode)
{
  gsw::SeaStateSpec s;
  EXPECT_TRUE(gsw::SeaStateFromCode(0, s));
  EXPECT_DOUBLE_EQ(s.significantWaveHeight, 0.0);
  EXPECT_TRUE(gsw::SeaStateFromCode(5, s));
  EXPECT_GT(s.significantWaveHeight, 0.0);
  EXPECT_GT(s.peakPeriod, 0.0);
  EXPECT_GT(s.windSpeed, 0.0);

  gsw::SeaStateSpec untouched;
  EXPECT_FALSE(gsw::SeaStateFromCode(-1, untouched));
  EXPECT_FALSE(gsw::SeaStateFromCode(10, untouched));
  EXPECT_DOUBLE_EQ(untouched.significantWaveHeight, 0.0);
}

//////////////////////////////////////////////////
TEST(Helpers, WithSeaState)
{
  gsw::WaveParameters unset;  // seaState defaults to -1
  unset.period = 7.0;
  EXPECT_DOUBLE_EQ(gsw::WithSeaState(unset).period, 7.0);  // unchanged

  gsw::WaveParameters calm;
  calm.seaState = 0;
  EXPECT_DOUBLE_EQ(gsw::WithSeaState(calm).gain, 0.0);  // glassy

  gsw::WaveParameters rough;
  rough.seaState = 5;
  rough.model = "CWR";
  const auto applied = gsw::WithSeaState(rough);
  EXPECT_DOUBLE_EQ(applied.gain, 1.0);
  EXPECT_GT(applied.period, 0.0);
  EXPECT_GT(applied.amplitude, 0.0);  // CWR sets amplitude = Hs/2

  gsw::WaveParameters bad;
  bad.seaState = 99;  // out of range -> unchanged
  bad.period = 6.0;
  EXPECT_DOUBLE_EQ(gsw::WithSeaState(bad).period, 6.0);
}

// ===========================================================================
// WavesSystemBase — the source-system plumbing, driven through a real ECM
// ===========================================================================
/// \brief Fixture for the `WavesSystemBase` tests: a world entity carrying the
/// components `Configure` needs (World/Name/Gravity), plus helpers to build the
/// plugin SDF element and `UpdateInfo` values.
class WavesSystemTest : public ::testing::Test
{
  /// \brief Create the world entity with the World/Name/Gravity components
  /// Configure reads.
  protected: void SetUp() override
  {
    this->world = this->ecm.CreateEntity();
    this->ecm.CreateComponent(this->world, gz::sim::components::World());
    this->ecm.CreateComponent(this->world,
      gz::sim::components::Name("test_world"));
    this->ecm.CreateComponent(this->world,
      gz::sim::components::Gravity(gz::math::Vector3d(0.0, 0.0, -9.8)));
  }

  /// \brief Build the plugin's SDF element (kept alive for the test via
  /// `roots`).
  /// \param[in] _inner The XML inside the `<plugin>` element.
  /// \return The parsed `<plugin>` element.
  protected: sdf::ElementPtr Plugin(const std::string &_inner)
  {
    auto root = std::make_shared<sdf::Root>();
    const std::string xml =
      "<sdf version='1.9'><world name='test_world'>"
      "<plugin name='waves' filename='libstub.so'>" + _inner +
      "</plugin></world></sdf>";
    const auto errs = root->LoadSdfString(xml);
    EXPECT_TRUE(errs.empty());
    this->roots.push_back(root);
    return root->WorldByIndex(0)->Element()->GetElement("plugin");
  }

  /// \brief Build an UpdateInfo at sim time `_t` [s], optionally paused.
  /// \param[in] _t      Simulation time [s].
  /// \param[in] _paused Whether the update is paused.
  /// \return The populated UpdateInfo.
  protected: static gz::sim::UpdateInfo Info(double _t, bool _paused = false)
  {
    gz::sim::UpdateInfo info;
    info.simTime =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(_t));
    info.paused = _paused;
    return info;
  }

  /// \brief The Wavefield component on the world entity, or null if absent.
  protected: const gz::sim::components::Wavefield *Comp()
  {
    return this->ecm.Component<gz::sim::components::Wavefield>(this->world);
  }

  /// \brief The entity-component manager under test.
  protected: gz::sim::EntityComponentManager ecm;
  /// \brief Event manager passed to Configure.
  protected: gz::sim::EventManager evm;
  /// \brief The world entity carrying the Wavefield component.
  protected: gz::sim::Entity world{gz::sim::kNullEntity};
  /// \brief Keeps parsed SDF roots alive for the duration of a test.
  protected: std::vector<std::shared_ptr<sdf::Root>> roots;
};

//////////////////////////////////////////////////
TEST_F(WavesSystemTest, ConfigureParsesSdfAndWritesComponent)
{
  StubWavesSystem sys;
  sys.Configure(this->world, this->Plugin(
    "<update_rate>40</update_rate>"
    "<wave><model>CWR</model><period>3.0</period><gain>0.7</gain>"
    "<grid_size>64</grid_size><spectrum>pms</spectrum>"
    "<sea_state>2</sea_state><filter_invert>true</filter_invert></wave>"),
    this->ecm, this->evm);

  const auto *comp = this->Comp();
  ASSERT_NE(comp, nullptr);
  EXPECT_EQ(comp->Data().algorithm, "stub");
  EXPECT_EQ(comp->Data().generation, 1u);
  EXPECT_DOUBLE_EQ(comp->Data().updateRate, 40.0);
  EXPECT_EQ(comp->Data().params.model, "CWR");
  EXPECT_DOUBLE_EQ(comp->Data().params.period, 3.0);
  EXPECT_EQ(comp->Data().params.gridSize, 64u);
  EXPECT_EQ(comp->Data().params.spectrum, "pms");
  EXPECT_EQ(comp->Data().params.seaState, 2);
  EXPECT_TRUE(comp->Data().params.filterInvert);
  // Gravity taken from the world's Gravity component.
  EXPECT_DOUBLE_EQ(comp->Data().params.gravity, 9.8);
  // The engine was built and configured with the parsed params.
  auto *engine = dynamic_cast<StubWaveField *>(comp->Data().simulation.get());
  ASSERT_NE(engine, nullptr);
  EXPECT_DOUBLE_EQ(engine->lastParams.period, 3.0);
}

//////////////////////////////////////////////////
TEST_F(WavesSystemTest, ConfigureWithoutWaveUsesDefaults)
{
  StubWavesSystem sys;
  sys.Configure(this->world, this->Plugin("<update_rate>15</update_rate>"),
    this->ecm, this->evm);

  const auto *comp = this->Comp();
  ASSERT_NE(comp, nullptr);
  EXPECT_DOUBLE_EQ(comp->Data().updateRate, 15.0);
  EXPECT_EQ(comp->Data().params.model, gsw::WaveParameters{}.model);
  EXPECT_DOUBLE_EQ(comp->Data().params.period, gsw::WaveParameters{}.period);
}

//////////////////////////////////////////////////
TEST_F(WavesSystemTest, PreUpdateAdvancesEngineWithThrottle)
{
  StubWavesSystem sys;
  sys.Configure(this->world,
    this->Plugin("<update_rate>40</update_rate>"), this->ecm, this->evm);
  auto *engine = dynamic_cast<StubWaveField *>(this->Comp()->Data().simulation.get());
  ASSERT_NE(engine, nullptr);

  sys.PreUpdate(this->Info(1.000), this->ecm);   // advances
  EXPECT_EQ(engine->updateCount, 1);
  EXPECT_DOUBLE_EQ(engine->lastUpdate, 1.0);
  sys.PreUpdate(this->Info(1.010), this->ecm);   // < 1/40 s later -> throttled
  EXPECT_EQ(engine->updateCount, 1);
  sys.PreUpdate(this->Info(1.050), this->ecm);   // enough elapsed -> advances
  EXPECT_EQ(engine->updateCount, 2);
  sys.PreUpdate(this->Info(2.0, /*paused=*/true), this->ecm);  // paused -> skip
  EXPECT_EQ(engine->updateCount, 2);
}

//////////////////////////////////////////////////
TEST_F(WavesSystemTest, PreUpdateRepointsClearedEngine)
{
  StubWavesSystem sys;
  sys.Configure(this->world,
    this->Plugin("<update_rate>40</update_rate>"), this->ecm, this->evm);

  // Simulate an ECM deserialize (operator>> nulls the engine pointer).
  this->ecm.Component<gz::sim::components::Wavefield>(this->world)
    ->Data().simulation.reset();
  sys.PreUpdate(this->Info(1.0), this->ecm);
  EXPECT_NE(this->Comp()->Data().simulation, nullptr);  // re-pointed
}

//////////////////////////////////////////////////
TEST_F(WavesSystemTest, ResetRewindsThrottle)
{
  StubWavesSystem sys;
  sys.Configure(this->world,
    this->Plugin("<update_rate>40</update_rate>"), this->ecm, this->evm);
  auto *engine = dynamic_cast<StubWaveField *>(this->Comp()->Data().simulation.get());
  ASSERT_NE(engine, nullptr);

  sys.PreUpdate(this->Info(5.0), this->ecm);
  EXPECT_EQ(engine->updateCount, 1);
  sys.Reset(this->Info(0.0), this->ecm);          // rewind throttle
  sys.PreUpdate(this->Info(0.001), this->ecm);    // small t, but advances again
  EXPECT_EQ(engine->updateCount, 2);
}

//////////////////////////////////////////////////
TEST_F(WavesSystemTest, SetParametersServiceAppliesUpdate)
{
  StubWavesSystem sys;
  sys.Configure(this->world,
    this->Plugin("<update_rate>40</update_rate><wave><gain>0.1</gain></wave>"),
    this->ecm, this->evm);

  gz::transport::Node node;
  gz::msgs::Param req;
  auto &m = *req.mutable_params();
  auto anyD = [](double v) { gz::msgs::Any a; a.set_type(gz::msgs::Any::DOUBLE);
                             a.set_double_value(v); return a; };
  auto anyS = [](const std::string &v) { gz::msgs::Any a;
                 a.set_type(gz::msgs::Any::STRING); a.set_string_value(v);
                 return a; };
  auto anyI = [](int v) { gz::msgs::Any a; a.set_type(gz::msgs::Any::INT32);
                          a.set_int_value(v); return a; };
  m["gain"] = anyD(0.7);
  m["spectrum"] = anyS("pms");
  m["grid_size"] = anyI(64);
  m["sea_state"] = anyI(3);
  m["filter_invert"] = anyD(1.0);
  m["number"] = anyI(5);
  m["unknown_key"] = anyS("ignored");

  gz::msgs::Boolean rep;
  bool ok = false;
  const bool got = node.Request(
    "/world/test_world/wave/set_parameters", req, 5000, rep, ok);
  ASSERT_TRUE(got);
  ASSERT_TRUE(ok);
  EXPECT_TRUE(rep.data());  // at least one key recognised

  // Queued only — not applied until the ECM thread (PreUpdate) drains it.
  EXPECT_EQ(this->Comp()->Data().generation, 1u);
  sys.PreUpdate(this->Info(1.0), this->ecm);

  const auto &d = this->Comp()->Data();
  EXPECT_EQ(d.generation, 2u);
  EXPECT_DOUBLE_EQ(d.params.gain, 0.7);
  EXPECT_EQ(d.params.spectrum, "pms");
  EXPECT_EQ(d.params.gridSize, 64u);
  EXPECT_EQ(d.params.seaState, 3);
  EXPECT_TRUE(d.params.filterInvert);
}

//////////////////////////////////////////////////
TEST_F(WavesSystemTest, SetParametersServiceRejectsUnknownOnly)
{
  StubWavesSystem sys;
  sys.Configure(this->world,
    this->Plugin("<update_rate>40</update_rate>"), this->ecm, this->evm);

  gz::transport::Node node;
  gz::msgs::Param req;
  gz::msgs::Any a;
  a.set_type(gz::msgs::Any::STRING);
  a.set_string_value("x");
  (*req.mutable_params())["bogus"] = a;

  gz::msgs::Boolean rep;
  bool ok = false;
  ASSERT_TRUE(node.Request(
    "/world/test_world/wave/set_parameters", req, 5000, rep, ok));
  EXPECT_TRUE(ok);            // the service call itself succeeded
  EXPECT_FALSE(rep.data());   // but no recognised parameter keys
}

//////////////////////////////////////////////////
TEST_F(WavesSystemTest, SetParametersServiceCoercesTypesAndWarns)
{
  StubWavesSystem sys;
  sys.Configure(this->world,
    this->Plugin("<update_rate>40</update_rate>"), this->ecm, this->evm);

  gz::transport::Node node;
  gz::msgs::Param req;
  auto &m = *req.mutable_params();
  auto any = [](gz::msgs::Any::ValueType _t) { gz::msgs::Any a; a.set_type(_t);
                                               return a; };
  { auto a = any(gz::msgs::Any::INT32);   a.set_int_value(7);
    m["period"] = a; }         // double key fed an int  -> ReadDouble INT32
  { auto a = any(gz::msgs::Any::BOOLEAN); a.set_bool_value(true);
    m["filter_invert"] = a; }  // double key fed a bool  -> ReadDouble BOOLEAN
  { auto a = any(gz::msgs::Any::DOUBLE);  a.set_double_value(128.0);
    m["grid_size"] = a; }      // int key fed a double   -> ReadInt DOUBLE
  { auto a = any(gz::msgs::Any::STRING);  a.set_string_value("x");
    m["tau"] = a; }            // double key, non-numeric -> warn + skip
  { auto a = any(gz::msgs::Any::STRING);  a.set_string_value("x");
    m["seed"] = a; }           // int key, non-numeric    -> warn + skip
  { auto a = any(gz::msgs::Any::DOUBLE);  a.set_double_value(1.0);
    m["spectrum"] = a; }       // string key, non-string  -> warn + skip

  gz::msgs::Boolean rep;
  bool ok = false;
  ASSERT_TRUE(node.Request(
    "/world/test_world/wave/set_parameters", req, 5000, rep, ok));
  EXPECT_TRUE(rep.data());  // period / filter_invert / grid_size were recognised
  sys.PreUpdate(this->Info(1.0), this->ecm);

  const auto &d = this->Comp()->Data().params;
  EXPECT_DOUBLE_EQ(d.period, 7.0);     // int coerced to double
  EXPECT_TRUE(d.filterInvert);         // bool coerced to double != 0
  EXPECT_EQ(d.gridSize, 128u);         // double truncated to int
}

//////////////////////////////////////////////////
TEST_F(WavesSystemTest, ConfigureAbortsWithoutWorldEntity)
{
  gz::sim::EntityComponentManager bareEcm;  // no world entity
  gz::sim::EventManager bareEvm;
  StubWavesSystem sys;
  // worldEntity() returns kNullEntity -> Configure logs and returns early.
  sys.Configure(gz::sim::kNullEntity,
    this->Plugin("<update_rate>40</update_rate>"), bareEcm, bareEvm);
  SUCCEED();  // the contract is: no crash, nothing created
}

//////////////////////////////////////////////////
TEST_F(WavesSystemTest, ConfigureAbortsWhenEngineCreationFails)
{
  /// \brief A source system whose engine factory fails (returns nullptr).
  class NullEngineSystem : public gz::sim::systems::WavesSystemBase
  {
    protected: std::string EngineToken() const override { return "null"; }
    protected: std::shared_ptr<gsw::IWaveField> MakeEngine(
      const gsw::WaveParameters &) const override { return nullptr; }
  };
  NullEngineSystem sys;
  sys.Configure(this->world,
    this->Plugin("<update_rate>40</update_rate>"), this->ecm, this->evm);
  EXPECT_EQ(this->Comp(), nullptr);  // engine null -> no component written
}

//////////////////////////////////////////////////
TEST_F(WavesSystemTest, PreUpdateBeforeConfigureIsNoop)
{
  StubWavesSystem sys;  // never Configured -> no engine
  sys.PreUpdate(this->Info(1.0), this->ecm);  // early return on null engine
  EXPECT_EQ(this->Comp(), nullptr);
}
