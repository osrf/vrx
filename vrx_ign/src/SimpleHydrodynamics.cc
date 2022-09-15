/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <string>
#include <Eigen/Eigen>
#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <sdf/sdf.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"

#include "SimpleHydrodynamics.hh"

using namespace gz;
using namespace sim;
using namespace vrx;

class vrx::SimpleHydrodynamicsPrivate
{
  /// \brief The link entity.
  public: sim::Link link;

  /// \brief Model interface.
  public: Model model{kNullEntity};

  /// \brief Added mass in surge, X_\dot{u}.
  public: double paramXdotU{0.0};

  /// \brief Added mass in sway, Y_\dot{v}.
  public: double paramYdotV{0.0};

  /// \brief Added mass in heave, Z_\dot{w}.
  public: double paramZdotW{0.0};

  /// \brief Added mass in roll, K_\dot{p}.
  public: double paramKdotP{0.0};

  /// \brief Added mass in pitch, M_\dot{q}.
  public: double paramMdotQ{0.0};

  /// \brief Added mass in yaw, N_\dot{r}.
  public: double paramNdotR{0.0};

  /// \brief Linear drag in surge.
  public: double paramXu{0.0};

  /// \brief Quadratic drag in surge.
  public: double paramXuu{0.0};

  /// \brief Linear drag in sway.
  public: double paramYv{0.0};

  /// \brief Quadratic drag in sway.
  public: double paramYvv{0.0};

  /// \brief Linear drag in heave.
  public: double paramZw{0.0};

  /// \brief Quadratic drag in heave.
  public: double paramZww{0.0};

  /// \brief Linear drag in roll.
  public: double paramKp{0.0};

  /// \brief Quadratic drag in roll.
  public: double paramKpp{0.0};

  /// \brief Linear drag in pitch.
  public: double paramMq{0.0};

  /// \brief Quadratic drag in pitch.
  public: double paramMqq{0.0};

  /// \brief Linear drag in yaw.
  public: double paramNr{0.0};

  /// \brief Quadratic drag in yaw.
  public: double paramNrr{0.0};

  /// \brief Added mass of vehicle.
  /// See: https://en.wikipedia.org/wiki/Added_mass
  public: Eigen::MatrixXd Ma;
};


//////////////////////////////////////////////////
SimpleHydrodynamics::SimpleHydrodynamics()
  : dataPtr(std::make_unique<SimpleHydrodynamicsPrivate>())
{
}

//////////////////////////////////////////////////
void SimpleHydrodynamics::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  // Parse required elements.
  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "No <link_name> specified" << std::endl;
    return;
  }

  std::string linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->link = Link(this->dataPtr->model.LinkByName(_ecm, linkName));
  if (!this->dataPtr->link.Valid(_ecm))
  {
    gzerr << "Could not find link named [" << linkName
           << "] in model" << std::endl;
    return;
  }

  this->dataPtr->link.EnableVelocityChecks(_ecm);
  this->dataPtr->link.EnableAccelerationChecks(_ecm);

  this->dataPtr->paramXdotU       = _sdf->Get<double>("xDotU", 5  ).first;
  this->dataPtr->paramYdotV       = _sdf->Get<double>("yDotV", 5  ).first;
  this->dataPtr->paramZdotW       = _sdf->Get<double>("zDotW", 0.1).first;
  this->dataPtr->paramKdotP       = _sdf->Get<double>("kDotP", 0.1).first;
  this->dataPtr->paramMdotQ       = _sdf->Get<double>("mDotQ", 0.1).first;
  this->dataPtr->paramNdotR       = _sdf->Get<double>("nDotR", 1  ).first;
  this->dataPtr->paramXu          = _sdf->Get<double>("xU",   20  ).first;
  this->dataPtr->paramXuu         = _sdf->Get<double>("xUU",   0  ).first;
  this->dataPtr->paramYv          = _sdf->Get<double>("yV",   20  ).first;
  this->dataPtr->paramYvv         = _sdf->Get<double>("yVV",   0  ).first;
  this->dataPtr->paramZw          = _sdf->Get<double>("zW",   20  ).first;
  this->dataPtr->paramZww         = _sdf->Get<double>("zWW",   0  ).first;
  this->dataPtr->paramKp          = _sdf->Get<double>("kP",   20  ).first;
  this->dataPtr->paramKpp         = _sdf->Get<double>("kPP",   0  ).first;
  this->dataPtr->paramMq          = _sdf->Get<double>("mQ",   20  ).first;
  this->dataPtr->paramMqq         = _sdf->Get<double>("mQQ",   0  ).first;
  this->dataPtr->paramNr          = _sdf->Get<double>("nR",   20  ).first;
  this->dataPtr->paramNrr         = _sdf->Get<double>("nRR",   0  ).first;

  // Added mass according to Fossen's equations (p 37).
  this->dataPtr->Ma = Eigen::MatrixXd::Zero(6, 6);

  this->dataPtr->Ma(0, 0) = this->dataPtr->paramXdotU;
  this->dataPtr->Ma(1, 1) = this->dataPtr->paramYdotV;
  this->dataPtr->Ma(2, 2) = this->dataPtr->paramZdotW;
  this->dataPtr->Ma(3, 3) = this->dataPtr->paramKdotP;
  this->dataPtr->Ma(4, 4) = this->dataPtr->paramMdotQ;
  this->dataPtr->Ma(5, 5) = this->dataPtr->paramNdotR;

  gzdbg << "SimpleHydrodynamics plugin successfully configured with the "
        << "following parameters:"                        << std::endl;
  gzdbg << "  <link_name>: " << linkName                  << std::endl;
  gzdbg << "  <xDotU>: "     << this->dataPtr->paramXdotU << std::endl;
  gzdbg << "  <yDotV>: "     << this->dataPtr->paramYdotV << std::endl;
  gzdbg << "  <zDotW>: "     << this->dataPtr->paramZdotW << std::endl;
  gzdbg << "  <kDotP>: "     << this->dataPtr->paramKdotP << std::endl;
  gzdbg << "  <mDotQ>: "     << this->dataPtr->paramMdotQ << std::endl;
  gzdbg << "  <nDotR>: "     << this->dataPtr->paramNdotR << std::endl;
  gzdbg << "  <xU>: "        << this->dataPtr->paramXu    << std::endl;
  gzdbg << "  <xUU>: "       << this->dataPtr->paramXuu   << std::endl;
  gzdbg << "  <yV>: "        << this->dataPtr->paramYv    << std::endl;
  gzdbg << "  <yVV>: "       << this->dataPtr->paramYvv   << std::endl;
  gzdbg << "  <zW>: "        << this->dataPtr->paramZw    << std::endl;
  gzdbg << "  <zWW>: "       << this->dataPtr->paramZww   << std::endl;
  gzdbg << "  <kP>: "        << this->dataPtr->paramKp    << std::endl;
  gzdbg << "  <kPP>: "       << this->dataPtr->paramKpp   << std::endl;
  gzdbg << "  <mQ>: "        << this->dataPtr->paramMq    << std::endl;
  gzdbg << "  <mQQ>: "       << this->dataPtr->paramMqq   << std::endl;
  gzdbg << "  <nR>: "        << this->dataPtr->paramNr    << std::endl;
  gzdbg << "  <nRR>: "       << this->dataPtr->paramNrr   << std::endl;
}

//////////////////////////////////////////////////
void SimpleHydrodynamics::PreUpdate(
    const sim::UpdateInfo &_info,
    sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("SimpleHydrodynamics::PreUpdate");

  if (_info.paused)
    return;

  if (!this->dataPtr->link.Valid(_ecm))
    return;

  Eigen::VectorXd stateDot = Eigen::VectorXd(6);
  Eigen::VectorXd state    = Eigen::VectorXd(6);
  Eigen::MatrixXd Cmat     = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Dmat     = Eigen::MatrixXd::Zero(6, 6);

  // Get vehicle state.
  auto worldAngularVel = this->dataPtr->link.WorldAngularVelocity(_ecm);
  auto worldLinearVel = this->dataPtr->link.WorldLinearVelocity(_ecm);
  auto worldAngularAccel = this->dataPtr->link.WorldAngularAcceleration(_ecm);
  auto worldLinearAccel = this->dataPtr->link.WorldLinearAcceleration(_ecm);

  // Sanity check: Make sure that we can read the full state.
  if (!worldAngularVel)
  {
    gzerr << "No angular velocity" <<"\n";
    return;
  }

  if (!worldLinearVel)
  {
    gzerr << "No linear velocity" <<"\n";
    return;
  }

  if (!worldAngularAccel)
  {
    gzerr << "No angular acceleration" <<"\n";
    return;
  }

  if (!worldLinearAccel)
  {
    gzerr << "No linear acceleration" <<"\n";
    return;
  }

  // Transform from world to local frame.
  auto comPose = this->dataPtr->link.WorldInertialPose(_ecm);
  auto localAngularVel   = comPose->Rot().Inverse() * (*worldAngularVel);
  auto localLinearVel    = comPose->Rot().Inverse() * (*worldLinearVel);
  auto localAngularAccel = comPose->Rot().Inverse() * (*worldAngularAccel);
  auto localLinearAccel  = comPose->Rot().Inverse() * (*worldLinearAccel);

  stateDot << localLinearAccel.X(), localLinearAccel.Y(), localLinearAccel.Z(),
   localAngularAccel.X(), localAngularAccel.Y(), localAngularAccel.Z();

  state << localLinearVel.X(), localLinearVel.Y(), localLinearVel.Z(),
    localAngularVel.X(), localAngularVel.Y(), localAngularVel.Z();

  // Added Mass.
  const Eigen::VectorXd kAmassVec = -1.0 * this->dataPtr->Ma * stateDot;

  // Coriolis - added mass components.
  Cmat(0, 5) = this->dataPtr->paramYdotV * localLinearVel.Y();
  Cmat(1, 5) = this->dataPtr->paramXdotU * localLinearVel.X();
  Cmat(5, 0) = this->dataPtr->paramYdotV * localLinearVel.Y();
  Cmat(5, 1) = this->dataPtr->paramXdotU * localLinearVel.X();

  // Drag.
  Dmat(0, 0) = this->dataPtr->paramXu +
    this->dataPtr->paramXuu * std::abs(localLinearVel.X());
  Dmat(1, 1) = this->dataPtr->paramYv +
    this->dataPtr->paramYvv * std::abs(localLinearVel.Y());
  Dmat(2, 2) = this->dataPtr->paramZw +
    this->dataPtr->paramZww * std::abs(localLinearVel.Z());
  Dmat(3, 3) = this->dataPtr->paramKp +
    this->dataPtr->paramKpp * std::abs(localAngularVel.X());
  Dmat(4, 4) = this->dataPtr->paramMq +
    this->dataPtr->paramMqq * std::abs(localAngularVel.Y());
  Dmat(5, 5) = this->dataPtr->paramNr +
    this->dataPtr->paramNrr * std::abs(localAngularVel.Z());

  const Eigen::VectorXd kDvec = -1.0 * Dmat * state;

  // Sum all forces - in body frame.
  const Eigen::VectorXd kForceSum = kAmassVec + kDvec;

  // Transform the force and torque to the world frame.
  math::Vector3d forceWorld = (*comPose).Rot().RotateVector(
    math::Vector3d(kForceSum(0), kForceSum(1), kForceSum(2)));
  math::Vector3d torqueWorld = (*comPose).Rot().RotateVector(
    math::Vector3d(kForceSum(3), kForceSum(4), kForceSum(5)));

  // Apply the force and torque at COM.
  this->dataPtr->link.AddWorldWrench(_ecm, forceWorld, torqueWorld);
}

GZ_ADD_PLUGIN(vrx::SimpleHydrodynamics,
              sim::System,
              SimpleHydrodynamics::ISystemConfigure,
              SimpleHydrodynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::SimpleHydrodynamics,
                    "vrx::SimpleHydrodynamics")
