/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the usv_gazebo_dynamics_plugin package,
known as this Package.

This Package free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This Package s distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this package.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

#include <cmath>
#include <functional>
#include <sstream>

#include <gazebo/math/Pose.hh>
#include "usv_gazebo_plugins/usv_gazebo_dynamics_plugin.hh"

#define GRAVITY 9.815

using namespace gazebo;

//////////////////////////////////////////////////
UsvDynamicsPlugin::UsvDynamicsPlugin()
{
}

//////////////////////////////////////////////////
double UsvDynamicsPlugin::SdfParamDouble(sdf::ElementPtr _sdfPtr,
  const std::string &_paramName, const double _defaultVal) const
{
  if (!_sdfPtr->HasElement(_paramName))
  {
    ROS_INFO_STREAM("Parameter <" << _paramName << "> not found: "
                    "Using default value of <" << _defaultVal << ">.");
    return _defaultVal;
  }

  double val = _sdfPtr->Get<double>(_paramName);
  ROS_DEBUG_STREAM("Parameter found - setting <" << _paramName <<
                  "> to <" << val << ">.");
  return val;
}

//////////////////////////////////////////////////
void UsvDynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_DEBUG("Loading usv_gazebo_dynamics_plugin");
  this->world = _model->GetWorld();

  // Get parameters from SDF
  std::string linkName;
  if (!_sdf->HasElement("bodyName") ||
      !_sdf->GetElement("bodyName")->GetValue())
  {
    this->link = _model->GetLink();
    linkName = this->link->GetName();
    ROS_INFO_STREAM("Did not find SDF parameter bodyName");
  }
  else
  {
    linkName = _sdf->GetElement("bodyName")->Get<std::string>();
    this->link = _model->GetLink(linkName);

    ROS_DEBUG_STREAM("Found SDF parameter bodyName as <" << linkName<< ">");
  }
  if (!this->link)
  {
    ROS_FATAL("usv_gazebo_dynamics_plugin error: bodyName: %s does not exist\n",
      linkName.c_str());
    return;
  }
  else
  {
    ROS_DEBUG_STREAM("USV Dynamics Model Link Name = " << linkName);
  }

  this->waterLevel       = this->SdfParamDouble(_sdf, "waterLevel"  , 0.5);
  double waterDensity    = this->SdfParamDouble(_sdf, "waterDensity", 997.7735);
  this->paramXdotU       = this->SdfParamDouble(_sdf, "xDotU"       , 5);
  this->paramYdotV       = this->SdfParamDouble(_sdf, "yDotV"       , 5);
  this->paramNdotR       = this->SdfParamDouble(_sdf, "nDotR"       , 1);
  this->paramXu          = this->SdfParamDouble(_sdf, "xU"          , 20);
  this->paramXuu         = this->SdfParamDouble(_sdf, "xUU"         , 0);
  this->paramYv          = this->SdfParamDouble(_sdf, "yV"          , 20);
  this->paramYvv         = this->SdfParamDouble(_sdf, "yVV"         , 0);
  this->paramZw          = this->SdfParamDouble(_sdf, "zW"          , 20);
  this->paramKp          = this->SdfParamDouble(_sdf, "kP"          , 20);
  this->paramMq          = this->SdfParamDouble(_sdf, "mQ"          , 20);
  this->paramNr          = this->SdfParamDouble(_sdf, "nR"          , 20);
  this->paramNrr         = this->SdfParamDouble(_sdf, "nRR"         , 0);
  double paramBoatArea   = this->SdfParamDouble(_sdf, "boatArea"    , 0.48);
  double paramBoatWidth  = this->SdfParamDouble(_sdf, "boatWidth"   , 1.0);
  double paramBoatLength = this->SdfParamDouble(_sdf, "boatLength"  , 1.35);

  // Wave parameters
  std::ostringstream buf;
  std::vector<float> tmpv(2, 0);
  this->paramWaveN = _sdf->GetElement("wave_n")->Get<int>();
  for (int i = 0; i < this->paramWaveN; ++i)
  {
    buf.str("");
    buf << "wave_amp" << i;
    this->paramWaveAmps.push_back(_sdf->GetElement(buf.str())->Get<float>());
    ROS_DEBUG_STREAM("Wave Amplitude " << i << ": " << this->paramWaveAmps[i]);
    buf.str("");
    buf << "wave_period" << i;
    this->paramWavePeriods.push_back(_sdf->GetElement(buf.str())->Get<float>());
    buf.str("");
    buf << "wave_direction" << i;
    math::Vector2d tmpm = _sdf->GetElement(buf.str())->Get<math::Vector2d>();
    tmpv[0] = tmpm.x;
    tmpv[1] = tmpm.y;
    this->paramWaveDirections.push_back(tmpv);
    ROS_DEBUG_STREAM("Wave Direction " << i << ": " <<
      this->paramWaveDirections[i][0] << ", " <<
      this->paramWaveDirections[i][1]);
  }

  // Get inertia and mass of vessel
  const math::Vector3 kInertia =
    this->link->GetInertial()->GetPrincipalMoments();
  const double kMass = this->link->GetInertial()->GetMass();

  // Report some of the pertinent parameters for verification
  ROS_DEBUG("USV Dynamics Parameters: From URDF XACRO model definition");
  ROS_DEBUG_STREAM("Vessel Mass (rigid-body): " << kMass);
  ROS_DEBUG_STREAM("Vessel Inertia Vector (rigid-body): X:" << kInertia[0] <<
                  " Y:" << kInertia[1] << " Z:" << kInertia[2]);

  // Initialize time and odometry position
  this->prevUpdateTime = this->world->GetSimTime();

  // Listen to the update event broadcastes every physics iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvDynamicsPlugin::Update, this));

  // Initialize Added Mass Matrix
  this->Ma = Eigen::MatrixXd(6, 6);
  this->Ma <<
    this->paramXdotU, 0,                0,   0,   0,   0,
    0,                this->paramYdotV, 0,   0,   0,   0,
    0,                0,                0.1, 0,   0,   0,
    0,                0,                0,   0.1, 0,   0,
    0,                0,                0,   0,   0.1, 0,
    0,                0,                0,   0,   0,   this->paramNdotR;

  // must be factor of 2! - only 2 for now!!
  const int kNN = 2;

  // x,y grid step increments
  this->dx = paramBoatLength / kNN;
  this->dy = paramBoatWidth / kNN;

  // Vector for interating through grid points on boat
  for (int i = -kNN / 2; i < 0; ++i)
    this->II.push_back(i);

  for (int i = 1; i <= kNN / 2; ++i)
    this->II.push_back(i);

  // Precalculate this to save some time.
  this->buoyFrac = (paramBoatArea / (kNN * kNN)) * GRAVITY * waterDensity;
}

//////////////////////////////////////////////////
void UsvDynamicsPlugin::Update()
{
  const common::Time kTimeNow = this->world->GetSimTime();
  double dt = (kTimeNow - this->prevUpdateTime).Double();
  this->prevUpdateTime = kTimeNow;

  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  const math::Pose kPose = this->link->GetWorldPose();
  const math::Vector3 kEuler = kPose.rot.GetAsEuler();

  // Get body-centered linear and angular rates
  const math::Vector3 kVelLinearBody = this->link->GetRelativeLinearVel();
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Vel linear: " << kVelLinearBody);
  const math::Vector3 kVelAngularBody = this->link->GetRelativeAngularVel();
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Vel angular: " << kVelAngularBody);

  // Estimate the linear and angular accelerations.
  // Note the the GetRelativeLinearAccel() and AngularAccel() functions
  // appear to be unreliable
  const math::Vector3 kAccelLinearBody =
    (kVelLinearBody - this->prevLinVel) / dt;
  this->prevLinVel = kVelLinearBody;
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Accel linear: " << kAccelLinearBody);
  const math::Vector3 kAccelAngularBody =
    (kVelAngularBody - this->prevAngVel) / dt;
  this->prevAngVel = kVelAngularBody;
  ROS_DEBUG_STREAM_THROTTLE(0.5, "Accel angular: " << kAccelAngularBody);

  // Create state and derivative of state (accelerations)
  Eigen::VectorXd stateDot = Eigen::VectorXd(6);
  Eigen::VectorXd state    = Eigen::VectorXd(6);
  Eigen::MatrixXd Cmat     = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Dmat     = Eigen::MatrixXd::Zero(6, 6);

  stateDot << kAccelLinearBody.x, kAccelLinearBody.y, kAccelLinearBody.z,
    kAccelAngularBody.x, kAccelAngularBody.y, kAccelAngularBody.z;

  state << kVelLinearBody.x, kVelLinearBody.y, kVelLinearBody.z,
    kVelAngularBody.x, kVelAngularBody.y, kVelAngularBody.z;

  // Added Mass
  const Eigen::VectorXd kAmassVec = -1.0 * this->Ma * stateDot;
  ROS_DEBUG_STREAM_THROTTLE(1.0, "stateDot: \n" << stateDot);
  ROS_DEBUG_STREAM_THROTTLE(1.0, "amassVec :\n" << kAmassVec);

  // Coriolis - added mass components
  Cmat(0, 5) = this->paramYdotV * kVelLinearBody.y;
  Cmat(1, 5) = this->paramXdotU * kVelLinearBody.x;
  Cmat(5, 0) = this->paramYdotV * kVelLinearBody.y;
  Cmat(5, 1) = this->paramXdotU * kVelLinearBody.x;

  // Drag
  Dmat(0, 0) = this->paramXu + this->paramXuu * std::abs(kVelLinearBody.x);
  Dmat(1, 1) = this->paramYv + this->paramYvv * std::abs(kVelLinearBody.y);
  Dmat(2, 2) = this->paramZw;
  Dmat(3, 3) = this->paramKp;
  Dmat(4, 4) = this->paramMq;
  Dmat(5, 5) = this->paramNr + this->paramNrr * std::abs(kVelAngularBody.z);
  ROS_DEBUG_STREAM_THROTTLE(1.0, "Dmat :\n" << Dmat);
  const Eigen::VectorXd kDvec = -1.0 * Dmat * state;
  ROS_DEBUG_STREAM_THROTTLE(1.0, "Dvec :\n" << kDvec);

  // Vehicle frame transform
  tf2::Quaternion vq = tf2::Quaternion();
  tf2::Matrix3x3 m;
  m.setEulerYPR(kEuler.z, kEuler.y, kEuler.x);
  m.getRotation(vq);
  tf2::Transform xformV = tf2::Transform(vq);

  // Sum all forces - in body frame
  const Eigen::VectorXd kForceSum = kAmassVec + kDvec;

  // Forces in fixed frame
  ROS_DEBUG_STREAM_THROTTLE(1.0, "forceSum :\n" << kForceSum);

  // Add dynamic forces/torques to link at CG
  this->link->AddRelativeForce(
    math::Vector3(kForceSum(0), kForceSum(1), kForceSum(2)));
  this->link->AddRelativeTorque(
    math::Vector3(kForceSum(3), kForceSum(4), kForceSum(5)));

  // Loop over boat grid points
  for (const int i : this->II)
  {
    // grid points on boat
    tf2::Vector3 bpnt(0, 0, 0);

    // in world coordinates
    tf2::Vector3 bpntW(0, 0, 0);

    // grid point in boat fram
    bpnt.setX(i * this->dx);
    for (const int j : this->II)
    {
      bpnt.setY(j * this->dy);

      // Transform from vessel to water/world frame
      bpntW = xformV * bpnt;

      // Debug
      ROS_DEBUG_STREAM_THROTTLE(1.0, "[" << i << "," << j <<
          "] grid points" << bpnt.x() << "," << bpnt.y() << "," << bpnt.z());
      ROS_DEBUG_STREAM_THROTTLE(1.0, "v frame euler " << kEuler);
      ROS_DEBUG_STREAM_THROTTLE(1.0, "in water frame" << bpntW.x() << "," <<
          bpntW.y() << "," << bpntW.z());

      // Vertical location of boat grid point in world frame
      const float kDdz = kPose.pos.z + bpntW.z();
      ROS_DEBUG_STREAM("Z, pose: " << kPose.pos.z << ", bpnt: " << bpntW.z() <<
        ", dd: " << kDdz);

      // Find vertical displacement of wave field
      // World location of grid point
      math::Vector3 X;
      X.x = kPose.pos.x + bpntW.x();
      X.y = kPose.pos.y + bpntW.y();

      // sum vertical dsplacement over all waves
      double dz = 0.0;
      for (int k = 0; k < this->paramWaveN; ++k)
      {
        const double kDdotx = this->paramWaveDirections[k][0] * X.x +
          this->paramWaveDirections[k][1] * X.y;
        const double kW = 2.0 * M_PI / this->paramWavePeriods[k];
        const double kK = kW * kW / GRAVITY;
        dz += this->paramWaveAmps[k] * cos(kK * kDdotx - kW * kTimeNow.Float());
      }
      ROS_DEBUG_STREAM_THROTTLE(1.0, "wave disp: " << dz);

      // Buoyancy force at grid point
      const float kBuoyForce =
        ((this->waterLevel + dz) - kDdz) * this->buoyFrac;
      ROS_DEBUG_STREAM("buoyForce: " << kBuoyForce);

      // Apply force at grid point
      // From web, Appears that position is in the link frame
      // and force is in world frame
      this->link->AddForceAtRelativePosition(math::Vector3(0, 0, kBuoyForce),
        math::Vector3(bpnt.x(), bpnt.y(), bpnt.z()));
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(UsvDynamicsPlugin);
