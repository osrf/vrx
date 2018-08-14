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

#include <ros/time.h>

#include <functional>

#include "usv_gazebo_plugins/usv_gazebo_thrust_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
Thruster::Thruster(UsvThrust *_parent)
{
  // Set some defaults
  this->cmdTopic = "thruster_default_cmdTopic";
  this->maxCmd = 1.0;
  this->maxForceFwd = 100.0;
  this->maxForceRev = -100.0;
  this->mappingType = 0;
  this->plugin = _parent;

  // Initialize some things
  this->currCmd = 0.0;
  this->lastCmdTime = this->plugin->world->GetSimTime();
}

//////////////////////////////////////////////////
void Thruster::OnThrustCmd(const std_msgs::Float32::ConstPtr &_msg)
{
  // When we get a new thrust command!
  ROS_DEBUG_STREAM("New thrust command! " << _msg->data);
  std::lock_guard<std::mutex> lock(this->plugin->mutex);
  this->lastCmdTime = this->plugin->world->GetSimTime();
  this->currCmd = _msg->data;
}

//////////////////////////////////////////////////
double UsvThrust::SdfParamDouble(sdf::ElementPtr _sdfPtr,
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
void UsvThrust::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  ROS_DEBUG("Loading usv_gazebo_thrust_plugin");
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Get parameters from SDF
  std::string nodeNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
  }

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);

  ROS_DEBUG_STREAM("Loading thrusters from SDF");

  // For each thruster
  int thrusterCounter = 0;
  if (_sdf->HasElement("thruster"))
  {
    sdf::ElementPtr thrusterSDF = _sdf->GetElement("thruster");
    while (thrusterSDF)
    {
      // Instatiate
      Thruster thruster(this);

      ROS_DEBUG_STREAM("Thruster #" << thrusterCounter);

      // Find link by name in SDF
      if (thrusterSDF->HasElement("linkName"))
      {
        std::string linkName = thrusterSDF->Get<std::string>("linkName");
        thruster.link = this->model->GetLink(linkName);
        if (thruster.link == nullptr)
        {
          ROS_ERROR_STREAM("Could not find a link by the name <" << linkName
            << "> in the model!");
        }
        else
        {
          ROS_DEBUG_STREAM("Thruster added to link <" << linkName << ">");
        }
      }
      else
      {
        ROS_ERROR_STREAM("Please specify a link name for each thruster!");
      }

      // Parse out propellor joint name
      if (thrusterSDF->HasElement("propJointName"))
      {
        std::string propName =
          thrusterSDF->GetElement("propJointName")->Get<std::string>();
        thruster.propJoint = this->model->GetJoint(propName);
        if (thruster.propJoint == nullptr)
        {
          ROS_WARN_STREAM("Could not find a propellor joint by the name of <" <<
            propName << "> in the model!");
        }
        else
        {
          ROS_DEBUG_STREAM("Propellor joint <" << propName <<
            "> added to thruster");
        }
      }
      else
      {
        ROS_WARN_STREAM("No propJointName SDF parameter for thruster #"
          << thrusterCounter);
      }

      // Parse individual thruster SDF parameters
      thruster.maxCmd = this->SdfParamDouble(thrusterSDF, "maxCmd", 1.0);
      thruster.maxForceFwd =
        this->SdfParamDouble(thrusterSDF, "maxForceFwd", 250.0);
      thruster.maxForceRev =
        this->SdfParamDouble(thrusterSDF, "maxForceRev", -100.0);
      if (thrusterSDF->HasElement("mappingType"))
      {
        thruster.mappingType = thrusterSDF->Get<int>("mappingType");
        ROS_DEBUG_STREAM("Parameter found - setting <mappingType> to <" <<
          thruster.mappingType << ">.");
      }
      else
      {
        thruster.mappingType = 0;
        ROS_INFO_STREAM("Parameter <mappingType> not found: "
          "Using default value of <" << thruster.mappingType << ">.");
      }

      // Parse for subscription topic
      if (thrusterSDF->HasElement("cmdTopic"))
      {
        thruster.cmdTopic = thrusterSDF->Get<std::string>("cmdTopic");
      }
      else
      {
        ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) "
          "for each thruster!");
      }

      // Push to vector and increment
      this->thrusters.push_back(thruster);
      thrusterSDF = thrusterSDF->GetNextElement("thruster");
      thrusterCounter++;
    }
  }
  else
  {
    ROS_WARN_STREAM("No 'thruster' tags in description - how will you move?");
  }
  ROS_DEBUG_STREAM("Found " << thrusterCounter << " thrusters");

  // Initialize the ROS node and subscribe to cmd_drive
  this->rosnode.reset(new ros::NodeHandle(nodeNamespace));

  // Advertise joint state publisher to view propellers in rviz
  // TODO: consider throttling joint_state pub for performance
  // (every OnUpdate may be too frequent).
  this->jointStatePub =
    this->rosnode->advertise<sensor_msgs::JointState>("joint_states", 1);
  this->jointStateMsg.name.resize(thrusters.size());
  this->jointStateMsg.position.resize(thrusters.size());
  this->jointStateMsg.velocity.resize(thrusters.size());
  this->jointStateMsg.effort.resize(thrusters.size());

  for (size_t i = 0; i < this->thrusters.size(); ++i)
  {
    // Prefill the joint state message with the propeller joint.
    this->jointStateMsg.name[i] = this->thrusters[i].propJoint->GetName();

    // Subscribe to commands for each thruster.
    this->thrusters[i].cmdSub = this->rosnode->subscribe(
      this->thrusters[i].cmdTopic, 1, &Thruster::OnThrustCmd,
      &this->thrusters[i]);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvThrust::Update, this));
}

//////////////////////////////////////////////////
double UsvThrust::ScaleThrustCmd(const double _cmd, const double _maxCmd,
  const double _maxPos, const double _maxNeg) const
{
  double val = 0.0;
  if (_cmd >= 0.0)
  {
    val = _cmd / _maxCmd * _maxPos;
    val = std::min(val, _maxPos);
  }
  else
  {
    double absMaxNeg = std::abs(_maxNeg);
    val = _cmd / _maxCmd * absMaxNeg;
    val = std::max(val, -1.0 * absMaxNeg);
  }
  return val;
}

//////////////////////////////////////////////////
double UsvThrust::Glf(const double _x, const float _A, const float _K,
    const float _B, const float _v, const float _C, const float _M) const
{
  return _A + (_K - _A) / (pow(_C + exp(-_B * (_x - _M)), 1.0 / _v));
}

//////////////////////////////////////////////////
double UsvThrust::GlfThrustCmd(const double _cmd,
                               const double _maxPos,
                               const double _maxNeg) const
{
  double val = 0.0;
  if (_cmd > 0.01)
  {
    val = this->Glf(_cmd, 0.01f, 59.82f, 5.0f, 0.38f, 0.56f, 0.28f);
    val = std::min(val, _maxPos);
  }
  else if (_cmd < 0.01)
  {
    val = this->Glf(_cmd, -199.13f, -0.09f, 8.84f, 5.34f, 0.99f, -0.57f);
    val = std::max(val, _maxNeg);
  }
  return val;
}

//////////////////////////////////////////////////
void UsvThrust::Update()
{
  common::Time now = this->world->GetSimTime();

  for (size_t i = 0; i < this->thrusters.size(); ++i)
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      // Enforce command timeout
      double dtc = (now - this->thrusters[i].lastCmdTime).Double();
      if (dtc > this->cmdTimeout && this->cmdTimeout > 0.0)
      {
        this->thrusters[i].currCmd = 0.0;
        ROS_DEBUG_STREAM_THROTTLE(1.0, "[" << i << "] Cmd Timeout");
      }

      // Apply the thrust mapping
      math::Vector3 tforcev(0, 0, 0);
      switch (this->thrusters[i].mappingType)
      {
        case 0:
          tforcev.x = this->ScaleThrustCmd(this->thrusters[i].currCmd,
                                           this->thrusters[i].maxCmd,
                                           this->thrusters[i].maxForceFwd,
                                           this->thrusters[i].maxForceRev);
          break;
        case 1:
          tforcev.x = this->GlfThrustCmd(this->thrusters[i].currCmd,
                                         this->thrusters[i].maxForceFwd,
                                         this->thrusters[i].maxForceRev);
          break;
        default:
            ROS_FATAL_STREAM("Cannot use mappingType=" <<
              this->thrusters[i].mappingType);
            break;
      }

      // Apply force for each thruster
      this->thrusters[i].link->AddLinkForce(tforcev);

      // Spin the propellers
      this->SpinPropeller(this->thrusters[i].propJoint,
        this->thrusters[i].currCmd);
    }
  }

  // Publish the propeller joint state
  this->jointStateMsg.header.stamp = ros::Time::now();
  this->jointStatePub.publish(this->jointStateMsg);
}

//////////////////////////////////////////////////
void UsvThrust::SpinPropeller(physics::JointPtr &_propeller,
    const double _input)
{
  if (!_propeller)
    return;

  const double kMinInput = 0.1;
  const double kMaxInput = 1.0;
  const double kMaxEffort = 2.0;
  double effort = 0.0;

  if (std::abs(_input) > kMinInput)
    effort = (_input / kMaxInput) * kMaxEffort;

  _propeller->SetForce(0, effort);

  // Get index in joint state message for this propeller
  uint8_t index = -1;
  for (int i = 0; i < this->thrusters.size(); ++i)
  {
    if (_propeller->GetName() == this->jointStateMsg.name[i])
    {
      index = i;
      break;
    }
  }
  if (index < 0)
  {
    ROS_WARN("Propeller %s cannot be associated with a joint, "
             "skipping message.", _propeller->GetName().c_str());
    return;
  }

  // Set position, velocity, and effort of joint from gazebo
  gazebo::math::Angle position = _propeller->GetAngle(0);
  position.Normalize();
  this->jointStateMsg.position[index] = position.Radian();
  this->jointStateMsg.velocity[index] = _propeller->GetVelocity(0);
  this->jointStateMsg.effort[index] = effort;
}

GZ_REGISTER_MODEL_PLUGIN(UsvThrust);
