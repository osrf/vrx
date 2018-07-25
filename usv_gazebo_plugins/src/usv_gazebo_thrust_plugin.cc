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

#include "usv_gazebo_plugins/usv_gazebo_thrust_plugin.hh"

using namespace gazebo;

Thruster::Thruster()
{
}
void Thruster::OnThrustCmd(const std_msgs::Float32::ConstPtr & msg)
{
  // When we get a new thrust command
}

//////////////////////////////////////////////////
UsvThrust::UsvThrust()
{
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

  // Retrieve model parameters from SDF
  // Set default values
  // How long to allow no input on cmd_drive
  this->cmdTimeout         = 1.0;
  this->paramMappingType   = 0;
  this->paramMaxCmd        = 1.0;
  this->paramMaxForceFwd   = 100.0;
  this->paramMaxForceRev   = -100.0;
  this->paramBoatWidth     = 1.0;
  this->paramBoatLength    = 1.35;
  this->paramThrustZoffset = -0.01;

  // Get parameters from SDF
  std::string nodeNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
  }

  ROS_INFO_STREAM("Loading thrusters from SDF");

  // Temporary storage
  std::string linkName;

  // For each thruster
  int tcnt = 0;
  if (_sdf->HasElement("thruster"))
  {
    sdf::ElementPtr thrusterSDF = _sdf->GetElement("thruster");
    while (thrusterSDF){
      // Instatiate
      Thruster thruster;  

      // Find link by name in SDF
      if (thrusterSDF->HasElement("linkName")){
	linkName = thrusterSDF->Get<std::string>("linkName");
	thruster.link = this->model->GetLink(linkName);
	if (thruster.link == nullptr){
	  ROS_ERROR_STREAM("Could not find a link by the name <" << linkName <<"> in the model!");
	}
      }
      else{
	ROS_ERROR_STREAM("Please specify a link name for each thruster!");
      }

      // Parse individual thruster SDF parameters
      thruster.maxCmd = this->SdfParamDouble(thrusterSDF, "maxCmd",1.0);
      thruster.maxForceFwd = this->SdfParamDouble(thrusterSDF, "maxForceFwd",250.0);
      thruster.maxForceRev = this->SdfParamDouble(thrusterSDF, "maxForceRev",-100.0);
      if (thrusterSDF->HasElement("mappingType")){
	thruster.mappingType = thrusterSDF->Get<int>("mappingType");
	ROS_INFO_STREAM("Parameter found - setting <mappingType> to <" <<
			thruster.mappingType << ">.");
      }
      else
	{
	  thruster.mappingType = 0;
	  ROS_INFO_STREAM("Parameter <mappingType> not found: Using default value of "
			  "<" << thruster.mappingType << ">.");
	}

      // Parse for subscription topic 
      if (thrusterSDF->HasElement("cmdTopic")){
	thruster.cmdTopic = thrusterSDF->Get<std::string>("cmdTopic");
      }
      else{
	ROS_ERROR_STREAM("Please specify a cmdTopic (for ROS subscription) for each thruster!");
      }
      
      // Push to vector and increment
      this->thrusters.push_back(thruster);
      thrusterSDF = thrusterSDF->GetNextElement("thruster");
      tcnt++;
      
    } // end of while
  }
  else
  {
    ROS_WARN_STREAM("No 'thruster' tags in description - how will you move?");
  }
  ROS_INFO_STREAM("Found " << tcnt << " thrusters");
  
      

  
  if (_sdf->HasElement("bodyName"))
  {
    linkName = _sdf->Get<std::string>("bodyName");
    this->link = this->model->GetLink(linkName);
    ROS_DEBUG_STREAM("Found SDF parameter bodyName as <" << linkName << ">");
  }
  else
  {
    this->link = this->model->GetLink();
    linkName = this->link->GetName();
    ROS_INFO_STREAM("Did not find SDF parameter bodyName");
  }

  if (!this->link)
  {
    ROS_FATAL("usv_gazebo_thrust_plugin error: bodyName: %s does not exist\n",
      linkName.c_str());
    return;
  }

  ROS_DEBUG_STREAM("USV Model Link Name = " << linkName);

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", this->cmdTimeout);
  if (_sdf->HasElement("mappingType"))
  {
    this->paramMappingType = _sdf->Get<int>("mappingType");
    ROS_DEBUG_STREAM("Parameter found - setting <mappingType> to <" <<
                    this->paramMappingType << ">.");
  }
  else
  {
    ROS_INFO_STREAM("Parameter <mappingType> not found: Using default value of "
                    "<" << this->paramMappingType << ">.");
  }
  // Verify
  if (this->paramMappingType > 1 || this->paramMappingType < 0)
  {
    ROS_FATAL_STREAM("Cannot use a mappingType of " << this->paramMappingType);
    this->paramMappingType = 0;
  }

  this->paramMaxCmd = this->SdfParamDouble(_sdf, "maxCmd", this->paramMaxCmd);
  this->paramMaxForceFwd = this->SdfParamDouble(_sdf, "maxForceFwd",
      this->paramMaxForceFwd);
  this->paramMaxForceRev = this->SdfParamDouble(_sdf, "maxForceRev",
      this->paramMaxForceRev);
  // make negative
  this->paramMaxForceRev = -1.0 * std::abs(this->paramMaxForceRev);
  this->paramBoatWidth = this->SdfParamDouble(_sdf, "boatWidth",
      this->paramBoatWidth);
  this->paramBoatLength = this->SdfParamDouble(_sdf, "boatLength",
      this->paramBoatLength);
  this->paramThrustZoffset = this->SdfParamDouble(_sdf, "thrustOffsetZ",
      this->paramThrustZoffset);

  // Get the names of the propeller joints.
  this->ParsePropeller(
    _sdf, "left_propeller_joint", this->leftPropellerJoint);
  this->ParsePropeller(
    _sdf, "right_propeller_joint", this->rightPropellerJoint);

  // Initialize time and odometry position
  this->lastCmdDriveTime = this->world->GetSimTime();

  // Initialize the ROS node and subscribe to cmd_drive
  this->rosnode.reset(new ros::NodeHandle(nodeNamespace));

  // Advertise joint state publisher to view propellers in rviz
  // TODO: consider throttling joint_state pub for performance
  // (every OnUpdate may be too frequent).
  this->jointStatePub =
    this->rosnode->advertise<sensor_msgs::JointState>("joint_states", 1);
  this->jointStateMsg.name.resize(2);
  this->jointStateMsg.position.resize(2);
  this->jointStateMsg.velocity.resize(2);
  this->jointStateMsg.effort.resize(2);
  this->jointStateMsg.name[0] = this->leftPropellerJoint->GetName();
  this->jointStateMsg.name[1] = this->rightPropellerJoint->GetName();

  this->cmdDriveSub = this->rosnode->subscribe("cmd_drive", 1,
      &UsvThrust::OnCmdDrive, this);

  // Subscribe to commands for each thruster
  for (size_t i = 0; i < this->thrusters.size(); ++i){
    this->thrusters[i].cmdSub = this->rosnode->subscribe(this->thrusters[i].cmdTopic,1,&Thruster::OnThrustCmd,&this->thrusters[i]);
    //this->thrusters[i].cmdSub = this->rosnode->subscribe(this->thrusters[i].cmdTopic,1,&UsvThrust::OnCmdDrive, this);
  }
  
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvThrust::Update, this));
}

//////////////////////////////////////////////////
double UsvThrust::ScaleThrustCmd(const double _cmd) const
{
  double val = 0.0;
  if (_cmd >= 0.0)
  {
    val = _cmd / this->paramMaxCmd * this->paramMaxForceFwd;
    val = std::min(val, this->paramMaxForceFwd);
  }
  else
  {
    val = _cmd / this->paramMaxCmd * std::abs(this->paramMaxForceRev);
    val = std::max(val, this->paramMaxForceRev);
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
double UsvThrust::GlfThrustCmd(const double _cmd) const
{
  double val = 0.0;
  if (_cmd > 0.01)
  {
    val = this->Glf(_cmd, 0.01f, 59.82f, 5.0f, 0.38f, 0.56f, 0.28f);
    val = std::min(val, this->paramMaxForceFwd);
  }
  else if (_cmd < 0.01)
  {
    val = this->Glf(_cmd, -199.13f, -0.09f, 8.84f, 5.34f, 0.99f, -0.57f);
    val = std::max(val, this->paramMaxForceRev);
  }
  else
  {
    val = 0.0;
  }
  ROS_DEBUG_STREAM_THROTTLE(0.5,
      _cmd << ": " << val << " / " << val / this->paramMaxForceFwd);
  return val;
}

//////////////////////////////////////////////////
void UsvThrust::Update()
{
  common::Time time_now = this->world->GetSimTime();

  // Enforce command timeout
  double dcmd = (time_now - this->lastCmdDriveTime).Double();
  if (dcmd > this->cmdTimeout && this->cmdTimeout > 0.0)
  {
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Command timeout!");
    this->lastCmdDriveLeft = 0.0;
    this->lastCmdDriveRight = 0.0;
  }
  // Scale commands to thrust and torque forces
  ROS_DEBUG_STREAM_THROTTLE(1.0, "Last cmd: left:" << this->lastCmdDriveLeft
       << " right: " << this->lastCmdDriveRight);
  double thrust_left = 0.0;
  double thrust_right = 0.0;
  switch (this->paramMappingType)
  {
  // Simplest, linear
  case 0:
    thrust_left = this->ScaleThrustCmd(this->lastCmdDriveLeft);
    thrust_right = this->ScaleThrustCmd(this->lastCmdDriveRight);
    break;
  // GLF
  case 1:
    thrust_left = this->GlfThrustCmd(this->lastCmdDriveLeft);
    thrust_right = this->GlfThrustCmd(this->lastCmdDriveRight);
    break;
  default:
    ROS_FATAL_STREAM("Cannot use mappingType=" << this->paramMappingType);
    break;
  }

  double thrust = thrust_right + thrust_left;
  double torque = (thrust_right - thrust_left) * this->paramBoatWidth;
  ROS_DEBUG_STREAM_THROTTLE(1.0, "Thrust: left:" << thrust_left
      << " right: " << thrust_right);

  // Add torque
  this->link->AddRelativeTorque(math::Vector3(0, 0, torque));

  // Add input force with offset below vessel
  math::Vector3 relpos(-1.0 * this->paramBoatLength / 2.0, 0.0,
      this->paramThrustZoffset);  // relative pos of thrusters
  math::Vector3 inputforce3(thrust, 0, 0);

  // Get Pose
  math::Pose pose = this->link->GetWorldPose();

  inputforce3 = pose.rot.RotateVector(inputforce3);
  this->link->AddForceAtRelativePosition(inputforce3, relpos);

  // Spin the propellers
  this->SpinPropeller(this->leftPropellerJoint, this->lastCmdDriveLeft);
  this->SpinPropeller(this->rightPropellerJoint, this->lastCmdDriveRight);
 
  // Publish the propeller joint state
  this->jointStateMsg.header.stamp = ros::Time::now();
  this->jointStatePub.publish(this->jointStateMsg);
}

//////////////////////////////////////////////////
void UsvThrust::OnCmdDrive(const usv_gazebo_plugins::UsvDriveConstPtr &_msg)
{
  this->lastCmdDriveTime = this->world->GetSimTime();
  this->lastCmdDriveLeft = _msg->left;
  this->lastCmdDriveRight = _msg->right;
}

//////////////////////////////////////////////////
void UsvThrust::ParsePropeller(const sdf::ElementPtr _sdf,
    const std::string &_sdfName, physics::JointPtr &_propellerJoint) const
{
  if (_sdf->HasElement(_sdfName))
  {
    std::string propellerName = _sdf->GetElement(_sdfName)->Get<std::string>();
    _propellerJoint = this->model->GetJoint(propellerName);

    if (!_propellerJoint)
    {
      ROS_WARN_STREAM("Unable to find propeller joint <" <<
         propellerName << ">");
    }
  }
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
  uint8_t index;
  if (_propeller->GetName() == this->jointStateMsg.name[0])
    index = 0;
  else if (_propeller->GetName() == this->jointStateMsg.name[1])
    index = 1;
  else
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
