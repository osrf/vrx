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

#include <usv_gazebo_plugins/usv_gazebo_thrust_plugin.hh>

using namespace gazebo;

//////////////////////////////////////////////////
UsvThrust::UsvThrust()
{
}

//////////////////////////////////////////////////
double UsvThrust::SdfParamDouble(sdf::ElementPtr _sdfPtr,
  const std::string &_paramName, double _defaultVal)
{
  if (!_sdfPtr->HasElement(_paramName))
  {
    ROS_INFO_STREAM("Parameter <" << _paramName << "> not found: "
                    "Using default value of <" << _defaultVal << ">.");
    return _defaultVal;
  }

  double val = _sdfPtr->Get<double>(_paramName);
  ROS_INFO_STREAM("Parameter found - setting <" << _paramName << 
                  "> to <" << val << ">.");
  return val;
}

//////////////////////////////////////////////////
void UsvThrust::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  ROS_INFO("Loading usv_gazebo_thrust_plugin");
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Retrieve model parameters from SDF
  // Set default values
  this->nodeNamespace = "";
  // how long to allow no input on cmd_drive
  this->cmdTimeout = 1.0; 
  this->paramMappingType = 0;
  this->paramMaxCmd = 1.0;
  this->paramMaxForceFwd = 100.0;
  this->paramMaxForceRev = -100.0;
  this->paramBoatWidth = 1.0;
  this->paramBoatLength = 1.35;
  this->paramThrustZoffset = -0.01;

  // Get parameters from SDF
  if (_sdf->HasElement("robotNamespace"))
  {
    this->nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
  }

  if (_sdf->HasElement("bodyName"))
  {
    this->linkName = _sdf->Get<std::string>("bodyName");
    this->link = this->model->GetLink(this->linkName);
    ROS_INFO_STREAM("Found SDF parameter bodyName as <"<<this->linkName<<">");
  }
  else
  {
    this->link = this->model->GetLink();
    this->linkName = this->link->GetName();
    ROS_INFO_STREAM("Did not find SDF parameter bodyName");
  }

  if (!this->link)
  {
    ROS_FATAL("usv_gazebo_thrust_plugin error: bodyName: %s does not exist\n",
      this->linkName.c_str());
    return;
  }

  ROS_INFO_STREAM("USV Model Link Name = " << this->linkName);

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout",this->cmdTimeout);
  if (_sdf->HasElement("mappingType"))
  {
    this->paramMappingType = _sdf->Get<int>("mappingType");
    ROS_INFO_STREAM("Parameter found - setting <mappingType> to <" <<
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
  ParsePropeller(_sdf, "left_propeller_joint", this->leftPropellerJoint);
  ParsePropeller(_sdf, "right_propeller_joint", this->rightPropellerJoint);

  // Initialize time and odometry position
  this->prevUpdateTime = this->lastCmdDriveTime = this->world->GetSimTime();

  // Initialize the ROS node and subscribe to cmd_drive
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "usv_thrust_gazebo",
	    ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  this->rosnode = new ros::NodeHandle( this->nodeNamespace );

  this->cmdDriveSub = this->rosnode->subscribe("cmd_drive", 1, 
      &UsvThrust::OnCmdDrive, this );

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvThrust::UpdateChild, this));
}

//////////////////////////////////////////////////
double UsvThrust::scaleThrustCmd(double cmd)
{
  double val = 0.0;
  if (cmd >= 0.0)
  {
    val = cmd / this->paramMaxCmd * this->paramMaxForceFwd;
    val = std::min(val,this->paramMaxForceFwd);
  }
  else  // cmd is less than zero
  {
    val = -1.0 * std::abs(cmd)/this->paramMaxCmd *
          std::abs(this->paramMaxForceRev);
    val = std::max(val,this->paramMaxForceRev);
  }
  return val;
}

//////////////////////////////////////////////////
double UsvThrust::glf(double x, float A, float K, float B,
		      float v, float C, float M){
  return A + (K-A) / (pow(C + exp(-B * (x-M)), 1.0 / v));
}

//////////////////////////////////////////////////
double UsvThrust::glfThrustCmd(double cmd)
{
  double val = 0.0;
  if (cmd > 0.01)
  {
    val = glf(cmd,0.01, 59.82, 5.0, 0.38, 0.56, 0.28);
    val = std::min(val, this->paramMaxForceFwd);
  }
  else if (cmd < 0.01)
  {
    val = glf(cmd, -199.13, -0.09, 8.84, 5.34, 0.99, -0.57);
    val = std::max(val, this->paramMaxForceRev);
  }
  else
  {
    val = 0.0;
  }
  ROS_INFO_STREAM_THROTTLE(0.5,
      cmd << ": " << val << " / " << val / this->paramMaxForceFwd);
  return val;

}

//////////////////////////////////////////////////
void UsvThrust::UpdateChild()
{
  common::Time time_now = this->world->GetSimTime();
  this->prevUpdateTime = time_now;

  // Enforce command timeout
  double dcmd = (time_now - this->lastCmdDriveTime).Double();
  if (dcmd > this->cmdTimeout && this->cmdTimeout > 0.0)
  {
    ROS_INFO_STREAM_THROTTLE(1.0,"Command timeout!");
    this->lastCmdDriveLeft = 0.0;
    this->lastCmdDriveRight = 0.0;
  }
  // Scale commands to thrust and torque forces
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Last cmd: left:" << this->lastCmdDriveLeft
		   << " right: " << this->lastCmdDriveRight);
  double thrust_left = 0.0;
  double thrust_right = 0.0;
  switch(this->paramMappingType)
  {
  case 0: // Simplest, linear
    thrust_left = scaleThrustCmd(this->lastCmdDriveLeft);
    thrust_right = scaleThrustCmd(this->lastCmdDriveRight);
    break;
  case 1: // GLF
    thrust_left = glfThrustCmd(this->lastCmdDriveLeft);
    thrust_right = glfThrustCmd(this->lastCmdDriveRight);
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
  this->link->AddRelativeTorque(math::Vector3(0,0,torque));

  // Add input force with offset below vessel
  math::Vector3 relpos(-1.0 * this->paramBoatLength / 2.0, 0.0,
		  this->paramThrustZoffset);  // relative pos of thrusters
  math::Vector3 inputforce3(thrust, 0,0);

  // Get Pose
  math::Pose pose = this->link->GetWorldPose();

  inputforce3 = pose.rot.RotateVector(inputforce3);
  this->link->AddForceAtRelativePosition(inputforce3,relpos);

  // Spin the propellers
  SpinPropeller(this->leftPropellerJoint, this->lastCmdDriveLeft);
  SpinPropeller(this->rightPropellerJoint, this->lastCmdDriveRight);
}

//////////////////////////////////////////////////
void UsvThrust::OnCmdDrive( const usv_gazebo_plugins::UsvDriveConstPtr &msg)
{
  this->lastCmdDriveTime = this->world->GetSimTime();
  this->lastCmdDriveLeft = msg->left;
  this->lastCmdDriveRight = msg->right;
}

//////////////////////////////////////////////////
void UsvThrust::ParsePropeller(const sdf::ElementPtr sdf,
    const std::string &sdf_name, physics::JointPtr &propeller_joint)
{
  if (sdf->HasElement(sdf_name))
  {
    std::string propeller_name;
    propeller_name = sdf->GetElement(sdf_name)->Get<std::string>();
    propeller_joint = this->model->GetJoint(propeller_name);

    if (!propeller_joint)
      ROS_WARN_STREAM("Unable to find propeller joint <"<<propeller_name<<">");
  }
}

//////////////////////////////////////////////////
void UsvThrust::SpinPropeller(const physics::JointPtr &propeller,
    const double input)
{
  const double min_input = 0.1;
  const double max_input = 1.0;
  const double max_effort = 2.0;
  double effort = 0.0;
  
  if (!propeller)
    return;

  if (std::abs(input) > min_input)
    effort = (input / max_input) * max_effort;

  propeller->SetForce(0, effort);
}

GZ_REGISTER_MODEL_PLUGIN(UsvThrust);
