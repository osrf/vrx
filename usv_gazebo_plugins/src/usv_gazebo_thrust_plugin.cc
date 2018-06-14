/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the usv_gazebo_dynamics_plugin package, known as this Package.

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

#include <boost/thread.hpp>
#include <ros/time.h>

#include <robotx_gazebo/usv_gazebo_thrust_plugin.hh>

using namespace gazebo;

UsvThrust::UsvThrust()
{
}

UsvThrust::~UsvThrust()
{
  rosnode_->shutdown();
  spinner_thread_->join();
  delete rosnode_;
  delete spinner_thread_;
}

void UsvThrust::FiniChild()
{
}


double UsvThrust::getSdfParamDouble(sdf::ElementPtr sdfPtr, const std::string &param_name, double default_val)
{
  double val = default_val;
  if (sdfPtr->HasElement(param_name) && sdfPtr->GetElement(param_name)->GetValue())
  {
    val = sdfPtr->GetElement(param_name)->Get<double>();
    ROS_INFO_STREAM("Parameter found - setting <" << param_name << "> to <" << val << ">.");

  }
  else{
    ROS_INFO_STREAM("Parameter <" << param_name << "> not found: Using default value of <" << val << ">.");
  }
  return val;
}

void UsvThrust::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  ROS_INFO("Loading usv_gazebo_thrust_plugin");
  model_ = _parent;
  world_ = model_->GetWorld();

  // Retrieve model paramters from SDF
  // Set default values
  node_namespace_ = "";
  cmd_timeout_ = 1.0; // how long to allow no input on cmd_drive
  param_mapping_type_ = 0;
  param_max_cmd_ = 1.0;
  param_max_force_fwd_ = 100.0;
  param_max_force_rev_ = -100.0;
  param_boat_width_ = 1.0;
  param_boat_length_ = 1.35;
  param_thrust_z_offset_ = -0.01;

  //  Enumerating model
  ROS_INFO_STREAM("Enumerating Model...");
  ROS_INFO_STREAM("Model name = "<< model_->GetName());
  physics::Link_V links = model_->GetLinks();
  for (unsigned int ii=0; ii<links.size(); ii++){
    ROS_INFO_STREAM("Link: "<<links[ii]->GetName());
  }

  // Get parameters from SDF
  if (_sdf->HasElement("robotNamespace"))
  {
    node_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  if (!_sdf->HasElement("bodyName") || !_sdf->GetElement("bodyName")->GetValue())
  {
    link_ = model_->GetLink();
    link_name_ = link_->GetName();
    ROS_INFO_STREAM("Did not find SDF parameter bodyName");
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    //link_name_ = "thrust_link";
    link_ = model_->GetLink(link_name_);

    ROS_INFO_STREAM("Found SDF parameter bodyName as <"<<link_name_<<">");
  }
  if (!link_)
  {
    ROS_FATAL("usv_gazebo_thrust_plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }
  else
  {
    ROS_INFO_STREAM("USV Model Link Name = " << link_name_);
  }

  cmd_timeout_ = getSdfParamDouble(_sdf,"cmdTimeout",cmd_timeout_);
  if (_sdf->HasElement("mappingType") && _sdf->GetElement("mappingType")->GetValue()) {
    param_mapping_type_ = _sdf->GetElement("mappingType")->Get<int>();
    ROS_INFO_STREAM("Parameter found - setting <mappingType> to <" << param_mapping_type_ <<">.");
  }
  else{
    ROS_INFO_STREAM("Parameter <mappingType> not found: Using default value of <" << param_mapping_type_ << ">.");
  }
  // verify
  if ( (param_mapping_type_ > 1) || (param_mapping_type_ < 0) ){
    ROS_FATAL_STREAM("Cannot use a mappingType of " << param_mapping_type_);
    param_mapping_type_ = 0;
  }
  param_max_cmd_ = getSdfParamDouble(_sdf,"maxCmd",param_max_cmd_);
  param_max_force_fwd_ = getSdfParamDouble(_sdf,"maxForceFwd",
					   param_max_force_fwd_);
  param_max_force_rev_ = getSdfParamDouble(_sdf,"maxForceRev",
					   param_max_force_rev_);
  param_max_force_rev_ = -1.0*std::abs(param_max_force_rev_);  // make negative
  param_boat_width_ = getSdfParamDouble(_sdf,"boatWidth",param_boat_width_);
  param_boat_length_ = getSdfParamDouble(_sdf,"boatLength",param_boat_length_);
  param_thrust_z_offset_ = getSdfParamDouble(_sdf,"thrustOffsetZ",
					 param_thrust_z_offset_);

  //initialize time and odometry position
  prev_update_time_ = last_cmd_drive_time_ = this->world_->GetSimTime();

  // Initialize the ROS node and subscribe to cmd_drive
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "usv_thrust_gazebo",
	    ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle( node_namespace_ );

  cmd_drive_sub_ = rosnode_->subscribe("cmd_drive", 1, &UsvThrust::OnCmdDrive, this );

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->spinner_thread_ = new boost::thread( boost::bind( &UsvThrust::spin, this) );
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&UsvThrust::UpdateChild, this));

}

double UsvThrust::scaleThrustCmd(double cmd)
{
  double val = 0.0;
  if (cmd >= 0.0){
    val = cmd/param_max_cmd_*param_max_force_fwd_;
    val = std::min(val,param_max_force_fwd_);
  }
  else  // cmd is less than zero
  {
    val = -1.0*std::abs(cmd)/param_max_cmd_*std::abs(param_max_force_rev_);
    val = std::max(val,param_max_force_rev_);
  }
  return val;
}


double UsvThrust::glf(double x, float A, float K, float B,
		      float v, float C, float M){
  return A+ (K-A) / (pow(C+exp(-B*(x-M)),1.0/v));
}

double UsvThrust::glfThrustCmd(double cmd)
{
  double val = 0.0;
  if (cmd > 0.01){
    val = glf(cmd,0.01,59.82,5.0,0.38,0.56,0.28);
    val = std::min(val,param_max_force_fwd_);
  }
  else if (cmd < 0.01){
    val = glf(cmd,-199.13,-0.09,8.84,5.34,0.99,-0.57);
    val = std::max(val,param_max_force_rev_);
  }
  else{
    val = 0.0;
  }
  ROS_INFO_STREAM_THROTTLE(0.5,cmd << ": " << val << " / " << val/param_max_force_fwd_);
  return val;

}


void UsvThrust::UpdateChild()
{
  common::Time time_now = this->world_->GetSimTime();
  prev_update_time_ = time_now;

  // Enforce command timeout
  double dcmd = (time_now - last_cmd_drive_time_).Double();
  if ( (dcmd > cmd_timeout_) && (cmd_timeout_ > 0.0) )
  {
    ROS_INFO_STREAM_THROTTLE(1.0,"Command timeout!");
    last_cmd_drive_left_ = 0.0;
    last_cmd_drive_right_ = 0.0;
  }
  // Scale commands to thrust and torque forces
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Last cmd: left:" << last_cmd_drive_left_
		   << " right: " << last_cmd_drive_right_);
  double thrust_left = 0.0;
  double thrust_right = 0.0;
  switch(param_mapping_type_){
  case 0: // Simplest, linear
    thrust_left = scaleThrustCmd(last_cmd_drive_left_);
    thrust_right = scaleThrustCmd(last_cmd_drive_right_);
    break;
  case 1: // GLF
    thrust_left = glfThrustCmd(last_cmd_drive_left_);
    thrust_right = glfThrustCmd(last_cmd_drive_right_);
    break;
  default:
    ROS_FATAL_STREAM("Cannot use mappingType="<<param_mapping_type_);
    break;


  } // eo switch

  double thrust = thrust_right + thrust_left;
  double torque = (thrust_right - thrust_left)*param_boat_width_;
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Thrust: left:" << thrust_left
			    << " right: " << thrust_right);

  // Add torque
  link_->AddRelativeTorque(math::Vector3(0,0,torque));

  // Add input force with offset below vessel
  math::Vector3 relpos(-1.0*param_boat_length_/2.0, 0.0 ,
		       param_thrust_z_offset_);  // relative pos of thrusters
  math::Vector3 inputforce3(thrust, 0,0);

  // Get Pose
  pose_ = link_->GetWorldPose();

  //link_->AddLinkForce(inputforce3,relpos);
  inputforce3 = pose_.rot.RotateVector(inputforce3);
  //link_->AddRelativeForce(inputforce3);
  link_->AddForceAtRelativePosition(inputforce3,relpos);
}

void UsvThrust::OnCmdDrive( const robotx_gazebo::UsvDriveConstPtr &msg)
{
    last_cmd_drive_time_ = this->world_->GetSimTime();
    last_cmd_drive_left_ = msg->left;
    last_cmd_drive_right_ = msg->right;
}


void UsvThrust::spin()
{
    while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(UsvThrust);

