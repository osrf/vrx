/*

Copyright (c) 2018, Brian Bingham
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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/time.h>
#include <tf2/LinearMath/Transform.h>

#include <robotx_gazebo/usv_gazebo_wind_plugin.hh>

#define GRAVITY 9.815

using namespace gazebo;

UsvWindPlugin::UsvWindPlugin()
{
}

UsvWindPlugin::~UsvWindPlugin()
{
  rosnode_->shutdown();
  spinner_thread_->join();
  delete rosnode_;
  delete spinner_thread_;
}

void UsvWindPlugin::FiniChild()
{
}


double UsvWindPlugin::getSdfParamDouble(sdf::ElementPtr sdfPtr, const std::string &param_name, double default_val)
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

void UsvWindPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  ROS_INFO("Loading usv_gazebo_wind_plugin");
  model_ = _parent;
  world_ = model_->GetWorld();

  // Retrieve model paramters from SDF
  // Set default values
  node_namespace_ = "";

  //  Enumerating model
  ROS_INFO_STREAM("Enumerating Model...");
  ROS_INFO_STREAM("Model name = "<< model_->GetName());
  physics::Link_V links = model_->GetLinks();
  for (auto ii=0u; ii<links.size(); ii++){
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
    ROS_FATAL("usv_gazebo_wind_plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }
  else
  {
    ROS_INFO_STREAM("USV Model Link Name = " << link_name_);
  }

  if (_sdf->HasElement("wind_velocity_vector")){
    param_wind_velocity_vector_ = _sdf->GetElement("wind_velocity_vector")->Get<math::Vector3>();
  }
  else{
    param_wind_velocity_vector_ = math::Vector3(0,0,0);
  }
  ROS_INFO_STREAM("Wind velocity vector = "<<param_wind_velocity_vector_.x << " , " << param_wind_velocity_vector_.y << " , " << param_wind_velocity_vector_.z);


  if (_sdf->HasElement("wind_coeff_vector")){
    param_wind_coeff_vector_ = _sdf->GetElement("wind_coeff_vector")->Get<math::Vector3>();
  }
  else{
    param_wind_coeff_vector_ = math::Vector3(0,0,0);
  }
  ROS_INFO_STREAM("Wind coefficient vector = "<<param_wind_coeff_vector_.x << " , " << param_wind_coeff_vector_.y << " , " << param_wind_coeff_vector_.z);


  // Initialize the ROS node and subscribe to cmd_drive
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "usv_wind_gazebo",
	    ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle( node_namespace_ );

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->spinner_thread_ = new boost::thread( boost::bind( &UsvWindPlugin::spin, this) );
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&UsvWindPlugin::UpdateChild, this));
}
void UsvWindPlugin::UpdateChild()
{
  // Wind
  // Transform wind from world coordinates to body coordinates
  math::Vector3 relative_wind = link_->GetWorldPose().rot.GetInverse().RotateVector(param_wind_velocity_vector_);
  // Calculate apparent wind
  math::Vector3 apparent_wind = relative_wind - link_->GetRelativeLinearVel();
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Relative wind: " << relative_wind);
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Apparent wind: " << apparent_wind);
  // Calculate wind force - body coordinates
  math::Vector3 wind_force(
			   param_wind_coeff_vector_.x * relative_wind.x * abs(relative_wind.x),
			   param_wind_coeff_vector_.y * relative_wind.y * abs(relative_wind.y),
			   -2.0*param_wind_coeff_vector_.z * relative_wind.x * relative_wind.y);


  // Add forces/torques to link at CG
  link_->AddRelativeForce(math::Vector3(wind_force.x, wind_force.y, 0.0));
  link_->AddRelativeTorque(math::Vector3(0.0,0.0,wind_force.z));
}

void UsvWindPlugin::spin()
{
    while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(UsvWindPlugin);

