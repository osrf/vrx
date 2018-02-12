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
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <functional>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/time.h>

#include <robotx_gazebo/usv_gazebo_dynamics_plugin.hh>

#define GRAVITY 9.815

using namespace gazebo;

UsvPlugin::UsvPlugin()
{
}

UsvPlugin::~UsvPlugin()
{
  rosnode_->shutdown();
  spinner_thread_->join();
  delete rosnode_;
  delete spinner_thread_;
}

void UsvPlugin::FiniChild()
{
}


double UsvPlugin::getSdfParamDouble(sdf::ElementPtr sdfPtr, const std::string &param_name, double default_val)
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

void UsvPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  ROS_INFO("Loading usv_gazebo_dynamics_plugin");
  model_ = _parent;
  world_ = model_->GetWorld();

  // Retrieve model paramters from SDF
  // Set default values
  node_namespace_ = "";


  water_level_ = 0.5;
  water_density_ = 997.7735;
  cmd_timeout_ = 1.0; // how long to allow no input on cmd_drive

  param_X_dot_u_ = 5;  // Added mass
  param_Y_dot_v_ = 5;
  param_N_dot_r_ = 1;
  param_X_u_ = 20;  // linear drag
  param_X_uu_ = 0.0;
  param_Y_v_ = 20;
  param_Y_vv_ = 0.0;
  param_Z_w_ = 20;
  param_K_p_ = 20.0;
  param_M_q_ = 20.0;
  param_N_r_ = 20;
  param_N_rr_ = 0.0;
  param_max_cmd_ = 1.0;
  param_max_force_fwd_ = 100.0;
  param_max_force_rev_ = -100.0;
  param_boat_width_ = 1.0;
  param_boat_length_ = 1.35;
  param_thrust_z_offset_ = -0.01;
  param_metacentric_length_ = 0.4 ; // From clearpath
  param_metacentric_width_ = 0.4;  // ditto
  param_boat_area_ = 0.48;  // Clearpath: 1.2m in length, 0.2m in width, 2 pontoons.


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
    ROS_FATAL("usv_gazebo_dynamics_plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }
  else
  {
    ROS_INFO_STREAM("USV Model Link Name = " << link_name_);
  }

  water_level_ = getSdfParamDouble(_sdf,"waterLevel",water_level_);
  water_density_ = getSdfParamDouble(_sdf,"waterDensity",water_density_);
  cmd_timeout_ = getSdfParamDouble(_sdf,"cmdTimeout",cmd_timeout_);
  param_X_dot_u_ = getSdfParamDouble(_sdf,"xDotU",param_X_dot_u_);
  param_Y_dot_v_ = getSdfParamDouble(_sdf,"yDotV",param_Y_dot_v_);
  param_N_dot_r_ = getSdfParamDouble(_sdf,"nDotR",param_N_dot_r_);

  param_X_u_ = getSdfParamDouble(_sdf,"xU",param_X_u_);
  param_X_uu_ = getSdfParamDouble(_sdf,"xUU",param_X_uu_);
  param_Y_v_ = getSdfParamDouble(_sdf,"yV",param_Y_v_);
  param_Y_vv_ = getSdfParamDouble(_sdf,"yVV",param_Y_vv_);
  param_Z_w_ = getSdfParamDouble(_sdf,"zW",param_Z_w_);
  param_K_p_ = getSdfParamDouble(_sdf,"kP",param_K_p_);
  param_M_q_ = getSdfParamDouble(_sdf,"mQ",param_M_q_);
  param_N_r_ = getSdfParamDouble(_sdf,"nR",param_N_r_);
  param_N_rr_ = getSdfParamDouble(_sdf,"nRR",param_N_rr_);

  param_max_cmd_ = getSdfParamDouble(_sdf,"maxCmd",param_max_cmd_);
  param_max_force_fwd_ = getSdfParamDouble(_sdf,"maxForceFwd",
					   param_max_force_fwd_);
  param_max_force_rev_ = getSdfParamDouble(_sdf,"maxForceRev",
					   param_max_force_rev_);
  param_max_force_rev_ = -1.0*std::abs(param_max_force_rev_);  // make negative

  param_boat_area_ = getSdfParamDouble(_sdf,"boatArea",param_boat_area_);
  param_boat_width_ = getSdfParamDouble(_sdf,"boatWidth",param_boat_width_);
  param_boat_length_ = getSdfParamDouble(_sdf,"boatLength",param_boat_length_);
  param_thrust_z_offset_ = getSdfParamDouble(_sdf,"thrustOffsetZ",
					 param_thrust_z_offset_);
  param_metacentric_length_ = getSdfParamDouble(_sdf,"metacentricLength",
						param_metacentric_length_);
  param_metacentric_width_ = getSdfParamDouble(_sdf,"metacentricWidth",
						param_metacentric_width_);
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

  // Get inertia and mass of vessel
  math::Vector3 inertia = link_->GetInertial()->GetPrincipalMoments();
  double mass = link_->GetInertial()->GetMass();

  // Report some of the pertinent parameters for verification
  ROS_INFO("USV Dynamics Parameters: From URDF XACRO model definition");
  ROS_INFO_STREAM("Vessel Mass (rigid-body): " << mass);
  ROS_INFO_STREAM("Vessel Inertia Vector (rigid-body): X:" << inertia[0] <<
		  " Y:"<<inertia[1] <<
		  " Z:"<<inertia[2]);
  ROS_INFO("USV Dynamics Parameters: Plugin Parameters");

  //initialize time and odometry position
  prev_update_time_ = last_cmd_drive_time_ = this->world_->GetSimTime();

  // Initialize the ROS node and subscribe to cmd_drive
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "usv_gazebo",
	    ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle( node_namespace_ );

  cmd_drive_sub_ = rosnode_->subscribe("cmd_drive", 1, &UsvPlugin::OnCmdDrive, this );

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->spinner_thread_ = new std::thread( std::bind( &UsvPlugin::spin, this) );
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvPlugin::UpdateChild, this));

  // Initialize Added Mass Matrix
  Ma_ = Eigen::MatrixXd(6,6);
  Ma_ << param_X_dot_u_ ,   0,   0, 0, 0, 0,
        0,   param_Y_dot_v_,   0, 0, 0, 0,
        0,   0,   0.1, 0, 0, 0,
        0,   0,   0, 0.1, 0, 0,
        0,   0,   0, 0, 0.1, 0,
        0,   0,   0, 0, 0, param_N_dot_r_ ;
}

double UsvPlugin::scaleThrustCmd(double cmd, double max_cmd, double max_pos, double max_neg)
{
  double val = 0.0;
  if (cmd >= 0.0){
    val = cmd/max_cmd*max_pos;
    if (val > max_pos)
    {
      val = max_pos;
    }
  }
  else  // cmd is less than zero
  {
    val = -1.0*std::abs(cmd)/max_cmd*std::abs(max_neg);
    if (std::abs(val) > std::abs(max_neg))
    {
      val = -1.0*std::abs(max_neg);  // ensure it is negative
    }
  }
  return val;
}
void UsvPlugin::UpdateChild()
{
  common::Time time_now = this->world_->GetSimTime();
  common::Time step_time = time_now - prev_update_time_;

  double dt = step_time.Double();
  prev_update_time_ = time_now;

  // Enforce command timeout
  common::Time cmd_time = time_now - last_cmd_drive_time_;
  double dcmd = cmd_time.Double();
  if ( dcmd > cmd_timeout_ )
  {
    ROS_INFO_STREAM_THROTTLE(1.0,"Command timeout!");
    last_cmd_drive_left_ = 0.0;
    last_cmd_drive_right_ = 0.0;
  }
  // Scale commands to thrust and torque forces
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Last cmd: left:" << last_cmd_drive_left_
		   << " right: " << last_cmd_drive_right_);
  double thrust_left = scaleThrustCmd(last_cmd_drive_left_, param_max_cmd_,
				      param_max_force_fwd_,
				      param_max_force_rev_);

  double thrust_right = scaleThrustCmd(last_cmd_drive_right_, param_max_cmd_,
				      param_max_force_fwd_,
				      param_max_force_rev_);
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Thrust: left:" << thrust_left
		   << " right: " << thrust_right);
  double thrust = thrust_right + thrust_left;
  double torque = (thrust_right - thrust_left)*param_boat_width_;




  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  pose = link_->GetWorldPose();
  euler = pose.rot.GetAsEuler();
  math::Quaternion curr_orientation = pose.rot;

  // Get body-centered linear and angular rates
  math::Vector3 vel_linear_body = link_->GetRelativeLinearVel();
  ROS_DEBUG_STREAM_THROTTLE(0.5,"Vel linear: " << vel_linear_body);
  math::Vector3 vel_angular_body = link_->GetRelativeAngularVel();
  ROS_DEBUG_STREAM_THROTTLE(0.5,"Vel angular: " << vel_angular_body);
  // Estimate the linear and angular accelerations.
  // Note the the GetRelativeLinearAccel() and AngularAccel() functions
  // appear to be unreliable
  math::Vector3 accel_linear_body = (vel_linear_body - prev_lin_vel_) /dt;
  prev_lin_vel_ = vel_linear_body;
  ROS_DEBUG_STREAM_THROTTLE(0.5,"Accel linear: " << accel_linear_body);
  math::Vector3 accel_angular_body = (vel_angular_body - prev_ang_vel_) /dt;
  prev_ang_vel_ = vel_angular_body;
  ROS_DEBUG_STREAM_THROTTLE(0.5,"Accel angular: " << accel_angular_body);

  // Wind
  // Transform wind from world coordinates to body coordinates
  math::Vector3 relative_wind = pose.rot.GetInverse().RotateVector(param_wind_velocity_vector_);
  // Calculate apparent wind
  math::Vector3 apparent_wind = relative_wind - vel_linear_body;
  ROS_DEBUG_STREAM_THROTTLE(0.5,"Relative wind: " << relative_wind);
  ROS_DEBUG_STREAM_THROTTLE(0.5,"Apparent wind: " << apparent_wind);
  // Calculate wind force - body coordinates
  math::Vector3 wind_force(
			   param_wind_coeff_vector_.x * relative_wind.x * abs(relative_wind.x),
			   param_wind_coeff_vector_.y * relative_wind.y * abs(relative_wind.y),
			   -2.0*param_wind_coeff_vector_.z * relative_wind.x * relative_wind.y);

  // Create state and derivative of state (accelerations)
  Eigen::VectorXd state_dot(6);
  state_dot << accel_linear_body.x, accel_linear_body.y, accel_linear_body.z,
    accel_angular_body.x, accel_angular_body.y, accel_angular_body.z;

  Eigen::VectorXd state(6);
  state << vel_linear_body.x, vel_linear_body.y, vel_linear_body.z,
    vel_angular_body.x, vel_angular_body.y, vel_angular_body.z;

  // Added Mass
  Eigen::VectorXd amassVec = -1.0*Ma_*state_dot;
  ROS_DEBUG_STREAM_THROTTLE(1.0,"state_dot: \n" << state_dot);
  ROS_DEBUG_STREAM_THROTTLE(1.0,"amassVec :\n" << amassVec);

  // Coriolis - added mass components
  Eigen::MatrixXd Cmat = Eigen::MatrixXd::Zero(6,6);
  Cmat(0,5) = param_Y_dot_v_ * vel_linear_body.y;
  Cmat(1,5) = param_X_dot_u_ * vel_linear_body.x;
  Cmat(5,0) = param_Y_dot_v_ * vel_linear_body.y;
  Cmat(5,1) = param_X_dot_u_ * vel_linear_body.x;
  Eigen::VectorXd Cvec = -1.0*Cmat*state;
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Cvec :\n" << Cvec);

  // Drag
  //Eigen::MatrixXd Dmat = Eigen::MatrixXd(6,6);
  Eigen::MatrixXd Dmat = Eigen::MatrixXd::Zero(6,6);
  Dmat(0,0) = param_X_u_ + param_X_uu_*std::abs(vel_linear_body.x);
  Dmat(1,1) = param_Y_v_ + param_Y_vv_*std::abs(vel_linear_body.y);
  Dmat(2,2) = param_Z_w_;
  Dmat(3,3) = param_K_p_;
  Dmat(4,4) = param_M_q_;
  Dmat(5,5) = param_N_r_ + param_N_rr_*std::abs(vel_angular_body.z);
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Dmat :\n" << Dmat);
  Eigen::VectorXd Dvec = -1.0*Dmat*state;
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Dvec :\n" << Dvec);

  // Restoring/Buoyancy Forces
  double buoy_force = (water_level_ - pose.pos.z)*param_boat_area_*GRAVITY*water_density_;
  Eigen::VectorXd buoyVec = Eigen::VectorXd::Zero(6);
  buoyVec(2) = buoy_force;  // Z direction - shoudl really be in XYZ frame
  buoyVec(3) = -param_metacentric_width_*sin(euler.x)*buoy_force; // roll
  buoyVec(4) = -param_metacentric_length_*sin(euler.y)*buoy_force; // pitch
  ROS_DEBUG_STREAM_THROTTLE(1.0,"buoyVec :\n" << buoyVec);

  // Inputs
  Eigen::VectorXd inputVec = Eigen::VectorXd::Zero(6);
  //inputVec(0) = thrust;
  inputVec(5) = torque;
  ROS_DEBUG_STREAM_THROTTLE(1.0,"inputVec :\n" << inputVec);

  // Sum all forces
  // note, inputVec only includes torque component
  Eigen::VectorXd forceSum = inputVec + amassVec + Dvec + buoyVec;

  ROS_DEBUG_STREAM_THROTTLE(1.0,"forceSum :\n" << forceSum);
  math::Vector3 totalLinear(forceSum(0)+wind_force.x,
			    forceSum(1)+wind_force.y,
			    forceSum(2));
  math::Vector3 totalAngular(forceSum(3),forceSum(4),
			     forceSum(5)+wind_force.z);

  // Add dynamic forces/torques to link at CG
  link_->AddRelativeForce(totalLinear);
  link_->AddRelativeTorque(totalAngular);

  // Add input force with offset below vessel
  math::Vector3 relpos(-1.0*param_boat_length_/2.0, 0.0 ,
		       param_thrust_z_offset_);  // relative pos of thrusters
  math::Vector3 inputforce3(thrust, 0,0);

  //link_->AddLinkForce(inputforce3,relpos);
  inputforce3 = curr_orientation.RotateVector(inputforce3);
  //link_->AddRelativeForce(inputforce3);
  link_->AddForceAtRelativePosition(inputforce3,relpos);
}

void UsvPlugin::OnCmdDrive( const robotx_gazebo::UsvDriveConstPtr &msg)
{
    last_cmd_drive_time_ = this->world_->GetSimTime();
    last_cmd_drive_left_ = msg->left;
    last_cmd_drive_right_ = msg->right;
}

void UsvPlugin::spin()
{
    while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(UsvPlugin);

