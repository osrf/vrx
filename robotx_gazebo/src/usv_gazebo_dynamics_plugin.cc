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

#include <functional>
#include <thread>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/time.h>
#include <tf2/LinearMath/Transform.h>

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

  param_boat_width_ = 1.0;
  param_boat_length_ = 1.35;

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


  param_boat_area_ = getSdfParamDouble(_sdf,"boatArea",param_boat_area_);
  param_boat_width_ = getSdfParamDouble(_sdf,"boatWidth",param_boat_width_);
  param_boat_length_ = getSdfParamDouble(_sdf,"boatLength",param_boat_length_);

  param_metacentric_length_ = getSdfParamDouble(_sdf,"metacentricLength",
						param_metacentric_length_);
  param_metacentric_width_ = getSdfParamDouble(_sdf,"metacentricWidth",
						param_metacentric_width_);

  // Wave parameters
  std::ostringstream buf;
  math::Vector2d tmpm;
  std::vector<float> tmpv(2,0);
  param_wave_n_ = _sdf->GetElement("wave_n")->Get<int>();
  for (int ii=0; ii < param_wave_n_; ii++){
    buf.str("");
    buf<<"wave_amp"<<ii;
    param_wave_amps_.push_back(_sdf->GetElement(buf.str())->Get<float>());
    ROS_INFO_STREAM("Wave Amplitude "<<ii<<": "<<param_wave_amps_[ii]);
    buf.str("");
    buf<<"wave_period"<<ii;
    param_wave_periods_.push_back(_sdf->GetElement(buf.str())->Get<float>());
    buf.str("");
    buf<<"wave_direction"<<ii;
    tmpm=_sdf->GetElement(buf.str())->Get<math::Vector2d>();
    tmpv[0] = tmpm.x;
    tmpv[1] = tmpm.y;
    param_wave_directions_.push_back(tmpv);
    ROS_INFO_STREAM("Wave Direction "<<ii<<": "<<param_wave_directions_[ii][0]
		    << ", " << param_wave_directions_[ii][1]);

  }


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
  prev_update_time_ = this->world_->GetSimTime();

  // Initialize the ROS node
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "usv_dynamics_gazebo",
	    ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle( node_namespace_ );

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

  Cmat_ =  Eigen::MatrixXd::Zero(6,6);
  Dmat_ =  Eigen::MatrixXd::Zero(6,6);
  state_dot_ = Eigen::VectorXd(6);
  state_ = Eigen::VectorXd(6);
  amassVec_ = Eigen::VectorXd(6);

  int NN = 2; // must be factor of 2! - only 2 for now!!
  // x,y grid step increments
  dx_ = param_boat_length_/NN;
  dy_ = param_boat_width_/NN;
  // Vector for interating throug grid points on boat
  for (int ii=-NN/2; ii<0; ii++){
    II_.push_back(ii);
  }
  for (int ii=1; ii<=NN/2; ii++){
    II_.push_back(ii);
  }
  // Precalculate this to save some time.
  buoy_frac_ = (param_boat_area_/(NN*NN))*GRAVITY*water_density_;

}


void UsvPlugin::UpdateChild()
{
  common::Time time_now = this->world_->GetSimTime();
  //common::Time step_time = time_now - prev_update_time_;
  double dt = (time_now - prev_update_time_).Double();
  prev_update_time_ = time_now;

  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  pose_ = link_->GetWorldPose();
  euler_ = pose_.rot.GetAsEuler();

  // Get body-centered linear and angular rates
  vel_linear_body_ = link_->GetRelativeLinearVel();
  ROS_DEBUG_STREAM_THROTTLE(0.5,"Vel linear: " << vel_linear_body_);
  vel_angular_body_ = link_->GetRelativeAngularVel();
  ROS_DEBUG_STREAM_THROTTLE(0.5,"Vel angular: " << vel_angular_body_);
  // Estimate the linear and angular accelerations.
  // Note the the GetRelativeLinearAccel() and AngularAccel() functions
  // appear to be unreliable
  math::Vector3 accel_linear_body = (vel_linear_body_ - prev_lin_vel_) /dt;
  prev_lin_vel_ = vel_linear_body_;
  ROS_DEBUG_STREAM_THROTTLE(0.5,"Accel linear: " << accel_linear_body);
  math::Vector3 accel_angular_body = (vel_angular_body_ - prev_ang_vel_) /dt;
  prev_ang_vel_ = vel_angular_body_;
  ROS_DEBUG_STREAM_THROTTLE(0.5,"Accel angular: " << accel_angular_body);

  // Create state and derivative of state (accelerations)
  state_dot_ << accel_linear_body.x, accel_linear_body.y, accel_linear_body.z,
    accel_angular_body.x, accel_angular_body.y, accel_angular_body.z;

  state_ << vel_linear_body_.x, vel_linear_body_.y, vel_linear_body_.z,
    vel_angular_body_.x, vel_angular_body_.y, vel_angular_body_.z;

  // Added Mass
  amassVec_ = -1.0*Ma_*state_dot_;
  ROS_DEBUG_STREAM_THROTTLE(1.0,"state_dot_: \n" << state_dot_);
  ROS_DEBUG_STREAM_THROTTLE(1.0,"amassVec :\n" << amassVec_);

  // Coriolis - added mass components
  Cmat_(0,5) = param_Y_dot_v_ * vel_linear_body_.y;
  Cmat_(1,5) = param_X_dot_u_ * vel_linear_body_.x;
  Cmat_(5,0) = param_Y_dot_v_ * vel_linear_body_.y;
  Cmat_(5,1) = param_X_dot_u_ * vel_linear_body_.x;
  Cvec_ = -1.0*Cmat_*state_;
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Cvec :\n" << Cvec_);

  // Drag
  //Eigen::MatrixXd Dmat = Eigen::MatrixXd(6,6);
  Dmat_(0,0) = param_X_u_ + param_X_uu_*std::abs(vel_linear_body_.x);
  Dmat_(1,1) = param_Y_v_ + param_Y_vv_*std::abs(vel_linear_body_.y);
  Dmat_(2,2) = param_Z_w_;
  Dmat_(3,3) = param_K_p_;
  Dmat_(4,4) = param_M_q_;
  Dmat_(5,5) = param_N_r_ + param_N_rr_*std::abs(vel_angular_body_.z);
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Dmat :\n" << Dmat_);
  Dvec_ = -1.0*Dmat_*state_;
  ROS_DEBUG_STREAM_THROTTLE(1.0,"Dvec :\n" << Dvec_);

  // Vehicle frame transform
  tf2::Quaternion vq = tf2::Quaternion();
  tf2::Matrix3x3 m;
  m.setEulerYPR(euler_.z,euler_.y,euler_.x);
  m.getRotation(vq);
  tf2::Transform xform_v = tf2::Transform(vq);

  // Sum all forces - in body frame
  Eigen::VectorXd forceSum = amassVec_ + Dvec_;// + buoyVec;
  // Forces in fixed frame
  ROS_DEBUG_STREAM_THROTTLE(1.0,"forceSum :\n" << forceSum);

  // Add dynamic forces/torques to link at CG
  link_->AddRelativeForce(math::Vector3(forceSum(0),forceSum(1),forceSum(2)));
  link_->AddRelativeTorque(math::Vector3(forceSum(3),forceSum(4),forceSum(5)));

  // Distribute upward buoyancy force

  float ddz, buoy_force;
  double w, k, dz, Ddotx;
  math::Vector3 X;  // location of vehicle base link
  tf2::Vector3 bpnt(0,0,0);     // grid points on boat
  tf2::Vector3 bpnt_w(0,0,0);   // in world coordinates

  // Loop over boat grid points
  //for (std::list<int>::iterator ii=Ilist.begin();ii != Ilist.end(); ii++){
  for (std::vector<int>::iterator it=II_.begin(); it != II_.end(); ++it){
    bpnt.setX((*it)*dx_);  // grid point in boat fram
    for (std::vector<int>::iterator jt=II_.begin(); jt != II_.end(); ++jt){
      //for (std::list<int>::iterator jj=Jlist.begin(); jj != Jlist.end(); jj++)
      bpnt.setY((*jt)*dy_);

      // Transform from vessel to water/world frame
      bpnt_w = xform_v*bpnt;

      // Debug
      ROS_DEBUG_STREAM_THROTTLE(1.0,"[" << (*it) <<","<<(*jt)<< "] grid points"<<bpnt.x() <<","<<bpnt.y() <<","<<bpnt.z());
      ROS_DEBUG_STREAM_THROTTLE(1.0,"v frame euler "<<euler_);
      ROS_DEBUG_STREAM_THROTTLE(1.0,"in water frame"<<bpnt_w.x() <<","<<bpnt_w.y() <<","<<bpnt_w.z());

      // Vertical location of boat grid point in world frame
      ddz = pose_.pos.z+bpnt_w.z();
      ROS_DEBUG_STREAM("Z, pose: " << pose_.pos.z <<", bpnt: "<<bpnt_w.z() << ", dd: " << ddz);

      // Find vertical displacement of wave field
      X.x = pose_.pos.x+bpnt_w.x();  // World location of grid point
      X.y = pose_.pos.y+bpnt_w.y();
      // sum vertical dsplacement over all waves
      dz = 0.0;
      for (int ii=0; ii < param_wave_n_; ii++){
	Ddotx=param_wave_directions_[ii][0]*X.x
	  +param_wave_directions_[ii][1]*X.y;
	w = 2.0*3.14159 / param_wave_periods_[ii];
	k = w*w/9.81;
	dz += param_wave_amps_[ii]*cos(k*Ddotx-w*time_now.Float());;
      }
      ROS_DEBUG_STREAM_THROTTLE(1.0,"wave disp: " << dz);

      // Buoyancy force at grid point
      buoy_force = (((water_level_+dz)-(ddz))*(buoy_frac_));
      ROS_DEBUG_STREAM("buoy_force: " << buoy_force);
      // Apply force at grid point
      // From web, Appears that position is in the link frame
      // and force is in world frame
      link_->AddForceAtRelativePosition(math::Vector3(0,0,buoy_force),
					math::Vector3(bpnt.x(),bpnt.y(),bpnt.z()));
    }
  }
}

void UsvPlugin::spin()
{
    while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(UsvPlugin);

