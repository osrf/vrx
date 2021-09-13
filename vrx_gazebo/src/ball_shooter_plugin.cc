/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include "vrx_gazebo/ball_shooter_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
void BallShooterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gazebo::physics::WorldPtr world =
    gazebo::physics::get_world();

  GZ_ASSERT(_model != nullptr, "Received NULL model pointer");

  // Make sure the ROS node for Gazebo has already been initialised.
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("ball_shooter_plugin", "A ROS node for"
      " Gazebo hasn't been initialised, unable to load plugin. Load the Gazebo "
      "system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Parse the required <projectile> element.
  if (!_sdf->HasElement("projectile"))
  {
    gzerr << "BallShooterPlugin: Missing <projectile> element" << std::endl;
    return;
  }

  sdf::ElementPtr projectileElem = _sdf->GetElement("projectile");

  // Parse the required <projectile><model_name> used as projectile.
  if (!projectileElem->HasElement("model_name"))
  {
    gzerr << "BallShooterPlugin: Missing <projectile><model_name> element\n";
    return;
  }

  std::string projectileName =
    projectileElem->GetElement("model_name")->Get<std::string>();
#if GAZEBO_MAJOR_VERSION >= 8
  this->projectileModel = world->ModelByName(projectileName);
#else
  this->projectileModel = world->GetModel(projectileName);
#endif
  if (!this->projectileModel)
  {
    gzerr << "BallShooterPlugin: The model '" << projectileName
          << "' does not exist" << std::endl;
    return;
  }

  // Parse the required <projectile><link_name>
  if (!projectileElem->HasElement("link_name"))
  {
    gzerr << "BallShooterPlugin: Missing <projectile><link_name> element\n";
    return;
  }

  std::string projectileLinkName =
    projectileElem->GetElement("link_name")->Get<std::string>();

  this->projectileLink = this->projectileModel->GetLink(projectileLinkName);
  if (!this->projectileLink)
  {
    gzerr << "BallShooterPlugin: The link '" << projectileLinkName
          << "' does not exist within '" << projectileName << "'" << std::endl;
    return;
  }

  // Parse <frame> if available.
  std::string frameName;
  if (projectileElem->HasElement("frame"))
  {
    frameName = projectileElem->Get<std::string>("frame");
#if GAZEBO_MAJOR_VERSION >= 8
    this->frame = world->EntityByName(frameName);
#else
    this->frame = world->GetEntity(frameName);
#endif
    if (!this->frame)
    {
      gzerr << "The frame '" << frameName << "' does not exist" << std::endl;
      frameName = "";
    }
    else if (!this->frame->HasType(physics::Base::LINK) &&
             !this->frame->HasType(physics::Base::MODEL))
    {
      this->frame = nullptr;
      frameName = "";
      gzerr << "<frame> tag must list the name of a link or model" << std::endl;
    }
  }

  // Parse <pose> if available.
  if (projectileElem->HasElement("pose"))
    this->pose = projectileElem->Get<ignition::math::Pose3d>("pose");

  // Parse <topic> if available.
  std::string topic = "/ball_shooter/fire";
  if (_sdf->HasElement("topic"))
    topic = _sdf->GetElement("topic")->Get<std::string>();

  // Parse <num_shots> if available.
  if (_sdf->HasElement("num_shots"))
    this->remainingShots = _sdf->GetElement("num_shots")->Get<unsigned int>();

  // Parse <shot_force> if available.
  if (_sdf->HasElement("shot_force"))
    this->shotForce = _sdf->GetElement("shot_force")->Get<double>();

  // Initialise the ros handle.
  this->rosNodeHandle.reset(new ros::NodeHandle());

  this->fireSub = this->rosNodeHandle->subscribe(
    topic, 1, &BallShooterPlugin::OnFire, this);

  // Connect the update function to the world update event.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&BallShooterPlugin::Update, this));

  // Debug output.
  ROS_INFO_NAMED("ball_shooter_plugin", "<projectile><model_name>: %s",
    projectileName.c_str());
  ROS_INFO_NAMED("ball_shooter_plugin", "<projectile><link_name>: %s",
    projectileLinkName.c_str());
  ROS_INFO_NAMED("ball_shooter_plugin", "<frame>: %s", frameName.c_str());
  ROS_INFO_NAMED("ball_shooter_plugin",
    "<num_shots>: %u", this->remainingShots);
  ROS_INFO_NAMED("ball_shooter_plugin", "<pose>: {%f %f %f %f %f %f",
    this->pose.Pos().X(), this->pose.Pos().Y(), this->pose.Pos().Z(),
    this->pose.Rot().Roll(), this->pose.Rot().Pitch(), this->pose.Rot().Yaw());
  ROS_INFO_NAMED("ball_shooter_plugin", "<shot_force>: %f", this->shotForce);
  ROS_INFO_NAMED("ball_shooter_plugin", "<topic>: %s", topic.c_str());
}

//////////////////////////////////////////////////
void BallShooterPlugin::Update()
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (!shotReady || !this->projectileModel || !this->projectileLink)
    return;

  // Set the new pose of the projectile based on the frame.
  ignition::math::Pose3d projectilePose = this->pose;
  if (this->frame)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    auto framePose = this->frame->WorldPose();
#else
    auto framePose = this->frame->GetWorldPose().Ign();
#endif
    ignition::math::Matrix4d transMat(framePose);
    ignition::math::Matrix4d poseLocal(this->pose);
    projectilePose = (transMat * poseLocal).Pose();
  }

  // Teleport the projectile to the ball shooter.
  this->projectileModel->SetWorldPose(projectilePose);
  this->projectileModel->SetLinearVel(ignition::math::Vector3d::Zero);
  this->projectileModel->SetAngularVel(ignition::math::Vector3d::Zero);

  // Ignition!
  this->projectileLink->AddLinkForce({this->shotForce, 0, 0});

  --this->remainingShots;
  shotReady = false;
}

//////////////////////////////////////////////////
void BallShooterPlugin::OnFire(const std_msgs::Empty::ConstPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (this->remainingShots <= 0)
  {
    gzdbg << "BallShooterPlugin: Maximum number of shots already reached. "
          << "Request ignored" << std::endl;
    return;
  }

  this->shotReady = true;
}

GZ_REGISTER_MODEL_PLUGIN(BallShooterPlugin);
