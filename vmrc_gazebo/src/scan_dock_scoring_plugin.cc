/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <cmath>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include "vmrc_gazebo/scan_dock_scoring_plugin.hh"

/////////////////////////////////////////////////
ScanDockScoringPlugin::ScanDockScoringPlugin()
{
  gzmsg << "scan and dock scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void ScanDockScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  ScoringPlugin::Load(_world, _sdf);

  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  if (!this->ParseSDF(_sdf))
    return;

  // Quit if ros plugin was not loaded
  if (!ros::isInitialized())
  {
    ROS_ERROR("ROS was not initialized.");
    return;
  }

  this->nh = ros::NodeHandle(this->ns);
  this->colorSequenceServer = this->nh.advertiseService(
    this->topic, &ScanDockScoringPlugin::OnColorSequence, this);

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ScanDockScoringPlugin::Update, this));
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::Update()
{

}

//////////////////////////////////////////////////
bool ScanDockScoringPlugin::ParseSDF(sdf::ElementPtr _sdf)
{
  // Required: The expected color pattern.
  for (auto colorIndex : {"color_1", "color_2", "color_3"})
  {
    if (!_sdf->HasElement(colorIndex))
    {
      ROS_ERROR("<%s> missing", colorIndex);
      return false;
    }

    auto color = _sdf->GetElement(colorIndex)->Get<std::string>();

    // Sanity check: color should be red, green, blue or yellow.
    if (color != "RED"  && color != "GREEN" &&
        color != "BLUE" && color != "YELLOW")
    {
      ROS_ERROR("Invalid color [%s]", color.c_str());
      return false;
    }

    this->expectedSequence.push_back(color);
  }

  // Optional: ROS namespace.
  if (_sdf->HasElement("robot_namespace"))
    this->ns = _sdf->GetElement("robot_namespace")->Get<std::string>();

  // Required if shuffle enabled: ROS topic.
  if (_sdf->HasElement("topic"))
  {
    this->topic = _sdf->GetElement("topic")->Get<std::string>();
  }

  return true;
}

//////////////////////////////////////////////////
bool ScanDockScoringPlugin::OnColorSequence(std_srvs::Trigger::Request &_req,
  std_srvs::Trigger::Response &_res)
{
  ROS_INFO_NAMED("ScanDockScoringPlugin", "Color sequence submitted");

  // _res.message = "New pattern: " + _res.message;
  _res.success = true;
  return _res.success;
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(ScanDockScoringPlugin)
