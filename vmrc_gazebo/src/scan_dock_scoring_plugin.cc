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

#include <algorithm>
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

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ScanDockScoringPlugin::Update, this));
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::Update()
{
  {
    // We only allow one color sequence submission.
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->colorSequenceReceived)
      this->colorSequenceServer.shutdown();
  }
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
    std::transform(color.begin(), color.end(), color.begin(), ::tolower);

    // Sanity check: color should be red, green, blue or yellow.
    if (color != "red"  && color != "green" &&
        color != "blue" && color != "yellow")
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
  if (_sdf->HasElement("color_sequence_service"))
  {
    this->colorSequenceService =
      _sdf->GetElement("color_sequence_service")->Get<std::string>();
  }

  return true;
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::OnRunning()
{
  this->colorSequenceServer = this->nh.advertiseService(
    this->colorSequenceService, &ScanDockScoringPlugin::OnColorSequence, this);
}

//////////////////////////////////////////////////
bool ScanDockScoringPlugin::OnColorSequence(
  ros::ServiceEvent<vmrc_gazebo::ColorSequence::Request,
  vmrc_gazebo::ColorSequence::Response> &_event)
{
  ROS_INFO_NAMED("ScanDockScoringPlugin", "Color sequence submission received");

  const vmrc_gazebo::ColorSequence::Request &req = _event.getRequest();
  vmrc_gazebo::ColorSequence::Response &res = _event.getResponse();

  {
    // Sanity check: Only one color sequence submission is allowed.
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->colorSequenceReceived)
    {
      ROS_ERROR("The color sequence has already been submitted");
      res.success = false;
      return false;
    }
  
    this->colorSequenceReceived = true;
  }

  // Sanity check: Make sure that we have the expected color sequence.
  if (this->expectedSequence.size() != 3u)
  {
    res.success = false;
    return false;
  }

  std::string color1 = req.color1;
  std::string color2 = req.color2;
  std::string color3 = req.color3;

  std::transform(color1.begin(), color1.end(), color1.begin(), ::tolower);
  std::transform(color2.begin(), color2.end(), color2.begin(), ::tolower);
  std::transform(color3.begin(), color3.end(), color3.begin(), ::tolower);

  // Incorrect color sequence.
  if (color1 == this->expectedSequence[0] &&
      color2 == this->expectedSequence[1] &&
      color3 == this->expectedSequence[2])
  {
    // Add points for a correct color sequence.
    this->SetScore(this->Score() + 10);
  }

  // The submission is considered correct even if the sequence is wrong.
  res.success = true;
  return true;
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(ScanDockScoringPlugin)
