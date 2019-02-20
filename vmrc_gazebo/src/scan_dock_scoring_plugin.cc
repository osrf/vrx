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

  // Subscriber to receive world updates (e.g.: a notification after a cloning).
  this->node.reset(new gazebo::transport::Node());
  this->node->Init(this->world->GetName());

  this->timer1.Stop();
  this->timer1.Reset();
  this->timer2.Stop();
  this->timer2.Reset();

  this->containSub1 = this->node->Subscribe(this->bay1Topic,
    &ScanDockScoringPlugin::OnActivationZoneBay1, this);
  this->containSub2 = this->node->Subscribe(this->bay2Topic,
    &ScanDockScoringPlugin::OnActivationZoneBay2, this);

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ScanDockScoringPlugin::Update, this));
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

  // Required parameter.
  if (!_sdf->HasElement("bay1_topic"))
  {
    ROS_ERROR("<bay1_topic> missing");
    return false;
  }
  this->bay1Topic = _sdf->GetElement("bay1_topic")->Get<std::string>();

  // Required parameter.
  if (!_sdf->HasElement("bay2_topic"))
  {
    ROS_ERROR("<bay2_topic> missing");
    return false;
  }
  this->bay2Topic = _sdf->GetElement("bay2_topic")->Get<std::string>();

  // Required parameter..
  if (!_sdf->HasElement("correct_bay_topic"))
  {
    ROS_ERROR("<correct_bay_topic> missing");
    return false;
  }
  this->correctBayTopic =
    _sdf->GetElement("correct_bay_topic")->Get<std::string>();

  // Sanity check: Make sure that the correct bay topic matches an existing
  // bay topic.
  if (this->correctBayTopic != this->bay1Topic &&
      this->correctBayTopic != this->bay2Topic)
  {
    ROS_ERROR("<correct_bay_topic> should match <bay1_topic> or <bay2_topic>");
    return false;
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
void ScanDockScoringPlugin::Update()
{
  // We only allow one color sequence submission.
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->colorSequenceReceived)
    this->colorSequenceServer.shutdown();

  // Check whether the vehicle docked.
  if (this->timer1.GetElapsed() >= gazebo::common::Time(10.0))
  {
    if (this->correctBayTopic == this->bay1Topic)
      this->SetScore(this->Score() + 10);

    this->Finish();
  }

  if (this->timer2.GetElapsed() >= gazebo::common::Time(10.0))
  {
    if (this->correctBayTopic == this->bay2Topic)
      this->SetScore(this->Score() + 10);

    this->Finish();
  }
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::OnRunning()
{
  this->colorSequenceServer = this->nh.advertiseService(
    this->colorSequenceService, &ScanDockScoringPlugin::OnColorSequence, this);
}

/////////////////////////////////////////////////
void ScanDockScoringPlugin::OnActivationZoneBay1(ConstIntPtr &_msg)
{
  if (_msg->data() == 1)
    this->timer1.Start();

  if (_msg->data() == 0)
  {
    this->timer1.Stop();
    this->timer1.Reset();
  }

  gzmsg << "OnActivationZoneBay1(): " << _msg->data() << std::endl;
}

/////////////////////////////////////////////////
void ScanDockScoringPlugin::OnActivationZoneBay2(ConstIntPtr &_msg)
{
  if (_msg->data() == 1)
    this->timer2.Start();

  if (_msg->data() == 0)
  {
    this->timer2.Stop();
    this->timer2.Reset();
  }

  gzmsg << "OnActivationZoneBay2(): " << _msg->data() << std::endl;
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
