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
#include <gazebo/common/Console.hh>
#include "vrx_gazebo/scan_dock_scoring_plugin.hh"

/////////////////////////////////////////////////
ColorSequenceChecker::ColorSequenceChecker(
  const std::vector<std::string> &_expectedColors,
  const std::string &_rosColorSequenceService,
  gazebo_ros::Node::SharedPtr _node)
  : expectedSequence(_expectedColors),
    colorSequenceService(_rosColorSequenceService),
    node(_node)
{
  // Quit if ros plugin was not loaded
  if (!rclcpp::ok())
  {
    RCLCPP_ERROR(node->get_logger(), "ROS was not initialized.");
    return;
  }
}

/////////////////////////////////////////////////
void ColorSequenceChecker::Enable()
{
  this->colorSequenceServer = this->node->create_service<vrx_gazebo::srv::ColorSequence>
      (this->colorSequenceService, 
      std::bind(&ColorSequenceChecker::OnColorSequence, this, std::placeholders::_1, std::placeholders::_2));
}

/////////////////////////////////////////////////
void ColorSequenceChecker::Disable()
{
  this->colorSequenceServer.reset();
}

/////////////////////////////////////////////////
bool ColorSequenceChecker::SubmissionReceived() const
{
  return this->colorSequenceReceived;
}

/////////////////////////////////////////////////
bool ColorSequenceChecker::Correct() const
{
  return this->correctSequence;
}

/////////////////////////////////////////////////
bool ColorSequenceChecker::OnColorSequence(
  const std::shared_ptr<vrx_gazebo::srv::ColorSequence::Request> req,
          std::shared_ptr<vrx_gazebo::srv::ColorSequence::Response> res)
{
  RCLCPP_INFO(node->get_logger(), "Color sequence submission received");

  {
    // Sanity check: Only one color sequence submission is allowed.
    if (this->colorSequenceReceived)
    {
      RCLCPP_ERROR(node->get_logger(), "The color sequence has already been submitted");
      res->success = false;
      return false;
    }

    this->colorSequenceReceived = true;
  }

  // Sanity check: Make sure that we have the expected color sequence.
  if (this->expectedSequence.size() != 3u)
  {
    RCLCPP_ERROR(node->get_logger(), "The color sequence is not of size 3 - will be ignored.");
    res->success = false;
    return false;
  }

  std::string color1 = req->color1;
  std::string color2 = req->color2;
  std::string color3 = req->color3;

  std::transform(color1.begin(), color1.end(), color1.begin(), ::tolower);
  std::transform(color2.begin(), color2.end(), color2.begin(), ::tolower);
  std::transform(color3.begin(), color3.end(), color3.begin(), ::tolower);

  // Incorrect color sequence.
  this->correctSequence =
      color1 == this->expectedSequence[0] &&
      color2 == this->expectedSequence[1] &&
      color3 == this->expectedSequence[2];
  if (this->correctSequence)
  {
    RCLCPP_INFO(node->get_logger(), "Received color sequence is "
      "correct.  Additional points will be scored.");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Received color sequence is "
      "not correct. No additional points.");
  }

  // The submission is considered correct even if the sequence is wrong.
  res->success = true;
  return true;
}

/////////////////////////////////////////////////
DockChecker::DockChecker(const std::string &_name,
  const std::string &_internalActivationTopic,
  const std::string &_externalActivationTopic,
  const double _minDockTime,
  const bool _dockAllowed, const std::string &_worldName,
  const std::string &_announceSymbol,
  const std::string &_gzSymbolTopic,
  gazebo_ros::Node::SharedPtr _node)
  : name(_name),
    internalActivationTopic(_internalActivationTopic),
    externalActivationTopic(_externalActivationTopic),
    minDockTime(_minDockTime),
    dockAllowed(_dockAllowed),
    gzSymbolTopic(_gzSymbolTopic),
    node(_node)
{
  this->timer.Stop();
  this->timer.Reset();

  this->announceSymbol.data = _announceSymbol;

  this->ignNode.Subscribe(this->internalActivationTopic,
    &DockChecker::OnInternalActivationEvent, this);
  this->ignNode.Subscribe(this->externalActivationTopic,
    &DockChecker::OnExternalActivationEvent, this);
}

/////////////////////////////////////////////////
bool DockChecker::AnytimeDocked() const
{
  return this->anytimeDocked;
}

/////////////////////////////////////////////////
bool DockChecker::AtEntrance() const
{
  return this->atEntrance;
}

/////////////////////////////////////////////////
bool DockChecker::Allowed() const
{
  return this->dockAllowed;
}

/////////////////////////////////////////////////
void DockChecker::AnnounceSymbol()
{
  // Override the docks own sdf parameters
  this->dockPlacardPub = this->node->create_publisher
    <dock_placard_msgs::msgs::DockPlacard>(gzSymbolTopic, 1);
  dock_placard_msgs::msgs::DockPlacard symbol;
  symbol.set_color(announceSymbol.data.substr
    (0, announceSymbol.data.find("_")));
  symbol.set_shape(announceSymbol.data.substr
    (announceSymbol.data.find("_")+1));
  this->dockPlacardPub->publish(symbol);

  if (this->dockAllowed)
  {
    this->symbolPub =
      this->node->create_publisher<std_msgs::msg::String>(this->symbolTopic, 1);

    this->symbolPub->publish(this->announceSymbol);
  }
}

/////////////////////////////////////////////////
void DockChecker::Update()
{
  if (this->anytimeDocked)
    return;

  this->anytimeDocked =
    this->timer.GetElapsed() >= gazebo::common::Time(this->minDockTime);

  if (this->anytimeDocked)
  {
    gzmsg  << "Successfully stayed in dock for " << this->minDockTime
           << " seconds, transitioning to <docked> state" << std::endl;
  }
}

/////////////////////////////////////////////////
void DockChecker::OnInternalActivationEvent(const ignition::msgs::Boolean &_msg)
{
  // Currently docked.
  if (_msg.data() == 1)
  {
    this->timer.Start();
    gzmsg << "Entering internal dock activation zone, transitioning to "
          << "<docking> state in [" << this->name << "]." << std::endl;
  }

  // Currently undocked.
  if (_msg.data() == 0)
  {
    this->timer.Stop();
    this->timer.Reset();
    if (this->AnytimeDocked())
    {
      gzmsg << "Leaving internal dock activation zone in [" << this->name
            << "] after required time - transitioning to <exited> state."
            << std::endl;
    }
    else
    {
      gzmsg << "Leaving internal dock activation zone in [" << this->name
            << "] early - transitioning back to <undocked> state."
            << std::endl;
    }
  }

  gzdbg << "[" << this->name << "] OnInternalActivationEvent(): "
        << _msg.data() << std::endl;
}

/////////////////////////////////////////////////
void DockChecker::OnExternalActivationEvent(const ignition::msgs::Boolean &_msg)
{
  this->atEntrance = _msg.data() == 1;

  if (this->atEntrance)
  {
    gzmsg << "Entering external dock activation zone in [" << this->name
          << "]" << std::endl;
  }
  else
  {
    gzmsg << "Leaving external dock activation zone in [" << this->name
          << "]" << std::endl;
  }

  gzdbg << "[" << this->name << "] OnExternalActivationEvent(): "
        << _msg.data() << std::endl;
}

//////////////////////////////////////////////////
ScanDockScoringPlugin::ScanDockScoringPlugin()
{
}

/////////////////////////////////////////////////
void ScanDockScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  ScoringPlugin::Load(_world, _sdf);

  gzmsg << "Task [" << this->TaskName() << "]" << std::endl;

  if (!this->ParseSDF(_sdf))
    return;

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ScanDockScoringPlugin::Update, this));

  this->lightBuoySequencePub = this->node->create_publisher
    <light_buoy_colors_msgs::msgs::LightBuoyColors>(this->colorTopic, 10);
}

//////////////////////////////////////////////////
bool ScanDockScoringPlugin::ParseSDF(sdf::ElementPtr _sdf)
{
  // Optional: ROS namespace.
  std::string ns;
  if (_sdf->HasElement("robot_namespace"))
    ns = _sdf->GetElement("robot_namespace")->Get<std::string>();

  // Enable color checker - default is true
  this->enableColorChecker = true;
  if (_sdf->HasElement("enable_color_checker"))
  {
    enableColorChecker =
      _sdf->GetElement("enable_color_checker")->Get<bool>();
  }
  // Optional: ROS service.
  std::string colorSequenceService = "/vrx/scan_dock/color_sequence";
  if (_sdf->HasElement("color_sequence_service"))
  {
    colorSequenceService =
      _sdf->GetElement("color_sequence_service")->Get<std::string>();
  }

  // Required: The expected color pattern.
  for (auto colorIndex : {"color_1", "color_2", "color_3"})
  {
    if (!_sdf->HasElement(colorIndex))
    {
      RCLCPP_ERROR(node->get_logger(), "<%s> missing", colorIndex);
      return false;
    }

    auto color = _sdf->GetElement(colorIndex)->Get<std::string>();
    std::transform(color.begin(), color.end(), color.begin(), ::tolower);

    // Sanity check: color should be red, green, blue or yellow.
    if (color != "red"  && color != "green" &&
        color != "blue" && color != "yellow")
    {
      RCLCPP_ERROR(node->get_logger(), "Invalid color [%s]", color.c_str());
      return false;
    }
    this->expectedSequence.push_back(color);
  }

  // Optional: gazebo topic where light buoy sequence is published
  if (!_sdf->HasElement("color_topic"))
  {
    this->colorTopic = "/vrx/light_buoy/new_pattern";
  }
  else
  {
    this->colorTopic = _sdf->GetElement("color_topic")->Get<std::string>();
  }

  // Optional: the points granted when reported the correct color sequence.
  if (_sdf->HasElement("color_bonus_points"))
  {
    this->colorBonusPoints =
      _sdf->GetElement("color_bonus_points")->Get<double>();
  }

  // Instantiate the color checker.
  if (this->enableColorChecker)
  {
    this->colorChecker.reset(
      new ColorSequenceChecker(this->expectedSequence, 
                                colorSequenceService, node));
  }

  // Required: Parse the bays.
  if (!_sdf->HasElement("bays"))
  {
    RCLCPP_ERROR(node->get_logger(), "<bays> missing");
    return false;
  }

  auto baysElem = _sdf->GetElement("bays");
  if (!baysElem->HasElement("bay"))
  {
    RCLCPP_ERROR(node->get_logger(), "<bay> missing");
    return false;
  }

  auto bayElem = baysElem->GetElement("bay");
  while (bayElem)
  {
    // Required: bay name.
    if (!bayElem->GetElement("name"))
    {
      RCLCPP_ERROR(node->get_logger(), "<bays::bay::name> missing");
      return false;
    }
    std::string bayName = bayElem->Get<std::string>("name");

    // Required: internal_activation topic.
    if (!bayElem->GetElement("internal_activation_topic"))
    {
      RCLCPP_ERROR(node->get_logger(), "<bays::bay::internal_activation_topic> missing");
      return false;
    }
    std::string internalActivationTopic =
      bayElem->Get<std::string>("internal_activation_topic");

    // Required: external_activation topic.
    if (!bayElem->GetElement("external_activation_topic"))
    {
      RCLCPP_ERROR(node->get_logger(), "<bays::bay::external_activation_topic> missing");
      return false;
    }
    std::string externalActivationTopic =
      bayElem->Get<std::string>("external_activation_topic");

    // Required: gazebo symbol topic.
    if (!bayElem->GetElement("symbol_topic"))
    {
      RCLCPP_ERROR(node->get_logger(), "<bays::bay::symbol_topic> missing");
      return false;
    }
    std::string symbolTopic = bayElem->Get<std::string>("symbol_topic");

    // Required: minimum time to be considered "docked".
    if (!bayElem->GetElement("min_dock_time"))
    {
      RCLCPP_ERROR(node->get_logger(), "<bays::bay::min_dock_time> missing");
      return false;
    }
    double minDockTime = bayElem->Get<double>("min_dock_time");

    // Required: dock allowed.
    if (!bayElem->GetElement("dock_allowed"))
    {
      RCLCPP_ERROR(node->get_logger(), "<bays::bay::dock_allowed> missing");
      return false;
    }
    bool dockAllowed = bayElem->Get<bool>("dock_allowed");

    std::string announceSymbol = "";
    if (!bayElem->HasElement("symbol"))
    {
      RCLCPP_ERROR(node->get_logger(), "<bays::bay::symbol> not found");
    }
    announceSymbol =
      bayElem->GetElement("symbol")->Get<std::string>();


    // Create a new dock checker.
    std::unique_ptr<DockChecker> dockChecker(
      new DockChecker(bayName, internalActivationTopic,
        externalActivationTopic, minDockTime, dockAllowed,
        this->world->Name(), 
        announceSymbol, symbolTopic, node));

    // Add the dock checker.
    this->dockCheckers.push_back(std::move(dockChecker));

    // Process the next checker.
    bayElem = bayElem->GetNextElement();
  }

  // Optional: the points granted when the vehicle docks in any bay.
  if (_sdf->HasElement("dock_bonus_points"))
  {
    this->dockBonusPoints =
      _sdf->GetElement("dock_bonus_points")->Get<double>();
  }

  // Optional: the points granted when the vehicle docks in the expected bay.
  if (_sdf->HasElement("correct_dock_bonus_points"))
  {
    this->correctDockBonusPoints =
      _sdf->GetElement("correct_dock_bonus_points")->Get<double>();
  }

  // Optional: the shooting targets.
  if (_sdf->HasElement("targets"))
  {
    // We require at least one <target> element.
    auto targetsElem = _sdf->GetElement("targets");
    if (!targetsElem->HasElement("target"))
    {
      RCLCPP_ERROR(node->get_logger(), "<targets><target> not found");
      return false;
    }

    auto targetElem = targetsElem->GetElement("target");
    while (targetElem)
    {
      if (!targetElem->HasElement("topic"))
      {
        RCLCPP_ERROR(node->get_logger(), "<targets><target><topic> not found");
        return false;
      }
      std::string topic = targetElem->Get<std::string>("topic");

      if (!targetElem->HasElement("bonus_points"))
      {
        RCLCPP_ERROR(node->get_logger(), "<targets><target><bonus_points> not found");
        return false;
      }
      double bonusPoints = targetElem->Get<double>("bonus_points");

#if GAZEBO_MAJOR_VERSION >= 8
      std::function<void(const ignition::msgs::Boolean&)> subCb =
        [this, bonusPoints](const ignition::msgs::Boolean &_msg)
      {
        // The projectile hit the target!
        if (_msg.data())
        {
          std::lock_guard<std::mutex> lock(this->mutex);
          this->shootingBonus = bonusPoints;
        }
      };

      this->ignNode.Subscribe(topic, subCb);
#endif

      // Process the next target.
      targetElem = targetElem->GetNextElement();
    }
  }

  return true;
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::Update()
{
  if (this->enableColorChecker)
  {
    // Verify the color checker.
    if (!this->colorSubmissionProcessed &&
        this->colorChecker->SubmissionReceived())
    {
      // We need to decide if we grant extra points.
      if (this->colorChecker->Correct())
      {
        gzmsg << "Adding <" << this->colorBonusPoints << "> points for correct "
          << "reporting of color sequence" << std::endl;
        this->SetScore(this->Score() + this->colorBonusPoints);
      }

      // We only allow one color sequence submission.
      this->colorChecker->Disable();
      this->colorSubmissionProcessed = true;
    }
  }

  // Verify the dock checkers.
  for (auto &dockChecker : this->dockCheckers)
  {
    // We always need to update the checkers.
    dockChecker->Update();

    // Nothing to do if nobody ever docked or we're still inside the bay.
    if (!dockChecker->AnytimeDocked() || !dockChecker->AtEntrance())
      continue;

    // Points granted for docking!
    this->SetScore(this->Score() + this->dockBonusPoints);
    if (this->TaskState() == "running")
    {
    gzmsg  << "Successfully docked in [" << dockChecker->name << "]"
      << ". Awarding " << this->dockBonusPoints << " points." <<std::endl;
    }

    // Is this the right bay?
    if (dockChecker->Allowed())
    {
      this->SetScore(this->Score() + this->correctDockBonusPoints);
      if (this->TaskState() == "running")
      {
        gzmsg << "Docked in correct dock [" << dockChecker->name << "]"
              << ". Awarding " << this->correctDockBonusPoints
              << " more points." << std::endl;
      }
    }
    else
    {
      if (this->TaskState() == "running")
      {
        gzmsg  << "Docked in incorrect dock [" << dockChecker->name << "]"
        << ". No additional points." <<std::endl;
      }
    }

    // Time to finish the task as the vehicle docked.
    // Note that we only allow to dock one time. This is to prevent teams
    // docking in all possible bays.
    this->Finish();
    break;
  }

  // Check the shooting targets.
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->shootingBonus > 0)
    {
      this->SetScore(this->Score() + this->shootingBonus);
      shootingBonus = 0;
    }
  }
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::OnReady()
{
  gzmsg << "OnReady" << std::endl;
  // Announce the symbol if needed.
  for (auto &dockChecker : this->dockCheckers)
    dockChecker->AnnounceSymbol();
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::OnRunning()
{
  gzmsg << "OnRunning" << std::endl;
  light_buoy_colors_msgs::msgs::LightBuoyColors colors;
  colors.set_color_1(this->expectedSequence[0]);
  colors.set_color_2(this->expectedSequence[1]);
  colors.set_color_3(this->expectedSequence[2]);
  lightBuoySequencePub->publish(colors);

  if (this->enableColorChecker)
  {
    this->colorChecker->Enable();
  }
  // Announce the symbol if needed.
  for (auto &dockChecker : this->dockCheckers)
    dockChecker->AnnounceSymbol();
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(ScanDockScoringPlugin)
