/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <ignition/math/Rand.hh>

#include "vmrc_gazebo/light_buoy_controller.hh"

const std::array<LightBuoyController::Colors_t, 5> LightBuoyController::kColors
  = {LightBuoyController::Colors_t(CreateColor(1.0, 0.0, 0.0, 1.0), "RED"),
     LightBuoyController::Colors_t(CreateColor(0.0, 1.0, 0.0, 1.0), "GREEN"),
     LightBuoyController::Colors_t(CreateColor(0.0, 0.0, 1.0, 1.0), "BLUE"),
     LightBuoyController::Colors_t(CreateColor(1.0, 1.0, 0.0, 1.0), "YELLOW"),
     LightBuoyController::Colors_t(CreateColor(0.0, 0.0, 0.0, 1.0), "OFF")};

//////////////////////////////////////////////////
std_msgs::ColorRGBA LightBuoyController::CreateColor(const double _r,
  const double _g, const double _b, const double _a)
{
  static std_msgs::ColorRGBA color;
  color.r = _r;
  color.g = _g;
  color.b = _b;
  color.a = _a;
  return color;
}

//////////////////////////////////////////////////
void LightBuoyController::Load(gazebo::physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");

  // Quit if ros plugin was not loaded
  if (!ros::isInitialized())
  {
    ROS_ERROR("ROS was not initialized.");
    return;
  }

  std::string ns = "";
  if (_sdf->HasElement("robotNamespace"))
    ns = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
  {
    ROS_INFO_NAMED("light_bouy_controller",
                   "missing <robotNamespace>, defaulting to %s", ns.c_str());
  }

  this->nh = ros::NodeHandle(ns);

  // Create publisher to set color on each panel
  this->panelPubs[0] = this->nh.advertise<std_msgs::ColorRGBA>("panel1", 1u);
  this->panelPubs[1] = this->nh.advertise<std_msgs::ColorRGBA>("panel2", 1u);
  this->panelPubs[2] = this->nh.advertise<std_msgs::ColorRGBA>("panel3", 1u);

  // Generate random initial pattern
  std::string initial;
  this->ChangePattern(initial);

  this->changePatternServer = this->nh.advertiseService(
    "new_pattern", &LightBuoyController::ChangePattern, this);

  this->timer = this->nh.createTimer(
    ros::Duration(1.0), &LightBuoyController::IncrementState, this);
}

//////////////////////////////////////////////////
void LightBuoyController::IncrementState(const ros::TimerEvent &/*_event*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  // Start over if at end of pattern
  if (this->state > 3)
      this->state = 0;
  auto msg = this->kColors[this->pattern[this->state]].first;
  // Publish current color to each panel
  for (size_t i = 0; i < 3; ++i)
    this->panelPubs[i].publish(msg);
  // Increment index for next timer callback
  ++this->state;
}

//////////////////////////////////////////////////
bool LightBuoyController::ChangePattern(std_srvs::Trigger::Request &_req,
  std_srvs::Trigger::Response &_res)
{
  this->ChangePattern(_res.message);
  _res.message = "New pattern: " + _res.message;
  _res.success = true;
  return _res.success;
}

//////////////////////////////////////////////////
void LightBuoyController::ChangePattern(std::string &_message)
{
  Pattern_t newPattern;
  // Last phase in pattern is always off
  newPattern[3] = 4;
  // Loop until random pattern is different from current one
  do {
    // Generate random sequence of 3 colors among RED, GREEN, BLUE, and YELLOW
    for (size_t i = 0; i < 3; ++i)
      newPattern[i] = ignition::math::Rand::IntUniform(0, 3);
    // Ensure there is no CONSECUTIVE repeats
    while (newPattern[0] == newPattern[1] || newPattern[1] == newPattern[2])
      newPattern[1] = ignition::math::Rand::IntUniform(0, 3);
  } while (newPattern == this->pattern);

  std::lock_guard<std::mutex> lock(this->mutex);
  // Copy newly generated pattern to pattern
  this->pattern = newPattern;
  // Start in OFF state so pattern restarts at beginning
  this->state = 3;
  // Generate string representing pattern, ex: "RGB"
  for (size_t i = 0; i < 3; ++i)
    _message += this->kColors[newPattern[i]].second[0];
  // Log the new pattern
  ROS_INFO_NAMED("light_bouy_controller", "Pattern is %s", _message.c_str());
}

// Register plugin with gazebo
GZ_REGISTER_MODEL_PLUGIN(LightBuoyController)
