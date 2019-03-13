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
#include <gazebo/rendering/Scene.hh>
#include <ignition/math/Rand.hh>
#include "vrx_gazebo/light_buoy_plugin.hh"

const std::array<LightBuoyPlugin::Colors_t, 5> LightBuoyPlugin::kColors
  = {LightBuoyPlugin::Colors_t(CreateColor(1.0, 0.0, 0.0, 1.0), "red"),
     LightBuoyPlugin::Colors_t(CreateColor(0.0, 1.0, 0.0, 1.0), "green"),
     LightBuoyPlugin::Colors_t(CreateColor(0.0, 0.0, 1.0, 1.0), "blue"),
     LightBuoyPlugin::Colors_t(CreateColor(1.0, 1.0, 0.0, 1.0), "yellow"),
     LightBuoyPlugin::Colors_t(CreateColor(0.0, 0.0, 0.0, 1.0), "off")};

//////////////////////////////////////////////////
std_msgs::ColorRGBA LightBuoyPlugin::CreateColor(const double _r,
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
uint8_t LightBuoyPlugin::IndexFromColor(const std::string &_color)
{
  uint8_t index = 0u;
  for (auto color : kColors)
  {
    if (_color == color.second)
      return index;

    ++index;
  }

  return std::numeric_limits<uint8_t>::max();
}

//////////////////////////////////////////////////
void LightBuoyPlugin::Load(gazebo::rendering::VisualPtr _parent,
  sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");

  this->scene = _parent->GetScene();
  GZ_ASSERT(this->scene != nullptr, "NULL scene");

  // This is important to disable the visual plugin running inside the GUI.
  if (!this->scene->EnableVisualizations())
    return;

  if (!this->ParseSDF(_sdf))
    return;

  // Quit if ros plugin was not loaded
  if (!ros::isInitialized())
  {
    ROS_ERROR("ROS was not initialized.");
    return;
  }

  if (this->shuffleEnabled)
  {
    this->nh = ros::NodeHandle(this->ns);
    this->changePatternServer = this->nh.advertiseService(
      this->topic, &LightBuoyPlugin::ChangePattern, this);
  }

  this->timer.Start();

  this->updateConnection = gazebo::event::Events::ConnectPreRender(
    std::bind(&LightBuoyPlugin::Update, this));
}

//////////////////////////////////////////////////
bool LightBuoyPlugin::ParseSDF(sdf::ElementPtr _sdf)
{
  // Required: Sequence of colors.
  uint8_t i = 0u;
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

    this->pattern[i++] = IndexFromColor(color);
  }

  // The last color of the pattern is always black.
  this->pattern[3] = IndexFromColor("off");

  // Required: visuals.
  if (!_sdf->HasElement("visuals"))
  {
    ROS_ERROR("<visuals> missing");
    return false;
  }

  auto visualsElem = _sdf->GetElement("visuals");
  if (!visualsElem->HasElement("visual"))
  {
    ROS_ERROR("<visual> missing");
    return false;
  }

  auto visualElem = visualsElem->GetElement("visual");
  while (visualElem)
  {
    std::string visualName = visualElem->Get<std::string>();
    this->visualNames.push_back(visualName);
    visualElem = visualElem->GetNextElement();
  }

  // Optional: Is shuffle enabled?
  if (_sdf->HasElement("shuffle"))
  {
    this->shuffleEnabled = _sdf->GetElement("shuffle")->Get<bool>();

    // Required if shuffle enabled: ROS topic.
    if (!_sdf->HasElement("topic"))
    {
      ROS_ERROR("<topic> missing");
    }
    this->topic = _sdf->GetElement("topic")->Get<std::string>();
  }

  // Optional: ROS namespace.
  if (_sdf->HasElement("robot_namespace"))
    this->ns = _sdf->GetElement("robot_namespace")->Get<std::string>();

  return true;
}

//////////////////////////////////////////////////
void LightBuoyPlugin::Update()
{
  // Get the visuals if needed.
  if (this->visuals.empty())
  {
    for (auto visualName : this->visualNames)
    {
      auto visualPtr = this->scene->GetVisual(visualName);
      if (visualPtr)
        this->visuals.push_back(visualPtr);
      else
        ROS_ERROR("Unable to find [%s] visual", visualName.c_str());
    }
  }

  if (this->timer.GetElapsed() < gazebo::common::Time(1.0))
    return;

  // Restart the timer.
  this->timer.Reset();
  this->timer.Start();

  std::lock_guard<std::mutex> lock(this->mutex);

  // Start over if at end of pattern
  if (this->state > 3)
    this->state = 0;

  auto color = this->kColors[this->pattern[this->state]].first;
  #if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Color gazeboColor(color.r, color.g, color.b, color.a);
  #else 
    gazebo::common::Color gazeboColor(color.r, color.g, color.b, color.a);
  #endif
  // Update the visuals.
  for (auto visual : this->visuals)
  {
    visual->SetAmbient(gazeboColor);
    visual->SetDiffuse(gazeboColor);
  }

  // Increment index for next timer callback
  ++this->state;
}

//////////////////////////////////////////////////
bool LightBuoyPlugin::ChangePattern(std_srvs::Trigger::Request &_req,
  std_srvs::Trigger::Response &_res)
{
  this->ChangePattern(_res.message);
  _res.message = "New pattern: " + _res.message;
  _res.success = true;
  return _res.success;
}

//////////////////////////////////////////////////
void LightBuoyPlugin::ChangePattern(std::string &_message)
{
  Pattern_t newPattern;
  // Last phase in pattern is always off
  newPattern[3] = IndexFromColor("off");

  // Loop until random pattern is different from current one
  do {
    // Generate random sequence of 3 colors among RED, GREEN, BLUE, and YELLOW
    for (size_t i = 0; i < 3; ++i)
      newPattern[i] = ignition::math::Rand::IntUniform(0, 3);
    // Ensure there is no consecutive repeats
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
  ROS_INFO_NAMED("LightBuoyPlugin", "Pattern is %s", _message.c_str());
}

// Register plugin with gazebo
GZ_REGISTER_VISUAL_PLUGIN(LightBuoyPlugin)
