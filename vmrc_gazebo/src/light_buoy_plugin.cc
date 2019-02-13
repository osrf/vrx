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

#include <gazebo/rendering/Scene.hh>
#include <ignition/math/Rand.hh>

#include "vmrc_gazebo/light_buoy_plugin.hh"

LightBuoyPlugin::Colors_t LightBuoyPlugin::kColors =
{
  {"RED",    CreateColor(1.0, 0.0, 0.0, 1.0)},
  {"GREEN",  CreateColor(0.0, 1.0, 0.0, 1.0)},
  {"BLUE",   CreateColor(0.0, 0.0, 1.0, 1.0)},
  {"YELLOW", CreateColor(1.0, 1.0, 0.0, 1.0)},
  {"OFF",    CreateColor(0.0, 0.0, 0.0, 1.0)},
};

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
void LightBuoyPlugin::Load(gazebo::rendering::VisualPtr _parent,
  sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");
  
  this->scene = _parent->GetScene();
  GZ_ASSERT(this->scene != nullptr, "NULL scene");

  if (!this->ParseSDF(_sdf))
    return;

  // // Quit if ros plugin was not loaded
  // if (!ros::isInitialized())
  // {
  //   ROS_ERROR("ROS was not initialized.");
  //   return;
  // }

  // std::string ns = "";
  // if (_sdf->HasElement("robotNamespace"))
  //   ns = _sdf->GetElement("robotNamespace")->Get<std::string>();
  // else
  // {
  //   ROS_INFO_NAMED("light_bouy_plugin",
  //                  "missing <robotNamespace>, defaulting to %s", ns.c_str());
  // }

  // this->nh = ros::NodeHandle(ns);

  // // Create publisher to set color on each panel
  // this->panelPubs[0] = this->nh.advertise<std_msgs::ColorRGBA>("panel1", 1u);
  // this->panelPubs[1] = this->nh.advertise<std_msgs::ColorRGBA>("panel2", 1u);
  // this->panelPubs[2] = this->nh.advertise<std_msgs::ColorRGBA>("panel3", 1u);

  // // Generate random initial pattern
  // std::string initial;
  // this->ChangePattern(initial);

  // this->changePatternServer = this->nh.advertiseService(
  //   "new_pattern", &LightBuoyController::ChangePattern, this);

  // this->timer = this->nh.createTimer(
  //   ros::Duration(1.0), &LightBuoyController::IncrementState, this);
}

//////////////////////////////////////////////////
bool LightBuoyPlugin::ParseSDF(sdf::ElementPtr _sdf)
{
  // Parse the sequence of colors.
  uint8_t i = 0u;
  for (auto colorIndex : {"color_1", "color_2", "color_3"})
  {
    if (!_sdf->HasElement(colorIndex))
    {
      ROS_ERROR("<%s> missing", colorIndex);
      return false;
    }

    auto color = _sdf->GetElement(colorIndex)->Get<std::string>();

    // Sanity check: color should be red, green, blue or yellow.
    if (color != "RED" && color != "GREEN" &&
        color != "BLUE" && color != "YELLOW")
    {
      ROS_ERROR("Invalid color [%s]", color.c_str());
      return false;
    }

    this->pattern[i] = kColors[color];
  }

  // Parse visuals.
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
    auto visualPtr = this->scene->GetVisual(visualName);
    if (visualPtr)
      this->visuals.push_back(visualPtr);      
    else
      ROS_ERROR("Unable to find [%s] visual", visualName.c_str());

    visualElem = visualElem->GetNextElement();
  }
}

// Register plugin with gazebo
GZ_REGISTER_VISUAL_PLUGIN(LightBuoyPlugin)
