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

#include <string>

#include "vmrc_gazebo/gazebo_ros_color.hh"

//////////////////////////////////////////////////
void GazeboRosColor::Load(gazebo::rendering::VisualPtr _parent,
  sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");

  // Quit if ROS plugin was not loaded
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("gazebo_ros_color_plugin", "A ROS node for Gazebo "
      "has not been initialized, unable to load plugin. " << "Load the Gazebo "
      "system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Load namespace from SDF if available. Otherwise, use the model name.
  std::string modelName = _parent->GetName();
  auto delim = modelName.find(":");
  if (delim != std::string::npos)
    modelName = modelName.substr(0, delim);

  std::string ns = modelName;
  if (_sdf->HasElement("robotNamespace"))
    ns = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
  {
    ROS_DEBUG_NAMED("gazebo_ros_color_plugin",
      "missing <robotNamespace>, defaulting to %s", ns.c_str());
  }

  // Load topic from sdf if available
  std::string topicName = "color";
  if (_sdf->HasElement("topicName"))
    topicName = _sdf->GetElement("topicName")->Get<std::string>();
  else
  {
    ROS_INFO_NAMED("gazebo_ros_color_plugin",
      "missing <topicName>, defaulting to %s", topicName.c_str());
  }

  // Copy parent into class for use in callback
  this->visual = _parent;

  // Setup node handle and subscriber
  this->nh = ros::NodeHandle(ns);
  this->colorSub = this->nh.subscribe(topicName, 1,
    &GazeboRosColor::ColorCallback, this);
}

//////////////////////////////////////////////////
void GazeboRosColor::ColorCallback(const std_msgs::ColorRGBAConstPtr &_msg)
{
  // Convert ROS color to gazebo color
  gazebo::common::Color gazebo_color(_msg->r, _msg->g, _msg->b, _msg->a);

  // Set parent's color to message color
  this->visual->SetAmbient(gazebo_color);
  this->visual->SetDiffuse(gazebo_color);
}

// Register plugin with gazebo
GZ_REGISTER_VISUAL_PLUGIN(GazeboRosColor)
