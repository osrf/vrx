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

#ifndef VMRC_GAZEBO_GAZEBO_ROS_COLOR_HH_
#define VMRC_GAZEBO_GAZEBO_ROS_COLOR_HH_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Visual.hh>
#include <sdf/sdf.hh>

/// \brief Visual plugin for changing the color of some visual elements using
/// ROS messages. This plugin accepts the following SDF parameters:
///
/// <robotNamespace>: The ROS namespace for this node. If not present,
///                   the model name without any "::"" will be used.
///                   E.g.: The plugin under a visual named
///                   "model1::my_submodel::link::visual" will use "model1"
///                   as namespace unless a value is specified.
/// <topicName>: The topic used to request color changes. The default topic
///              name is /color
class GazeboRosColor : public gazebo::VisualPlugin
{
  // Documentation inherited.
  public: void Load(gazebo::rendering::VisualPtr _parent,
                    sdf::ElementPtr _sdf);

  /// \brief Pointer to the visual element to modify.
  private: gazebo::rendering::VisualPtr visual = nullptr;

  /// \brief Subscriber to accept color change requests.
  ros::Subscriber colorSub;

  /// \brief ROS Node handle.
  ros::NodeHandle nh;

  /// Callback for processing color change requests.
  /// \param[in] _msg The message containing the color request.
  void ColorCallback(const std_msgs::ColorRGBAConstPtr &_msg);
};

#endif
