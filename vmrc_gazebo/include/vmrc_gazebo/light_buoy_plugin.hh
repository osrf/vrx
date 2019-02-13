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

#ifndef VMRC_GAZEBO_LIGHT_BUOY_PLUGIN_HH_
#define VMRC_GAZEBO_LIGHT_BUOY_PLUGIN_HH_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <array>
#include <map>
#include <vector>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Visual.hh>
#include <sdf/sdf.hh>

/// \brief Visual plugin for changing the color of some visual elements using
/// ROS messages. This plugin accepts the following SDF parameters:
///
/// <color_1>: The first color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_2>: The second color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_3>: The third color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <shuffle>: True if the topic for shuffling the sequence is enabled. 
/// <robotNamespace>: The ROS namespace for this node. If not present,
///                   the model name without any "::"" will be used.
///                   E.g.: The plugin under a visual named
///                   "model1::my_submodel::link::visual" will use "model1"
///                   as namespace unless a value is specified.
/// <topicName>: The ROS topic used to request color changes.
/// <visuals>: The collection of visuals that change in color. It accepts N
///            elements of <visual> elements.
///
/// Here's an example:
///   <plugin name="panel1plugin" filename="libgazebo_ros_color.so">
///     <color_1>RED</color_1>
///     <color_2>GREEN</color_2>
///     <color_3>BLUE</color_3>
///     <visuals>
///       <visual>panel_1</visual>
///       <visual>panel_2</visual>
///       <visual>panel_3</visual>
///     </visuals>
///     <shuffle>true</shuffle>
///     <robotNamespace>light_buoy</robotNamespace>
///     <topicName>light_buoy/shuffle</topicName>
///   </plugin>
class LightBuoyPlugin : public gazebo::VisualPlugin
{
  // Documentation inherited.
  public: void Load(gazebo::rendering::VisualPtr _parent,
                    sdf::ElementPtr _sdf);

  /// \brief Creates a std_msgs::ColorRGBA message from 4 doubles.
  /// \param[in] _r Red.
  /// \param[in] _g Green.
  /// \param[in] _b Blue.
  /// \param[in] _a Alpha.
  /// \return The ColorRGBA message.
  private: static std_msgs::ColorRGBA CreateColor(const double _r,
                                                  const double _g,
                                                  const double _b,
                                                  const double _a);

  /// \brief Parse all SDF parameters.
  /// \param[in] _sdf SDF elements.
  private: bool ParseSDF(sdf::ElementPtr _sdf);

  /// Callback for processing color change requests.
  /// \param[in] _msg The message containing the color request.
  private: void ShuffleCallback(const std_msgs::ColorRGBAConstPtr &_msg);

  /// \def Colors_t
  /// \brief A pair of RGBA color and its name as a string.
  private: using Colors_t = std::map<std::string, std_msgs::ColorRGBA>;

  /// \def Pattern_t
  /// \brief The current pattern to display, pattern_[3] is always OFF.
  private: using Pattern_t = std::array<std_msgs::ColorRGBA, 4>;

  /// \brief List of the color options (red, green, blue, yellow and no color)
  /// with their string name for logging.
  private: static Colors_t kColors;

  /// \brief Pointer to the visual elements to modify.
  private: std::vector<gazebo::rendering::VisualPtr> visuals;

  /// \brief Whether shuffle is enabled via a ROS topic or not.
  private: bool shuffleEnabled = true;

  /// \brief Subscriber to accept color change requests.
  private: ros::Subscriber shuffleSub;

  /// \brief ROS Node handle.
  private: ros::NodeHandle nh;

  /// \brief The color pattern.
  private: Pattern_t pattern;

  /// Pointer to the scene node.
  private: gazebo::rendering::ScenePtr scene;
};

#endif
