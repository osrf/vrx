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

#ifndef VRX_GAZEBO_LIGHT_BUOY_PLUGIN_HH_
#define VRX_GAZEBO_LIGHT_BUOY_PLUGIN_HH_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Trigger.h>

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>

/// \brief Visual plugin for changing the color of some visual elements using
/// ROS messages. This plugin accepts the following SDF parameters:
///
/// <color_1>: The first color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_2>: The second color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_3>: The third color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <shuffle>: True if the topic for shuffling the sequence is enabled.
/// <robot_namespace>: The ROS namespace for this node. If not present,
///                   the model name without any "::"" will be used.
///                   E.g.: The plugin under a visual named
///                   "model1::my_submodel::link::visual" will use "model1"
///                   as namespace unless a value is specified.
/// <topic>: The ROS topic used to request color changes.
/// <visuals>: The collection of visuals that change in color. It accepts N
///            elements of <visual> elements.
///
/// Here's an example:
///   <plugin name="light_buoy_plugin" filename="liblight_buoy_plugin.so">
///     <color_1>RED</color_1>
///     <color_2>GREEN</color_2>
///     <color_3>BLUE</color_3>
///     <visuals>
///       <visual>robotx_light_buoy::base_link::panel_1</visual>
///       <visual>robotx_light_buoy::base_link::panel_2</visual>
///       <visual>robotx_light_buoy::base_link::panel_3</visual>
///     </visuals>
///     <shuffle>true</shuffle>
///     <robot_namespace>vrx</robot_namespace>
///     <topic>light_buoy/shuffle</topic>
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

  /// \brief Return the index of the color from its string.
  /// \param[in] _color The color
  /// \return The index in kColors.
  private: static uint8_t IndexFromColor(const std::string &_color);

  /// \brief Parse all SDF parameters.
  /// \param[in] _sdf SDF elements.
  private: bool ParseSDF(sdf::ElementPtr _sdf);

  /// \brief Callback for change pattern service, calls other changePattern
  /// internaly.
  /// \param[in] _req Not used.
  /// \param[out] _res The Response containing a message with the new pattern.
  /// \return True when the operation succeed or false otherwise.
  private: bool ChangePattern(std_srvs::Trigger::Request &_req,
                              std_srvs::Trigger::Response &_res);

  /// \brief Generate a new pattern and reset state to OFF.
  /// \param[in, out] _message The current pattern in string format, where
  /// each color is represented with its initial letter.
  /// E.g.: "RYG".
  private: void ChangePattern(std::string &_message);

  /// \brief Display the next color in the sequence, or start over if at the end
  private: void Update();

  /// \def Colors_t
  /// \brief A pair of RGBA color and its name as a string.
  private: using Colors_t = std::pair<std_msgs::ColorRGBA, std::string>;

  /// \def Pattern_t
  /// \brief The current pattern to display, pattern_[3] is always OFF.
  private: using Pattern_t = std::array<uint8_t, 4>;

  /// \brief List of the color options (red, green, blue, yellow and no color)
  /// with their string name for logging.
  private: static const std::array<Colors_t, 5> kColors;

  /// \brief Collection of visual names.
  private: std::vector<std::string> visualNames;

  /// \brief Pointer to the visual elements to modify.
  private: std::vector<gazebo::rendering::VisualPtr> visuals;

  /// \brief Whether shuffle is enabled via a ROS topic or not.
  private: bool shuffleEnabled = true;

  /// \brief Service to generate and display a new color sequence.
  private: ros::ServiceServer changePatternServer;

  /// \brief ROS Node handle.
  private: ros::NodeHandle nh;

  /// \brief The color pattern.
  private: Pattern_t pattern;

  /// \brief Track current index in pattern.
  /// \sa IncrementState(const ros::TimerEvent &_event)
  private: uint8_t state = 0u;

  /// \brief ROS namespace.
  private: std::string ns;

  /// \brief ROS topic.
  private: std::string topic;

  /// Pointer to the scene node.
  private: gazebo::rendering::ScenePtr scene;

  /// \brief Connects to rendering update event.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief Timer used to switch colors every second
  private: gazebo::common::Timer timer;

  /// \brief Locks state and pattern member variables.
  private: std::mutex mutex;
};

#endif
