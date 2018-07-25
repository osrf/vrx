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

#ifndef ROBOTX_GAZEBO_SYMBOL_CONTROLLER_HH_
#define ROBOTX_GAZEBO_SYMBOL_CONTROLLER_HH_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Trigger.h>

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>
#include <gazebo/gazebo.hh>

/// \brief Controls the shape and color of a symbol.
///
/// It selects a shape (triangle, cross, circle) and a color (red, green, blue)
/// and applies to the current symbol
/// This plugin can be configured with the following SDF tags:
///
///   <robotNamespace> ROS namespace of Node, can be used to have multiple
///                    plugins.
///   <shape> "triangle", "cross" or "circle". If ommited, the shape will be
///           randomly selected.
///   <color> "red", "green" or "blue". If ommited, the color will be
///           randomly selected.
class SymbolController : public gazebo::ModelPlugin
{
  /// \brief Constructor.
  public: SymbolController() = default;

  // Documentation inherited.
  public: void Load(gazebo::physics::ModelPtr _parent,
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

  /// \def Colors_t
  /// \brief A pair of RGBA color and its name as a string.
  private: using Colors_t = std::pair<std_msgs::ColorRGBA, std::string>;

  /// \brief The current shape to display.
  private: std::string shape;

  /// \brief The current color to display.
  private: Colors_t color;

  /// \brief Publisher to set the color to each of the 3 potential shapes.
  private: std::array<ros::Publisher, 3> symbolPubs;

  /// \brief Node handle.
  private: ros::NodeHandle nh;

  /// \brief Locks state and pattern member variables.
  private: std::mutex mutex;
};

#endif
