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

#ifndef VMRC_GAZEBO_SYMBOL_CONTROLLER_HH_
#define VMRC_GAZEBO_SYMBOL_CONTROLLER_HH_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Trigger.h>

#include <map>
#include <string>
#include <vector>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>

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
  public: virtual void Load(gazebo::physics::ModelPtr _parent,
                            sdf::ElementPtr _sdf);

  /// \brief Create a new combination of shape/color.
  public: void Shuffle();

  /// \brief ROS service callback for creating a new shape/color combination.
  /// \param[in] _req Unused.
  /// \param[out] _res Service result.
  /// \return True on success or false otherwise.
  private: bool Shuffle(std_srvs::Trigger::Request &_req,
                        std_srvs::Trigger::Response &_res);

  /// \brief Choose a new random shape.
  private: void ShuffleShape();

  /// \brief Choose a new random color.
  private: void ShuffleColor();

  /// \brief Publish the current shape/color combination.
  /// This callback is used for publishing the current combination after some
  /// time to make sure that all subscribers are up and runnig.
  /// \param[in] _event Unused.
  private: void Publish(const ros::TimerEvent &_event);

  /// \brief Publish the current shape/color combination.
  private: void Publish();

  /// \brief List of the color options (red, green, blue, and no color)
  /// with their string name for logging.
  private: static std::map<std::string, std_msgs::ColorRGBA> kColors;

  /// \brief List of the shape options (circle, cross, triangle)
  /// with their string name for logging.
  private: static std::vector<std::string> kShapes;

  /// \brief The current color.
  private: std::string color;

  /// \brief The current shape.
  private: std::string shape;

  /// \brief Publisher to set the color to each of the 3 potential shapes.
  private: std::vector<ros::Publisher> symbolPubs;

  /// \brief Service to generate a new random shape/color.
  private: ros::ServiceServer changePatternServer;

  /// \brief Node handle.
  private: ros::NodeHandle nh;

  /// \brief Timer triggered in simulated time to update the shape/color for
  // the first time.
  private: ros::Timer timer;
};

#endif
