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

#ifndef VMRC_GAZEBO_SCAN_DOCK_SCORING_PLUGIN_HH_
#define VMRC_GAZEBO_SCAN_DOCK_SCORING_PLUGIN_HH_

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>
#include "vmrc_gazebo/scoring_plugin.hh"

/// \brief A plugin for computing the score of the scan and dock task.
/// This plugin derives from the generic ScoringPlugin class. Check out that
/// plugin for other required SDF elements.
/// This plugin requires the following SDF parameters:
///
/// <color_1>: Expected first color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_2>: Expected second color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <color_3>: Expected third color of the sequence (RED, GREEN, BLUE, YELLOW).
/// <robot_namespace>: The ROS namespace for this plugin.
/// <topic>: The ROS topic used to send the color pattern.
///
/// Here's an example:
/// <plugin name="scan_dock_scoring_plugin"
///         filename="libscan_code_scoring_plugin.so">
/// </plugin>
class ScanDockScoringPlugin : public ScoringPlugin
{
  // Constructor.
  public: ScanDockScoringPlugin();

  // Documentation inherited.
  private: void Load(gazebo::physics::WorldPtr _world,
                     sdf::ElementPtr _sdf);

  /// \brief Parse all SDF parameters.
  /// \param[in] _sdf SDF elements.
  private: bool ParseSDF(sdf::ElementPtr _sdf);

  /// \brief Callback executed at every world update.
  private: void Update();

  /// \brief Callback for change pattern service, calls other changePattern
  /// internaly.
  /// \param[in] _req Not used.
  /// \param[out] _res The Response containing a message with the new pattern.
  /// \return True when the operation succeed or false otherwise.
  private: bool OnColorSequence(std_srvs::Trigger::Request &_req,
                                std_srvs::Trigger::Response &_res);

  /// \brief The expected color sequence.
  private: std::vector<std::string> expectedSequence;

  /// \brief Service to generate and display a new color sequence.
  private: ros::ServiceServer colorSequenceServer;

  /// \brief ROS Node handle.
  private: ros::NodeHandle nh;

  /// \brief ROS namespace.
  private: std::string ns;

  /// \brief ROS topic where the color sequence should be sent.
  private: std::string topic = "/scan_dock/color_sequence";
};

#endif
