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

#ifndef VRX_LIGHT_BUOY_PLUGIN_HH_
#define VRX_LIGHT_BUOY_PLUGIN_HH_

#include <memory>
#include <sdf/sdf.hh>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>

namespace vrx
{
  /// \brief Plugin for changing the color of some visual elements using
  //  messages.
  /// This plugin accepts the following SDF parameters:
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
  /// <ros_shuffle_topic>: The ROS topic used to request color changes.
  /// <gz_colors_topic>: The gazebo topic used to request specific color
  ///                    changes. Defaults to /vrx/light_buoy/new_pattern
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
  ///     <ros_shuffle_topic>light_buoy/shuffle</ros_shuffle_topic>
  ///   </plugin>
  class LightBuoyPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
  {
    // \brief Constructor
    public: LightBuoyPlugin();

    /// \brief Destructor.
    public: ~LightBuoyPlugin() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif
