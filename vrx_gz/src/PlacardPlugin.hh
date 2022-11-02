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

#ifndef VRX_PLACARD_PLUGIN_HH_
#define VRX_PLACARD_PLUGIN_HH_

#include <memory>
#include <sdf/sdf.hh>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>

namespace vrx
{

/// \brief Controls the shape and color of a symbol.
///
/// It selects a shape (triangle, cross, circle, rectangle) and a color
/// (red, green, blue, yellow) and applies to the current symbol.
/// This plugin can be configured with the following SDF tags:
///
///   <shape> "triangle", "cross", "circle" or "rectangle".
///           If ommited, the shape will be randomly selected.
///   <color> "red", "green", "blue" or "yellow". If ommited, the color will be
///           randomly selected.
///   <shuffle>: True if the topic for shuffling the sequence is enabled.
///   <robot_namespace> Topic namespace.
///   <shuffle_topic>: The topic used to request color changes.
///   <symbol_topic>: The gazebo topic subscribed to set symbol changes
///                   defaults to /<robot_namespace>/symbol
///   <visuals>: The set of visual symbols. It contains at least one visual:
///              <visual>: A visual displaying a shape.
///
/// Here's an example:
///   <plugin name="vrx::PlacardPlugin" filename="libPlacardPlugin.so">
///     <shape>triangle</shape>
///     <color>red</color>
///     <visuals>
///       <visual>dock_2018_placard1::visual_circle</visual>
///       <visual>dock_2018_placard1::visual_h_cross</visual>
///       <visual>dock_2018_placard1::visual_v_cross</visual>
///       <visual>dock_2018_placard1::visual_triangle</visual>
///     </visuals>
///     <shuffle>true</shuffle>
///     <robot_namespace>vrx</robot_namespace>
///     <shuffle_topic>dock/placard/shuffle</shuffle_topic>
///   </plugin>
class PlacardPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure
{
  // Documentation inherited.
  public: PlacardPlugin();

  /// \brief Destructor.
  public: ~PlacardPlugin() override = default;

  // Documentation inherited.
  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) override;

  /// \brief Private data pointer.
  GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
};
}

#endif
