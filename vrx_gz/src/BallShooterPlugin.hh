/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef VRX_BALL_SHOOTER_PLUGIN_HH_
#define VRX_BALL_SHOOTER_PLUGIN_HH_

#include <memory>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

namespace vrx
{
/// \brief Simulate a ball shooter. A projectile is launched when a
/// ROS message is received. The ball is reused (teleported) from previous
/// shots.
///
/// The plugin accepts the following SDF parameters:
/// * Required parameters:
/// <projectile> The object used as projectile should be declared as a
///              <projectile> element with the following parameters:
///                 * <model_name> The name of the model used as projectile.
///                                Required parameter.
///                 * <link_name> The name of the link within the projectile
///                               model where the force will be applied when
///                               firing the shooter. Required parameter.
///                 * <frame> If present, the <pose> parameter will be in the
///                           frame of this link/model. Otherwise the world
///                           frame is used. Optional parameter.
///                 * <pose> - Pose of the projectile right before being shot.
///                            Optional parameter. Default to {0 0 0 0 0 0}.
///
/// * Optional parameters:
/// <num_shots> - Number of shots allowed. Default to UINT_MAX.
/// <shot_force> - Force (N) applied to the projectile. Default to 250 N.
/// <topic> - Name of the ROS topic to shoot. Default to "/ball_shooter/fire".
///
/// Here's an example:
/// <plugin name="ball_shooter_plugin" filename="libball_shooter_plugin.so">
///   <projectile>
///     <model_name>a_projectile</model_name>
///     <link_name>link</link_name>
///     <frame>my_robot/ball_shooter_link</frame>
///     <pose>0.2 0 0 0 0 0</pose>
///   </projectile>
///   <shot_force>250</shot_force>
///   <topic>my_robot/ball_shooter/fire</topic>
/// </plugin>
class BallShooterPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  // \brief Constructor.
  public: BallShooterPlugin();

  /// \brief Destructor.
  public: ~BallShooterPlugin() override = default;

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
