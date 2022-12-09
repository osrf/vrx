/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef VRX_GAZEBO_GYMKHANA_SCORING_PLUGIN_HH_
#define VRX_GAZEBO_GYMKHANA_SCORING_PLUGIN_HH_

#include <gz/msgs/param.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

#include "ScoringPlugin.hh"

namespace vrx
{
  /// \brief A plugin for computing the score of the gymkhana task.
  /// This plugin derives from the generic ScoringPlugin class. Refer to that
  /// plugin for an explanation of the four states defined (Initial, Ready,
  /// Running and Finished) as well as other required SDF elements.
  ///
  /// The gymkhana task is divided into two parts: a navigation channel and
  /// an obstacle field. The vehicle should first traverse the navigation
  /// channel, then minimize its distance to an acoustic pinger "blackbox"
  /// located in the obstacle field.
  ///  
  /// This plugin accepts the following optional SDF parameters:
  ///
  /// <obstacle_penalty>: Specifies how many points are deducted per collision.
  /// <pinger_position>: Specifies the location of the pinger in cartesian
  /// coordinates. The first two coordinates represent cartesian X and Y. These
  /// default to 0. The third value is always ignored.
  /// <set_position_topic_name>: Specifies the topic used to set the pinger
  /// position. The default is /pinger/set_pinger_position. Note this topic is
  /// for debugging only; the pinger position will not be changed dynamically
  /// during an actual competition trial.
  ///
  /// To manage the navigation channel and obstacle field parts of the task, 
  /// this plugin also relies on the NavigationScoringPlugin and the
  /// StationKeepingScoringPlugin. These must be included in the sdf and
  /// configured according to their documentation, with the following
  /// additional constraints:
  /// 
  /// NavigationScoringPlugin:
  ///   <per_plugin_exit_plugin_on_completion> must be set to false
  ///   <silent> is recommended to be set to true to reduce redundant messages 
  ///
  /// StationkeepingScoringPlugin:
  ///   <goal_pose_cart> must have the same value as the <pinger_position>
  ///   set in the GymKhanaScoringPlugin.
  ///   <head_error_on> must be set to false
  ///   <per_plugin_exit_plugin_on_completion> must be set to false
  ///   <silent> is recommended to be set to true to reduce redundant messages 
    class GymkhanaScoringPlugin : public ScoringPlugin
  {
    /// \brief Constructor.
    public: GymkhanaScoringPlugin();
  
    /// \brief Destructor.
    public: virtual ~GymkhanaScoringPlugin() override = default;
  
    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;
  
    // Documentation inherited.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;
  
    /// \brief Callback for channel navigation portion's scoring plugin
    public: void ChannelCallback(const gz::msgs::Param &_msg);

    // Documentation inherited.
    protected: void OnFinished() override;
 
    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
#endif
