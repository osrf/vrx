/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifndef GAZEBO_POPULATION_PLUGIN_HH_
#define GAZEBO_POPULATION_PLUGIN_HH_

#include <memory>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  // Forward declare private data class
  class PopulationPluginPrivate;

  /// \brief A plugin that allows models to be spawned at a given location in
  /// a specific simulation time. As an example, this plugin might be used for
  /// placing objects in a conveyor belt.

  /// The plugin accepts the following SDF parameters:

  /// <object_sequence>: Contains the list of objects to be populated. An object
  ///                    should be declared as an <object> element with the
  ///                    following parameters:
  ///                      <time> Simulation time to be spawned.
  ///                      <type> Model.
  ///                      <pose> Initial object pose.
  ///
  /// <loop_forever>: Optional parameter. If true, all objects will be spawned
  /// as a circular buffer. After spawning the last element of the collection,
  /// the first one will be inserted.
  ///
  /// <frame>: Optional parameter. If present, the poses of the objects will be
  /// in the frame of this link/model. Otherwise the world frame is used.
  ///
  /// <activation_topic>: Optional parameter. If present, the objects won't be
  /// inserted in simulation until the proper command is received in the
  /// specified topic. Available commands:
  ///   "restart" : The object population will be restarted.
  ///   "stop"    : Stop the object insertion.
  ///
  /// Here's an example of a valid SDF:
  ///
  /// <plugin filename="libPopulationPlugin.so" name="populate_conveyor">
  ///   <loop_forever>true</loop_forever>
  ///   <object_sequence>
  ///     <object>
  ///       <time>8.0</time>
  ///       <type>coke_can</type>
  ///       <pose>1 6 1 0 0 0</pose>
  ///     </object>
  ///     <object>
  ///       <time>4.0</time>
  ///       <type>cordless_drill/</type>
  ///       <pose>1 6 1 0 0 0</pose>
  ///     </object>
  ///     <object>
  ///       <time>12.0</time>
  ///       <type>beer</type>
  ///       <pose>1 6 1 0 0 0</pose>
  ///     </object>
  ///   </object_sequence>
  /// </plugin>

  class GAZEBO_VISIBLE PopulationPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: PopulationPlugin();

    /// \brief Destructor.
    public: virtual ~PopulationPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Pause the object population.
    public: void Pause();

    /// \brief Resume the object population after a pause.
    public: void Resume();

    /// \brief Restart the the object population.
    public: virtual void Restart();

    /// \brief Update the plugin.
    protected: void OnUpdate();

    /// \brief Callback that receives activation messages. If the
    /// <activation_topic> is set in SDF, the plugin won't populate any object
    /// until the activation is received.
    /// \param[in] _msg String message that indicates the activation command.
    ///   * start|restart: Start/restart the object population.
    protected: void OnActivation(ConstGzStringPtr &_msg);

    /// \brief Callback that receives rate modifier messages. If the
    /// <rate_modifier_topic> is set in SDF, the plugin will modify the population rate
    /// by the received factor.
    /// \param[in] _msg String message that indicates the rate modifier.
    protected: void OnRateModification(ConstGzStringPtr &_msg);

    /// \brief True when the plugin is enabled or false if it's paused.
    protected: bool Enabled() const;

    /// \brief Determine whether is time to give the plugin an update based on
    /// the plugin's update rate.
    protected: bool TimeToExecute();

    /// \brief Overwrite this method for sending periodic updates with the
    /// plugin state.
    private: virtual void Publish() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<PopulationPluginPrivate> dataPtr;
  };
}
#endif
