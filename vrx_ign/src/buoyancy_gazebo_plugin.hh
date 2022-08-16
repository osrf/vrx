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

#ifndef VRX_BUOYANCY_HH_
#define VRX_BUOYANCY_HH_

#include <map>
#include <string>
#include <vector>
#include <ignition/math/Vector3.hh>
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Link.hh"
#include <ignition/gazebo/System.hh>
#include "ignition/gazebo/Util.hh"
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include "shape_volume.hh"
#include "Wavefield.hh"

using namespace ignition;

namespace vrx
{
  /// \brief A class for storing buoyancy object properties
  class BuoyancyObject
  {
    /// \brief Default constructor
    public: BuoyancyObject();

    /// \brief Default move constructor
    public: BuoyancyObject(BuoyancyObject&& obj) noexcept; // NOLINT

    /// \brief No copy constructor
    public: BuoyancyObject(BuoyancyObject& obj) = delete;

    /// \brief Load buoyancy object from SDF
    /// \param[in] _sdf The SDF Element tree containing the object parameters.
    public: void Load(const gazebo::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief Display string for buoyancy object
    public: std::string Disp();

    /// \brief The link entity.
    public: gazebo::Link link;

    /// \brief Associated link ID
    public: int linkId;

    /// \brief Associated link name
    public: std::string linkName;

    /// \brief Pose of buoyancy relative to link
    public: ignition::math::Pose3d pose;

    /// \brief Object mass (from inertial elem)
    public:double mass;

    /// \brief Buoyancy object's shape properties
    public: ShapeVolumePtr shape;
  };

  /// \brief This plugin simulates buoyancy of an object in fluid.
  ///   <wave_model>:    Name of the wave model object (optional)
  ///
  ///   <fluid_density>: Sets the density of the fluid that surrounds the
  ///                    buoyant object [kg/m^3].
  ///                    This parameter is optional (default value 997 kg/m^3).
  ///
  ///   <fluid_level>:   The height of the fluid/air interface [m].
  ///                    This parameter is optional (default value 0 m).
  ///
  ///   <linear_drag>:   Linear drag coefficent [N/(m/s)].
  ///                    Translational drag implement as linear function
  ///                    of velocity.
  ///                    This parameter is optional.
  ///
  ///   <angular_drag>:  Angular drag coefficent [(Nm)/(rad/s)].
  ///                    Rotational drag implemented as linear function
  ///                    of velocity.
  ///                    This parameter is optional.
  ///
  ///   <buoyancy>:      Describes the volume properties
  ///                    For example:
  ///
  ///                    <buoyancy name="buoyancy1">
  ///                      <link_name>link</link_name>
  ///                      <geometry>
  ///                        ...
  ///                      </geometry>
  ///                    </buoyancy>
  ///
  ///     <link>:        Name of associated link element
  ///
  ///     <geometry>:    Geometry element specifying buoyancy object's
  ///                    volume properties.
  ///                    Supported shapes: box, sphere, cylinder
  class Buoyancy
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
  {
    /// \brief Constructor.
    public: Buoyancy();

    /// \brief Destructor.
    public: ~Buoyancy() override = default;

    // Documentation inherited.
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief The wavefield.
    protected: Wavefield wavefield;

    /// \brief The density of the fluid in which the object is submerged in
    /// kg/m^3. Defaults to 1000, the fluid density of water at 15 Celsius.
    protected: double fluidDensity;

    /// \brief The height of the fluid/air interface [m]. Defaults to 0.
    protected: double fluidLevel = 0;

    /// \brief Linear drag coefficient. Defaults to 0.
    protected: double linearDrag;

    /// \brief Angular drag coefficient. Defaults to 0.
    protected: double angularDrag;

    /// \brief List of buoyancy objects for model
    protected: std::vector<BuoyancyObject> buoyancyObjects;

    /// \brief Map of <link ID, link pointer>
    protected: std::map<int, gazebo::Link> linkMap;

  //   /// \brief Pointer to base model
  //   protected: physics::ModelPtr model;

  //   /// \brief Pointer to the Gazebo world
  //   /// Retrieved when the model is loaded.
  //   protected: physics::WorldPtr world;

  //   /// \brief The name of the wave model
  //   protected: std::string waveModelName;

    /// \brief Map of water height at each link from previous timestep
    protected: std::map<int, double> linkHeights;

    /// \brief Map of water velocity at each link
    protected: std::map<int, double> linkHeightDots;

    /// \brief Previous update time
    protected: double lastSimTime{0};

    /// \brief The world's gravity [m/s^2].
    protected: math::Vector3d gravity;

  //   /// \brief The wave parameters.
  //   protected: std::shared_ptr<const asv::WaveParameters> waveParams;
  };
}

#endif
