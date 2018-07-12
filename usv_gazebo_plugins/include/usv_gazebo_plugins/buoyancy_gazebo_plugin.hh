/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef USV_GAZEBO_PLUGINS_BUOYANCY_GAZEBO_PLUGIN_HH_
#define USV_GAZEBO_PLUGINS_BUOYANCY_GAZEBO_PLUGIN_HH_

#include <map>
#include <gazebo/common/common.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief A class for storing the volume properties of a link.
  class VolumeProperties
  {
    /// \brief Default constructor.
    public: VolumeProperties()
      : area(0), height(0)
    {
    }

    /// \brief Center of volume in the link frame.
    public: math::Vector3 cov;

    /// \brief Horizontal area of this link.
    public: double area;

    /// \brief Vertical height for this link.
    public: double height;
  };

  /// \brief A plugin that simulates buoyancy of an object immersed in fluid.
  /// All SDF parameters are optional.
  ///   <fluid_density>: Sets the density of the fluid that surrounds the
  ///                    buoyant object [kg/m^3].
  ///                    This paramater is optional.
  ///
  ///   <fluid_level>:   The height of the fluid/air interface [m].
  ///                    This parameter is optional.
  ///
  ///   <fluid_drag>:    Quadratic drag generally applied to Z velocity.
  ///                    This parameter is optional.
  ///
  ///   <link>:          Describe the volume properties of individual links in
  ///                    the model.
  ///                    For example:
  ///
  ///                    <link name="body">
  ///                      <center_of_volume>1 2 3</center_of_volume>
  ///                      <area>10</volume>
  ///                      <height>5</height>
  ///                    </link>
  ///
  ///     <center_of_volume>: A point representing the volumetric center of the
  ///                         link in the link frame. This is where the buoyancy
  ///                         force will be applied. This field is required.
  ///
  ///     <area>:             Horizontal area of this link.
  ///                         This field is required
  ///
  ///     <height>:           Vertical height of this link.
  ///                         This field is required.
  class BuoyancyPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: BuoyancyPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief The density of the fluid in which the object is submerged in
    /// kg/m^3. Defaults to 1000, the fluid density of water at 15 Celsius.
    protected: double fluidDensity;

    /// \brief The height of the fluid/air interface [m]. Defaults to 0.
    protected: double fluidLevel;

    /// \brief Quadratic drag generally applied to Z velocity. Defaults to 0.
    protected: double fluidDrag;

    /// \brief Map of <link ID, point> pairs mapping link IDs to the CoV
    /// (center of volume) and volume of the link.
    protected: std::map<int, VolumeProperties> volPropsMap;

    /// \brief Vector of links in the model for which we will apply buoyancy
    /// forces.
    protected: physics::Link_V buoyancyLinks;
  };
}

#endif
