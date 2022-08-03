/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef VRX_WAMVSURFACE_HH_
#define VRX_WAMVSURFACE_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

#include "Surface.hh"

namespace vrx
{
  /// \brief A system that simulates the buoyancy of an object at the surface of
  /// a fluid. This system must be attached to a model and the system will apply
  /// buoyancy to a collection of points around a given link.
  ///
  /// This system models the vehicle's buoyancy assuming two hulls with a
  /// cylindrical shape. It's possible to derive from this plugin and provide
  /// your own buoyancy function at each point. For this purpose you
  /// should override `BuoyancyAtPoint()` in the derived plugin.
  ///
  /// This plugin also supports waves. If you provide a wavefield via SDF, the
  /// plugin will account for the delta Z that the waves generate at each point.
  ///
  /// ## Required system parameters
  ///
  /// * `<link_name>` is the name of the link used to apply forces.
  ///
  /// ## Optional system parameters
  ///
  /// * `<vehicle_length>` is the length of the vessel [m].
  /// * `<hull_radius>` is the radius of the vessel's hull [m].
  /// * `<fluid_level>` is the depth at which the fluid should be in the vehicle
  /// * `<fluid_density>` is the density of the fluid.
  /// * `<points>` contains a collection of points where the forces generated
  ///              by this plugin will be applied. See the format of each point
  ///              next:
  /// *   `<point><position>` Relative position of the point relative to
  ///                         `link_name`.
  /// * <wavefield>: The wavefield parameters. See `Wavefield.hh`.
  ///
  /// ## Example
  /// <plugin
  ///   filename="ignition-gazebo-surface-system"
  ///   name="ignition::gazebo::systems::Surface">
  ///   <link_name>base_link</link_name>
  ///   <vehicle_length>4.9</vehicle_length>
  ///   <vehicle_width>2.4</vehicle_width>
  ///   <hull_radius>0.213</hull_radius>
  ///   <fluid_level>0</fluid_level>

  ///   <!-- Points -->
  ///   <points>
  ///     <point>
  ///       <position>1.225 1.2 0</position>
  ///     </point>
  ///     <point>
  ///       <position>1.225 -1.2 0</position>
  ///     </point>
  ///     <point>
  ///       <position>-1.225 1.2 0</position>
  ///     </point>
  ///     <point>
  ///       <position>-1.225 -1.2 0</position>
  ///     </point>
  ///   </points>

  ///   <!-- Waves -->
  ///   <wavefield>
  ///     <size><%= $wavefield_size%> <%= $wavefield_size%></size>
  ///     <cell_count><%= $wavefield_cell_count%> <%=$wavefield_cell_count%></cell_count>
  ///     <wave>
  ///       <model>PMS</model>
  ///       <period>5.0</period>
  ///       <number>3</number>
  ///       <scale>1.1</scale>
  ///       <gain>0.5</gain>
  ///       <direction>1 0</direction>
  ///       <angle>0.4</angle>
  ///       <tau>2.0</tau>
  ///       <amplitude>0.0</amplitude>
  ///       <steepness>0.0</steepness>
  ///     </wave>
  ///   </wavefield>
  /// </plugin>
  class WamvSurface
      : public vrx::Surface
  {
    /// \brief Constructor.
    public: WamvSurface();

    /// \brief Destructor.
    public: ~WamvSurface() override = default;

    /// \brief This method is called when there is a timestep in the simulator.
    /// \param[in] _info Simulator information about the current timestep.
    ///                         will become the new registry.
    /// \param[in] _ecm - Ignition's ECM.
    public: virtual double BuoyancyAtPoint(
                    const ignition::gazebo::UpdateInfo &_info,
                    const ignition::math::Vector3d &_point,
                    double _deltaZ,
                    ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    IGN_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}

#endif
