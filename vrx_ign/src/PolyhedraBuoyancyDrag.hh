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

#ifndef VRX_POLYHEDRABUOYANCYDRAG_HH_
#define VRX_POLYHEDRABUOYANCYDRAG_HH_

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

namespace vrx
{
  /// \brief This plugin simulates buoyancy of an object in fluid.
  /// The code computes buoyancy and drag forces on rigid bodies.
  /// The algorithm determines an exact buoyancy force for a polyhedron in a
  /// water volume.
  /// \ref Erin Catto. “Exact Buoyancy for Polyhedra”.
  ///      In Game Programming Gems 6, Charles River Media, 2006, pp. 175–187.
  ///
  /// ## Optional system parameters
  ///   <wavefield>: The wavefield parameters. See `Wavefield.hh`.
  ///
  ///   <fluid_density>: Sets the density of the fluid that surrounds the
  ///                    buoyant object [kg/m^3]. Default value is 1000 kg/m^3).
  ///
  ///   <fluid_level>:   The height of the fluid/air interface [m].
  ///                    The default value is 0 m.
  ///
  ///   <linear_drag>:   Linear drag coefficent [N/(m/s)].
  ///                    Translational drag implement as linear function
  ///                    of velocity. The default value is 0.
  ///
  ///   <angular_drag>:  Angular drag coefficent [(Nm)/(rad/s)].
  ///                    Rotational drag implemented as linear function
  ///                    of velocity. The default value is 0.
  ///
  ///   <buoyancy>:      Describes the volume properties. It might contain the
  ///                    following elements:
  ///
  ///                      <link_name>: Required element containing the name of
  ///                                   the link used to apply forces.
  ///                      <geometry>: Required element modeling the buoyancy
  ///                                  of the object.
  ///                      <pose>: Optional element containing the offset
  ///                              from `link_name` where forces will be applied
  ///
  ///                    For example:
  ///
  ///                    <buoyancy name="buoyancy1">
  ///                      <link_name>link</link_name>
  ///                      <geometry>
  ///                       <cylinder>
  ///                         <length>5</length>
  ///                         <radius>0.17</radius>
  ///                       </cylinder>
  ///                      </geometry>
  ///                    </buoyancy>
  ///
  /// ## Example
  /// <plugin name="vrx::PolyhedraBuoyancyDrag"
  ///         filename="libPolyhedraBuoyancyDrag.so">
  ///   <fluid_density>1000</fluid_density>
  ///   <fluid_level>0.0</fluid_level>
  ///   <linear_drag>25.0</linear_drag>
  ///   <angular_drag>2.0</angular_drag>
  ///   <buoyancy name="buoyancy_cylinder">
  ///     <link_name>link</link_name>
  ///     <pose>0 0 0 0 0 0</pose>
  ///     <geometry>
  ///       <cylinder>
  ///         <length>4</length>
  ///         <radius>0.2</radius>
  ///       </cylinder>
  ///     </geometry>
  ///   </buoyancy>
  ///   <wavefield>
  ///     <size>1000 1000</size>
  ///     <cell_count>50 50</cell_count>
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
  class PolyhedraBuoyancyDrag
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor.
    public: PolyhedraBuoyancyDrag();

    /// \brief Destructor.
    public: ~PolyhedraBuoyancyDrag() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}

#endif
