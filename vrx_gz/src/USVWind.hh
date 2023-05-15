/*
 * Copyright (C) 2023 Jessica Herman
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

#ifndef VRX_USV_WIND_HH_
#define VRX_USV_WIND_HH_

#include <memory>
#include <random>
#include <gz/sim/System.hh>
#include <sdf/sdf.hh>

namespace vrx
{
    /// \brief A plugin that simulates a simple wind model,
    /// based directly off of Brian Bingham's plugin for VRX in
    /// [Gazebo Classic](https://github.com/osrf/vrx/tree/gazebo_classic)
    ///
    /// This plugin accepts thefollowing parameters:
    ///
    /// <wind_objs>: Block of objects (models) to be effected by the wind.
    ///   <wind_obj>: A wind object. NOTE: may include as many objects as you like
    ///     <name>: Name of the model (object) that will be effected by the wind
    ///     <link_name>: Link on that model which will feel the force of the wind
    ///                  (limited to ONE per model).
    ///     <coeff_vector>: Coefficient vector of the particluar wind object.
    ///
    /// <wind_direction>: Wind direction vector. Wind direction is specified as
    /// the positive direction of the wind velocity vector in the horizontal plane
    /// in degrees using the ENU coordinate convention
    ///
    /// <wind_mean_velocity>: The wind average velocity.
    ///
    /// <var_wind_gain_constants>: Variable wind speed gain constant.
    ///
    /// <var_wind_time_constants>: Variable wind speed time constant.
    ///
    /// <random_seed>: Set the seed for wind speed randomization.
    ///
    /// <update_rate>: Publishing rate of the wind topic. If set to 0, it will not
    /// publish, if set to a -1 it will publish every simulation iteration.
    /// "Station-keeping control of an unmanned surface vehicle exposed to
    /// current and wind disturbances".
    ///
    /// <topic_wind_speed>: The debug topic to advertise the wind speed.
    ///
    /// <topic_wind_direction>: The debug topic to advertise the wind direction.
    ///
    /// TODO: Example sdf

    class USVWind
        : public gz::sim::System,
          public gz::sim::ISystemConfigure,
          public gz::sim::ISystemPreUpdate
    {
        /// \brief Constructor.
    public:
        USVWind();

        /// \brief Destructor.
    public:
        ~USVWind() override = default;

        // Documentation inherited.
    public:
        void Configure(const gz::sim::Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       gz::sim::EntityComponentManager &_ecm,
                       gz::sim::EventManager &_eventMgr) override;

        // Documentation inherited.
    public:
        void PreUpdate(
            const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) override;

        /// \brief Private data pointer.
    private:
         GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr);
    };
}

#endif
