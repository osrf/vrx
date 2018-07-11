/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the usv_gazebo_dynamics_plugin package,
known as this Package.

This Package free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This Package s distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef USV_GAZEBO_PLUGINS_DYNAMICS_PLUGIN_HH_
#define USV_GAZEBO_PLUGINS_DYNAMICS_PLUGIN_HH_

#include <Eigen/Core>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief ToDo.
  class UsvDynamicsPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    ///
    /// <bodyName>: ToDo.
    /// <boatArea>: Horizontal surface area [m^2]. Default value is 0.48.
    /// <boatLength>: Boat length [m]. Default value is 1.35.
    /// <boatWidth>: Boat width [m]. Default value is 1.
    /// <waterDensity>: Water density [kg/m^3]. Default value is 997.7735.
    /// <waterLevel>: Water height [m]. Default value is 0.5.
    /// <xDotU>: ToDo.
    /// <yDotV>: ToDo.
    /// <nDotR>: ToDo.
    /// <xU>: ToDo.
    /// <xUU>: ToDo.
    /// <yV>: ToDo.
    /// <yVV>: ToDo.
    /// <zW>: ToDo.
    /// <kP>: ToDo.
    /// <mQ>: ToDo.
    /// <nR>: ToDo.
    /// <nRR>: ToDo.
    /// <wave_n>: ToDo.
    /// <wave_amp<N>>: ToDo.
    /// <wave_period<N>>: ToDo.
    /// <wave_direction<N>>: ToDo.
    public: UsvDynamicsPlugin();

    /// \brief Destructor.
    public: virtual ~UsvDynamicsPlugin() = default;

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf);

    /// \brief Callback for Gazebo simulation engine.
    protected: virtual void Update();

    /// \brief Convenience function for getting SDF parameters.
    /// \param[in] _sdfPtr Pointer to an SDF element to parse.
    /// \param[in] _paramName The name of the element to parse.
    /// \param[in] _defaultVal The default value returned if the element
    /// does not exist.
    /// \return The value parsed.
    private: double SdfParamDouble(sdf::ElementPtr _sdfPtr,
                                   const std::string &_paramName,
                                   const double _defaultVal) const;

    /// \brief Pointer to the Gazebo world, retrieved when the model is loaded.
    private: physics::WorldPtr world;

    /// \brief Pointer to model link in gazebo,
    /// optionally specified by the bodyName parameter.
    /// The states are taken from this link and forces applied to this link.
    private: physics::LinkPtr link;

    /// \brief Simulation time of the last update.
    private: common::Time prevUpdateTime;

    /// \brief ToDo.
    private: math::Vector3 prevLinVel;

    /// \brief ToDo.
    private: math::Vector3 prevAngVel;

    /// \brief For Buoyancy calculation.
    private: float buoyFrac;

    /// \brief ToDo.
    private: float dx;

    /// \brief ToDo.
    private: float dy;

    /// \brief ToDo.
    private: std::vector<int> II;

    /// \brief Values to set via Plugin Parameters.
    /// Plugin Parameter: Added mass in surge, X_\dot{u}.
    private: double paramXdotU;

    /// \brief Plugin Parameter: Added mass in sway, Y_\dot{v}.
    private: double paramYdotV;

    /// \brief Plugin Parameter: Added mass in yaw, N_\dot{r}.
    private: double paramNdotR;

    /// \brief Plugin Parameter: Linear drag in surge.
    private: double paramXu;

    /// \brief Plugin Parameter: Quadratic drag in surge.
    private: double paramXuu;

    /// \brief Plugin Parameter: Linear drag in sway.
    private: double paramYv;

    /// \brief Plugin Parameter: Quadratic drag in sway.
    private: double paramYvv;

    /// \brief ToDo.
    private: double paramZw;

    /// \brief ToDo.
    private: double paramKp;

    /// \brief ToDo.
    private: double paramMq;

    /// \brief Plugin Parameter: Linear drag in yaw.
    private: double paramNr;
    /// \brief Plugin Parameter: Quadratic drag in yaw.
    private: double paramNrr;

    /// \brief Water height [m].
    private: double waterLevel;

    /// \brief Added mass matrix, 6x6.
    private: Eigen::MatrixXd Ma;

    /// \brief Wave parameters.
    private: int paramWaveN;

    /// \brief ToDo.
    private: std::vector<float> paramWaveAmps;

    /// \brief ToDo.
    private: std::vector<float> paramWavePeriods;

    /// \brief ToDo.
    private: std::vector<std::vector<float>> paramWaveDirections;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;
  };
}

#endif
