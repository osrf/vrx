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
  /// \brief Plugin class to implement hydrodynamics and wave response.
  /// This plugin accepts the following SDF parameters:
  ///
  /// <bodyName>: Name of base link for receiving pose and and applying forces.
  /// <boatArea>: Horizontal surface area [m^2]. Default value is 0.48.
  /// <boatLength>: Boat length [m]. Default value is 1.35.
  /// <boatWidth>: Boat width [m]. Default value is 1.
  /// <waterDensity>: Water density [kg/m^3]. Default value is 997.7735.
  /// <waterLevel>: Water height [m]. Default value is 0.5.
  /// <xDotU>: Added mass coeff, surge.
  /// <yDotV>: Added mass coeff, sway.
  /// <nDotR>: Added mass coeff, yaw
  /// <xU>: Linear drag coeff surge.
  /// <xUU>: Quadratic drag coeff surge.
  /// <yV>: Linear drag coeff sway.
  /// <yVV>: Quadratic drag coeff sway
  /// <zW>: Linear drag coeff heave.
  /// <kP>: Linear drag coeff pitch.
  /// <mQ>: Linear drag coeff roll.
  /// <nR>: Linear drag coeff yaw.
  /// <nRR>: Quadratic drag coeff yaw.
  /// <wave_n>: Number of waves to generate wave field.
  /// <wave_amp<N>>: Amplitude for each component [m].
  /// <wave_period<N>>: Period for each compenent [s].
  /// <wave_direction<N>>: Direction of motion for each component ENU [rad].
  class UsvDynamicsPlugin : public ModelPlugin
  {
    /// \brief Constructor.
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

    /// \brief Linear velocity from previous time step,
    /// for estimating acceleration.
    private: math::Vector3 prevLinVel;

    /// \brief Angular velocity from previous time step,
    /// for estimating acceleration.
    private: math::Vector3 prevAngVel;

    /// \brief For Buoyancy fraction for each discrete element.
    private: float buoyFrac;

    /// \brief Grid size in x direction for buoyancy discretization.
    private: float dx;

    /// \brief Grid size in y direction for buoyancy discretization.
    private: float dy;

    /// \brief Grid indicies for buoyancy discretization.
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

    /// \brief Plugin Parameter: Linear drag in heave.
    private: double paramZw;

    /// \brief Plugin Parameter: Linear drag in roll.
    private: double paramKp;

    /// \brief Plugin Parameter: Linear drag in pitch.
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

    /// \brief Wave amplitude values for N components.
    private: std::vector<float> paramWaveAmps;

    /// \brief Wave period values for N components.
    private: std::vector<float> paramWavePeriods;

    /// \brief Wave direction values for N components.
    private: std::vector<std::vector<float>> paramWaveDirections;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;
  };
}

#endif
