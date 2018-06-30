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
along with this package.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef USV_GAZEBO_PLUGINS_THRUST_HH
#define USV_GAZEBO_PLUGINS_THRUST_HH

#include <ros/ros.h>
#include <string>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <usv_gazebo_plugins/UsvDrive.h>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief ToDo.
  class UsvThrust : public ModelPlugin
  {
    /// \brief Constructor.
    public: UsvThrust();

    /// \brief Destructor.
    public: virtual ~UsvThrust() = default;

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _parent,
                              sdf::ElementPtr _sdf);
  
    /// \brief Callback executed at every physics update.
    protected: virtual void UpdateChild();

    /// \brief Callback for Drive commands.
    /// \param msg usv_msgs UsvDrive message
    private:void OnCmdDrive(const usv_gazebo_plugins::UsvDriveConstPtr &_msg);

    /// \brief Convenience function for getting SDF parameters.
    private: double SdfParamDouble(sdf::ElementPtr _sdfPtr,
                                   const std::string &_paramName,
                                   double defaultVal);

    /// \brief Takes ROS Kingfisher Drive commands and scales them by max thrust
    /// \param cmd ROS drive command
    /// \param max_cmd  Maximum value expected for commands - scaling factor
    /// \param max_pos  Maximum positive force value
    /// \param max_neg  Maximum negative force value
    /// \return Value scaled and saturated
    private: double scaleThrustCmd(double _cmd);

    /// \brief ToDo.
    private: double glf(double _x, float _A, float _K, float _B,
                        float _v, float _C, float _M);

    /// \brief ToDo.
    private: double glfThrustCmd(double _cmd);

    /// \brief Parse the propeller name from SDF.
    /// \param sdf The entire model SDF
    /// \param sdfName The SDF element to parse
    /// \param propellerJoint The joint pointer to initialize
    private: void ParsePropeller(const sdf::ElementPtr _sdf,
                                 const std::string &_sdfName,
                                 physics::JointPtr &_propellerHoint);

    /// \brief Spin a propeller based on its input
    /// \param propeller Pointer to the propeller joint to spin
    /// \param input Last input received for this propeller
    private: void SpinPropeller(const physics::JointPtr &_propeller,
                                const double _input);

    /// \brief ToDo.
    private: std::string nodeNamespace;

    /// \brief ToDo.
    private: std::string linkName;

    /// \brief ToDo.
    private: ros::NodeHandle *rosnode;

    /// \brief ToDo.
    private: ros::Subscriber cmdDriveSub;

    /// \brief Pointer to the Gazebo world, retrieved when the model is loaded.
    private: physics::WorldPtr world;

    /// \brief Pointer to Gazebo parent model, retrieved when the model is
    /// loaded.
    private: physics::ModelPtr model;

    /// \brief Pointer to model link in gazebo.
    ///  optionally specified by the bodyName parameter,
    ///  The states are taken from this link and forces applied to this link.
    private: physics::LinkPtr link;

    /// \brief Timeout for receiving Drive commands [s].
    private: double cmdTimeout;

    /// \brief ToDo.
    private: common::Time prevUpdateTime;

    /// \brief ToDo.
    private: common::Time lastCmdDriveTime;

    /// \brief ToDo.
    private: double lastCmdDriveLeft;

    /// \brief ToDo.
    private: double lastCmdDriveRight;

    /// \brief ToDo.
    private: int paramMappingType;

    /// \brief Plugin Parameter: Maximum (abs val) of Drive commands.
    /// typ. +/-1.0
    private: double paramMaxCmd;

    /// \brief Plugin Parameter: Maximum forward force [N].
    private: double paramMaxForceFwd;

    /// \brief Plugin Parameter: Maximum reverse force [N].
    private: double paramMaxForceRev;

    /// \brief Plugin Parameter: Boat width [m].
    private: double paramBoatWidth;

    /// \brief Plugin Parameter: Boat length [m].
    private: double paramBoatLength;

    ///  \brief Plugin Parameter: Z offset for applying forward thrust.
    private: double paramThrustZoffset;

    /// \brief Joint controlling the left propeller.
    private: physics::JointPtr leftPropellerJoint;

    /// \brief Joint controlling the right propeller.
    private: physics::JointPtr rightPropellerJoint;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;
  };
}

#endif
