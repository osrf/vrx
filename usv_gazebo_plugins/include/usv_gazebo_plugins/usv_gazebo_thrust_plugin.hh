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
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <memory>
#include <string>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo
{

  // Foward declaration of UsvThrust class
  class UsvThrust;
  
  /// \brief Thruster class
  class Thruster
  {

    /// \brief Constructor
    /// \param[in] _sdfPtr Pointer to an SDF element to parse.
    public: Thruster(UsvThrust* parent);

    /// \brief Callback for new thrust commands
    public: void OnThrustCmd(const std_msgs::Float32::ConstPtr & msg);
    
    /// \brief Maximum abs val of incoming command
    public: double maxCmd;

    /// \brief Max forward force in Newtons
    public: double maxForceFwd;

    /// \brief Max reverse force in Newtons
    public: double maxForceRev;

    /// \brief Link where thrust force is applied
    public: physics::LinkPtr link;

    /// \brief Thruster mapping (0=linear; 1=GLF, nonlinear)
    public: int mappingType;

    /// \brief Topic name for incoming ROS thruster commands
    public: std::string cmdTopic;

    /// \brief Subscription to thruster commands
    public: ros::Subscriber cmdSub;

    /// \brief Current, most recent command
    public: double currCmd;

    /// \brief Plugin parent pointer - for accessing world, etc.
    protected: UsvThrust* plugin;

    /// \brief Last time received a command via ROS topic
    public: common::Time lastCmdTime;

    /// \brief Joint controlling the propeller.
    public: physics::JointPtr propJoint;
    
  };
  
  /// \brief A plugin to simulate a propulsion system under water.
  /// This plugin accepts the following SDF parameters.
  /// See https://github.com/bsb808/robotx_docs/blob/master/theoryofoperation/theory_of_operation.pdf
  /// for more information.
  ///
  ///   <robotNamespace>: Namespace prefix for USV.
  ///   <cmdTimeout>:  Timeout, after which thrust is set to zero [s].
  ///   <left_propeller_joint>: The left's propeller joint.
  ///   <right_propeller_joint>: The right's propeller joint.
  ///
  ///   Multiple thrusters are declared by including <thruster> SDF elements,
  ///   where each thruster includes the following SDF parameters specific to
  ///   the individual thruster
  ///   <linkName>: Name of the link on which to apply thrust forces.
  ///   <mappingType>: Thruster mapping (0=linear; 1=GLF, nonlinear)
  ///   <maxCmd>:Maximum (abs val) of thrust commands, typ. 1.0.
  ///   <maxForceFwd>: Maximum forward force [N].
  ///   <maxForceRev>: Maximum reverse force [N].

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

    /// \brief Takes ROS Drive commands and scales them by max thrust.
    /// \param[in] _cmd ROS drive command.
    /// \return Value scaled and saturated.
  private: double ScaleThrustCmd(const double _cmd, double max_cmd, double max_pos, double max_neg) const;

    /// \brief Generalized logistic function (GLF) used for non-linear
    /// thruster model.
    /// \param[in] _x Independent variable (input) of GLF.
    /// \param[in] _A Lower asymptote.
    /// \param[in] _K Upper asymptote.
    /// \param[in] _B Growth rate
    /// \param[in] _v Affects near which asymptote max. growth occurs.
    /// \param[in] _C Typically near 1.0.
    /// \param[in] _M Offset to input.
    /// \return 
    private: double Glf(const double _x,
                        const float  _A,
                        const float  _K,
                        const float  _B,
                        const float  _v,
                        const float  _C,
                        const float  _M) const;

    /// \brief Uses GLF function to map thrust command to thruster force
    /// in Newtons.
    /// \param[in] _cmd Thrust command {-1.0,1.0}.
    /// \return Thrust force [N].
  private: double GlfThrustCmd(const double _cmd, double max_pos, double max_neg) const;

    /// \brief Spin a propeller based on its input
    /// \param[in] _propeller Pointer to the propeller joint to spin
    /// \param[in] _input Last input received for this propeller
    private: void SpinPropeller(physics::JointPtr &_propeller,
                                const double _input);

    /// \brief The ROS node handler used for communications.
    private: std::unique_ptr<ros::NodeHandle> rosnode;

    /// \brief Pointer to the Gazebo world, retrieved when the model is loaded.
    public: physics::WorldPtr world;

    /// \brief Pointer to Gazebo parent model, retrieved when the model is
    /// loaded.
    private: physics::ModelPtr model;

    /// \brief Timeout for receiving Drive commands [s].
    private: double cmdTimeout;

    /// \brief Vector of thruster instances
    private: std::vector<Thruster> thrusters;

       /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief For publishing to /joint_state with propeller state.
    private: ros::Publisher jointStatePub;

    /// \brief The propeller message state.
    private: sensor_msgs::JointState jointStateMsg;
  };

  

}

#endif
