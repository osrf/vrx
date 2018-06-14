/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the usv_gazebo_dynamics_plugin package, known as this Package.

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

#ifndef USV_GAZEBO_THRUST_H
#define USV_GAZEBO_THRUST_H

// C++
#include <algorithm>  // min/mzx
#include <math.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo_plugins/gazebo_ros_utils.h>

//ROS
#include <robotx_gazebo/UsvDrive.h>
#include <ros/ros.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

namespace gazebo
{
  class UsvThrust : public ModelPlugin
  {
  public:
    UsvThrust();
    virtual ~UsvThrust();
    /*! Loads the model in gets dynamic parameters from SDF. */
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  protected:
    /*! Callback for Gazebo simulation engine */
    virtual void UpdateChild();
    virtual void FiniChild();
  private:
    /*!
      Callback for Drive commands
      \param msg usv_msgs UsvDrive message
    */
    void OnCmdDrive( const robotx_gazebo::UsvDriveConstPtr &msg);

    /*! ROS spin once */
    void spin();

    /*! Convenience function for getting SDF parameters

     */
    double getSdfParamDouble(sdf::ElementPtr sdfPtr,const std::string &param_name,double default_val);

    /*! Takes ROS Kingfisher Drive commands and scales them by max thrust

      \param cmd ROS drive command
      \param max_cmd  Maximum value expected for commands - scaling factor
      \param max_pos  Maximum positive force value
      \param max_neg  Maximum negative force value
      \return Value scaled and saturated
     */
    double scaleThrustCmd(double cmd);

    double glf(double x, float A, float K, float B,
	       float v, float C, float M);

    double glfThrustCmd(double cmd);

    /// Parameters
    std::string node_namespace_;
    std::string link_name_;

    ros::NodeHandle *rosnode_;

    ros::Subscriber cmd_drive_sub_;

    //GazeboRosPtr gazebo_ros_;
    //physics::ModelPtr parent;
    event::ConnectionPtr update_connection_;
    boost::thread *spinner_thread_;

    /*! Pointer to the Gazebo world, retrieved when the model is loaded */
    physics::WorldPtr world_;
    /*! Pointer to Gazebo parent model, retrieved when the model is loaded */
    physics::ModelPtr model_;
    /*! Pointer to model link in gazebo,
      optionally specified by the bodyName parameter,
      The states are taken from this link and forces applied to this link.*/
    physics::LinkPtr link_;
    math::Pose pose_;
    /*! Timeout for recieving Drive commands [s]*/
    double cmd_timeout_;
    common::Time prev_update_time_;
    common::Time last_cmd_drive_time_;
    double last_cmd_drive_left_;
    double last_cmd_drive_right_;


    int param_mapping_type_;
    /*! Plugin Parameter: Maximum (abs val) of Drive commands. typ. +/-1.0 */
    double param_max_cmd_;
    /*! Plugin Parameter: Maximum forward force [N] */
    double param_max_force_fwd_;
    /*! Plugin Parameter: Maximum reverse force [N] */
    double param_max_force_rev_;

    /*! Plugin Parameter: Boat width [m] */
    double param_boat_width_;
    /*! Plugin Parameter: Boat length [m] */
    double param_boat_length_;
    /*! Plugin Parameter: Z offset for applying forward thrust */
    double param_thrust_z_offset_;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
  };  // class UsvThrust
} // namespace gazebo

#endif //USV_GAZEBO_THRUST_H
