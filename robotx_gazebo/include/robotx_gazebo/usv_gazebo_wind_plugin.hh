/*

Copyright (c) 2018, Brian Bingham
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

#ifndef USV_GAZEBO_WIND_H
#define USV_GAZEBO_WIND_H


// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo_plugins/gazebo_ros_utils.h>

//ROS
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <Eigen/Core>
				    //#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

namespace gazebo
{
  class UsvWindPlugin : public ModelPlugin
  {
  public:
    UsvWindPlugin();
    virtual ~UsvWindPlugin();
    /*! Loads the model in gets dynamic parameters from SDF. */
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  protected:
    /*! Callback for Gazebo simulation engine */
    virtual void UpdateChild();
    virtual void FiniChild();
  private:

    double getSdfParamDouble(sdf::ElementPtr sdfPtr, const std::string &param_name, double default_val);

    /*! ROS spin once */
    void spin();

    /// Parameters
    std::string node_namespace_;
    std::string link_name_;

    ros::NodeHandle *rosnode_;

    //GazeboRosPtr gazebo_ros_;
    //physics::ModelPtr parent;
    event::ConnectionPtr update_connection_;

    /*! Pointer to the Gazebo world, retrieved when the model is loaded */
    physics::WorldPtr world_;
    /*! Pointer to Gazebo parent model, retrieved when the model is loaded */
    physics::ModelPtr model_;
    /*! Pointer to model link in gazebo,
      optionally specified by the bodyName parameter,
      The states are taken from this link and forces applied to this link.*/
    physics::LinkPtr link_;

    /*! Wind velocity in Gazebo coordinates [m/s] */
    math::Vector3 param_wind_velocity_vector_;

    /*! Wind force coefficients */
    math::Vector3 param_wind_coeff_vector_;

    boost::thread *spinner_thread_;

    event::ConnectionPtr contact_event_;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

  };  // class UsvWindPlugin
} // namespace gazebo

#endif //USV_GAZEBO_WIND_H
