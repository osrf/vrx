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
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef USV_GAZEBO_DYNAMICS_H
#define USV_GAZEBO_DYNAMICS_H

// C++
#include <vector>
#include <iostream>     // std::cout, std::ios
#include <sstream>      // std::ostringstream
#include <thread>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo_plugins/gazebo_ros_utils.h>

//ROS
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
//#include <usv_msgs/UsvDrive.h>
//#include <usv_gazebo_plugins/UsvDrive.h>

#include <Eigen/Core>
				    //#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

namespace gazebo
{
  class UsvPlugin : public ModelPlugin
  {
  public:
    UsvPlugin();
    virtual ~UsvPlugin();
    /*! Loads the model in gets dynamic parameters from SDF. */
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  protected:
    /*! Callback for Gazebo simulation engine */
    virtual void UpdateChild();
    virtual void FiniChild();
  private:
    /*! Presumably this would get called when there is a collision,
      but not implemented! */
    void OnContact(const std::string &name, const physics::Contact &contact);

    /*! ROS spin once */
    void spin();

    /*! Convenience function for getting SDF parameters

     */
    double getSdfParamDouble(sdf::ElementPtr sdfPtr,const std::string &param_name,double default_val);

    /// Parameters
    std::string node_namespace_;
    std::string link_name_;

    ros::NodeHandle *rosnode_;

    ros::Publisher sensor_state_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher joint_state_pub_;

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


    // Simulation time of the last update
    common::Time prev_update_time_;
    math::Vector3 prev_lin_vel_;
    math::Vector3 prev_ang_vel_;
    math::Pose pose_;
    math::Vector3 euler_;
    math::Vector3 vel_linear_body_;
    math::Vector3 vel_angular_body_;
    math::Vector3 acceleration;
    math::Vector3 angular_velocity_;
    math::Vector3 angular_acceleration_;
    Eigen::VectorXd state_dot_;
    Eigen::VectorXd state_;
    Eigen::VectorXd amassVec_;
    Eigen::MatrixXd Cmat_;
    Eigen::VectorXd Cvec_;
    Eigen::MatrixXd Dmat_;
    Eigen::VectorXd Dvec_;

    // For Buoyancy calculation
    float buoy_frac_;
    float dx_;
    float dy_;
    std::vector<int> II_;

    // Values to set via Plugin Parameters
    /*! Plugin Parameter: Added mass in surge, X_\dot{u} */
    double param_X_dot_u_;
    /*! Plugin Parameter: Added mass in sway, Y_\dot{v} */
    double param_Y_dot_v_;
    /*! Plugin Parameter: Added mass in yaw, N_\dot{r}*/
    double param_N_dot_r_;

    /*! Plugin Parameter: Linear drag in surge */
    double param_X_u_;
    /*! Plugin Parameter: Quadratic drag in surge */
    double param_X_uu_;
    /*! Plugin Parameter: Linear drag in sway */
    double param_Y_v_;
    /*! Plugin Parameter: Quadratic drag in sway */
    double param_Y_vv_;

    double param_Z_w_;
    double param_K_p_;
    double param_M_q_;

    /*! Plugin Parameter: Linear drag in yaw */
    double param_N_r_;
    /*! Plugin Parameter: Quadratic drag in yaw*/
    double param_N_rr_;
    /*! Plugin Parameter: Boat width [m] */
    double param_boat_width_;
    /*! Plugin Parameter: Boat length [m] */
    double param_boat_length_;
    /*! Plugin Parameter: Horizontal surface area [m^2] */
    double param_boat_area_ ;
    /*! Plugin Parameter: Metacentric length [m] */
    double param_metacentric_length_;
    /*! Plugin Parameter: Metacentric width[m] */
    double param_metacentric_width_;


    double xyz_damping_;
    double yaw_damping_;
    double rp_damping_;
    /* Water height [m]*/
    double water_level_;
    /* Water density [kg/m^3] */
    double water_density_;
    /*! Added mass matrix, 6x6 */
    Eigen::MatrixXd Ma_;

    /* Wave parameters */
    int param_wave_n_;
    std::vector<float> param_wave_amps_;
    std::vector<float> param_wave_periods_;
    std::vector< std::vector<float> > param_wave_directions_;
    /* Old - for single wave
    double param_wave_amp_;
    math::Vector2d param_wave_dir_;
    double param_wave_period_;
    */

    /*! Wind velocity in Gazebo coordinates [m/s] */
    math::Vector3 param_wind_velocity_vector_;

    /*! Wind force coefficients */
    math::Vector3 param_wind_coeff_vector_;

    std::thread *spinner_thread_;

    event::ConnectionPtr contact_event_;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

  };  // class UsvPlugin
} // namespace gazebo

#endif //USV_GAZEBO_DYNAMICS_H
