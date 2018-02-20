// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo_plugins/gazebo_ros_utils.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Scalar.h>
#include <usv_gazebo_plugins/usv_gazebo_dynamics_plugin.h>



int main(int argc, char **argv){

  gazebo::UsvPlugin usvp;
  usvp.Load(gazebo::physics::ModelPtr(),sdf::ElementPtr());
  usvp.UpdateChild();
}

