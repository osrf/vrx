#ifndef ROBOTX_TEST_HELPERS_HH
#define ROBOTX_TEST_HELPERS_HH
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

bool ModelExists(const std::string &_name, ros::WallDuration _timeout = ros::WallDuration(5, 0))
{
  ros::WallTime timeout = ros::WallTime::now() + _timeout;
  while (ros::WallTime::now() < timeout)
  {
      gazebo_msgs::ModelStatesConstPtr modelStates = ros::topic::waitForMessage<gazebo_msgs::ModelStates>(std::string("/gazebo/model_states"), ros::Duration(0.1));
      if (!modelStates) continue;
      for(auto model: modelStates->name)
      {
        if(model == _name)
          return true;
      }
  } 
  return false;
}
#endif
