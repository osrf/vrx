#ifndef ANIMATED_BOX_H
#define ANIMATED_BOX_H

#include <boost/bind.hpp>
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/rendering/Visual.hh"
#include <std_msgs/ColorRGBA.h>
#include "ros/ros.h"

class GazeboRosColor : public gazebo::VisualPlugin
{
public:
  void Load(gazebo::rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/);

private:
  // Pointer to the model
  gazebo::rendering::VisualPtr visual_ = nullptr;
  // Subscriber to color 
  ros::Subscriber color_sub_;
  // Node handle
  ros::NodeHandle nh_;
  // Callback
  void colorCallback(const std_msgs::ColorRGBAConstPtr& msg);
};

#endif  // ANIMATED_BOX_H
