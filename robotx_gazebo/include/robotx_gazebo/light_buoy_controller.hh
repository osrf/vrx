#pragma once

#include <boost/bind.hpp>
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/rendering/Visual.hh"
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Trigger.h>
#include "ros/ros.h"

namespace light_buoy_controller
{


class LightBuoyController : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  static void init();
private:
  static std_msgs::ColorRGBA color(double, double, double, double);
  static const std_msgs::ColorRGBA RED;
  static const std_msgs::ColorRGBA GREEN;
  static const std_msgs::ColorRGBA BLUE;
  static const std_msgs::ColorRGBA YELLOW;
  static const std_msgs::ColorRGBA OFF;
  typedef std::pair<std_msgs::ColorRGBA, std::string> colors_t;
  static const colors_t colors[4];

  // Subscriber to color 
  ros::Publisher panel_pubs_[3];
  // Node handle
  ros::NodeHandle nh_;
  // Service
  ros::ServiceServer change_pattern_server_;
  // Timer
  ros::Timer timer_;
  // Track index
  uint8_t state_ = 0;
  // Callback
  void incrementState(const ros::TimerEvent& _event);
  void incrementState();
  std_msgs::ColorRGBA pattern_[4];
  // Callback
  bool changePattern(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  void changePattern(std::string& new_pattern);
  // Lock
  std::mutex mutex_;
};

} // namespace light_buoy_controller
