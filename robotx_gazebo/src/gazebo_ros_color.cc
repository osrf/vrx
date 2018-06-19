#include <robotx_gazebo/gazebo_ros_color.hh>

// Register plugin with gazebo
GZ_REGISTER_VISUAL_PLUGIN(GazeboRosColor)

void GazeboRosColor::Load(gazebo::rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");

  // Quit if ros plugin was not loaded
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("camera", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Load namespace from sdf if available
  std::string ns = "";
  if (_sdf->HasElement("robotNamespace"))
    ns = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    ROS_INFO_NAMED("gazebo_ros_color_plugin", "missing <robotNamespace>, defaulting to %s", ns.c_str());

  // Load topic from sdf if available
  std::string topic_name = "color";
  if (_sdf->HasElement("topicName"))
    topic_name = _sdf->GetElement("topicName")->Get<std::string>();
  else
    ROS_INFO_NAMED("gazebo_ros_color_plugin", "missing <topicName>, defaulting to %s", topic_name.c_str());

  // Copy parent into class for use in callback
  visual_ = _parent;

  // Setup node handle and subscriber
  nh_ = ros::NodeHandle(ns);
  color_sub_ = nh_.subscribe(topic_name, 1, &GazeboRosColor::colorCallback, this);
}

void GazeboRosColor::colorCallback(const std_msgs::ColorRGBAConstPtr& msg)
{
  // Convert ROS color to gazebo color
  gazebo::common::Color gazebo_color(msg->r, msg->g, msg->b, msg->a);

  // Set parent's color to message color
  visual_->SetAmbient(gazebo_color);
  visual_->SetDiffuse(gazebo_color);
}
