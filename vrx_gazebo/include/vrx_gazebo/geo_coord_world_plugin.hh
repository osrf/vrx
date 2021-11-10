#ifndef VRX_GAZEBO_GEO_COORD_WORLD_PLUGIN_HH_
#define VRX_GAZEBO_GEO_COORD_WORLD_PLUGIN_HH_

#include <ros/ros.h>
#include <vector>
#include "gazebo/msgs/msgs.hh"
#include "geometry_msgs/Vector3.h"
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>

class GeoCoordWorldPlugin : public gazebo::WorldPlugin
{
    /// \brief Class constructor.
  public: GeoCoordWorldPlugin();

  // Documentation inherited.
  protected: void Load(gazebo::physics::WorldPtr _world,
                       sdf::ElementPtr _sdf);

  protected: void mouseCallback(ConstVector3dPtr &msg_loc);

  /// \brief A world pointer.
  protected: gazebo::physics::WorldPtr world;  
  
  /// \brief Topic where the transformed mouse coordinate is published.
  private: std::string mouseGeoTopic = "/vrx/mouse_geo_loc";

    /// \brief Topic where world frame mouse coordinate is published.
  private: std::string mouseWorldTopic = "/vrx/mouse_world_loc";
  
  /// \brief gazebo node pointer
  private: gazebo::transport::NodePtr gzNode;

    /// \brief ROS node handle.
  private: std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief Publisher for the geo frame mouse coordinate.
  private: gazebo::transport::PublisherPtr mouseGeoPub;

  /// \brief Subscriber for the world frame mouse coordinate.
    private: gazebo::transport::SubscriberPtr mouseWorldSub;
};

#endif