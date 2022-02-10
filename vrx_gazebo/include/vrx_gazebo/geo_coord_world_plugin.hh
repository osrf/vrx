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

  /// \brief A world pointer.
  protected: gazebo::physics::WorldPtr world; 

  /// \brief A pointer to the sdf.
  protected: sdf::ElementPtr _sdf; 
  
  /// \brief Topic where the transformed coordinate is published.
  private: std::string geoTopic = "/vrx/mouse_geo_loc";

    /// \brief Topic where world frame coordinate is published.
  private: std::string worldTopic = "/vrx/mouse_world_loc";
  
  /// \brief gazebo node pointer
  private: gazebo::transport::NodePtr gzNode;

  /// \brief Publisher for the geo frame coordinates.
  private: gazebo::transport::PublisherPtr geoPub;

  /// \brief Subscriber for the world frame coordinates.
    private: gazebo::transport::SubscriberPtr worldSub;

  /// \brief Callback for worldSub topic.  Converts received world
  ///        frame coordinates to lat/ long and republishes on geoPub.
  protected: void callback(ConstVector3dPtr &msg_loc);
};

#endif