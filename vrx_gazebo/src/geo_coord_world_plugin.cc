#include "vrx_gazebo/geo_coord_world_plugin.hh"

/////////////////////////////////////////////////
GeoCoordWorldPlugin::GeoCoordWorldPlugin()
    : WorldPlugin(), gzNode(new gazebo::transport::Node())
{
}

void GeoCoordWorldPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "ScoringPlugin::Load(): NULL world pointer");
  GZ_ASSERT(_sdf,   "ScoringPlugin::Load(): NULL _sdf pointer");

  this->world = _world;
  //this->sdf = _sdf;

  // Initialize ROS transport.
  this->rosNode.reset(new ros::NodeHandle());
  this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gzNode->Init();
  mouseGeoPub = this->gzNode->Advertise<gazebo::msgs::Vector3d>
    (mouseGeoTopic);
  mouseWorldSub = this->gzNode->Subscribe(mouseWorldTopic, &GeoCoordWorldPlugin::mouseCallback, this);
}

void GeoCoordWorldPlugin::mouseCallback(ConstVector3dPtr &msg_loc)
{
  // Invert local coordinates to account for SphericalCoordinates local frame bug
  ignition::math::Vector3d mouse_loc(-msg_loc->x(), -msg_loc->y(), msg_loc->z());

        // Conversion from Gazebo Cartesian coordinates to spherical.
#if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Vector3d latlon =
      this->world->SphericalCoords()->SphericalFromLocal(mouse_loc);
#else
    const ignition::math::Vector3d latlon =
      this->world->GetSphericalCoordinates()->SphericalFromLocal(mouse_loc);
#endif
gazebo::msgs::Vector3d latlon_msg;
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::msgs::Set(&latlon_msg, gazebo::math::Vector3(latlon.X, latlon.Y, latlon.Z));
#else
  gazebo::msgs::Set(&latlon_msg, latlon);
#endif

this->mouseGeoPub->Publish(latlon_msg);
}