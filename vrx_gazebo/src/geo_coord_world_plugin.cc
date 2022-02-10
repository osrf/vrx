#include "vrx_gazebo/geo_coord_world_plugin.hh"

/////////////////////////////////////////////////
GeoCoordWorldPlugin::GeoCoordWorldPlugin()
    : WorldPlugin(), gzNode(new gazebo::transport::Node())
{
}

void GeoCoordWorldPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "GeoCoordWorldPlugin::Load(): NULL world pointer");
  this->world = _world;
  if (_sdf) {
    if (_sdf->HasElement("world_topic"))
      this->worldTopic = _sdf->Get<std::string>("world_topic");
    if (_sdf->HasElement("geo_topic"))
      this->geoTopic = _sdf->Get<std::string>("geo_topic");
  }
  // Initialize transport.

  this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gzNode->Init("world_to_geo_handler");
  geoPub = this->gzNode->Advertise<gazebo::msgs::Vector3d>
    (geoTopic);
  worldSub = this->gzNode->Subscribe(worldTopic,
      &GeoCoordWorldPlugin::callback, this);
}

void GeoCoordWorldPlugin::callback(ConstVector3dPtr &msg_loc)
{
  // Invert local coords to account for SphericalCoords frame bug
  ignition::math::Vector3d mod_loc(-msg_loc->x(), -msg_loc->y(),
      msg_loc->z());

  // Conversion from Gazebo Cartesian coordinates to spherical.
#if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Vector3d latlon =
      this->world->SphericalCoords()->SphericalFromLocal(mod_loc);
#else
    const ignition::math::Vector3d latlon =
      this->world->GetSphericalCoordinates()->SphericalFromLocal(mod_loc);
#endif
gazebo::msgs::Vector3d latlon_msg;
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::msgs::Set(&latlon_msg,
      gazebo::math::Vector3(latlon.X, latlon.Y, latlon.Z));
#else
  gazebo::msgs::Set(&latlon_msg, latlon);
#endif
gzmsg << "Local->Spherical Lat: " << latlon.X() <<
          " Lon: " << latlon.Y() << std::endl;
this->geoPub->Publish(latlon_msg);
}

// Register plugin with gazebo
GZ_REGISTER_WORLD_PLUGIN(GeoCoordWorldPlugin)