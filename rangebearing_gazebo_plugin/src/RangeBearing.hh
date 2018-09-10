#ifndef _RANGEBEARING_HH_
#define _RANGEBEARING_HH_

#include <cmath>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/GaussianNoiseModel.hh"

#include "gazebo/util/system.hh"

#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>

#include <gazebo/common/common.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Pose.hh>

#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include "update_timer.h"

#include <algorithm>    // std::min

// Constrain agle to be -pi --- pi
inline double constrainAngle(double x){
  double pi = 3.14159265359;
  x = std::fmod(x + pi,2.0*pi);
  if (x < 0)
    x += 2*pi;
  return x - pi;
}

// Saturate
inline double clamp(double val, double top, double bottom)
{
  return std::max(bottom, std::min(val, top));
}


namespace gazebo
{
  /// \class RaySensor RaySensor.hh sensors/sensors.hh
  /// \brief Sensor with one or more rays.
  ///
  /// This sensor cast rays into the world, tests for intersections, and
  /// reports the range to the nearest object.  It is used by ranging
  /// sensor models (e.g., sonars and scanning laser range finders).
  //class GZ_SENSORS_VISIBLE RangeBearing : public Sensor
  class RangeBearing : public ModelPlugin
  {
    /// \brief Constructor
    public: RangeBearing();
    
    /// \brief Destructor
    public: virtual ~RangeBearing() = default;

    // Documentation inherited
    protected: virtual void Load(physics::ModelPtr _model,
				 sdf::ElementPtr _sdf);

    // Documentation inherited
    protected: virtual void Update();

    private:
      /// \brief The parent World
      physics::WorldPtr world;

      /// \brief Pointer to the model
      physics::ModelPtr model;

      /// \brief Pointer to link
      physics::LinkPtr link;

      /// \brief Name of the link the plugin is attached to
      std::string linkName;

      /// \brief Pointer to the update event connection.
      event::ConnectionPtr updateConnection;

      /// \brief Update timer
      UpdateTimer updateTimer;

      /// \brief ROS node handler 
      ros::NodeHandle* nodeHandle;

      /// \brief ROS publiser for range bearing measurements
      ros::Publisher rbPublisher;

      /// \brief Topic for publishing range bearing meaurements
      std::string rbTopic;

      /// \brief Message for publication
      std_msgs::Float32MultiArray rbMsg;

      /// \brief ROS namespace
      std::string rosNamespace;

      /// \brief The x,y,z location of the beacon we are ranging to, Gazebo coordinates
      math::Vector3 beaconPoint;

      /// \brief Gazebo noise object for range
      gazebo::sensors::NoisePtr rangeNoise;

      /// \brief Gazebo noise object for bearing angle
      gazebo::sensors::NoisePtr bearingNoise;

      /// \brief Gazebo noise object for elevation angle
      gazebo::sensors::NoisePtr elevationNoise;
    
  };
}

#endif
