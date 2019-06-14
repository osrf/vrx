/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author Dave Coleman
 *  \desc   Example ROS plugin for Gazebo
 *  \Original plugin from gazebo_plugins package 
 *  \(https://github.com/ros-simulation/gazebo_ros_pkgs), 
 *  \modified to implement a ROS interface to gazebo's GPS sensor 
 *  \by Jonathan Wheare.
 *  
 */

#include <usv_gazebo_plugins/usv_gazebo_gps_plugin.hh>
#include <gazebo/common/CommonIface.hh>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(USVGazeboGPS);

////////////////////////////////////////////////////////////////////////////////
// Constructor
USVGazeboGPS::USVGazeboGPS()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
USVGazeboGPS::~USVGazeboGPS()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void USVGazeboGPS::Load( sensors::SensorPtr _parent, sdf::ElementPtr _sdf )
{

  //initialise the gazebo communications
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init();

  // Store pointer to model for later use.
  this->sensor = _parent;

  // From gazebo_ros_color plugin
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");

  // Make sure the ROS node for Gazebo has already been initialised
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("usv_gazebo_gps_plugin", "A ROS node for Gazebo \
    has not been initialised, unable to load plugin. " << "Load the Gazebo \
    system plugin 'libusv_gazebo_gps_plugin.so' in the gazebo_ros package)");
    return;
  }

  // From gazebo_ros_color plugin
  // Load namespace from SDF if available. Otherwise, use the model name.
  std::string modelName = _parent->Name();
  auto delim = modelName.find(":");
  if (delim != std::string::npos)
  {
    modelName = modelName.substr(0, delim);
  }

  // Initialise the namespace
  std::string ns = modelName;
  if (_sdf->HasElement("robotNamespace"))
    ns = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
  {
    ROS_DEBUG_NAMED("usv_gazebo_pinger_plugin",
      "missing <robotNamespace>, defaulting to %s", ns.c_str());
  }

  // Set the frame_id.  Defaults to "gps".
  this->frameId = "gps";
  if (_sdf->HasElement("frameId"))
  {
    this->frameId = _sdf->GetElement("frameId")->Get<std::string>();
  }

  // Load topic from sdf if available
  std::string topicName = "/gps/fix";
  if (_sdf->HasElement("topicName"))
    topicName = _sdf->GetElement("topicName")->Get<std::string>();
  else
  {
    ROS_INFO_NAMED("usv_gazebo_gps_plugin",
      "missing <topicName>, defaulting to %s", topicName.c_str());
  }
  
  // Load topic from sdf if available
  std::string velTopicName = "/gps/vel";
  if (_sdf->HasElement("velTopicName"))
    velTopicName = _sdf->GetElement("velTopicName")->Get<std::string>();
  else
  {
    ROS_INFO_NAMED("usv_gazebo_gps_plugin",
      "missing <velTopicName>, defaulting to %s", velTopicName.c_str());
  }

  // Load covariance
  this->covarianceNorth = 1.0;
  if (_sdf->HasElement("covarianceNorth"))
    this->covarianceNorth = _sdf->GetElement("covarianceNorth")->Get<double>();

  // Load covariance
  this->covarianceEast = 1.0;
  if (_sdf->HasElement("covarianceEast"))
    this->covarianceEast = _sdf->GetElement("covarianceEast")->Get<double>();

  // Load covariance
  this->covarianceUp = 1.0;
  if (_sdf->HasElement("covarianceUp"))
    this->covarianceUp = _sdf->GetElement("covarianceUp")->Get<double>();

  // Load covariance type
  this->covarianceType =1;
  if (_sdf->HasElement("covarianceType"))
    this->covarianceType = _sdf->GetElement("covarianceType")->Get<int>();  

  // initialise the ros handle
  this->rosNodeHandle.reset(new ros::NodeHandle(ns));

  // setup the publishers
  // Publisher for the GPS fix
  this->fixPub = this->rosNodeHandle \
  ->advertise<sensor_msgs::NavSatFix>(std::string(topicName), 10);
  
  // Publisher for velocity
  this->velPub = this->rosNodeHandle \
  ->advertise<geometry_msgs::Vector3Stamped>(std::string(velTopicName), 10);  
  
  // Manually create the topic name.  This would normally be done using the 
  // Topic() method on the sensor object, but this currently gives an incorrect 
  // result.  Both methods are currently being printed for debug purposes.
  std::string internalTopicName = "~/" + _parent->ParentName() + "/" + 
      _parent->Name(); 
  common::replaceAll(internalTopicName, internalTopicName, "::", "/");      
  
  // Log generated and sensor created topic names
  ROS_INFO_NAMED("usv_gazebo_gps_plugin","reported callback topic %s", 
      _parent->Topic().c_str());
  ROS_INFO_NAMED("usv_gazebo_gps_plugin","starting callback using topic %s", 
      internalTopicName.c_str());
            
  ignition::math::Vector3d poseCenter(0,0,0);
  //ignition::math::Vector3d spherical = this->sensor->dataPtr->sphericalCoordinates->SphericalFromLocal(poseCenter);
      
  // connect the update function to the world update event.  
  this->gps_sub_ = this->gazebo_node_->Subscribe(internalTopicName, 
    &USVGazeboGPS::OnGPS, this);

  // Turn the sensor on
  _parent->SetActive(true);    
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void USVGazeboGPS::OnGPS(ConstGPSPtr &_msg)
{
    // Publish a ROS message.    
    sensor_msgs::NavSatFix msg;
    // generate ROS header.  Sequence number is automatically populated.
    msg.header.stamp = ros::Time::now();
    // frame_id is neccesary for finding the tf transform.  The frame_id is 
    // specified in the sdf file.
    msg.header.frame_id = this->frameId;
    // Fill out the members of the message.
    msg.longitude = _msg->longitude_deg();    
    msg.latitude = _msg->latitude_deg();
    msg.altitude = _msg->altitude();
    // Fill covariance.  Currently only diagonal covariance is supported.
    msg.position_covariance_type = this->covarianceType;
    msg.position_covariance[0] = this->covarianceEast;
    msg.position_covariance[4] = this->covarianceNorth;
    msg.position_covariance[8] = this->covarianceUp;            
    // publish the fix position
    this->fixPub.publish(msg);
    // publish the velocity
    geometry_msgs::Vector3Stamped velMsg;
    velMsg.header.stamp = ros::Time::now();
    velMsg.header.frame_id = this->frameId;
    velMsg.vector.x = _msg->velocity_east();
    velMsg.vector.y = _msg->velocity_north();
    velMsg.vector.z = _msg->velocity_up();    
    this->velPub.publish(velMsg);
}

}

