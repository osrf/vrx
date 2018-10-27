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
 *  \modified to implement a simulated pinger sensor for Maritime ASVs
 *  \Modifications by Jonathan Wheare.
 *  
 */

#include <usv_gazebo_plugins/usv_gazebo_pinger_plugin.hh>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(USVGazeboPinger);

////////////////////////////////////////////////////////////////////////////////
// Constructor
USVGazeboPinger::USVGazeboPinger()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
USVGazeboPinger::~USVGazeboPinger()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void USVGazeboPinger::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{

  // Store pointer to model for later use.
  this->model = _parent;

  // From gazebo_ros_color plugin
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");

  // Make sure the ROS node for Gazebo has already been initialised
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("usv_gazebo_pinger_plugin", "A ROS node for Gazebo \
    has not been initialised, unable to load plugin. " << "Load the Gazebo \
    system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // From gazebo_ros_color plugin
  // Load namespace from SDF if available. Otherwise, use the model name.
  std::string modelName = _parent->GetName();
  auto delim = modelName.find(":");
  if (delim != std::string::npos)
  {
    modelName = modelName.substr(0, delim);
  }

  //Initialise the namespace
  std::string ns = modelName;
  if (_sdf->HasElement("robotNamespace"))
    ns = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
  {
    ROS_DEBUG_NAMED("usv_gazebo_pinger_plugin",
      "missing <robotNamespace>, defaulting to %s", ns.c_str());
  }

  // Set the frame_id.  Defaults to "pinger".
  this->frameId="pinger";
  if (_sdf->HasElement("frameId"))
  {
    this->frameId = _sdf->GetElement("frameId")->Get<std::string>();
  }

  // Load topic from sdf if available
  std::string topicName = "/pinger/range_bearing";
  if (_sdf->HasElement("topicName"))
    topicName = _sdf->GetElement("topicName")->Get<std::string>();
  else
  {
    ROS_INFO_NAMED("gazebo_pinger_plugin",
      "missing <topicName>, defaulting to %s", topicName.c_str());
  }
  
  // Set the topic to be used to publish the senor message
  std::string setPostionTopicName = "/pinger/set_pinger_position";
  if (_sdf->HasElement("setPositionTopicName"))
  {
    setPostionTopicName = _sdf->GetElement("setPositionTopicName")\
    ->Get<std::string>();
  }
  else
  {
    ROS_INFO_NAMED("gazebo_pinger_plugin",
      "missing <setPositionTopicName>, defaulting to %s", topicName.c_str());
  }
  
  // Initialise pinger position.  Defaults to origin
  this->position = math::Vector3(0,0,0); //Default to origin
  if (_sdf->HasElement("position"))
  {
    this->position = _sdf->Get<math::Vector3>("position");
  }

  // Initialise update rate
  this->updateRate = 1.0; //default to 1 reading per second.
  if (_sdf->HasElement("updateRate"))
  {
    this->updateRate = _sdf->Get<float>("updateRate");
  }

  // From Brian Binghams rangebearing_gazebo_plugin.  
  // Noise setup and parse SDF  
  if (_sdf->HasElement("rangeNoise"))
  {
    sdf::ElementPtr rangeNoiseElem = _sdf->GetElement("rangeNoise");
    // Note - it is hardcoded into the NoiseFactory.cc that the SDF
    // element be "noise".
    if (rangeNoiseElem->HasElement("noise"))
    {
      this->rangeNoise =
	sensors::NoiseFactory::NewNoiseModel(rangeNoiseElem->GetElement("noise"));
    }
    else{
      this->rangeNoise = nullptr;
      ROS_WARN("RangeBearing Plugin: The rangeNoise SDF element must contain noise tag");
    }
  }
  else
  {
    this->rangeNoise = nullptr;
    ROS_INFO("RangeBearing Plugin: No rangeNoise tag found, no noise added to measurements");
  }
  
  // Load the noise model from the SDF file.
  if (_sdf->HasElement("bearingNoise"))
  {
    sdf::ElementPtr bearingNoiseElem = _sdf->GetElement("bearingNoise");
    // Note - it is hardcoded into the NoiseFactory.cc that the SDF
    // element be "noise".
    if (bearingNoiseElem->HasElement("noise"))
    {
      this->bearingNoise =
	sensors::NoiseFactory::NewNoiseModel(bearingNoiseElem->GetElement("noise"));
    }
    else{
      this->bearingNoise = nullptr;
      ROS_WARN("The bearingNoise SDF element must contain noise tag");
    }
  }
  else
  {
    this->bearingNoise = nullptr;
    ROS_INFO("RangeBearing Plugin: No bearingNoise tag found, no noise added to measurements");
  }

  if (_sdf->HasElement("elevationNoise"))
  {
    sdf::ElementPtr elevationNoiseElem = _sdf->GetElement("elevationNoise");
    // Note - it is hardcoded into the NoiseFactory.cc that the SDF
    // element be "noise".
    if (elevationNoiseElem->HasElement("noise"))
    {
      this->elevationNoise =
	sensors::NoiseFactory::NewNoiseModel(elevationNoiseElem->GetElement("noise"));
    }
    else{
      this->elevationNoise = nullptr;
      ROS_WARN("The elevationNoise SDF element must contain noise tag");
    }
  }
  else
  {
    this->elevationNoise = nullptr;
    ROS_INFO("RangeBearing Plugin: No elevationNoise tag found, no noise added to measurements");
  }

  // initialise the ros handle
  this->rosNodeHandle.reset(new ros::NodeHandle(ns));

  // setup the publisher
  this->rangeBearingPub = this->rosNodeHandle \
  ->advertise<usv_msgs::RangeBearing>(std::string(topicName), 1);

  this->setPositionSub = this->rosNodeHandle \
  ->subscribe(setPostionTopicName,1,&USVGazeboPinger::pingerPositionCallback,this);

  // intialise the time with world time
  this->lastUpdateTime = this->model->GetWorld()->GetSimTime();

  // connect the update function to the world update event.
  this->updateConnection = \
  event::Events::ConnectWorldUpdateBegin(std::bind(&USVGazeboPinger::UpdateChild, this));

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void USVGazeboPinger::UpdateChild()
{
  // Test to see if it's time to generate a sensor reading
  if ((this->model->GetWorld()->GetSimTime() - this->lastUpdateTime) \
  > (float{1.0}/updateRate))
  {
    // lock the thread to protect this->position vector.
    std::lock_guard<std::mutex> lock(this->mutex);
    this->lastUpdateTime = this->model->GetWorld()->GetSimTime();

    // Find the pose of the model.
    math::Pose model_pose = this->model->GetWorldPose();
    
    // Direction vector to the pinger from the USV
    math::Vector3 direction = this->position - model_pose.pos;
    
    // Sensor reading is in the sensor frame.  Rotate the direction vector into
    // the frame of reference of the sensor.
    math::Vector3 direction_sensor_frame = \
      model_pose.rot.RotateVectorReverse(direction);
    
    // Generate a 2d vector for elevation calculation.
    math::Vector3 direction_sensor_frame_2d = \
        math::Vector3(direction_sensor_frame.x,direction_sensor_frame.y,0);

    // bearing is calculated by finding the world frame direction vector
    // and subtracting the pose of the vehicle.  A noise value is added.
    double bearing = atan2(direction_sensor_frame.y,direction_sensor_frame.x);
    double range = direction_sensor_frame.GetLength(); 
    double elevation = atan2(direction_sensor_frame.z, \
      direction_sensor_frame_2d.GetLength());
        
    // Apply noise to each measurement.  
    // From Brian Binghams rangebearing_gazebo_plugin.
    if (this->rangeNoise != nullptr)
      range = this->rangeNoise->Apply(range);
    if (this->bearingNoise != nullptr)
      bearing = this->bearingNoise->Apply(bearing);
    if (this->elevationNoise != nullptr)
      elevation  = this->elevationNoise->Apply(elevation);	

    // Publish a ROS message.    
    usv_msgs::RangeBearing msg;
    // generate ROS header.  Sequence number is automatically populated.
    msg.header.stamp = ros::Time(this->lastUpdateTime.sec, this->lastUpdateTime.nsec);
    // frame_id is neccesary for finding the tf transform.  The frame_id is 
    // specified in the sdf file.
    msg.header.frame_id = this->frameId;
    // Fill out the members of the message.
    msg.range = range;
    msg.bearing = bearing;
    msg.elevation = elevation;    

    // publish range and bearing message
    this->rangeBearingPub.publish(msg);
  }  
}

void USVGazeboPinger::pingerPositionCallback(const geometry_msgs::Vector3ConstPtr &msg)
{
  // Mutex added to prevent simulataneous reads and writes of mutex.
  // May not be neccesary, as this thread will only write, while the UpdateChild 
  // thread only reads.
  std::lock_guard<std::mutex> lock(this->mutex);
  this->position = math::Vector3(msg->x,msg->y,msg->z);
}

}

