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

#include <usv_gazebo_plugins/usv_gazebo_v3d_plugin.hh>
#include <gazebo/common/CommonIface.hh>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(USVV3D);

////////////////////////////////////////////////////////////////////////////////
// Constructor
USVV3D::USVV3D()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
USVV3D::~USVV3D()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void USVV3D::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{

  // Store pointer to model for later use.
  this->model = _parent;

  // From gazebo_ros_color plugin
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");

  // Make sure the ROS node for Gazebo has already been initialised
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("usv_gazebo_v3d_plugin", "A ROS node for Gazebo \
    has not been initialised, unable to load plugin.");
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

  // Initialise the namespace
  std::string ns = modelName;
  if (_sdf->HasElement("robotNamespace"))
    ns = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
  {
    ROS_DEBUG_NAMED("usv_gazebo_v3d_plugin",
      "missing <robotNamespace>, defaulting to %s", ns.c_str());
  }

  // Set the frame_id.  Defaults to "link".
  this->frameId = "link";
  if (_sdf->HasElement("frameId"))
  {
    this->frameId = _sdf->GetElement("frameId")->Get<std::string>();
  }

  // Initialise update rate
  this->updateRate = 10.0; //default to 1 reading per second.
  if (_sdf->HasElement("updateRate"))
  {
    this->updateRate = _sdf->Get<float>("updateRate");
  }

  // Load topic from sdf if available
  std::string topicName = "/v3d/vel";
  if (_sdf->HasElement("topicName"))
    topicName = _sdf->GetElement("topicName")->Get<std::string>();
  else
  {
    ROS_INFO_NAMED("usv_v3d_plugin",
      "missing <topicName>, defaulting to %s", topicName.c_str());
  }
  
  // initialise the ros handle
  this->rosNodeHandle.reset(new ros::NodeHandle(ns));
  
  // Publisher for velocity
  this->velPub = this->rosNodeHandle \
  ->advertise<geometry_msgs::Vector3Stamped>(std::string(topicName), 10);  

  // intialise the time with world time
  this->lastUpdateTime = this->model->GetWorld()->GetSimTime();

  // Listen to the update event broadcastes every physics iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&USVV3D::Update, this));
              
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void USVV3D::Update()
{
  // Test to see if it's time to generate a sensor reading
  if ((this->model->GetWorld()->GetSimTime() - this->lastUpdateTime) \
  > (float{1.0}/updateRate))
  {
    geometry_msgs::Vector3Stamped velMsg;
    velMsg.header.stamp = ros::Time::now();
    velMsg.header.frame_id = this->frameId;
    velMsg.vector.x = this->model->GetWorldLinearVel().x;
    velMsg.vector.y = this->model->GetWorldLinearVel().y;
    velMsg.vector.z = this->model->GetWorldLinearVel().z;    
    this->velPub.publish(velMsg);
    this->lastUpdateTime = this->model->GetWorld()->GetSimTime();
  }
}

}
