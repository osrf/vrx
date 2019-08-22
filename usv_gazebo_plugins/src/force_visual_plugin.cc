/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <algorithm>
#include <functional>
#include "usv_gazebo_plugins/force_visual_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
void ForceVisualPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // parse parameters
  if (_sdf->HasElement("scaling"))
  {
    this->scaling = _sdf->GetElement("scaling")->Get<double>();
  }

  // Store the pointer to the model
  this->model = _parent;

  // Get vector of links in model
  this->links = this->model->GetLinks();

#if GAZEBO_MAJOR_VERSION >= 8
  // advertise on marker topic
  if (!this->node.Advertise<ignition::msgs::Marker>(this->markerTopic))
  {
    gzerr << "Error advertising service ["
          << this->markerTopic << "]" << std::endl;
    return;
  }
#endif

  // set namespace for marker
  this->ns = "force_visualize/" + this->model->GetName();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ForceVisualPlugin::Update, this));
}

//////////////////////////////////////////////////
void ForceVisualPlugin::Update()
{
  for (const auto & link : this->links)
  {
    if (!this->PublishMarker(link))
    {
      gzwarn << "Error publishing marker message" << std::endl;
    }
  }
}

//////////////////////////////////////////////////
bool ForceVisualPlugin::PublishMarker(const gazebo::physics::LinkPtr& link)
{
#if GAZEBO_MAJOR_VERSION >= 8
  auto pos = link->WorldPose().Pos();
  auto force = link->WorldForce() / this->scaling;

  ignition::msgs::Marker markerMsg;
  markerMsg.set_ns(this->ns);
  markerMsg.set_id(link->GetId());
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);

  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name("Gazebo/Red");
    ignition::msgs::Set(markerMsg.add_point(), pos);
  ignition::msgs::Set(markerMsg.add_point(), pos + force);
  return this->node.Request(this->markerTopic, markerMsg);
#else
  gzwarn << "Gazebo markers not published (Gazebo version < 8)" << std::endl;
  return false;
#endif
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ForceVisualPlugin)
