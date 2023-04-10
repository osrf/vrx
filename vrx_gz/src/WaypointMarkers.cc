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

#include <gz/msgs/marker.pb.h>
#include <memory>
#include <string>
#include <gz/msgs/Utility.hh>
#include "WaypointMarkers.hh"

using namespace gz;
using namespace vrx;

/////////////////////////////////////////////////
WaypointMarkers::WaypointMarkers(const std::string &_namespace)
  : ns(_namespace), scaling{0.2, 0.2, 1.5}, height(4.0)
{
}

/////////////////////////////////////////////////
void WaypointMarkers::Load(const std::shared_ptr<const sdf::Element> &_sdf)
{
  if (_sdf->HasElement("scaling"))
    this->scaling = _sdf->Get<math::Vector3d>("scaling");

  if (_sdf->HasElement("height"))
    this->height = _sdf->Get<double>("height");

  if (_sdf->HasElement("initial_id"))
    this->id = _sdf->Get<int>("initial_id");
}

/////////////////////////////////////////////////
bool WaypointMarkers::DrawMarker(double _x, double _y, double _yaw)
{
  return this->DrawMarker(this->id++, _x, _y, _yaw);
}

/////////////////////////////////////////////////
bool WaypointMarkers::DrawMarker(int _markerId, double _x, double _y,
  double _yaw)
{
  msgs::Marker markerMsg;
  markerMsg.set_ns(this->ns);
  markerMsg.set_action(msgs::Marker_Action_ADD_MODIFY);
  markerMsg.set_visibility(gz::msgs::Marker::GUI);
  markerMsg.mutable_material()->mutable_ambient()->set_r(1);
  markerMsg.mutable_material()->mutable_ambient()->set_g(1);
  markerMsg.mutable_material()->mutable_ambient()->set_b(1);
  markerMsg.mutable_material()->mutable_ambient()->set_a(1);
  markerMsg.mutable_material()->mutable_diffuse()->set_r(1);
  markerMsg.mutable_material()->mutable_diffuse()->set_g(1);
  markerMsg.mutable_material()->mutable_diffuse()->set_b(1);
  markerMsg.mutable_material()->mutable_diffuse()->set_a(1);

  // draw cylinder
  markerMsg.set_type(msgs::Marker_Type_CYLINDER);
  msgs::Set(markerMsg.mutable_scale(), this->scaling);
  msgs::Set(markerMsg.mutable_pose(),
      math::Pose3d(_x, _y, this->height + this->scaling.Z() / 2.0, 0, 0, 0));
  markerMsg.set_id(_markerId);
  bool result = node.Request("/marker", markerMsg);
  if (!result)
    return false;

  // draw direction cylinder
  markerMsg.set_type(msgs::Marker_Type_CYLINDER);
  msgs::Set(markerMsg.mutable_scale(), this->scaling);
  msgs::Set(markerMsg.mutable_pose(),  math::Pose3d(
    _x + cos(_yaw), _y + sin(_yaw), this->height + this->scaling.Z() / 2.0,
    0, M_PI / 2, _yaw));
  markerMsg.set_id((_markerId + 1) * 1000);
  result = node.Request("/marker", markerMsg);

  return result;
}
