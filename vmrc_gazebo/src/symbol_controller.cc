/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <gazebo/physics/Model.hh>
#include <ignition/math/Rand.hh>

#include "vmrc_gazebo/symbol_controller.hh"

//////////////////////////////////////////////////
/// \brief Creates a std_msgs::ColorRGBA message from 4 doubles.
/// \param[in] _r Red.
/// \param[in] _g Green.
/// \param[in] _b Blue.
/// \param[in] _a Alpha.
/// \return The ColorRGBA message.
static std_msgs::ColorRGBA createColor(const double _r,
  const double _g, const double _b, const double _a)
{
  static std_msgs::ColorRGBA color;
  color.r = _r;
  color.g = _g;
  color.b = _b;
  color.a = _a;
  return color;
}

// Static initialization.
std::map<std::string, std_msgs::ColorRGBA> SymbolController::kColors =
  {
    {"red",   createColor(1.0, 0.0, 0.0, 1.0)},
    {"green", createColor(0.0, 1.0, 0.0, 1.0)},
    {"blue",  createColor(0.0, 0.0, 1.0, 1.0)},
  };
std::vector<std::string> SymbolController::kShapes =
  {"circle", "cross", "triangle"};

//////////////////////////////////////////////////
void SymbolController::Load(gazebo::physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");

  // Quit if ros plugin was not loaded
  if (!ros::isInitialized())
  {
    ROS_ERROR("ROS was not initialized.");
    return;
  }

  // Load namespace from SDF if available. Otherwise, use the model name.
  std::string modelName = _parent->GetName();
  auto delim = modelName.find(":");
  if (delim != std::string::npos)
    modelName = modelName.substr(0, delim);


  std::string ns = modelName;
  if (_sdf->HasElement("robotNamespace"))
    ns = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
  {
    ROS_DEBUG_NAMED("symbolController",
                    "missing <robotNamespace>, defaulting to %s", ns.c_str());
  }

  // Parse the shape. We initialize it with a random shape.
  this->ShuffleShape();
  if (_sdf->HasElement("shape"))
  {
    std::string aShape = _sdf->GetElement("shape")->Get<std::string>();
    std::transform(aShape.begin(), aShape.end(), aShape.begin(), ::tolower);
    // Sanity check: Make sure the shape is allowed.
    if (std::find(this->kShapes.begin(), this->kShapes.end(), aShape) !=
          this->kShapes.end())
    {
      this->shape = aShape;
    }
    else
    {
      ROS_INFO_NAMED("symbolController",
                  "incorrect [%s] <shape>, using random shape", aShape.c_str());
    }
  }

  // Parse the color. We initialize it with a random color.
  this->ShuffleColor();
  if (_sdf->HasElement("color"))
  {
    std::string aColor = _sdf->GetElement("color")->Get<std::string>();
    std::transform(aColor.begin(), aColor.end(), aColor.begin(), ::tolower);
    // Sanity check: Make sure the color is allowed.
    if (this->kColors.find(aColor) != this->kColors.end())
    {
      this->color = aColor;
    }
    else
    {
      ROS_INFO_NAMED("symbolController",
                  "incorrect [%s] <color>, using random color", aColor.c_str());
    }
  }

  this->nh = ros::NodeHandle(ns);

  // Create publisher to set symbols
  auto iterShapes = this->kShapes.begin();
  for (auto i = 0; i < this->kShapes.size(); ++i)
  {
    std::string topic = *iterShapes;
    std::advance(iterShapes, 1);
    this->symbolPubs.push_back(
      this->nh.advertise<std_msgs::ColorRGBA>(topic, 1u, true));
  }

  // This timer is used to defer the symbol publication a few seconds to make
  // sure that all subscribers receive the message.
  this->timer = this->nh.createTimer(
    ros::Duration(5.0), &SymbolController::Publish, this, true);

  // Advertise the service to shuffle shape and color.
  this->changePatternServer = this->nh.advertiseService(
    "shuffle", &SymbolController::Shuffle, this);
}

//////////////////////////////////////////////////
bool SymbolController::Shuffle(std_srvs::Trigger::Request &/*_req*/,
  std_srvs::Trigger::Response &_res)
{
  this->Shuffle();
  _res.success = true;
  return _res.success;
}

//////////////////////////////////////////////////
void SymbolController::Shuffle()
{
  this->ShuffleShape();
  this->ShuffleColor();
  this->Publish();
}

//////////////////////////////////////////////////
void SymbolController::ShuffleShape()
{
  std::string newShape;
  do
  {
    newShape =
      this->kShapes[ignition::math::Rand::IntUniform(0,
        this->kShapes.size() - 1)];
  }
  while (newShape == this->shape);
  this->shape = newShape;
}

//////////////////////////////////////////////////
void SymbolController::ShuffleColor()
{
  std::string newColor;
  do
  {
    auto iterColor = this->kColors.begin();
    std::advance(iterColor,
               ignition::math::Rand::IntUniform(0, this->kColors.size() - 1));
    newColor = (*iterColor).first;
  }
  while (newColor == this->color);
  this->color = newColor;
}

//////////////////////////////////////////////////
void SymbolController::Publish(const ros::TimerEvent &/*_event*/)
{
  this->Publish();
}

//////////////////////////////////////////////////
void SymbolController::Publish()
{
  for (auto i = 0; i < this->kShapes.size(); ++i)
  {
    std_msgs::ColorRGBA msg;
    msg.a = 0.0;

    auto topic = this->symbolPubs[i].getTopic();
    auto delim = topic.rfind("/");
    auto aShape = topic.substr(delim + 1);
    if (aShape == this->shape)
      msg = this->kColors[this->color];

    this->symbolPubs[i].publish(msg);
  }
}

// Register plugin with gazebo
GZ_REGISTER_MODEL_PLUGIN(SymbolController)
