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

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include "vmrc_gazebo/scoring_plugin.hh"

/////////////////////////////////////////////////
ScoringPlugin::ScoringPlugin()
{
}

/////////////////////////////////////////////////
void ScoringPlugin::Load(gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "ScoringPlugin::Load(): NULL world pointer");
  GZ_ASSERT(_sdf,   "ScoringPlugin::Load(): NULL _sdf pointer");

  this->world = _world;

  // This is a required element.
  if (!_sdf->HasElement("task_name"))
  {
    gzerr << "Unable to find <task_name> element in SDF." << std::endl;
    return;
  }
  this->taskName = _sdf->Get<std::string>("task_name");

  // This is a required element.
  if (!_sdf->HasElement("max_time"))
  {
    gzerr << "Unable to find <max_time> element in SDF." << std::endl;
    return;
  }
  this->maxTime = _sdf->Get<uint32_t>("max_time");

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ScoringPlugin::Update, this));
}

//////////////////////////////////////////////////
void ScoringPlugin::Update()
{

}
