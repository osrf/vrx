/*
 * Copyright (C) 20109 Brian Bingham
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

#include <functional>
#include <string>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>

#include "wave_gazebo_plugins/wavegauge_plugin.hh"
#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/WavefieldEntity.hh"
#include "wave_gazebo_plugins/WavefieldModelPlugin.hh"

using namespace asv;
using namespace gazebo;

/////////////////////////////////////////////////
WaveguagePlugin::WaveguagePlugin()
  : fluidLevel(0.0)
{
}

/////////////////////////////////////////////////
void WaveguagePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");
  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");

  // Capture the model pointer.
  this->model = _model;

  if (_sdf->HasElement("wave_model"))
  {
    this->waveModelName = _sdf->Get<std::string>("wave_model");
  }
  if (_sdf->HasElement("fluid_level"))
  {
    this->fluidLevel = _sdf->Get<double>("fluid_level");
  }
  if (_sdf->HasElement("link"))
  {
    gzmsg << "Found that SDF has at least one link element, looking at each "
          << "link..." << std::endl;
    int counter = 0;
    for (sdf::ElementPtr linkElem = _sdf->GetElement("link"); linkElem;
         linkElem = linkElem->GetNextElement("link"))
    {
      // Print each attribute for the link
      gzmsg << "Looking for name attribute in link number " << counter
            << ", which has " << linkElem->GetAttributeCount()
            << " attributes" << std::endl;
      counter++;
      int id = -1;
      std::string name = "";

      if (linkElem->HasElement("name"))
      {
        name = linkElem->GetElement("name")->Get<std::string>();
        gzmsg << "Found link name in SDF [" << name << "]" << std::endl;
        physics::LinkPtr link = _model->GetLink(name);
        if (!link)
        {
          gzwarn << "Specified link [" << name << "] not found." << std::endl;
          continue;
        }
        id = link->GetId();
        // Add this link to our list for applying buoy forces
        this->buoyancyLinks.push_back(link);
      }
      else
      {
        gzwarn << "Missing 'name' element within link number ["
               << counter - 1 << "] in SDF" << std::endl;
        // Exit if we didn't set ID
        continue;
      }

      if (this->volPropsMap.count(id) != 0)
      {
        gzwarn << "Properties for link [" << name << "] already set, skipping "
               << "second property block" << std::endl;
        continue;
      }

      if (linkElem->HasElement("center_of_volume"))
      {
        ignition::math::Vector3d cov = linkElem->GetElement("center_of_volume")
            ->Get<ignition::math::Vector3d>();
        this->volPropsMap[id].cov = cov;
      }
      else
      {
        gzwarn << "Required element center_of_volume missing from link ["
               << name << "] in WaveguagePlugin SDF" << std::endl;
        continue;
      }
    }
  }
  }

/////////////////////////////////////////////////
void WaveguagePlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WaveguagePlugin::OnUpdate, this));

}

/////////////////////////////////////////////////
void WaveguagePlugin::OnUpdate()
{
  // Retrieve the wave model...
  std::shared_ptr<const WaveParameters> waveParams 
    = WavefieldModelPlugin::GetWaveParams(
      this->model->GetWorld(), this->waveModelName);

  // No ocean waves...
  if (waveParams == nullptr)
  {
    return;
  }
	
  double simTime = this->model->GetWorld()->SimTime().Double();
  for (auto &link : this->buoyancyLinks)
  {
    #if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Pose3d linkFrame = link->WorldPose();
    #else
      ignition::math::Pose3d linkFrame = link->GetWorldPose().Ign();
    #endif

    // Compute the wave displacement at the centre of the link frame.
    // Wavefield height at the link, relative to the mean water level.
		//double lastz =  linkFrame.Pos().Z();
		//double waveHeight = WavefieldSampler::ComputeDepthDirectly(
    //  *waveParams, linkFrame.Pos(),simTime);
		double waveHeightS = WavefieldSampler::ComputeDepthSimply(
      *waveParams, linkFrame.Pos(),simTime);
		/*
		gzdbg << "Waveheight: directly = " << waveHeight
					<< ", simply = " << waveHeightS << std::endl;
		*/
		
		waveHeightS += this->fluidLevel;
		// Set vertical location to match the wave height
		// Use simple method for now - seems more consistent with visual render.
		//linkFrame.Pos().Z(waveHeight);
		linkFrame.Pos().Z(waveHeightS);
		
    link->SetWorldPose(linkFrame);
		/*
		gzdbg << "linkFrame.Pos().Z(), before =  " << lastz
					<< ", after = " <<  linkFrame.Pos().Z()
					<< ", waveHeight = " << waveHeight << std::endl;
		*/
  }
}

GZ_REGISTER_MODEL_PLUGIN(WaveguagePlugin)
