/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
//#include "plugins/BuoyancyPlugin.hh"
#include <robotx_gazebo/buoyancy_gazebo_plugin.hh>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin)

/////////////////////////////////////////////////
BuoyancyPlugin::BuoyancyPlugin()
  // Density of liquid water at 1 atm pressure and 15 degrees Celsius.
  : fluidDensity(999.1026),
  fluidLevel(0.0),
  fluidDrag(0.0)
{
  // pass
}

/////////////////////////////////////////////////
void BuoyancyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");
  this->model = _model;
  physics::WorldPtr world = _model->GetWorld();
  GZ_ASSERT(world != NULL, "Model is in a NULL world");

  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");
  this->sdf = _sdf;

  // Print the SDF document to the screen - helpful for debugging SDF
  _sdf->PrintValues("PluginSDF: ");

  if (this->sdf->HasElement("fluid_density"))
  {
    this->fluidDensity = this->sdf->Get<double>("fluid_density");
  }
  if (this->sdf->HasElement("fluid_level"))
  {
    this->fluidLevel = this->sdf->Get<double>("fluid_level");
  }
  if (this->sdf->HasElement("fluid_drag"))
  {
    this->fluidDrag = this->sdf->Get<double>("fluid_drag");
  }

  // Get "center of volume" and "volume" that were inputted in SDF
  // SDF input is recommended for mesh or polylines collision shapes

  if (this->sdf->HasElement("link"))
  {
    gzmsg << "Found that SDF has at least one link element, looking at each link..." << std::endl;
    int cnt = 0;
    for (sdf::ElementPtr linkElem = this->sdf->GetElement("link"); linkElem;
         linkElem = linkElem->GetNextElement("link"))
    {
      // Print each attribute for the link
      gzmsg << "Looking for name attribute in link number " << cnt <<
	", which has " << linkElem->GetAttributeCount() <<
	" attributes" << std::endl;
      cnt++;
      int id = -1;
      std::string name = "";
      //if (linkElem->HasAttribute("name"))
      if (linkElem->HasElement("name"))
      {
        //name = linkElem->Get<std::string>("name");
	name = linkElem->GetElement("name")->Get<std::string>();
	gzmsg << "Found link name in SDF [" << name << "]" << std::endl;
        physics::LinkPtr link = this->model->GetLink(name);
        if (!link)
        {
          gzwarn << "Specified link [" << name << "] not found." << std::endl;
          continue;
        }
        id = link->GetId();
	// Add this link to our list for applying buoy forces
	this->buoyLinks.push_back(link);
      }
      else
      {
        gzwarn << "Missing 'name' element within link number ["
	       << cnt-1 << "] in SDF" << std::endl;
        // Exit if we didn't set ID
        continue;
      }

      if (this->volPropsMap.count(id) != 0)
      {
        gzwarn << "Properties for link [" << name << "] already set, skipping "
               << "second property block" << std::endl;
      }

      if (linkElem->HasElement("center_of_volume"))
      {
	math::Vector3 cov = linkElem->GetElement("center_of_volume")
            ->Get<math::Vector3>();
        this->volPropsMap[id].cov = cov;
      }
      else
      {
        gzwarn << "Required element center_of_volume missing from link ["
               << name
               << "] in BuoyancyPlugin SDF" << std::endl;
        continue;
      }
      /*
      if (linkElem->HasElement("volume"))
      {
        double volume = linkElem->GetElement("volume")->Get<double>();
        if (volume <= 0)
        {
          gzwarn << "Nonpositive volume specified in BuoyancyPlugin!"
                 << std::endl;
          // Remove the element from the map since it's invalid.
          this->volPropsMap.erase(id);
          continue;
        }
        this->volPropsMap[id].volume = volume;
      }
      else
      {
        gzwarn << "Required element volume missing from element link [" << name
               << "] in BuoyancyPlugin SDF" << std::endl;
        continue;
      }
      */
      if (linkElem->HasElement("area"))
      {
        double area = linkElem->GetElement("area")->Get<double>();
        if (area <= 0)
        {
          gzwarn << "Nonpositive area specified in BuoyancyPlugin!"
                 << std::endl;
          // Remove the element from the map since it's invalid.
          this->volPropsMap.erase(id);
          continue;
        }
        this->volPropsMap[id].area = area;
      }
      else
      {
        gzwarn << "Required element 'area' missing from element link [" << name
               << "] in BuoyancyPlugin SDF" << std::endl;
        continue;
      }

      if (linkElem->HasElement("height"))
      {
        double height = linkElem->GetElement("height")->Get<double>();
        if (height <= 0)
        {
          gzwarn << "Nonpositive height specified in BuoyancyPlugin!"
                 << std::endl;
          // Remove the element from the map since it's invalid.
          this->volPropsMap.erase(id);
          continue;
        }
        this->volPropsMap[id].height = height;
      }
      else
      {
        gzwarn << "Required element 'height' missing from element link [" << name
               << "] in BuoyancyPlugin SDF" << std::endl;
        continue;
      }

    }
  }

  // For links the user didn't input, precompute the center of volume and
  // density. This will be accurate for simple shapes.
  //for (auto link : this->model->GetLinks()){

  // Remove this b/c we only want to apply buoyancy to links specifically
  // called out in SDF
  /*
  physics::Link_V links = this->model->GetLinks();
  physics::LinkPtr link;
  for (int ii=0; ii<links.size(); ii++){
    link = links[ii];

    int id = link->GetId();
    if (this->volPropsMap.find(id) == this->volPropsMap.end())
    {
      double volumeSum = 0;
      math::Vector3 weightedPosSum = math::Vector3::Zero;

      // The center of volume of the link is a weighted average over the pose
      // of each collision shape, where the weight is the volume of the shape
      //for (auto collision : link->GetCollisions()){
      physics::Collision_V colls = link->GetCollisions();
      physics::CollisionPtr collision;
      for (int jj=0; jj<colls.size();jj++){
	collision=colls[jj];
        //double volume = collision->GetShape()->ComputeVolume();
	double volume = 1.0;  // HACK
        volumeSum += volume;
	//weightedPosSum += volume*collision->WorldPose().Pos();
        weightedPosSum += volume*collision->GetWorldPose().pos;
      }
      // Subtract the center of volume into the link frame.
      this->volPropsMap[id].cov =
	weightedPosSum/volumeSum - link->GetWorldPose().pos;
	//weightedPosSum/volumeSum - link->WorldPose().Pos();
      this->volPropsMap[id].volume = volumeSum;
    }
  }
  */

}

/////////////////////////////////////////////////
void BuoyancyPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BuoyancyPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void BuoyancyPlugin::OnUpdate()
{


  //  for (auto link : this->model->GetLinks()){
  //physics::Link_V links = this->model->GetLinks();
  physics::LinkPtr link;
  //for (int ii=0; ii<links.size(); ii++){
  for (auto ii=0u; ii<buoyLinks.size(); ii++){
    link = this->buoyLinks[ii]; //link = links[ii];
    VolumeProperties volumeProperties = this->volPropsMap[link->GetId()];
    //double volume = volumeProperties.volume;
    double height = volumeProperties.height;
    double area = volumeProperties.area;
    double volume = height*area;  // default value

    //math::Pose linkFrame = link->WorldPose();
    math::Pose linkFrame = link->GetWorldPose();

    // Location of bottom of object relative to the fluid surface - assumes origin is at cog of the object
    double bottomRelSurf = this->fluidLevel - (linkFrame.pos.z - height/2.0);
    // Adjust volume of object depending on surface interaction
    //gzmsg << "level: " << this->fluidLevel << std::endl;
    //gzmsg << "height: " << height << std::endl;
    //gzmsg << "brs: " <<bottomRelSurf << std::endl;
    //gzmsg << "z: " << linkFrame.pos.z << std::endl;

    if ( bottomRelSurf <= 0 ) // out of water
      {
	volume = 0.0;
      }
    else if ( bottomRelSurf <= height ) // at surface
      {
	volume = bottomRelSurf *area;
      }
    else{
      volume = height*area;
    }

    //gzmsg << "volume = [" << volume << "]" << std::endl;
    GZ_ASSERT(volume >= 0, "Nonpositive volume found in volume properties!");

    // By Archimedes' principle,
    // buoyancy = -(mass*gravity)*fluid_density/object_density
    // object_density = mass/volume, so the mass term cancels.
    // Therefore,
    //    math::Vector3 buoyancy =
    //    -this->fluidDensity * volume * this->model->GetWorld()->Gravity();
    math::Vector3 gravity(0,0,-9.81);
    math::Vector3 buoyancy =
      -this->fluidDensity * volume * gravity;
    //gzmsg << "buoy " << buoyancy << std::endl;

    // Add some drag
    if (volume > 1e-6){
      math::Vector3 vel = link->GetWorldLinearVel();
      double dz = -1.0*this->fluidDrag * vel.z * std::abs(vel.z);
      //gzmsg << "drag: " << dz << std::endl;
      math::Vector3 drag(0,0,dz);
      buoyancy = buoyancy + drag;
      if (buoyancy.z < 0.0){
	buoyancy.z = 0.0;
	}
    }

    // rotate buoyancy into the link frame before applying the force.
    math::Vector3 buoyancyLinkFrame =
      linkFrame.rot.GetInverse().RotateVector(buoyancy);
      //linkFrame.Rot().Inverse().RotateVector(buoyancy);

    //link->AddLinkForce(buoyancyLinkFrame, volumeProperties.cov);
    //
    link->AddForceAtRelativePosition(buoyancy, volumeProperties.cov);
  }

}
