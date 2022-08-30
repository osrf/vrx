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

#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/World.hh>
#include "StationkeepingScoringPlugin.hh"
#include <ignition/msgs/float.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/quaternion.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Model.hh>

using namespace vrx;

/// \brief Private ScoringPlugin data class.
class StationkeepingScoringPlugin::Implementation
{
    /// \brief Transport node.
public:
    ignition::transport::Node node;

    /// \brief Topic where the task stats are published.
public:
    std::string goalTopic = "/vrx/stationkeeping/goal";

    /// \brief Topic where 2D pose error is published
public:
    std::string poseErrorTopic = "/vrx/stationkeeping/pose_error";

    /// \brief Topic where mean pose error is published.
public:
    std::string meanErrorTopic = "/vrx/stationkeeping/mean_pose_error";

    /// \brief Publisher for the goal.
public:
    ignition::transport::Node::Publisher goalPub;

    /// \brief Publisher for the combined 2D pose error.
public:
    ignition::transport::Node::Publisher poseErrorPub;

    /// \brief Publisher for the current mean error.
public:
    ignition::transport::Node::Publisher meanErrorPub;

    /// \brief Goal pose in local (Gazebo) coordinates.
public:
    double goalX;

    /// \brief Goal pose in local (Gazebo) coordinates.
public:
    double goalY;

    /// \brief Goal pose in local (Gazebo) coordinates.
public:
    double goalYaw;

    /// \brief Goal pose in spherical (WGS84) coordinates.
public:
    double goalLat;

    /// \brief Goal pose in spherical (WGS84) coordinates.
public:
    double goalLon;

    /// \brief Message to store goal pose (lat/long/yaw).
public:
    ignition::msgs::Pose goalMsg;

    /// \brief Combined 2D pose error (distance and yaw).
public:
    double poseError;

    /// \brief Number of instant pose error scores calculated so far .
public:
    unsigned int sampleCount = 0;

    /// \brief Sum of all pose error scores calculated so far.
public:
    double totalPoseError = 0;

    /// \brief Cumulative 2D RMS error in meters.
public:
    double meanError;

    /// \brief True to consider heading error or false otherwise.
public:
    bool headErrorOn = true;

    /// \brief Absolute time specifying the start of the docked time.
public:
    std::chrono::duration<double> dockTime;

    /// \brief Timer used to calculate the elapsed time docked in the bay.
public:
    std::chrono::duration<double> timer;

    /// \brief Spherical coordinates conversions.
public:
    ignition::math::SphericalCoordinates sc;

    /// \brief Vehicle to score.
public:
    ignition::gazebo::Entity vehicleEntity;

    /// \brief Waypoint visualization markers
public:
    bool markers = false;

public:
    void AddMarker();

public:
    bool silent = false;
};

void StationkeepingScoringPlugin::Implementation::AddMarker()
{
    ignition::msgs::Marker markerMsg;
    markerMsg.set_id(0);
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);

    ignition::msgs::Material *matMsg = markerMsg.mutable_material();
    matMsg->mutable_script()->set_name("Gazebo/Red");
    ignition::msgs::Set(markerMsg.mutable_pose(),
                        ignition::math::Pose3d(this->goalX, this->goalY,
                                               1, 0, 0, this->goalYaw));
    bool markerRequest = this->node.Request("/marker", markerMsg);
    this->markers = markerRequest;
}

/////////////////////////////////////////////////
StationkeepingScoringPlugin::StationkeepingScoringPlugin()
    : ScoringPlugin(), dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
    ignmsg << "Stationkeeping scoring plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void StationkeepingScoringPlugin::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
    ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);
    ignmsg << "Task [" << this->TaskName() << "]" << std::endl;
    auto worldEntity =
        _ecm.EntityByComponents(ignition::gazebo::components::World());
    ignition::gazebo::World world(worldEntity);
    this->dataPtr->sc = world.SphericalCoordinates(_ecm).value();

    // Get lat, lon and yaw from SDF
    if (!_sdf->HasElement("goal_pose") && !_sdf->HasElement("goal_pose_cart"))
    {
        ignerr << "Found neither <goal_pose> nor <goal_pose_cart> element in SDF." << std::endl;
        ignerr << "Using default pose: 0 0 0" << std::endl;
    }
    else if (_sdf->HasElement("goal_pose"))
    {
        if (_sdf->HasElement("goal_pose_cart"))
        {
            ignerr << "Both goal_pose and goal_pose_cart were specified."
                   << std::endl;
            ignerr << "Ignoring goal_pose_cart." << std::endl;
        }

        ignition::math::Vector3d latlonyaw =
            _sdf->Get<ignition::math::Vector3d>("goal_pose");

        // Store spherical 2D location
        this->dataPtr->goalLat = latlonyaw.X();
        this->dataPtr->goalLon = latlonyaw.Y();

        // Convert lat/lon to local
        // Snippet from UUV Simulator SphericalCoordinatesROSInterfacePlugin.cc
        ignition::math::Vector3d scVec(this->dataPtr->goalLat, this->dataPtr->goalLon, 0.0);
        ignition::math::Vector3d cartVec =
            this->dataPtr->sc.LocalFromSphericalPosition(scVec);

        // Store local 2D location and yaw
        this->dataPtr->goalX = cartVec.X();
        this->dataPtr->goalY = cartVec.Y();
        this->dataPtr->goalYaw = latlonyaw.Z();
    }
    else
    {
        ignition::math::Vector3d xyz =
            _sdf->Get<ignition::math::Vector3d>("goal_pose_cart");

        // Store local 2D location
        this->dataPtr->goalX = xyz.X();
        this->dataPtr->goalY = xyz.Y();

        // Convert local to lat/lon
        // Snippet from UUV Simulator SphericalCoordinatesROSInterfacePlugin.cc
        ignition::math::Vector3d cartVec(this->dataPtr->goalX, this->dataPtr->goalY, xyz.Z());

        auto in = ignition::math::SphericalCoordinates::CoordinateType::GLOBAL;
        auto out = ignition::math::SphericalCoordinates::CoordinateType::SPHERICAL;
        auto scVec = this->dataPtr->sc.PositionTransform(cartVec, in, out);
        scVec.X(IGN_RTOD(scVec.X()));
        scVec.Y(IGN_RTOD(scVec.Y()));

        // Store spherical 2D location
        this->dataPtr->goalLat = scVec.X();
        this->dataPtr->goalLon = scVec.Y();
        this->dataPtr->goalYaw = scVec.Z();
    }

    // Print some debugging messages
    ignmsg << "Stationkeeping Goal, Spherical: Lat = " << this->dataPtr->goalLat
           << " Lon = " << this->dataPtr->goalLon << std::endl;
    ignmsg << "Stationkeeping Goal, Local: X = " << this->dataPtr->goalX
           << " Y = " << this->dataPtr->goalY << " Yaw = " << this->dataPtr->goalYaw << std::endl;

    // Store goal pose in message for publishing
    ignition::math::Pose3d pose(this->dataPtr->goalLat, this->dataPtr->goalLon, 0,
                                0, 0, this->dataPtr->goalYaw);
    ignition::msgs::Set(this->dataPtr->goalMsg.mutable_position(), pose.Pos());
    ignition::msgs::Set(this->dataPtr->goalMsg.mutable_orientation(), pose.Rot());

    // Throttle messages to 1Hz
    ignition::transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(1u);
    if (_sdf->HasElement("goal_topic"))
    {
        this->dataPtr->goalTopic = _sdf->Get<std::string>("goal_topic");
    }
    this->dataPtr->goalPub =
        this->dataPtr->node.Advertise<ignition::msgs::Pose>(this->dataPtr->goalTopic, opts);

    if (_sdf->HasElement("pose_error_topic"))
    {
        this->dataPtr->poseErrorTopic = _sdf->Get<std::string>("pose_error_topic");
    }
    this->dataPtr->poseErrorPub = this->dataPtr->node.Advertise<ignition::msgs::Float>(this->dataPtr->poseErrorTopic, opts);

    if (_sdf->HasElement("mean_error_topic"))
    {
        this->dataPtr->meanErrorTopic = _sdf->Get<std::string>("mean_error_topic");
    }
    this->dataPtr->meanErrorPub = this->dataPtr->node.Advertise<ignition::msgs::Float>(
        this->dataPtr->meanErrorTopic, opts);

    if (_sdf->HasElement("head_error_on"))
        this->dataPtr->headErrorOn = _sdf->Get<bool>("head_error_on");

    if (_sdf->HasElement("markers"))
    {
        this->dataPtr->markers = true;
    }
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    ScoringPlugin::PreUpdate(_info, _ecm);

    // Check to see if we need to publish the marker(s)
    if (this->dataPtr->markers)
    {
        this->dataPtr->AddMarker();
    }

    // Start publishing the goal once in "ready" state
    if (this->ScoringPlugin::TaskState() == "ready")
        this->dataPtr->goalPub.Publish(this->dataPtr->goalMsg);

    // The vehicle might not be ready yet, let's try to get it.
    if (!this->dataPtr->vehicleEntity)
    {
        auto entity = _ecm.EntityByComponents(
            ignition::gazebo::components::Name(ScoringPlugin::VehicleName()));
        if (entity != ignition::gazebo::kNullEntity)
            this->dataPtr->vehicleEntity = entity;
        else
            return;
    }
    if (this->ScoringPlugin::TaskState() != "running")
        return;

    ignition::msgs::Float poseErrorMsg;
    ignition::msgs::Float meanErrorMsg;
    auto vehiclePose = _ecm.Component<ignition::gazebo::components::Pose>(
                               this->dataPtr->vehicleEntity)->Data();
    double currentHeading = vehiclePose.Rot().Euler().Z();
    double dx = this->dataPtr->goalX - vehiclePose.Pos().X();
    double dy = this->dataPtr->goalY - vehiclePose.Pos().Y();
    double dist = sqrt(pow(dx, 2) + pow(dy, 2));
    double dhdg = abs(this->dataPtr->goalYaw - currentHeading);
    double headError = M_PI - abs(dhdg - M_PI);

    if (this->dataPtr->headErrorOn)
    {
        double k = 0.75;
        this->dataPtr->poseError = dist + (pow(k, dist) * headError);
    }
    else
        this->dataPtr->poseError = dist;
    this->dataPtr->totalPoseError += this->dataPtr->poseError;
    this->dataPtr->sampleCount++;

    this->dataPtr->meanError =
        this->dataPtr->totalPoseError / this->dataPtr->sampleCount;

    poseErrorMsg.set_data(this->dataPtr->poseError);
    meanErrorMsg.set_data(this->dataPtr->meanError);

    this->dataPtr->poseErrorPub.Publish(poseErrorMsg);
    this->dataPtr->meanErrorPub.Publish(meanErrorMsg);
    this->dataPtr->goalPub.Publish(this->dataPtr->goalMsg);
    ScoringPlugin::SetScore(this->dataPtr->meanError);
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::OnReady()
{
    if (!this->dataPtr->silent)
    {
        ignmsg << "StationkeepingScoringPlugin::OnReady" << std::endl;
    }
    ScoringPlugin::OnReady();
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::OnRunning()
{
    if (!this->dataPtr->silent)
    {
        ignmsg << "StationkeepingScoringPlugin::OnRunning" << std::endl;
    }
    ScoringPlugin::OnRunning();
}

//////////////////////////////////////////////////
void StationkeepingScoringPlugin::OnFinished()
{
    if (!this->dataPtr->silent)
    {
        ignmsg << "StationkeepingScoringPlugin::OnFinished" << std::endl;
    }
    ScoringPlugin::OnFinished();
}

void StationkeepingScoringPlugin::OnCollision()
{
    if (!this->dataPtr->silent)
    {
        ignmsg << "StationkeepingScoringPlugin::OnCollision" << std::endl;
    }
    ScoringPlugin::OnCollision();
}

IGNITION_ADD_PLUGIN(vrx::StationkeepingScoringPlugin,
                    ignition::gazebo::System,
                    vrx::StationkeepingScoringPlugin::ISystemConfigure,
                    vrx::StationkeepingScoringPlugin::ISystemPreUpdate)
