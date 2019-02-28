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


/* Note - this code was originally derived from the ARIAC 
 * PopulationPlugin https://bitbucket.org/osrf/ariac/src/master/osrf_gear/include/osrf_gear/PopulationPlugin.hh
*/

#ifndef VRX_GAZEBO_PERCEPTION_SCORING_PLUGIN_HH_
#define VRX_GAZEBO_PERCEPTION_SCORING_PLUGIN_HH_

#include <geographic_msgs/GeoPoseStamped.h>
#include <ros/ros.h>
#include <memory>
#include <string>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>
#include "vrx_gazebo/scoring_plugin.hh"

/// \brief A class to monitor if the object and pose reported matches the
/// currently visible object and pose.
/// displayed in the light buoy.
class ObjectChecker
{
  /// \brief Constructor.
  /// \param[in] _rosNameSpace ROS namespace.
  /// \param[in] _rosObjectTopic The ROS topic used to receive
  /// the object identification and localization
  public: ObjectChecker(const std::string &_rosNameSpace,
                        const std::string &_rosObjectTopic,
                        gazebo::physics::WorldPtr _world);

  /// \brief Initialize a new trial
  /// \param[in] _objectName Name of the object for id purposes
  /// \param[in] _objectPose Pose of the object for localization purposes
  public: void NewTrial(const std::string &_objectName,
                        gazebo::physics::EntityPtr _object);

  /// Enable the ROS subscription.
  public: void Enable();

  /// Disable the ROS subscription.
  public: void Disable();

  /// \brief Whether a team submitted an identification for current trial
  /// \return True when the submission was received or false otherwise.
  public: bool SubmissionReceived() const;

  /// \brief Has the submission been scored?
  public: bool submissionScored = false;

  /// \brief Whether a team submitted a correct id or not.
  /// \return True when the team submitted the id and it is correct
  /// or false otherwise.
  public: bool Correct() const;

  /// \brief Callback executed when a new color submission is received.
  /// \param[in] _request Contains the submission.
  /// \param[out] _res The Response. Note that this will be true even if the
  /// reported sequence is incorrect.
  private: void OnObject(const geographic_msgs::GeoPoseStamped::ConstPtr &_msg);

  /// \brief Pose error of object localization.
  public: double objectError = -1.0;

  /// \brief ROS namespace.
  private: std::string ns;

  /// \brief ROS topic where the object id/pose is received.
  private: std::string objectTopic;

  /// \brief ROS Node handle.
  private: ros::NodeHandle nh;

  /// \brief ROS subscriber
  private: ros::Subscriber objectSub;

  /// \brief Whether the object has been received or not.
  private: bool objectReceived = false;

  /// \brief Whether the object ID received is correct or not.
  private: bool objectCorrect = false;

  /// \brief Count the trials.
  private: int trialCount = 0;

  /// \brief Current correct object name.
  private: std::string trueName;

  /// \brief Current object
  private: gazebo::physics::EntityPtr currObject;

  /// \brief World pointer. Need this for spherical/local conversion.
  private: gazebo::physics::WorldPtr world;
};

// Forward declare private data class
class PerceptionScoringPluginPrivate;

/// \brief A plugin that allows models to be spawned at a given location in
/// a specific simulation time and then takes care of scoring correct
/// identification and localization of the objects.
///
/// The plugin accepts the following SDF parameters:
///
/// <object_sequence>: Contains the list of objects to be populated. An object
///                    should be declared as an <object> element with the
///                    following parameters:
///                      <time> Simulation time to be spawned.
///                      <type> Model.
///                      <name> Landmark name.
///                      <pose> Initial object pose.
///
/// <loop_forever>: Optional parameter. If true, all objects will be spawned
/// as a circular buffer. After spawning the last element of the collection,
/// the first one will be inserted.
///
/// <frame>: Optional parameter. If present, the poses of the objects will be
/// in the frame of this link/model. Otherwise the world frame is used.
///
/// <robot_namespace>: Optional parameter.  If present, specifies ROS namespace.
///
/// <landmark_topic>: Optional parameter.  Specify the topic to which the
///   plugin subscribes for receiving identification and localization msgs.
///   Default is "/vrx/perception/landmark"
///
/// Here's an example of a valid SDF:
///
/// <plugin filename="libperception_scoring_plugin.so"
///         name="perception_scoring_plugin">
///   <vehicle>wamv</vehicle>
///   <task_name>perception</task_name>
///   <initial_state_duration>1</initial_state_duration>
///   <ready_state_duration>1</ready_state_duration>
///   <running_state_duration>300</running_state_duration>
///
///   <!-- Parameters for PopulationPlugin -->
///   <loop_forever>false</loop_forever>
///   <frame>wamv</frame>
///   <object_sequence>
///     <object>
///       <time>10.0</time>
///       <type>red</type>
///       <name>red_mark</name>
///       <pose>6 0 1 0 0 0</pose>
///     </object>
///     <object>
///       <time>10.0</time>
///       <type>green</type>
///       <name>green_mark</name>
///       <pose>6 6 1 0 0 0</pose>
///     </object>
///   </object_sequence>
/// </plugin>
class PerceptionScoringPlugin : public ScoringPlugin
{
  /// \brief Constructor.
  public: PerceptionScoringPlugin();

  /// \brief Destructor.
  public: virtual ~PerceptionScoringPlugin();

  // Documentation inherited.
  public: virtual void Load(gazebo::physics::WorldPtr _world,
                            sdf::ElementPtr _sdf);

  /// \brief Update the plugin.
  protected: void OnUpdate();

  /// \brief Restart the object population list
  private: void Restart();

  // Documentation inherited.
  private: void OnRunning() override;

  /// \brief Private data pointer.
  private: std::unique_ptr<PerceptionScoringPluginPrivate> dataPtr;
};

#endif
