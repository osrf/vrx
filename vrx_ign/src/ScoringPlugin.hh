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

#ifndef VRX_GAZEBO_SCORING_PLUGIN_HH_
#define VRX_GAZEBO_SCORING_PLUGIN_HH_

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <memory>
#include <string>
#include <vector>
#include <ignition/gazebo/System.hh>
#include <ignition/common.hh>
#include <ignition/transport.hh>
#include <ignition/msgs.hh>
#include <ignition/physics.hh>
#include <ignition/math/SphericalCoordinates.hh>
#include <ignition/gazebo/components/SphericalCoordinates.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/World.hh>
#include <sdf/sdf.hh>
#include <ignition/msgs/any.pb.h>
#include <ignition/msgs/param.pb.h>
#include <ignition/msgs/param_v.pb.h>
#include "ignition/gazebo/Util.hh"

#include "vrx_ros/msg/contact.hpp"
#include "vrx_ros/msg/task.hpp"


namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{

class ScoringPlugin 
        : public System,
          public ISystemConfigure,
          public ISystemPostUpdate 
    {
        /// \brief Class constructor
        public: ScoringPlugin();

        /// \brief Shutdown Gazebo and ROS
        public: void Exit();

        // Documentation inherited
        public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

        /// \brief Get the current score.
        /// \return The current score.
        protected: double Score() const;

        /// \brief Set the score.
        /// \param[in] _newScore The new score.
        protected: void SetScore(double _newScore);

        /// \brief Get the task name.
        /// \return Task name.
        protected: std::string TaskName() const;

        /// \brief Get the task state.
        /// \return Task state.
        protected: std::string TaskState() const;

        /// \brief Elapsed time in the running state.
        /// \return The elapsed time in the running state.
        protected: ignition::msgs::Time ElapsedTime() const;

        /// \brief Remaining time in the running state.
        /// \return The remaining time in the running state.
        protected: ignition::common::Time RemainingTime() const;

        /// \brief Finish the current task.
        /// This will set the "finished" flag in the task message to true.
        protected: void Finish();

        /// \brief Tries to release the vehicle in case is locked.
        protected: virtual void ReleaseVehicle();

        /// \brief Set the score in case of timeout
        protected: void SetTimeoutScore(double _timeoutScore);

        /// \brief Get the timeoutScore
        protected: double GetTimeoutScore() const;

        /// \brief Get running duration
        protected: double GetRunningStateDuration() const;

        /// \brief Get the number of WAM-V collisions.
        protected: unsigned int GetNumCollisions() const;

        /// \brief Callback executed at every world update.
        public: void PostUpdate(
                    const ignition::gazebo::UpdateInfo &_info,
                    const ignition::gazebo::EntityComponentManager &_ecm) override;

        /// \brief Update all time-related variables.
        std::chrono::duration<double> simTime;
        private: void UpdateTime(const std::chrono::duration<double> _simTime);

        /// \brief Update the state of the current task.
        private: void UpdateTaskState();

        /// \brief Update the task stats message.
        private: void UpdateTaskMessage();

        /// \brief Publish the task stats over a ROS topic.
        private: void PublishStats();

        /// \brief Callback executed when the task state transition into "ready".
        private: virtual void OnReady();

        /// \brief Callback executed when the task state transition into "running".
        private: virtual void OnRunning();

        /// \brief Callback executed when the task state transition into "finished".
        protected: virtual void OnFinished();

        /// \brief Callback executed when a collision is detected for the WAMV.
        private: virtual void OnCollision();

        /// \brief Callback function when collision occurs in the world.
        /// \param[in] _contacts List of all collisions from last simulation iteration
        private: void OnCollisionMsg(const ignition::msgs::Contacts &_contacts);

        /// \brief Parse all SDF parameters.
        /// \return True when all parameters were successfully parsed or false
        /// otherwise.
        private: bool ParseSDFParameters();

        /// \brief Parse the joints section of the SDF block.
        /// \return True when all parameters were successfully parsed or false
        /// otherwise.
        private: bool ParseJoints();

        /// \brief A world pointer.
        //TODO: How to define world ptr?
        // protected: std::shared_ptr<World::World> world;
        protected: World world;

        /// \brief The name of the task.
        protected: std::string taskName = "undefined";

        /// \brief The name of the vehicle to score.
        protected: std::string vehicleName;

        /// \brief Pointer to the vehicle to score.
        //TODO: How to define model ptr?
        // protected: ignition::physics::Model3dPtr vehicleModel;
        protected: std::unique_ptr<Entity> vehicleModel;

        /// \brief Last collision time.
        protected: ignition::common::Time lastCollisionTime;

        /// \brief Silent mode enabled?
        protected: bool silent = false;

        /// \brief Spherical coordinates conversions.
        protected: ignition::math::SphericalCoordinates sc;

        /// \brief gazebo node pointer
        private: std::unique_ptr<ignition::transport::Node> gzNode;

        /// \brief Collision detection node subscriber
        // TODO - Verify if ignition requires a subscriber object
        
        /// \brief gazebo server control publisher
        std::unique_ptr<ignition::transport::Node::Publisher> serverControlPub;

        /// \brief Topic where the task stats are published.
        private: std::string taskInfoTopic = "/vrx/task/info";

        /// \brief Bool flag for debug.
        private: bool debug = true;

        /// \brief Topic where debug collision is published.
        private: std::string contactDebugTopic = "/vrx/debug/contact";

        /// \brief The score.
        private: double score = 0.0;

        /// \brief Pointer to the SDF plugin element.
        private: std::shared_ptr<const sdf::Element> sdf; 

        /// \brief Duration (seconds) of the initial state.
        private: double initialStateDuration = 30.0;

        /// \brief Duration (seconds) of the ready state.
        private: double readyStateDuration = 60.0;

        /// \brief Duration (seconds) of the running state (max task time).
        protected: double runningStateDuration = 300.0;

        /// \brief Absolute time specifying the start of the ready state.
        private: ignition::common::Time readyTime;

        /// \brief Absolute time specifying the start of the running state.
        private: ignition::common::Time runningTime;

        /// \brief Absolute time specifying the start of the finish state.
        private: ignition::common::Time finishTime;

        /// \brief Current time (simulation).
        private: ignition::common::Time currentTime;

        // \brief Elapsed time since the start of the task (running state).
        private: ignition::common::Time elapsedTime;

        /// \brief Remaining time since the start of the task (running state).
        private: ignition::common::Time remainingTime;

        /// \brief Collision buffer.
        private: float collisionBuffer = 3.0;

        /// \brief Collisions counter.
        private: int collisionCounter = 0;

        /// \brief Collision list.
        private: std::vector<std::string> collisionList;

        /// \brief Collisions timestamps.
        private: std::vector<ignition::common::Time> collisionTimestamps;

        /// \brief Whether the current task has timed out or not.
        private: bool timedOut = false;

        /// \brief Time at which the last message was sent.
        private: ignition::common::Time lastStatsSent;

        /// \brief The task state.
        private: std::string taskState = "initial";
        
        /// \brief The next task message to be published.
        //TODO: Create Ros2 task msg, include and declare
        // private: vrx_ros::msg::Task taskMsg; //TODO: Relace with ignition message
        ignition::msgs::Param_V taskMsg;
        auto *param = taskMsg.add_param()->mutable_params();
        ignition::msgs::Any taskMsgName, taskMsgState, 
                taskMsgReadyTime, taskMsgRunningTime, taskMsgElapsedTime, taskMsgRemainingTime,
                taskMsgTimedOut, taskMsgNumCollisions, taskMsgScore;
        
        taskMsgName.set_type(ignition::msgs::Any_ValueType::Any_ValueType_STRING);
        taskMsgState.set_type(ignition::msgs::Any_ValueType::Any_ValueType_STRING);
        taskMsgReadyTime.set_type(ignition::msgs::Any_ValueType::Any_ValueType_TIME);
        taskMsgRunningTime.set_type(ignition::msgs::Any_ValueType::Any_ValueType_TIME);
        taskMsgElapsedTime.set_type(ignition::msgs::Any_ValueType::Any_ValueType_TIME);
        taskMsgRemainingTime.set_type(ignition::msgs::Any_ValueType::Any_ValueType_TIME);
        taskMsgTimedOut.set_type(ignition::msgs::Any_ValueType::Any_ValueType_BOOLEAN);
        taskMsgNumCollisions.set_type(ignition::msgs::Any_ValueType::Any_ValueType_INT32);
        taskMsgScore.set_type(ignition::msgs::Any_ValueType::Any_ValueType_DOUBLE);

        (*param)["name"] = taskMsgName;
        (*param)["state"] = taskMsgState;
        (*param)["ready_time"] = taskMsgReadyTime;
        (*param)["running_time"] = taskMsgRunningTime;
        (*param)["elapsed_time"] = taskMsgElapsedTime;
        (*param)["remaining_time"] = taskMsgRemainingTime;
        (*param)["timed_out"] = taskMsgTimedOut;
        (*param)["num_collisions"] = taskMsgNumCollisions;
        (*param)["score"] = taskMsgScore;

        /// \brief ROS Contact Msg.
        // private: vrx_ros::msg::Contact contactMsg; //Replaced with ignition messages
        private: ignition::msgs::Contact contactMsg;

        /// \brief The name of the joints to be dettached during ReleaseVehicle().
        private: std::vector<std::string> lockJointNames;

        /// \brief ROS node handle.
        //TODO: ROS2 equivalet of node handle??
        // private: std::unique_ptr<rclcpp::Node> rosNode; //Use gzNode

        /// \brief Publisher for the task state.
        //Replaced with ignition messages
        protected: std::unique_ptr<ignition::transport::Node::Publisher> taskPub;
        // protected: rclcpp::Publisher<vrx_ros::msg::Task>::SharedPtr taskPub;
        // protected: ros::Publisher taskPub;

        /// \brief Publisher for the collision.
        //Replaced with ignition messages
        private: std::unique_ptr<ignition::transport::Node::Publisher> contactPub;
        // private: rclcpp::Publisher<vrx_ros::msg::Contact>::SharedPtr contactPub;
        // private: ros::Publisher contactPub;


        /// \brief Score in case of timeout - added for Navigation task
        private: double timeoutScore = -1;

        /// \brief Whether to shut down after last gate is crossed.
        private: bool perPluginExitOnCompletion = true;

        /// \brief Number of WAM-V collisions.
        private: unsigned int numCollisions = 0u;

        };
}
}
}
}
#endif