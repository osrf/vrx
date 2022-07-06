/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>

using std::placeholders::_1;

class FramePublisher : public rclcpp::Node
{
  public: FramePublisher() : Node("frame_publisher")
  {
    this->declare_parameter<std::string>("world_frame", "");

    this->tfBroadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    this->tfBroadcasterStatic =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    this->subscription =
        this->create_subscription<tf2_msgs::msg::TFMessage>(
        "pose", 10,
        std::bind(&FramePublisher::HandlePose, this, _1));
    this->subscriptionStatic =
        this->create_subscription<tf2_msgs::msg::TFMessage>(
        "pose_static", 10,
        std::bind(&FramePublisher::HandlePoseStatic, this, _1));
  }

  private: void HandlePose(const std::shared_ptr<tf2_msgs::msg::TFMessage> _msg)
  {
    std::string world_frame;
    this->get_parameter("world_frame", world_frame); 
    if (!world_frame.empty())
    {
      std::vector<geometry_msgs::msg::TransformStamped> filtered;
      for (auto transform: _msg->transforms)
      {
        if (transform.header.frame_id != world_frame)
        {
          filtered.push_back(transform);
        }
      }
      this->tfBroadcaster->sendTransform(filtered);
    }
    else
    {
      // Send the transformation
      this->tfBroadcaster->sendTransform(_msg->transforms);
    }
  }

  private: void HandlePoseStatic(const std::shared_ptr<tf2_msgs::msg::TFMessage> _msg)
  {
    std::string world_frame;
    this->get_parameter("world_frame", world_frame); 
    if (!world_frame.empty())
    {
      std::vector<geometry_msgs::msg::TransformStamped> filtered;
      for (auto transform: _msg->transforms)
      {
        if (transform.header.frame_id != world_frame)
        {
          filtered.push_back(transform);
        }
      }
      this->tfBroadcasterStatic->sendTransform(filtered);
    }
    else
    {
      // Send the transformation
      this->tfBroadcasterStatic->sendTransform(_msg->transforms);
    }
  }

  private: rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription;
  private: rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr
               subscriptionStatic;
  private: std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
  private: std::unique_ptr<tf2_ros::StaticTransformBroadcaster>
               tfBroadcasterStatic;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}
