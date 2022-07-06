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
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

/// \brief A class that converts data from robot frame to optical frame
/// It subscribes to existing topic, modifies the frame_id of the message to
/// a new optical frame, then republishes the updated data to a new topic.
class OpticalFramePublisher : public rclcpp::Node
{
  /// \brief Constructor
  /// \param[in] _info True to publish camera info data in optical frame. False
  /// to publish only image data in optical frame
  public: OpticalFramePublisher(bool _info) : Node("optical_frame_publisher")
  {
    this->tfBroadcasterStatic =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    this->pub = this->create_publisher<sensor_msgs::msg::Image>(
        "output/image", 10);

    this->pubInfo = _info;
    if (this->pubInfo)
    {
      this->pubCi = this->create_publisher<sensor_msgs::msg::CameraInfo>(
          "output/camera_info", 10);
    }

    this->timer = this->create_wall_timer(100ms,
        std::bind(&OpticalFramePublisher::CheckSubscribers, this));
  }


  /// \brief Periodically check subcription count of the optical frame topics.
  /// If subscribers are present, we initiate subscription to the original image
  /// and camera_info topics, convert the frames, then republish the data these
  /// subscribers.
  private: void CheckSubscribers()
  {
    if (this->pub->get_subscription_count() > 0u)
      this->ImageConnect();
    if (this->pubCi && this->pubCi->get_subscription_count() > 0u)
      this->CameraInfoConnect();
  }

  /// \brief Callback when subscriber connects to the image topic
  private: void ImageConnect()
  {
    if (this->sub)
      return;
    this->sub =
        this->create_subscription<sensor_msgs::msg::Image>(
        "input/image", 10,
        std::bind(&OpticalFramePublisher::UpdateImageFrame, this, _1));
  }

  /// \brief Callback when subscriber connects to the camera_info topic
  private: void CameraInfoConnect()
  {
    if (this->subCi)
      return;
    this->subCi =
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "input/camera_info", 10,
        std::bind(&OpticalFramePublisher::UpdateCameraInfoFrame, this, _1));
  }

  /// \brief Subscriber callback which updates the frame id of the message
  /// and republishes the message.
  /// \param[in] _msg Message whose frame id is to be updated
  private: void UpdateImageFrame(
      const std::shared_ptr<sensor_msgs::msg::Image> _msg)
  {
    if (this->pub->get_subscription_count() == 0u && this->sub)
    {
      this->sub.reset();
      return;
    }
    if (this->newFrameId.empty())
    {
      this->newFrameId = _msg->header.frame_id + "_optical";
      this->PublishTF(_msg->header.frame_id, this->newFrameId);
    }
    auto m = *_msg.get();
    m.header.frame_id = this->newFrameId;
    this->pub->publish(m);
  }

  /// \brief Subscriber callback which updates the frame id of the message
  /// and republishes the message.
  /// \param[in] _msg Message whose frame id is to be updated
  private: void UpdateCameraInfoFrame(
      const std::shared_ptr<sensor_msgs::msg::CameraInfo> _msg)
  {
    if (this->pubCi->get_subscription_count() == 0u && this->subCi)
    {
      this->subCi.reset();
      return;
    }
    if (this->newFrameId.empty())
    {
      this->newFrameId = _msg->header.frame_id + "_optical";
      this->PublishTF(_msg->header.frame_id, this->newFrameId);
    }
    auto m = *_msg.get();
    m.header.frame_id = this->newFrameId;
    this->pubCi->publish(m);
  }

  /// \brief Publish static tf data between the original frame of the msg
  /// and the new optical frame
  /// \param[in] _frame Original message frame id
  /// \param[in] _childFrame Optical frame id
  private: void PublishTF(const std::string &_frame,
    const std::string &_childFrame)
  {
    geometry_msgs::msg::TransformStamped tfStamped;
    tfStamped.header.frame_id = _frame;
    tfStamped.child_frame_id = _childFrame;
    tfStamped.transform.translation.x = 0.0;
    tfStamped.transform.translation.y = 0.0;
    tfStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    // converts x forward to z forward
    q.setRPY(-M_PI/2.0, 0, -M_PI/2.0);
    tfStamped.transform.rotation.x = q.x();
    tfStamped.transform.rotation.y = q.y();
    tfStamped.transform.rotation.z = q.z();
    tfStamped.transform.rotation.w = q.w();
    this->tfBroadcasterStatic->sendTransform(tfStamped);
  }


  /// \brief timer with callback to check for publisher subscription count
  private: rclcpp::TimerBase::SharedPtr timer;

  /// \brief True to publish camera info in optical frame
  private: bool pubInfo = false;

  /// \brief Subscriber for the original image topic
  private: rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;

  /// \brief Subscriber for the original camera_info topic
  private: rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
               subCi;

  /// \brief Publisher for the image optical frame topic
  private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;

  /// \brief Publisher for the camera info optical frame topic
  private: rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pubCi;

  /// \brief Static transform broadcaster that broadcasts the optical frame id
  private: std::unique_ptr<tf2_ros::StaticTransformBroadcaster>
               tfBroadcasterStatic;

  /// \brief New of the new optical frame id
  private: std::string newFrameId;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // first arg is bool variable that specifies whether to re-publish camera info
  // in optical frame
  bool info = false;
  std::istringstream(argv[1]) >> info;

  rclcpp::spin(std::make_shared<OpticalFramePublisher>(info));
  rclcpp::shutdown();
  return 0;
}
