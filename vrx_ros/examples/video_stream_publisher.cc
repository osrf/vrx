/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;

/// \brief A class that demonstrates how to subscribe to images from a vehicle
/// camera, optionally updates its frame_id to indicate the source of the
/// the image, then publishes to the base station for validation.
class VideoStreamPublisher : public rclcpp::Node
{
  /// \brief Constructor
  /// \param[in] _robot Name of robot sending the image stream
  /// \param[in] _topic Image topic to subscribe to
  /// \param[in] _frameId Frame ID of the camera which the image comes from
  public: VideoStreamPublisher(const std::string &_robot,
      const std::string &_topic,
      const std::string &_frameId)
      : Node("video_stream_publisher")
  {
    this->frameId = _frameId;

    // subscribe to original image topic
    this->sub =
        this->create_subscription<sensor_msgs::msg::Image>(
        _topic, 10,
        std::bind(&VideoStreamPublisher::PublishTargetStream, this, _1));

    // publish image to base station
    this->pub = this->create_publisher<sensor_msgs::msg::Image>(
        "/" + _robot + "/mbzirc/target/stream/start", 10);
  }

  /// \brief Subscriber callback which updates the frame id of the image
  /// and republishes to the base station
  /// \param[in] _msg Image message
  private: void PublishTargetStream(
      const std::shared_ptr<sensor_msgs::msg::Image> _msg)
  {
    auto m = *_msg.get();
    // update the frame_id of the image if specified by user
    if (!this->frameId.empty())
      m.header.frame_id = this->frameId;
    this->pub->publish(m);
  }

  /// \brief Subscriber for the original image topic
  private: rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;

  /// \brief Publisher for the image stream to be sent to base station
  private: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;

  /// \brief Frame ID indicating which camera that the video stream is from
  private: std::string frameId;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  if (argc < 3)
  {
    std::cerr << "Usage: ros2 run mbzirc_ros video_stream_publisher "
              << "<robot_name> <topic name> [camera frame ID]"
              << std::endl;
    exit(1);
  }

  // Name of robot sending the image data to base station
  std::string robot = argv[1];
  // image topic to subscribe to
  std::string topic = argv[2];
  // user-specified frame_id
  // if this is empty, the original frame_id of the image will be used
  std::string frameId = argv[3];

  rclcpp::spin(std::make_shared<VideoStreamPublisher>(robot, topic, frameId));
  rclcpp::shutdown();
  return 0;
}
