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

#include <ignition/math/Angle.hh>

#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

using std::placeholders::_1;

/// \brief A bridge node that offers ROS2 APIs matching the ones provided by
/// the real USV hardware. The node processes the input cmd data and forwards
/// them to the sim ROS2 APIs.
class USVBridge : public rclcpp::Node
{
  /// \brief Constructor
  public: USVBridge() : Node("usv_bridge")
  {
    // cmd thrust pub
    this->leftCmdThrustPub = this->create_publisher<std_msgs::msg::Float64>(
        "left/thrust/cmd_thrust", 10);
    this->rightCmdThrustPub = this->create_publisher<std_msgs::msg::Float64>(
        "right/thrust/cmd_thrust", 10);

    // cmd pos pub
    this->leftCmdPosPub = this->create_publisher<std_msgs::msg::Float64>(
        "left/thrust/joint/cmd_pos", 10);
    this->rightCmdPosPub = this->create_publisher<std_msgs::msg::Float64>(
        "right/thrust/joint/cmd_pos", 10);

    // cmd thrust sub
    this->cmdThrustersSub =
       this->create_subscription<std_msgs::msg::Int16MultiArray>(
       "/cat/cmd_thrusters", 1,
       std::bind(&USVBridge::OnCmdThrusters, this, _1));

    // cmd pod sub
    this->cmdPodSub =
       this->create_subscription<std_msgs::msg::Int16MultiArray>(
       "/cat/cmd_pod", 1,
       std::bind(&USVBridge::OnCmdPod, this, _1));
  }

  /// \brief Callback when a thrust cmd is received
  /// \param[in] _msg Thrust cmd message
  private: void OnCmdThrusters(
      const std_msgs::msg::Int16MultiArray& _msg)
  {
    // there should only be 2 values
    //     index 0: left
    //     index 1: right
    int16_t leftValue = _msg.data[0];
    int16_t rightValue = _msg.data[1];

    // forward values to sim API
    std_msgs::msg::Float64 leftThrustMsg;
    leftThrustMsg.data = static_cast<double>(leftValue);
    this->leftCmdThrustPub->publish(leftThrustMsg);

    std_msgs::msg::Float64 rightThrustMsg;
    rightThrustMsg.data = static_cast<double>(rightValue);
    this->rightCmdThrustPub->publish(rightThrustMsg);
  }

  /// \brief Callback when a pod pos cmd is received
  /// \param[in] _msg Pod pos cmd message
  private: void OnCmdPod(
      const std_msgs::msg::Int16MultiArray& _msg)
  {
    // there should only be 2 values
    //     index 0: left
    //     index 1: right
    // values are expressed in hundreds of degrees
    int16_t leftValue = _msg.data[0];
    int16_t rightValue = _msg.data[1];

    // convert to sim frame
    // -90 deg in hw = 0 in sim
    // 90 deg in hw = -180 in sim
    double leftDeg = leftValue / 100.0;
    leftDeg += 90;
    ignition::math::Angle leftAngle;
    leftAngle.SetDegree(leftDeg);

    std_msgs::msg::Float64 leftPosMsg;
    leftPosMsg.data = static_cast<double>(leftAngle.Radian());
    this->leftCmdPosPub->publish(leftPosMsg);

    double rightDeg = rightValue / 100.0;
    rightDeg += 90;
    ignition::math::Angle rightAngle;
    rightAngle.SetDegree(rightDeg);

    std_msgs::msg::Float64 rightPosMsg;
    rightPosMsg.data = static_cast<double>(rightAngle.Radian());
    this->rightCmdPosPub->publish(rightPosMsg);
  }

  /// \brief Subscriber for the thruster commands
  private: rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr
      cmdThrustersSub;

  /// \brief Subscriber for the pod angular position commands
  private: rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr
      cmdPodSub;

  /// \brief Publisher for the left thruster
  private: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      leftCmdThrustPub;

  /// \brief Publisher for the right thruster
  private: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      rightCmdThrustPub;


  /// \brief Publisher for the left thruster joint pos cmd
  private: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      leftCmdPosPub;

  /// \brief Publisher for the right thruster joint pos cmd
  private: rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      rightCmdPosPub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<USVBridge>());
  rclcpp::shutdown();
  return 0;
}
