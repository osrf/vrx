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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> node;

// code adapted from ros_ign_bridge

//////////////////////////////////////////////////
/// \brief A class for testing ROS topic subscription.
template<typename ROS_T>
class MyTestClass
{
  /// \brief Class constructor.

public:
  explicit MyTestClass(const std::string & _topic)
  {
    this->sub =
      node->create_subscription<ROS_T>(_topic, 1000,
      std::bind(&MyTestClass::Cb, this, _1));
  }

  /// \brief Member function called each time a topic update is received.

public:
  void Cb(const typename ROS_T::SharedPtr _msg)
  {
    this->callbackExecuted = true;
  }

  /// \brief Member variables that flag when the actions are executed.

public:
  bool callbackExecuted = false;

/// \brief ROS subscriber;

private:
  typename rclcpp::Subscription<ROS_T>::SharedPtr sub;
};

/// \brief Wait until a boolean variable is set to true for a given number
/// of times. This function calls ros::spinOnce each iteration.
/// \param[in out] _boolVar The bool variable.
/// \param[in] _sleepEach Time duration to wait between each retry.
/// \param[in] _retries The number of retries.
///
/// E.g.:
///   using namespace std::chrono_literals;
///   waitUntilBoolVar(myVar, 1ms, 500);
template<class Rep, class Period>
void waitUntilBoolVarAndSpin(
  std::shared_ptr<rclcpp::Node> & node,
  bool & _boolVar,
  const std::chrono::duration<Rep, Period> & _sleepEach,
  const int _retries)
{
  int i = 0;
  while (!_boolVar && i < _retries) {
    ++i;
    std::this_thread::sleep_for(_sleepEach);
    rclcpp::spin_some(node);
  }
}

/////////////////////////////////////////////////
TEST(RosApiTest, SimTopics)
{
  // tf
  MyTestClass<tf2_msgs::msg::TFMessage> tf("/tf");
  waitUntilBoolVarAndSpin(
    node, tf.callbackExecuted, 10ms, 3500);
  EXPECT_TRUE(tf.callbackExecuted);

  // tf_static
  MyTestClass<tf2_msgs::msg::TFMessage> tfStatic("/tf_static");
  waitUntilBoolVarAndSpin(
    node, tfStatic.callbackExecuted, 10ms, 3500);
  EXPECT_TRUE(tfStatic.callbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("test_ros_api");

  return RUN_ALL_TESTS();
}
