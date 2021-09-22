/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef ROBOTX_GAZEBO_TEST_HELPERS_HH_
#define ROBOTX_GAZEBO_TEST_HELPERS_HH_

#include <gazebo_msgs/srv/get_model_list.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>

using namespace std::literals::chrono_literals;


/// \brief Check whether a model exists on simulation.
/// \param[in] _name The model name.
/// \param[in] _timeout Timeout to wait for the service.
/// \param[in] _retries Number of times it will call the service.
/// \return True when the model was found or false otherwise.
bool ModelExists(const std::string &_name, const int _timeout = 10000, const int _retries = 10)
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("sandisland_test");
  rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr client =
    node->create_client<gazebo_msgs::srv::GetModelList>("get_model_list");

  auto request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();

  while (!client->wait_for_service(std::chrono::milliseconds(_timeout))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service.");
      return false;
    }

    RCLCPP_INFO(node->get_logger(), "Service not available.");
    return false;
  }

  int tries(0);
  bool model_exists(false);

  rclcpp::Rate rate(std::chrono::milliseconds(5000));

  while (model_exists == false && tries < _retries) {
    auto result = client->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      for (auto model : result.get()->model_names)
      {
        if (model == _name) {
          model_exists = true;
          break;
        }
      }
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to call service get_model_list");
    }

    tries++;
    rate.sleep();
  }

  return model_exists;
}

#endif
