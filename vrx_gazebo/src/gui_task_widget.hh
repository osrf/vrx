/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef _GUI_TASK_WIDGET_HH_
#define _GUI_TASK_WIDGET_HH_

#include <rclcpp/rclcpp.hpp>
#include <vrx_gazebo/msg/task.hpp>
#include <vrx_gazebo/msg/contact.hpp>

#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <gazebo_msgs/msg/link_states.hpp>

namespace gazebo
{
  class GUITaskWidget : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: GUITaskWidget();

    /// \brief Destructor
    public: ~GUITaskWidget();

    /// \brief A signal used to set the task info line edit.
    /// \param[in] _string String representation of task info.
    signals: void SetTaskInfo(QString _string);

    /// \brief A signal used to set the wind and wamv direction
    /// \param[in] _string String representation of windspeed info.
    signals: void SetWindDirection(QPixmap _pixmap);

    /// \brief A signal used to set the contact widget
    /// \param[in] _string String representation of windspeed info.
    signals: void SetContact(QPixmap _pixmap);

    /// \brief Callback that received task info messages.
    /// \param[in] _msg Task info message that is received.
    protected: void OnTaskInfo(const vrx_gazebo::msg::Task::SharedPtr _msg);

    /// \brief Callback that received wind speed messages.
    /// \param[in] _msg windspeed info message that is received.
    protected: void OnWindSpeed(const std_msgs::msg::Float64::SharedPtr _msg);

    /// \brief Callback that received wind direction messages.
    /// \param[in] _msg wind direction info message that is received.
    protected: void OnWindDirection(const std_msgs::msg::Float64::SharedPtr _msg);

    /// \brief Callback that receives link state messages.
    /// \param[in] _msg wind direction info message that is received.
    protected: void OnLinkStates(const gazebo_msgs::msg::LinkStates::SharedPtr _msg);

    /// \brief Callback that receives Contact messages.
    /// \param[in] _msg wind direction info message that is received.
    protected: void OnContact(const vrx_gazebo::msg::Contact::SharedPtr _msg);

    /// \brief Helper function to format time string.
    /// \param[in] _msg Time message.
    /// \return Time formatted as a string.
    private: std::string FormatTime(unsigned int sec) const;

    /// \brief A ros NodeHandle
    private: std::shared_ptr<rclcpp::Node> node;

    /// \brief Subscriber to Task messages.
    private: rclcpp::Subscription<vrx_gazebo::msg::Task>::SharedPtr taskSub;

    /// \brief Subscriber to wind Speed messages.
    private: rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr windSpeedSub;

    /// \brief Subscriber to wind direction messages.
    private: rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr windDirectionSub;

    /// \brief Subscriber to link state messages.
    private: rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr linkStateSub;

    /// \brief Subscriber to contact messages.
    private: rclcpp::Subscription<vrx_gazebo::msg::Contact>::SharedPtr contactSub;

    /// \brief Last time contact occurred
    private: rclcpp::Time contactTime;

    /// \brief Pixmap for the wind and wamv direction compass
    private: QPixmap windPixmap;

    /// \brief Paiter for painting on the compass
    private: QPainter windPainter;

    /// \breif pixmap for the contact widget
    private: QPixmap contactPixmap;

    /// \brief painter for contact widget
    private: QPainter contactPainter;

    /// \brief general use pen
    private: QPen pen;

    /// \breif last reported windspeed
    private: double windSpeed = 0;

    /// \brief last reported wamvHeading(yaw)
    private: double wamvHeading = 0;
  };
}

#endif
