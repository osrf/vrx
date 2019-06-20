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

#include <ros/ros.h>
#include <vrx_gazebo/Task.h>

#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>
#include "std_msgs/Float64.h"

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

    /// \brief A signal used to set the wind speed info line edit.
    /// \param[in] _string String representation of windspeed info.
    signals: void SetWindSpeedInfo(QString _string);

    /// \brief A signal used to set the wind speed info line edit.
    /// \param[in] _string String representation of windspeed info.
    signals: void SetWindDirectionInfo(QPixmap _pixmap);

    /// \brief Callback that received task info messages.
    /// \param[in] _msg Task info message that is received.
    protected: void OnTaskInfo(const vrx_gazebo::Task::ConstPtr &_msg);
    
    /// \brief Callback that received wind speed messages.
    /// \param[in] _msg windspeed info message that is received.
    protected: void OnWindSpeed(const std_msgs::Float64::ConstPtr &_msg);
    
    /// \brief Callback that received wind direction messages.
    /// \param[in] _msg wind direction info message that is received.
    protected: void OnWindDirection(const std_msgs::Float64::ConstPtr &_msg);
    /*
    /// \brief Callback that received wind direction messages.
    /// \param[in] _msg wind direction info message that is received.
    protected: void OnContact(const std_msgs::Float64::ConstPtr &_msg);
    */
    /// \brief Helper function to format time string.
    /// \param[in] _msg Time message.
    /// \return Time formatted as a string.
    private: std::string FormatTime(unsigned int sec) const;

    /// \brief A ros NodeHandle
    private: std::unique_ptr<ros::NodeHandle> node;

    /// \brief Subscriber to Task messages.
    private: ros::Subscriber taskSub;

    /// \brief Subscriber to wind Speed messages.
    private: ros::Subscriber windSpeedSub;

    /// \brief Subscriber to wind direction messages.
    private: ros::Subscriber windDirectionSub;

    /// \brief Subscriber to wind direction messages.
    private: ros::Subscriber contactSub;

    private: QPixmap pixmap;

    private: QPainter painter;

    private: double windSpeed;
  };
}

#endif
