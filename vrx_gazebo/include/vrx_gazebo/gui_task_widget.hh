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

#include <string>

#include <boost/shared_ptr.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>
#include "ros/ros.h"
#include "vrx_gazebo/Task.h"

namespace gazebo
{
  class GAZEBO_VISIBLE GUITaskWidget : public GUIPlugin
  {
    //Q_OBJECT

    /// \brief Constructor
    public: GUITaskWidget();

    /// \brief Destructor
    public: virtual ~GUITaskWidget();

    /// \brief A signal used to set the task info line edit.
    /// \param[in] _string String representation of task info.
    signals: void SetTaskInfo(QString _string);

    /// \brief Callback that received task info messages.
    /// \param[in] _msg Task info message that is received.
    protected: void OnTaskInfo(const vrx_gazebo::Task::ConstPtr &_msg);

    /// \brief Helper function to format time string.
    /// \param[in] _msg Time message.
    /// \return Time formatted as a string.
    private: std::string FormatTime(unsigned int sec) const;

    /// \brief A ros NodeHandle
    private: boost::shared_ptr<ros::NodeHandle> node;

    /// \brief Subscriber to Task messages.
    private: ros::Subscriber taskSub;
  };
}

#endif
