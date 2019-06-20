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
#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "vrx_gazebo/Task.h"
#include "vrx_gazebo/Contact.h"
#include "gui_task_widget.hh"
#include <math.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GUITaskWidget)

/////////////////////////////////////////////////
GUITaskWidget::GUITaskWidget()
  : GUIPlugin(), pixmap(150,150), painter(&(this->pixmap))
{
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler |
              ros::init_options::AnonymousName);
  }

  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->setContentsMargins(0, 0, 0, 0);

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();
  // Create the layout that sits inside the frame
  QHBoxLayout *InfoLayout = new QHBoxLayout();
  InfoLayout->setContentsMargins(0, 0, 0, 0);

  // Task info Block
  // Create a time label
  QLabel *taskInfo = new QLabel; 
  // Add the label to the frame's layout
  InfoLayout->addWidget(taskInfo);
  connect(this, SIGNAL(SetTaskInfo(QString)),
      taskInfo, SLOT(setText(QString)), Qt::QueuedConnection); 

  // Wind direction block
  // Create a time label
  QLabel *windDirection = new QLabel; 
  // Add the label to the frame's layout
  InfoLayout->addWidget(windDirection);
  connect(this, SIGNAL(SetWindDirectionInfo(QPixmap)),
      windDirection, SLOT(setPixmap(QPixmap)), Qt::QueuedConnection); 
 
  // Wind speed block
  // Create a time label
  QLabel *windSpeed = new QLabel; 
  // Add the label to the frame's layout
  InfoLayout->addWidget(windSpeed);
  connect(this, SIGNAL(SetWindSpeedInfo(QString)),
      windSpeed, SLOT(setText(QString)), Qt::QueuedConnection); 

  this->pixmap.fill(Qt::gray);
  this->painter.setBrush(Qt::NoBrush);
  QPen pen;
  pen.setColor(Qt::black);
  pen.setWidth(10);
  this->painter.setPen(pen);
  this->painter.drawEllipse(5, 5, 140, 140);
  this->painter.setPen(Qt::red);
  this->painter.drawText(QRect(71, -2, 20, 20), 0, tr("N"), nullptr);

  // Add frameLayout to the frame
  mainFrame->setLayout(InfoLayout);
  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // set position and resize this widget
  this->setLayout(mainLayout);
  this->move(10, 10);
  this->resize(600, 170);

  // Subscribe to tasks topic (ROS)
  this->node.reset(new ros::NodeHandle);
  this->taskSub = this->node->subscribe("/vrx/task/info", 1,
      &GUITaskWidget::OnTaskInfo, this);
  this->windSpeedSub = this->node->subscribe("/vrx/debug/wind/speed", 1,
      &GUITaskWidget::OnWindSpeed, this);
  this->windDirectionSub = this->node->subscribe("/vrx/debug/wind/direction", 1,
      &GUITaskWidget::OnWindDirection, this);
}

/////////////////////////////////////////////////
GUITaskWidget::~GUITaskWidget()
{
}

/////////////////////////////////////////////////
void GUITaskWidget::OnWindDirection(const std_msgs::Float64::ConstPtr &_msg)
{
  double pi = 3.14159265;
  double scale = 5*this->windSpeed;
  double x = scale*cos((pi/180)*(_msg->data - 90)) + 75;
  double y = scale*sin((pi/180)*(_msg->data - 90)) + 75;
  this->painter.drawLine(QLine(75,75,x,y));
  this->SetWindDirectionInfo(this->pixmap);
}

/////////////////////////////////////////////////
void GUITaskWidget::OnWindSpeed(const std_msgs::Float64::ConstPtr &_msg)
{ 
  std::ostringstream windSpeedStream;
  windSpeedStream.str("");
  windSpeedStream << "Wind Speed: " << _msg->data << "\n";
  this->SetWindSpeedInfo(QString::fromStdString(windSpeedStream.str()));
  this->windSpeed = _msg->data;
}

/////////////////////////////////////////////////
void GUITaskWidget::OnTaskInfo(const vrx_gazebo::Task::ConstPtr &_msg)
{
  std::ostringstream taskInfoStream;
  taskInfoStream.str("");
  taskInfoStream << "Task Info:\n";
  taskInfoStream << "Task Name: " << _msg->name << "\n";
  taskInfoStream << "Task Stream: " << _msg->state << "\n";
  taskInfoStream << "Ready Time: " <<
    this->FormatTime(_msg->ready_time.toSec()) << "\n";
  taskInfoStream << "Running Time: " <<
    this->FormatTime(_msg->running_time.toSec()) << "\n";
  taskInfoStream << "Elapsed Time: " <<
    this->FormatTime(_msg->elapsed_time.toSec()) << "\n";
  taskInfoStream << "Remaining Time: " <<
    this->FormatTime(_msg->remaining_time.toSec()) << "\n";
  taskInfoStream << "Timed out: ";
  if (_msg->timed_out) taskInfoStream << "true" << "\n";
  else taskInfoStream << "false" << "\n";
  taskInfoStream << "Score: " << _msg->score << "\n";
  this->SetTaskInfo(QString::fromStdString(taskInfoStream.str()));
}

/////////////////////////////////////////////////
std::string GUITaskWidget::FormatTime(unsigned int sec) const
{
  std::ostringstream stream;
  unsigned int day, hour, min;

  stream.str("");

  day = sec / 86400;
  sec -= day * 86400;

  hour = sec / 3600;
  sec -= hour * 3600;

  min = sec / 60;
  sec -= min * 60;

  stream << std::setw(2) << std::setfill('0') << day << " ";
  stream << std::setw(2) << std::setfill('0') << hour << ":";
  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec << ".";

  return stream.str();
}

