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

#include <sstream>
#include <ignition/math/Vector3.hh>
#include <gazebo/gui/Conversions.hh>
#include <gazebo/gui/MouseEventHandler.hh>
#include <gazebo/gui/KeyEventHandler.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/UserCamera.hh>
#include "geo_coord_gui_plugin.hh"

using namespace gazebo;


//////////////////////////////////////////////////
GeoCoordGUIPlugin::GeoCoordGUIPlugin() : GUIPlugin()
{
#if GAZEBO_MAJOR_VERSION >= 8
    // set up frames for GUI overlay
    QHBoxLayout *mainLayout = new QHBoxLayout;
    QFrame *mainFrame = new QFrame();
    QVBoxLayout *frameLayout = new QVBoxLayout();
    mainFrame->setLayout(frameLayout);
    mainLayout->addWidget(mainFrame);

    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);
    this->resize(200, 15);

    // Add a QLineEdit widget.  Should be a QLabel but the text
    // is not clearing correctly between updates - fix TBD.
    QLineEdit *posDisplay = new QLineEdit();
    frameLayout->addWidget(posDisplay);

    // Bind GeoCoordGUIPlugin::OnMouseMove so the function is 
    // called when the mouse moves, and create the connection 
    // to update the displayed coordinates
    gui::MouseEventHandler::Instance()->AddMoveFilter("geo_coord",
        std::bind(&GeoCoordGUIPlugin::OnMouseMove, this,
        std::placeholders::_1));
    connect(this, SIGNAL(SetDispCoord(QString)), posDisplay,
        SLOT(setText(QString)), Qt::QueuedConnection);

    // Initialize transport and create pub/sub    
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init("mouse_loc_handler");
    this->mousePub = this->gzNode->Advertise<gazebo::msgs::Vector3d>
        (mouseWorldTopic);
    this->mouseSub = this->gzNode->Subscribe(mouseGeoTopic,
        &GeoCoordGUIPlugin::OnLatLong, this);

#endif
}

GeoCoordGUIPlugin::~GeoCoordGUIPlugin()
{
}

// Get the camera for use in transforming mouse positions.
void GeoCoordGUIPlugin::Load(sdf::ElementPtr _elem)
{
    this->camera = gazebo::gui::get_active_camera();
}

// Every time the mouse moves, this function is triggered.
// Convert mouse location to world frame coordinates of first contact,
// then publish the coordinates.
bool GeoCoordGUIPlugin::OnMouseMove(const common::MouseEvent &_event)
{
    ignition::math::Vector3d mouse_loc;
    if (!this->camera->GetScene()->FirstContact(
            this->camera, _event.Pos(), mouse_loc))
    {
        return false;
    }
    gazebo::msgs::Vector3d mouse_msg;

#if GAZEBO_MAJOR_VERSION < 6
    gazebo::msgs::Set(&mouse_msg, gazebo::math::Vector3(mouse_loc.X,
        mouse_loc.Y, mouse_loc.Z));
#else
    gazebo::msgs::Set(&mouse_msg, mouse_loc);
#endif

    this->mousePub->Publish(mouse_msg);
    return true;
}
// Subscriber callback for receipt of the transformed position.
// Triggers SetDispCoord SIGNAL to update the position GUI display.
void GeoCoordGUIPlugin::OnLatLong(ConstVector3dPtr &_msg)
{
    std::stringstream temp;
    temp << _msg->x() << ", " << _msg->y();
    this->SetDispCoord(QString::fromStdString(temp.str()));
}

// Register plugin with Gazebo
GZ_REGISTER_GUI_PLUGIN(GeoCoordGUIPlugin)