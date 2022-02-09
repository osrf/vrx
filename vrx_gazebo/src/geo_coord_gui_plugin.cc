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
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/UserCamera.hh>
#include "vrx_gazebo/geo_coord_gui_plugin.hh"

using namespace gazebo;

// Register plugin with Gazebo
GZ_REGISTER_GUI_PLUGIN(GeoCoordGUIPlugin)

//////////////////////////////////////////////////
GeoCoordGUIPlugin::GeoCoordGUIPlugin() : GUIPlugin()
{
    // set up frames for GUI overlay
    QHBoxLayout *mainLayout = new QHBoxLayout;
    QFrame *mainFrame = new QFrame();
    QVBoxLayout *frameLayout = new QVBoxLayout();
    mainFrame->setLayout(frameLayout);
    mainLayout->addWidget(mainFrame);

    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);
    this->resize(250, 100);

    // Create display box for geo position
    QLabel *posLabel = new QLabel(tr("00.0000N, 00.0000W"));
    frameLayout->addWidget(posLabel);

    gui::MouseEventHandler::Instance()->AddMoveFilter("geo_coord",
                                                      std::bind(&GeoCoordGUIPlugin::OnMouseMove, this, std::placeholders::_1));


    connect(this, SIGNAL(SetDispCoord(QString)),
            posLabel, SLOT(setText(QString)), Qt::QueuedConnection);
    this->gzNode = transport::NodePtr(new transport::Node());
    this->gzNode->Init();
    this->mousePub = this->gzNode->Advertise<gazebo::msgs::Vector3d>
    (mouseGeoTopic);
    gzmsg << "Publishing on " << mouseGeoTopic << std::endl;
}

GeoCoordGUIPlugin::~GeoCoordGUIPlugin()
{
}

void GeoCoordGUIPlugin::Load(sdf::ElementPtr _elem)
{
      //  this->camera = gazebo::gui::get_active_camera();
}

bool GeoCoordGUIPlugin::OnMouseMove(const common::MouseEvent &_event)
{
/*     ignition::math::Vector3d mouse_loc;
    if (!this->camera->GetScene()->FirstContact(
            this->camera, _event.Pos(), mouse_loc))
    {
        return false;
    }
    gazebo::msgs::Vector3d mouse_msg;
#if GAZEBO_MAJOR_VERSION < 6
    gazebo::msgs::Set(&mouse_msg, gazebo::math::Vector3(mouse_loc.X, mouse_loc.Y, mouse_loc.Z));
#else
    gazebo::msgs::Set(&mouse_msg, mouse_loc);
#endif

    this->mousePub->Publish(mouse_msg);
    // this->SetDispCoord(QString::fromStdString(
    //     latlon.X() + latlong.Y())); //convert to string? */
    return true;
}