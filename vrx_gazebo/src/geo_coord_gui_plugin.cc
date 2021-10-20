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
#include "geo_coord_gui_plugin.hh"

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
    QLabel *posLabel = new QLAbel(tr("00.0000N, 00.0000W"));
    frameLayout->addWidget(posLabel);

    gui::MouseEventHandler::Instance()->AddMoveFilter("geo_coord",
                                                      std::bind(&GeoCoordGUIPlugin::OnMouseMove, this, std::placeholders::_1));

    this->camera = gui::get_active_camera();
    connect(this, SIGNAL(SetDispCoord(QString)),
            posLabel, SLOT(setText(QString)), Qt::QueuedConnection);
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();
}

GeoCoordGUIPlugin::~GeoCoordGUIPlugin()
{
}

void GeoCoordGUIPlugin::Load(sdf::ElementPtr _elem)
{
}

bool GeoCoordGUIPlugin::OnMouseMove(const common::MouseEvent &_event)
{
    ignition::math::Vector3d mouse_loc;
    if (!this->camera->GetScene()->FirstContact(
            this->camera, _event.Pos(), mouse_loc))
    {
        return false;
    }
    ignition::math::Vector3d spherical_coord;
    spherical_coord = GET THE SPHERICAL COORDINATES SOMEHOW ? ::PositionTransform(mouse_loc, GLOBAL, SPHERICAL); //how do we import the spherical coordinates?
    this->SetDispCoord(QString::fromStdString(
        spherical_coord.X() + spherical_coord.Y())); //convert to string?
}