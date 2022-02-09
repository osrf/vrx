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

#ifndef VRX_GAZEBO_GEO_COORD_GUI_PLUGIN_HH_
#define VRX_GAZEBO_GEO_COORD_GUI_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <sdf/sdf.hh>

#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
// #include <gazebo/gui/gui.hh>
#endif
//#include "gazebo/gui/QTestFixture.hh"
//#include "gazebo/gui/GuiIface.hh"
//#include "gazebo/gui/MainWindow.hh"
//#include "gazebo/gui/Projection_TEST.hh"
namespace gazebo
{
    class GAZEBO_VISIBLE GeoCoordGUIPlugin : public GUIPlugin
    {
        Q_OBJECT
        // Constructor
    public:
        GeoCoordGUIPlugin();

        // Destructor
    public:
        virtual ~GeoCoordGUIPlugin();

    protected:
       void Load(sdf::ElementPtr _elem);

    signals:
        void SetDispCoord(QString _string);
        
        // Figure this out later - update the display every time the mouse moves.
    protected:
        bool OnMouseMove(const common::MouseEvent &_event);

        // Just guessing here - Node used to establish communication with gzserver.
    private:
        gazebo::transport::NodePtr gzNode;

    private:
        transport::PublisherPtr mousePub;

        // Camera to get scene information and convert mouse xy to scene coords.
    private:
    rendering::ScenePtr scene;
        rendering::UserCameraPtr camera;

          /// \brief Topic where the transformed mouse coordinate is published.
    private: std::string mouseGeoTopic = "/vrx/mouse_geo_loc";

        /// \brief Topic where world frame mouse coordinate is published.
    private: std::string mouseWorldTopic = "/vrx/mouse_world_loc";
    };
}
#endif