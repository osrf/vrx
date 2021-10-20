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

#ifndef VRX_GAZEBO_GEO_COORD_PLUGIN_HH_
#define VRX_GAZEBO_GEO_COORD_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>
#endif

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

    signals:
        void SetDispCoord(QString _string);
        
        // Figure this out later - update the display every time the mouse moves.
    protected:
        bool OnMouseMove(const common::MouseEvent &_event);

        // Just guessing here - Node used to establish communication with gzserver.
    private:
        transport::NodePtr node;

    private:
        transport::PublisherPtr factoryPub;

        // Camera to get scene information and convert mouse xy to scene coords.
    private:
        rendering::UserCameraPtr camera;
    }
}
#endif