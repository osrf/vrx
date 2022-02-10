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
#endif
#include "gazebo/gui/GuiIface.hh"


namespace gazebo
{
    class GAZEBO_VISIBLE GeoCoordGUIPlugin : public GUIPlugin
    {
        Q_OBJECT
        /// \brief Constructor
    public:
        GeoCoordGUIPlugin();

        /// \brief Destructor
    public:
        virtual ~GeoCoordGUIPlugin();

    protected:
       void Load(sdf::ElementPtr _elem);

    signals:
        void SetDispCoord(QString _string);
        
        /// \brief Update the display every time the mouse moves.
    protected:
        bool OnMouseMove(const common::MouseEvent &_event);

        /// \brief Node used to establish communication with gzserver.
    private:
        gazebo::transport::NodePtr gzNode;

    private:
        /// \brief Camera to get scene info and convert GUI xy to coords.
        rendering::UserCameraPtr camera;

    private:
        /// \brief Pub/sub for coordinates pre- and post- transform.
        transport::PublisherPtr mousePub;
        transport::SubscriberPtr mouseSub;
        /// \brief Topic where the transformed mouse coordinate is published.
        std::string mouseGeoTopic = "/vrx/mouse_geo_loc";
        /// \brief Topic where world frame mouse coordinate is published.
        std::string mouseWorldTopic = "/vrx/mouse_world_loc";
    protected: 
        /// \brief Callback to display received lat/ long position on GUI.
        void OnLatLong(ConstVector3dPtr &_msg);
    };
}
#endif