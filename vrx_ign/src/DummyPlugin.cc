#include <ignition/plugin/Register.hh>

#include "DummyPlugin.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

DummyPlugin::DummyPlugin()
{
    //
}

void DummyPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
    //
    return;
}

void DummyPlugin::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
    //
    return;
}




IGNITION_ADD_PLUGIN(DummyPlugin,
                    ignition::gazebo::System,
                    DummyPlugin::ISystemConfigure,
                    DummyPlugin::ISystemPostUpdate)