#include <ignition/common/Console.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/plugin/Register.hh>

#include "ScoringPlugin.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

IGNITION_ADD_PLUGIN(ScoringPlugin,
                    ignition::gazebo::System,
                    ScoringPlugin::ISystemConfigure,
                    ScoringPlugin::ISystemPostUpdate)