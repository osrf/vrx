#ifndef DUMMY_VRX_PLUGIN_HH_
#define DUMMY_VRX_PLUGIN_HH_

#include <memory>
#include <ignition/gazebo/System.hh>
#include <sdf/sdf.hh>

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
    // Forward declaration
    class DummyPluginPrivate;
    
    /// \brief Doc (MD)
    /// Doc (cont)
    class DummyPlugin
        : public System,
          public ISystemConfigure,
          public ISystemPostUpdate
    {
        /// \brief Constructor
        public: DummyPlugin();
        
        /// \brief Destructor
        public: ~DummyPlugin() override = default;

        // Documentation inherited
        public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

        // Documentation inherited
        public: void PostUpdate(
                    const ignition::gazebo::UpdateInfo &_info,
                    const ignition::gazebo::EntityComponentManager &_ecm) override;
        
    };
}
}
}
}

#endif