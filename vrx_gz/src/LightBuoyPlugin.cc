/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <gz/msgs/color.pb.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Visual.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/rendering/Events.hh>

#include <sdf/sdf.hh>

#include "LightBuoyPlugin.hh"

using namespace gz;
using namespace vrx;

/// \brief Private LightBuoyPlugin data class.
class LightBuoyPlugin::Implementation
{
  /// \def Pattern_t
  /// \brief The current pattern to display, pattern[3] and pattern[4]
  /// are always OFF.
  private: using Pattern_t = std::array<std::string, 5>;

  /// \brief Creates a gz::msgs::Color message from 4 doubles.
  /// \param[in] _r Red.
  /// \param[in] _g Green.
  /// \param[in] _b Blue.
  /// \param[in] _a Alpha.
  /// \return The Color message.
  public: static msgs::Color CreateColor(const double _r,
                                         const double _g,
                                         const double _b,
                                         const double _a);

  /// \brief Initialize all color/symbol sequences.
  public: void InitializeAllPatterns();

  /// \brief Parse all SDF parameters.
  /// \param[in] _sdf SDF elements.
  public: bool ParseSDF(sdf::ElementPtr _sdf);

  /// \brief Display the next color in the sequence, or start over if at the end
  public: void Update();

  /// \brief List of the color options (red, green, blue, yellow and no color)
  /// with their string name for logging.
  public: static std::map<std::string, gz::msgs::Color> kColors;

  /// \brief Collection of visual names.
  public: std::vector<std::string> visualNames;

  /// \brief Pointer to the visual elements to modify.
  public: std::vector<gz::rendering::VisualPtr> visuals;

  /// Pointer to the scene node.
  public: rendering::ScenePtr scene;

  /// \brief The color pattern.
  public: Pattern_t pattern;

  /// \brief Track current index in pattern.
  public: uint8_t state = 0u;

  /// \brief Next time where the plugin should be updated.
  public: std::chrono::duration<double> nextUpdateTime{0.0};

  /// \brief Current time (simulation).
  public: std::chrono::duration<double> currentTime;

  /// \brief Locks state and pattern member variables.
  public: std::mutex mutex;

  /// \brief Connection to pre-render event callback
  public: gz::common::ConnectionPtr connection{nullptr};

  /// \brief Vsual entity this plugin is attached to
  public: sim::Entity entity = sim::kNullEntity;
};

// Static initialization.
std::map<std::string, gz::msgs::Color>
  LightBuoyPlugin::Implementation::kColors =
  {
    {"red",    CreateColor(1.0, 0.0, 0.0, 1.0)},
    {"green",  CreateColor(0.0, 1.0, 0.0, 1.0)},
    {"blue",   CreateColor(0.0, 0.0, 1.0, 1.0)},
    {"yellow", CreateColor(1.0, 1.0, 0.0, 1.0)},
    {"off",    CreateColor(0.0, 0.0, 0.0, 1.0)},
  };

//////////////////////////////////////////////////
msgs::Color LightBuoyPlugin::Implementation::CreateColor(const double _r,
  const double _g, const double _b, const double _a)
{
  static msgs::Color color;
  color.set_r(_r);
  color.set_g(_g);
  color.set_b(_b);
  color.set_a(_a);
  return color;
}

/////////////////////////////////////////////////
bool LightBuoyPlugin::Implementation::ParseSDF(sdf::ElementPtr _sdf)
{
  // Required: Sequence of colors.
  uint8_t i = 0u;
  for (auto colorIndex : {"color_1", "color_2", "color_3"})
  {
    if (!_sdf->HasElement(colorIndex))
    {
      gzerr << "Missing <" << colorIndex << ">" << std::endl;
      return false;
    }

    auto color = _sdf->GetElement(colorIndex)->Get<std::string>();
    std::transform(color.begin(), color.end(), color.begin(), ::tolower);

    // Sanity check: color should be red, green, blue, yellow or off.
    if (color != "red"  && color != "green" &&
        color != "blue" && color != "yellow" && color != "off")
    {
      gzerr << "Invalid color [" << color.c_str() << "]" << std::endl;
      return false;
    }

    this->pattern[i++] = color;
  }

  // The last two colors of the pattern are always black.
  this->pattern[3] = "off";
  this->pattern[4] = "off";

  // Required: visuals.
  if (!_sdf->HasElement("visuals"))
  {
    gzerr << "<visuals> missing" << std::endl;
    return false;
  }

  auto visualsElem = _sdf->GetElement("visuals");
  if (!visualsElem->HasElement("visual"))
  {
    gzerr << "<visual> missing" << std::endl;
    return false;
  }

  auto visualElem = visualsElem->GetElement("visual");
  while (visualElem)
  {
    std::string visualName = visualElem->Get<std::string>();
    this->visualNames.push_back(visualName);
    visualElem = visualElem->GetNextElement();
  }

  return true;
}

//////////////////////////////////////////////////
void LightBuoyPlugin::Implementation::Update()
{
  if (!this->scene)
    this->scene = rendering::sceneFromFirstRenderEngine();

  if (!this->scene)
    return;

  // Get the visuals if needed.
  if (this->visuals.empty())
  {
    // this does a breadth first search for visual with the entity id
    auto rootVis = this->scene->RootVisual();
    std::list<rendering::NodePtr> nodes;
    nodes.push_back(rootVis);
    rendering::VisualPtr visual;
    while (!nodes.empty())
    {
      auto n = nodes.front();
      nodes.pop_front();
      if (n && n->HasUserData("gazebo-entity"))
      {
        auto variant = n->UserData("gazebo-entity");
        const uint64_t *value = std::get_if<uint64_t>(&variant);
        if (value && *value == static_cast<uint64_t>(this->entity))
        {
          visual = std::dynamic_pointer_cast<rendering::Visual>(n);
          break;
        }
      }
      for (unsigned int i = 0u; i < n->ChildCount(); ++i)
        nodes.push_back(n->ChildByIndex(i));
    }

    if (!visual)
    {
      gzerr << "Unable to find visual associated with entity id: "
            << this->entity << std::endl;
      return;
    }

    rendering::VisualPtr linkVisual =
      std::dynamic_pointer_cast<rendering::Visual>(visual->Parent());

    if (!linkVisual)
    {
      gzerr << "Unable to find parent link associated with visual entity id: "
            << this->entity << std::endl;
      return;
    }

    for (const auto &name : this->visualNames)
    {
      // find visual using specified name
      rendering::NodePtr node  = linkVisual->ChildByName(name);

      // if not found, try searching using short name
      if (!node)
      {
        auto delim = name.rfind("/");
        auto shortName = name.substr(delim + 1);
        node  = linkVisual->ChildByName(shortName);
      }

      if (node)
      {
        auto v = std::dynamic_pointer_cast<rendering::Visual>(node);
        if (v)
          this->visuals.push_back(v);
      }
      else
      {
        gzerr << "Unable to find visual: " << name << std::endl;
      }
    }
  }

  std::lock_guard<std::mutex> lock(this->mutex);

  if (this->currentTime < this->nextUpdateTime)
    return;

  this->nextUpdateTime += std::chrono::duration<double>(1.0);

  // Start over if at end of pattern
  if (this->state > 4u)
    this->state = 0u;

  auto color = this->kColors[this->pattern[this->state]];

  // Update the visuals.
  for (auto visual : this->visuals)
  {
    math::Color gazeboColor(color.r(), color.g(), color.b(), color.a());

    auto mat = visual->Material();
    if (!mat)
    {
      auto newMat = this->scene->CreateMaterial();
      // this sets the material with clone = true so internally the visual
      // makes a clone of the material. We can then remove newMat
      visual->SetMaterial(newMat);
      mat = visual->Material();
      this->scene->DestroyMaterial(newMat);
    }
    mat->SetAmbient(gazeboColor);
    mat->SetDiffuse(gazeboColor);
  }

  // Increment index for next timer callback
  ++this->state;
}

/////////////////////////////////////////////////
LightBuoyPlugin::LightBuoyPlugin()
  : dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void LightBuoyPlugin::Configure(const sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm,
    sim::EventManager &_eventMgr)
{
  this->dataPtr->entity = _entity;

  auto sdf = _sdf->Clone();
  if (!this->dataPtr->ParseSDF(sdf))
  {
    gzerr << "Error parsing SDF, plugin disabled." << std::endl;
    return;
  }

  // Connect to the SceneUpdate event.
  // The callback is executed in the rendering thread so do all
  // rendering operations in that thread.
  this->dataPtr->connection =
    _eventMgr.Connect<sim::events::SceneUpdate>(
      std::bind(&LightBuoyPlugin::Implementation::Update, this->dataPtr.get()));
}

//////////////////////////////////////////////////
void LightBuoyPlugin::PreUpdate(const sim::UpdateInfo &_info,
  sim::EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentTime = _info.simTime;
}

GZ_ADD_PLUGIN(LightBuoyPlugin,
              sim::System,
              LightBuoyPlugin::ISystemConfigure,
              LightBuoyPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::LightBuoyPlugin,
                    "vrx::LightBuoyPlugin")
