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
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

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
#include <gz/transport/Node.hh>

#include <sdf/sdf.hh>

#include "PlacardPlugin.hh"

using namespace gz;
using namespace vrx;

/// \brief Private PlacardPlugin data class.
class PlacardPlugin::Implementation
{
  /// \brief Creates a msgs::Color message from 4 doubles.
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

  /// \brief Display the symbol in the placard
  public: void Update();

  /// \brief ROS callback for changing a symbol and its color.
  /// \param[in] _msg Not used.
  public: void ChangeSymbol(const msgs::Empty &_msg);

  /// \brief Gazebo callback for changing light to a specific color pattern.
  /// \param[in] _msg New symbol.
  public: void ChangeSymbolTo(const msgs::StringMsg_V &_msg);

  /// \brief List of the color options (red, green, blue, and no color)
  /// with their string name for logging.
  public: static std::map<std::string, msgs::Color> kColors;

  /// \brief List of the shape options (circle, cross, triangle)
  /// with their string name for logging.
  public: static std::vector<std::string> kShapes;

  /// \brief The current color.
  public: std::string color;

  /// \brief The current shape.
  public: std::string shape;

  /// \brief All color/symbol sequences.
  public: std::vector<std::array<std::string, 2u>> allPatterns;

  /// \brief The index pointing to one of the potential color/symbol sequence.
  public: size_t allPatternsIdx = 0u;

  /// \brief Collection of visual names.
  public: std::vector<std::string> visualNames;

  /// \brief Pointer to the visual elements to modify.
  public: std::vector<rendering::VisualPtr> visuals;

  /// \brief Whether shuffle is enabled via a ROS topic or not.
  public: bool shuffleEnabled = true;

  /// \brief Topic namespace.
  public: std::string ns;

  /// \brief topic.
  public: std::string shuffleTopic;

  /// \brief gazebo Node
  public: transport::Node gzNode;

  /// \brief gazebo symbol sub topic
  public: std::string symbolSubTopic;

  /// Pointer to the scene node.
  public: rendering::ScenePtr scene;

  /// \brief True to change symbol shape and color in rendering thread
  public: bool symbolDirty = false;

  /// \brief Locks state and pattern member variables.
  public: std::mutex mutex;

  /// \brief Connection to pre-render event callback
  public: common::ConnectionPtr connection{nullptr};

  /// \brief Vsual entity this plugin is attached to
  public: sim::Entity entity = sim::kNullEntity;
};

// Static initialization.
std::map<std::string, msgs::Color> PlacardPlugin::Implementation::kColors =
  {
    {"red",    CreateColor(1.0, 0.0, 0.0, 1.0)},
    {"green",  CreateColor(0.0, 1.0, 0.0, 1.0)},
    {"blue",   CreateColor(0.0, 0.0, 1.0, 1.0)},
    {"yellow", CreateColor(1.0, 1.0, 0.0, 1.0)},
  };

std::vector<std::string> PlacardPlugin::Implementation::kShapes =
  {"circle", "cross", "triangle", "rectangle"};

/////////////////////////////////////////////////
PlacardPlugin::PlacardPlugin()
    : dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void PlacardPlugin::Configure(const sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm,
    sim::EventManager &_eventMgr)
{
  this->dataPtr->entity = _entity;
  this->dataPtr->InitializeAllPatterns();

  auto sdf = _sdf->Clone();
  if (!this->dataPtr->ParseSDF(sdf))
    return;

  if (this->dataPtr->shuffleEnabled)
  {
    std::string topic = this->dataPtr->ns.empty() ? "" : this->dataPtr->ns + "/";
    topic += this->dataPtr->shuffleTopic;
    this->dataPtr->gzNode.Subscribe(topic,
      &PlacardPlugin::Implementation::ChangeSymbol, this->dataPtr.get());
  }

  // connect to the SceneUpdate event
  // the callback is executed in the rendering thread so do all
  // rendering operations in that thread
  this->dataPtr->connection =
    _eventMgr.Connect<sim::events::SceneUpdate>(
      std::bind(&PlacardPlugin::Implementation::Update, this->dataPtr.get()));

  this->dataPtr->gzNode.Subscribe(this->dataPtr->symbolSubTopic,
    &PlacardPlugin::Implementation::ChangeSymbolTo, this->dataPtr.get());
}

//////////////////////////////////////////////////
msgs::Color PlacardPlugin::Implementation::CreateColor(const double _r,
  const double _g, const double _b, const double _a)
{
  static msgs::Color color;
  color.set_r(_r);
  color.set_g(_g);
  color.set_b(_b);
  color.set_a(_a);
  return color;
}

//////////////////////////////////////////////////
void PlacardPlugin::Implementation::InitializeAllPatterns()
{
  for (auto const &colorPair : this->kColors)
    for (auto const &shape : this->kShapes)
      this->allPatterns.push_back({colorPair.first, shape});
}

//////////////////////////////////////////////////
void PlacardPlugin::Implementation::ChangeSymbolTo(
    const msgs::StringMsg_V &_msg)
{
  if (_msg.data_size() < 2)
  {
    gzerr << "2 string values, [shape, color], are required "
          << "for changing symbol." << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(this->mutex);
  this->shape = _msg.data(0);
  this->color = _msg.data(1);
  this->symbolDirty = true;
}

//////////////////////////////////////////////////
bool PlacardPlugin::Implementation::ParseSDF(sdf::ElementPtr _sdf)
{
  // We initialize it with a random shape and color.
  msgs::Empty emptyMsg;
  this->ChangeSymbol(emptyMsg);

  // Parse the shape.
  if (_sdf->HasElement("shape"))
  {
    std::string aShape = _sdf->GetElement("shape")->Get<std::string>();
    std::transform(aShape.begin(), aShape.end(), aShape.begin(), ::tolower);
    // Sanity check: Make sure the shape is allowed.
    if (std::find(this->kShapes.begin(), this->kShapes.end(), aShape) !=
          this->kShapes.end())
    {
      this->shape = aShape;
    }
    else
    {
      gzerr << "Incorrect " << aShape << " <shape>, using random shape"
            << std::endl;
    }
  }

  // Parse the color. We initialize it with a random color.
  if (_sdf->HasElement("color"))
  {
    std::string aColor = _sdf->GetElement("color")->Get<std::string>();
    std::transform(aColor.begin(), aColor.end(), aColor.begin(), ::tolower);
    // Sanity check: Make sure the color is allowed.
    if (this->kColors.find(aColor) != this->kColors.end())
    {
      this->color = aColor;
    }
    else
    {
      gzerr << "Incorrect " << aColor << " <color>, using random color"
            << std::endl;
    }
  }

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

  // Optional: Is shuffle enabled?
  if (_sdf->HasElement("shuffle"))
  {
    this->shuffleEnabled = _sdf->GetElement("shuffle")->Get<bool>();

    // Required if shuffle enabled: ROS topic.
    if (!_sdf->HasElement("shuffle_topic"))
    {
      gzerr << "<shuffle_topic> missing" << std::endl;
      return false;
    }
    this->shuffleTopic = _sdf->GetElement("shuffle_topic")->Get<std::string>();
  }

  // Required: namespace.
  if (!_sdf->HasElement("robot_namespace"))
  {
    gzerr << "<robot_namespace> missing" << std::endl;
    return false;
  }
  this->ns = _sdf->GetElement("robot_namespace")->Get<std::string>();
  if (!_sdf->HasElement("symbol_topic"))
  {
    this->symbolSubTopic = "/" + this->ns + "/symbol";
  }
  else
  {
    this->symbolSubTopic = _sdf->GetElement("symbol_topic")->Get<std::string>();
  }

  return true;
}

//////////////////////////////////////////////////
void PlacardPlugin::Implementation::Update()
{
  if (!this->scene)
    this->scene = rendering::sceneFromFirstRenderEngine();

  if (!this->scene)
    return;

  // Get the visuals if needed.
  if (this->visuals.empty())
  {
    // this does a breadth first search for visual with the entity id
    auto rootVis = scene->RootVisual();
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
      for (unsigned int i = 0; i < n->ChildCount(); ++i)
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

  // Only update the plugin if new symbol cmd is received
  if (!this->symbolDirty)
    return;

  // Update the visuals.
  for (auto visual : this->visuals)
  {
    msgs::Color color;
    color.set_a(0.0);
    auto name = visual->Name();
    auto delim = name.rfind("/");
    auto shortName = name.substr(delim + 1);
    if (shortName.find(this->shape) != std::string::npos)
    {
      color = this->kColors[this->color];
    }
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

  this->symbolDirty = false;
}

//////////////////////////////////////////////////
void PlacardPlugin::Implementation::ChangeSymbol(const msgs::Empty &_msg)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->color = this->allPatterns[this->allPatternsIdx].at(0);
    this->shape = this->allPatterns[this->allPatternsIdx].at(1);
    this->allPatternsIdx =
      (this->allPatternsIdx + 1) % this->allPatterns.size();
    this->symbolDirty = true;
  }

  gzmsg << "New symbol is " << this->color << " " << this->shape << std::endl;
}

GZ_ADD_PLUGIN(PlacardPlugin,
              sim::System,
              PlacardPlugin::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(vrx::PlacardPlugin,
                    "vrx::PlacardPlugin")
