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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <memory>
#include <string>
#include <vector>
#include <gz/common/Profiler.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

#include "ScanDockScoringPlugin.hh"

using namespace gz;
using namespace vrx;

/// \brief A class to monitor if the color sequence reported matches the color
/// displayed in the light buoy.
class ColorSequenceChecker
{
  /// \brief Constructor.
  /// \param[in] _expectedColors The sequence of expected colors.
  /// \param[in] _colorSequenceTopic The topic used to receive the color
  /// submisison.
  public: ColorSequenceChecker(const std::vector<std::string> &_expectedColors,
                               const std::string &_colorSequenceTopic);
  /// \brief Enable submissions.
  public: void Enable();

  /// \brief Disable submissions.
  public: void Disable();

  /// \brief Whether a team submitted the color sequence or not.
  /// \return True when the submission was received or false otherwise.
  public: bool SubmissionReceived() const;

  /// \brief Whether a team submitted the color sequence and is correct or not.
  /// \return True when the team submitted the color sequence and it is correct
  /// or false otherwise.
  public: bool Correct() const;

  /// \brief Callback executed when a new color submission is received.
  /// \param[in] _sequence Contains the submission. The size of the vector
  /// should be three.
  private: void OnColorSequence(const msgs::StringMsg_V &_sequence);

  /// \brief The expected color sequence.
  private: std::vector<std::string> expectedSequence;

  /// \brief Topic where the color sequence should be sent.
  private: std::string colorSequenceTopic;

  /// \brief The transport node.
  private: transport::Node node;

  /// \brief Whether the color sequence has been received or not.
  private: bool colorSequenceReceived = false;

  /// \brief Whether the color sequence received is correct or not.
  private: bool correctSequence = false;
};

//////////////////////////////////////////////////
ColorSequenceChecker::ColorSequenceChecker(
  const std::vector<std::string> &_expectedColors,
  const std::string &_colorSequenceTopic)
  : expectedSequence(_expectedColors),
    colorSequenceTopic(_colorSequenceTopic)
{
}

//////////////////////////////////////////////////
void ColorSequenceChecker::Enable()
{
  this->node.Subscribe(this->colorSequenceTopic,
    &ColorSequenceChecker::OnColorSequence, this);
}

//////////////////////////////////////////////////
void ColorSequenceChecker::Disable()
{
  this->node.Unsubscribe(this->colorSequenceTopic);
}

/////////////////////////////////////////////////
bool ColorSequenceChecker::SubmissionReceived() const
{
  return this->colorSequenceReceived;
}

/////////////////////////////////////////////////
bool ColorSequenceChecker::Correct() const
{
  return this->correctSequence;
}

/////////////////////////////////////////////////
void ColorSequenceChecker::OnColorSequence(const msgs::StringMsg_V &_sequence)
{
  gzmsg << "ColorSequenceChecker: Color sequence submission received:"
        << _sequence.DebugString() << std::endl;
  std::cout << std::flush;

  // Sanity check: Only one submission is allowed.
  if (this->colorSequenceReceived)
  {
    gzerr << "The color sequence has already been submitted" << std::endl;
    return;
  }

  // Sanity check: We should receive three colors.
  if (_sequence.data_size() != 3u)
  {
    gzerr << "Received a color sequence with " << _sequence.data_size()
          << " values but expecting three instead. Submission ignored."
          << std::endl;
    return;
  }

  this->colorSequenceReceived = true;

  std::string color1 = _sequence.data(0);
  std::string color2 = _sequence.data(1);
  std::string color3 = _sequence.data(2);

  std::transform(color1.begin(), color1.end(), color1.begin(), ::tolower);
  std::transform(color2.begin(), color2.end(), color2.begin(), ::tolower);
  std::transform(color3.begin(), color3.end(), color3.begin(), ::tolower);

  this->correctSequence =
    color1 == this->expectedSequence[0] &&
    color2 == this->expectedSequence[1] &&
    color3 == this->expectedSequence[2];

  if (this->correctSequence)
  {
    gzmsg << "ColorSequenceChecker: Received color sequence is correct. "
          << " Additional points will be scored." << std::endl;
  }
  else
  {
    gzmsg << "ColorSequenceChecker: Received color sequence is not correct. "
          << "No additional points." << std::endl;
  }
  std::cout << std::flush;
}

/// \brief A class to monitor if the vehicle docked in a given bay.
class DockChecker
{
  /// \brief Constructor.
  /// \param[in] _name The name of the checker (only used for debugging).
  /// \param[in] _internalActivationTopic The gazebo topic used to receive
  /// notifications about the internal activation zone.
  /// \param[in] _externalActivationTopic The gazebo topic used to receive
  /// notifications about the external activation zone.
  /// from the "contain" plugin.
  /// \param[in] _minDockTime Minimum amount of seconds to stay docked to be
  /// considered a fully successfull dock.
  /// \param[in] _correctDock Whether this is the correct bay to dock in.
  /// \param[in] _announceSymbol Symbol to announce via msgs.
  /// E.g.: red_cross, blue_circle
  /// \param[in] _symbolTopic Optional topic to announce the symbol.
  /// \param[in] _vehicleName Name of the vehicle.
  public: DockChecker(const std::string &_name,
                      const std::string &_internalActivationTopic,
                      const std::string &_exteriorActivationTopic,
                      const std::chrono::duration<double> _minDockTime,
                      const bool _correctDock,
                      const std::string &_announceSymbol,
                      const std::string &_symbolTopic,
                      const std::string &_vehicleName);

  /// \brief The name of this checker.
  public: std::string name;

  /// \brief Whether the robot has been successfully docked in this bay or not.
  /// \return True when the robot has been docked or false otherwise.
  public: bool AnytimeDocked() const;

  /// \brief Whether the robot is currently at the entrance of the bay.
  /// \return True when the robot is at the entrance or false othwerwise.
  public: bool AtEntrance() const;

  /// \brief Whether this is the correct bay to dock in.
  public: bool Correct() const;

  /// \brief Announce the symbol of the bay.
  public: void AnnounceSymbol();

  /// \brief Update function that needs to be executed periodically.
  /// \param[in] _info Update information.
  public: void Update(const sim::UpdateInfo &_info);

  /// \brief Callback triggered when the vehicle enters or exits the activation
  /// zone.
  /// \param[in] _msg The current state (0: exiting, 1: entering).
  private: void OnInternalActivationEvent(const msgs::Pose &_msg);

  /// \brief Callback triggered when the vehicle enters or exits the activation
  /// zone.
  /// \param[in] _msg The current state (0: exiting, 1: entering).
  private: void OnExternalActivationEvent(const msgs::Pose &_msg);

  /// \brief The topic used to receive notifications from the internal
  /// activation zone.
  private: std::string internalActivationTopic;

  /// \brief The topic used to receive notifications from the external
  /// activation zone.
  private: std::string externalActivationTopic;

  /// \brief Minimum amount of seconds to stay docked to be considered a fully
  /// successfull dock.
  private: std::chrono::duration<double> minDockTime;

  /// \brief Current simulation time.
  private: std::chrono::duration<double> simTime;

  /// \brief Whether this is the correct bay to dock in.
  private: bool correctDock;

  /// \brief Timer used to calculate the elapsed time docked in the bay.
  private: std::chrono::duration<double> timer;

  /// \brief Whether the vehicle has successfully docked or not.
  private: bool anytimeDocked = false;

  /// \brief Whether the vehicle is at the entrance of the bay or not.
  private: bool atEntrance = false;

  /// \brief Shape and color of symbol to announce. E.g.: ["red", "cross"]
  private: msgs::StringMsg_V symbol;

  /// \brief Topic where the target symbol will be published.
  private: std::string symbolTopic = "/vrx/scan_dock/placard_symbol";

  /// \brief Vehicle name.
  private: std::string vehicleName;

  /// \brief The transport node.
  private: transport::Node node;

  /// \brief Publish the placard symbols.
  private: transport::Node::Publisher dockPlacardPub;
};

/////////////////////////////////////////////////
DockChecker::DockChecker(const std::string &_name,
  const std::string &_internalActivationTopic,
  const std::string &_externalActivationTopic,
  const std::chrono::duration<double> _minDockTime, const bool _correctDock,
  const std::string &_announceSymbol, const std::string &_symbolTopic,
  const std::string &_vehicleName)
  : name(_name),
    internalActivationTopic(_internalActivationTopic),
    externalActivationTopic(_externalActivationTopic),
    minDockTime(_minDockTime),
    correctDock(_correctDock),
    symbolTopic(_symbolTopic),
    vehicleName(_vehicleName)
{
  // Override the docks own sdf parameters
  this->dockPlacardPub = this->node.Advertise<msgs::StringMsg_V>(
    this->symbolTopic);

  // Add the shape.
  this->symbol.add_data(_announceSymbol.substr(_announceSymbol.find("_") + 1));
  // Add the color.
  this->symbol.add_data(_announceSymbol.substr(0, _announceSymbol.find("_")));

  this->timer =
    std::chrono::duration<double>(std::numeric_limits<double>::max());

  // Subscriber to receive ContainPlugin updates.
  if (!this->node.Subscribe(this->internalActivationTopic,
    &DockChecker::OnInternalActivationEvent, this))
  {
    gzerr << "Error subscribing to topic [" << this->internalActivationTopic
           << "]" << std::endl;
    return;
  }
  if (!this->node.Subscribe(this->externalActivationTopic,
    &DockChecker::OnExternalActivationEvent, this))
  {
    gzerr << "Error subscribing to topic [" << this->externalActivationTopic
           << "]" << std::endl;
    return;
  }
}

/////////////////////////////////////////////////
bool DockChecker::AnytimeDocked() const
{
  return this->anytimeDocked;
}

/////////////////////////////////////////////////
bool DockChecker::AtEntrance() const
{
  return this->atEntrance;
}

/////////////////////////////////////////////////
bool DockChecker::Correct() const
{
  return this->correctDock;
}

/////////////////////////////////////////////////
void DockChecker::AnnounceSymbol()
{
  this->dockPlacardPub.Publish(this->symbol);
}

/////////////////////////////////////////////////
void DockChecker::Update(const sim::UpdateInfo &_info)
{
  this->simTime = _info.simTime;

  if (this->anytimeDocked)
    return;

  auto elapsed = _info.simTime - this->timer;
  this->anytimeDocked = elapsed >= this->minDockTime;

  if (this->anytimeDocked)
  {
    gzmsg  << "Successfully stayed in dock for " << this->minDockTime.count()
           << " seconds, transitioning to <docked> state" << std::endl;
  }
}

/////////////////////////////////////////////////
void DockChecker::OnInternalActivationEvent(const msgs::Pose &_msg)
{
  // Sanity check: We're only interested in a performer that matches our
  // vehicle name.
  if (_msg.name() != this->vehicleName)
    return;

  // Get the state from the header.
  std::string state;
  for (const auto &data : _msg.header().data())
  {
    if (data.key() == "state" && !data.value().empty())
    {
      state = data.value(0);
      break;
    }
  }

  // Currently docked.
  if (state == "1")
  {
    this->timer = this->simTime;
    gzmsg << "Entering internal dock activation zone, transitioning to "
          << "<docking> state in [" << this->name << "]." << std::endl;
  }

  // Currently undocked.
  if (state == "0")
  {
    this->timer =
      std::chrono::duration<double>(std::numeric_limits<double>::max());
    if (this->AnytimeDocked())
    {
      gzmsg << "Leaving internal dock activation zone in [" << this->name
            << "] after required time - transitioning to <exited> state."
            << std::endl;
    }
    else
    {
      gzmsg << "Leaving internal dock activation zone in [" << this->name
            << "] early - transitioning back to <undocked> state."
            << std::endl;
    }
  }

  gzdbg << "[" << this->name << "] OnInternalActivationEvent(): "
        << state << std::endl;
  std::cout << std::flush;
}

/////////////////////////////////////////////////
void DockChecker::OnExternalActivationEvent(const msgs::Pose &_msg)
{
  // Sanity check: We're only interested in a performer that matches our
  // vehicle name.
  if (_msg.name() != this->vehicleName)
    return;

  // Get the state from the header.
  std::string state;
  for (const auto &data : _msg.header().data())
  {
    if (data.key() == "state" && !data.value().empty())
    {
      state = data.value(0);
      break;
    }
  }

  this->atEntrance = state == "1";

  if (this->atEntrance)
  {
    gzmsg << "Entering external dock activation zone in [" << this->name
          << "]" << std::endl;
  }
  else
  {
    gzmsg << "Leaving external dock activation zone in [" << this->name
          << "]" << std::endl;
  }

  gzdbg << "[" << this->name << "] OnExternalActivationEvent(): "
        << state << std::endl;
  std::cout << std::flush;
}

/// \brief Private ScanDockScoringPlugin data class.
class ScanDockScoringPlugin::Implementation
{
  /// \brief World entity.
  public: std::string worldName;

  /// \brief Parse all SDF parameters.
  /// \param[in] _sdf SDF elements.
  /// \param[in] _vehicleName Name of the vehicle.
  /// \return True when all params were successfully parsed or false otherwise.
  public: bool ParseSDF(sdf::ElementPtr _sdf,
                        const std::string _vehicleName);

  /// \brief In charge of receiving the team submission and compare it with
  /// the color sequence from the light buoy.
  public: std::unique_ptr<ColorSequenceChecker> colorChecker;

  /// \brief Monitor all the available bays to decide when the vehicle docks.
  public: std::vector<std::unique_ptr<DockChecker>> dockCheckers;

  /// \brief To check colors or not.
  public: bool enableColorChecker = true;

  /// \brief Whether we have processed the color sequence submission or not.
  public: bool colorSubmissionProcessed = false;

  /// \brief Points granted when the color sequence is correct.
  public: double colorBonusPoints = 10.0;

  /// \brief Points granted when the vehicle successfully
  /// dock-and-undock in any bay
  public: double dockBonusPoints = 10.0;

  /// \brief Points granted when the vehicle successfully
  /// dock-and-undock in the specified bay.
  public: double correctDockBonusPoints = 10.0;

  /// \brief Expected color sequence.
  public: std::vector<std::string> expectedSequence;

  /// \brief A transport node.
  public: transport::Node node;

  /// \brief A mutex.
  public: std::mutex mutex;

  /// \brief The shooting bonus.
  public: double shootingBonus = 0.0;
};

//////////////////////////////////////////////////
bool ScanDockScoringPlugin::Implementation::ParseSDF(sdf::ElementPtr _sdf,
  const std::string _vehicleName)
{
  // Enable color checker - default is true
  this->enableColorChecker = true;
  if (_sdf->HasElement("enable_color_checker"))
    this->enableColorChecker = _sdf->Get<bool>("enable_color_checker");

  // Optional: Color sequence topic.
  std::string colorSequenceTopic = "/vrx/scan_dock/color_sequence";
  if (_sdf->HasElement("color_sequence_topic"))
    colorSequenceTopic = _sdf->Get<std::string>("color_sequence_topic");

  // Required: The expected color pattern.
  for (auto colorIndex : {"color_1", "color_2", "color_3"})
  {
    if (!_sdf->HasElement(colorIndex))
    {
      gzerr << "ScanDockScoringPlugin: Missing <" << colorIndex << ">"
            << std::endl;
      return false;
    }

    auto color = _sdf->Get<std::string>(colorIndex);
    std::transform(color.begin(), color.end(), color.begin(), ::tolower);

    // Sanity check: color should be red, green, blue or yellow.
    if (color != "red"  && color != "green" &&
        color != "blue" && color != "yellow")
    {
      gzerr << "ScanDockScoringPlugin:: Invalid color [" << color << "]"
            << std::endl;
      return false;
    }

    this->expectedSequence.push_back(color);
  }

  // Optional: the points granted when reported the correct color sequence.
  if (_sdf->HasElement("color_bonus_points"))
    this->colorBonusPoints = _sdf->Get<double>("color_bonus_points");

  // Instantiate the color checker.
  if (this->enableColorChecker)
  {
    this->colorChecker.reset(
      new ColorSequenceChecker(this->expectedSequence, colorSequenceTopic));
  }

  // Required: Parse the bays.
  if (!_sdf->HasElement("bays"))
  {
    gzerr << "<bays> missing" << std::endl;
    return false;
  }

  auto baysElem = _sdf->GetElement("bays");
  if (!baysElem->HasElement("bay"))
  {
    gzerr << "<bay> missing" << std::endl;
    return false;
  }

  auto bayElem = baysElem->GetElement("bay");
  while (bayElem)
  {
    // Required: bay name.
    if (!bayElem->GetElement("name"))
    {
      gzerr << "<bays::bay::name> missing" << std::endl;
      return false;
    }
    std::string bayName = bayElem->Get<std::string>("name");

    // Required: internal_activation topic.
    if (!bayElem->GetElement("internal_activation_topic"))
    {
      gzerr << "<bays::bay::internal_activation_topic> missing" << std::endl;
      return false;
    }
    std::string internalActivationTopic =
      bayElem->Get<std::string>("internal_activation_topic");

    // Required: external_activation topic.
    if (!bayElem->GetElement("external_activation_topic"))
    {
      gzerr << "<bays::bay::external_activation_topic> missing" << std::endl;
      return false;
    }
    std::string externalActivationTopic =
      bayElem->Get<std::string>("external_activation_topic");

    // Required: gazebo symbol topic.
    if (!bayElem->GetElement("symbol_topic"))
    {
      gzerr << "<bays::bay::symbol_topic> missing" << std::endl;
      return false;
    }
    std::string symbolTopic = bayElem->Get<std::string>("symbol_topic");

    // Required: minimum time to be considered "docked".
    if (!bayElem->GetElement("min_dock_time"))
    {
      gzerr << "<bays::bay::min_dock_time> missing" << std::endl;
      return false;
    }
    double minDockTime = bayElem->Get<double>("min_dock_time");

    // Required: correct dock .
    if (!bayElem->GetElement("correct_dock"))
    {
      gzerr << "<bays::bay::correct_dock> missing" << std::endl;
      return false;
    }
    bool correctDock = bayElem->Get<bool>("correct_dock");

    std::string announceSymbol = "";
    if (!bayElem->HasElement("symbol"))
    {
      gzerr << "<bays::bay::symbol> not found" << std::endl;
    }
    announceSymbol = bayElem->GetElement("symbol")->Get<std::string>();

    // Create a new dock checker.
    std::unique_ptr<DockChecker> dockChecker(
      new DockChecker(bayName, internalActivationTopic,
        externalActivationTopic, std::chrono::duration<double>(minDockTime),
        correctDock, announceSymbol, symbolTopic, _vehicleName));

    // Add the dock checker.
    this->dockCheckers.push_back(std::move(dockChecker));

    // Process the next checker.
    bayElem = bayElem->GetNextElement();
  }

  // Optional: the points granted when the vehicle docks in any bay.
  if (_sdf->HasElement("dock_bonus_points"))
  {
    this->dockBonusPoints =
      _sdf->GetElement("dock_bonus_points")->Get<double>();
  }

  // Optional: the points granted when the vehicle docks in the expected bay.
  if (_sdf->HasElement("correct_dock_bonus_points"))
  {
    this->correctDockBonusPoints =
      _sdf->GetElement("correct_dock_bonus_points")->Get<double>();
  }

  // Optional: the shooting targets.
  if (_sdf->HasElement("targets"))
  {
    // We require at least one <target> element.
    auto targetsElem = _sdf->GetElement("targets");
    if (!targetsElem->HasElement("target"))
    {
      gzerr << "<targets><target> not found" << std::endl;
      return false;
    }

    auto targetElem = targetsElem->GetElement("target");
    while (targetElem)
    {
      if (!targetElem->HasElement("topic"))
      {
        gzerr << "<targets><target><topic> not found" << std::endl;
        return false;
      }
      std::string topic = targetElem->Get<std::string>("topic");

      if (!targetElem->HasElement("bonus_points"))
      {
        gzerr << "<targets><target><bonus_points> not found" << std::endl;
        return false;
      }
      double bonusPoints = targetElem->Get<double>("bonus_points");

      std::function<void(const msgs::Pose&)> subCb =
        [this, bonusPoints](const msgs::Pose &_msg)
      {
        // Get the state from the header.
        std::string state;
        for (const auto &data : _msg.header().data())
        {
          if (data.key() == "state" && !data.value().empty())
          {
            state = data.value(0);
            break;
          }
        }
        // The projectile hit the target!
        if (state == "1")
        {
          std::lock_guard<std::mutex> lock(this->mutex);
          this->shootingBonus = bonusPoints;

          gzmsg << "Target hit!" << std::endl;
          std::cout << std::flush;
        }
      };

      this->node.Subscribe(topic, subCb);

      // Process the next target.
      targetElem = targetElem->GetNextElement();
    }
  }

  return true;
}

//////////////////////////////////////////////////
ScanDockScoringPlugin::ScanDockScoringPlugin()
  : ScoringPlugin(),
    dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  ScoringPlugin::Configure(_entity, _sdf, _ecm, _eventMgr);

  sim::Entity worldEntity = _ecm.EntityByComponents(sim::components::World());
  this->dataPtr->worldName =
    _ecm.Component<sim::components::Name>(worldEntity)->Data();

  auto sdf = _sdf->Clone();
  if (!this->dataPtr->ParseSDF(sdf, this->VehicleName()))
  {
    gzerr << "Error parsing SDF, plugin disabled." << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::PreUpdate(const sim::UpdateInfo &_info,
  sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  ScoringPlugin::PreUpdate(_info, _ecm);

  if (this->TaskState() == "finished")
    return;

  if (this->dataPtr->enableColorChecker)
  {
    // Verify the color checker.
    if (!this->dataPtr->colorSubmissionProcessed &&
        this->dataPtr->colorChecker->SubmissionReceived())
    {
      // We need to decide if we grant extra points.
      if (this->dataPtr->colorChecker->Correct())
      {
        gzmsg << "Adding <" << this->dataPtr->colorBonusPoints
              << "> points for correct reporting of color sequence"
              << std::endl;
        this->SetScore(this->Score() + this->dataPtr->colorBonusPoints);
      }

      // We only allow one color sequence submission.
      this->dataPtr->colorChecker->Disable();
      this->dataPtr->colorSubmissionProcessed = true;
    }
  }

  // Verify the dock checkers.
  for (auto &dockChecker : this->dataPtr->dockCheckers)
  {
    // We always need to update the checkers.
    dockChecker->Update(_info);

    // Nothing to do if nobody ever docked or we're still inside the bay.
    if (!dockChecker->AnytimeDocked() || !dockChecker->AtEntrance())
      continue;

    // Points granted for docking!
    this->SetScore(this->Score() + this->dataPtr->dockBonusPoints);
    if (this->TaskState() == "running")
    {
      gzmsg  << "Successfully docked in [" << dockChecker->name << "]"
             << ". Awarding " << this->dataPtr->dockBonusPoints << " points."
             << std::endl;
    }

    // Is this the right bay?
    if (dockChecker->Correct())
    {
      this->SetScore(this->Score() + this->dataPtr->correctDockBonusPoints);
      if (this->TaskState() == "running")
      {
        gzmsg << "Docked in correct dock [" << dockChecker->name << "]"
              << ". Awarding " << this->dataPtr->correctDockBonusPoints
              << " more points." << std::endl;
      }
    }
    else
    {
      if (this->TaskState() == "running")
      {
        gzmsg  << "Docked in incorrect dock [" << dockChecker->name << "]"
               << ". No additional points." << std::endl;
      }
    }

    // Time to finish the task as the vehicle docked.
    // Note that we only allow to dock one time. This is to prevent teams
    // docking in all possible bays.
    this->Finish();
    break;
  }

  // Check the shooting targets.
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    if (this->dataPtr->shootingBonus > 0)
    {
      this->SetScore(this->Score() + this->dataPtr->shootingBonus);
      this->dataPtr->shootingBonus = 0;
    }
  }
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::OnReady()
{
  // Announce the symbol if needed.
  for (auto &dockChecker : this->dataPtr->dockCheckers)
    dockChecker->AnnounceSymbol();
  ScoringPlugin::OnReady();
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::OnRunning()
{
  std::function<void(const msgs::Boolean &_res, const bool)> f =
        [](const msgs::Boolean &_res, const bool)
      {
      };

  std::string performerTopic =
    "/world/" + this->dataPtr->worldName + "/level/set_performer";
  msgs::StringMsg performer;
  performer.set_data(this->VehicleName());

  // Register the vehicle as a performer.
  this->dataPtr->node.Request<msgs::StringMsg, msgs::Boolean>(
    performerTopic, performer, f);

  if (this->dataPtr->enableColorChecker)
    this->dataPtr->colorChecker->Enable();

  // Announce the symbol if needed.
  for (auto &dockChecker : this->dataPtr->dockCheckers)
    dockChecker->AnnounceSymbol();

  ScoringPlugin::OnRunning();
}

GZ_ADD_PLUGIN(ScanDockScoringPlugin,
              sim::System,
              ScanDockScoringPlugin::ISystemConfigure,
              ScanDockScoringPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::ScanDockScoringPlugin,
                    "vrx::ScanDockScoringPlugin")
