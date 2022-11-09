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

#include <gz/msgs/stringmsg_v.pb.h>
#include <memory>
#include <string>
#include <vector>
#include <gz/common/Profiler.hh>
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
  if (_sequence.data_size() < 3u)
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

/// \brief Private ScanDockScoringPlugin data class.
class ScanDockScoringPlugin::Implementation
{
  /// \brief Parse all SDF parameters.
  /// \param[in] _sdf SDF elements.
  /// \return True when all params were successfully parsed or false otherwise.
  public: bool ParseSDF(sdf::ElementPtr _sdf);

  /// \brief In charge of receiving the team submission and compare it with
  /// the color sequence from the light buoy.
  public: std::unique_ptr<ColorSequenceChecker> colorChecker;

  /// \brief To check colors or not.
  public: bool enableColorChecker = true;

  /// \brief Whether we have processed the color sequence submission or not.
  public: bool colorSubmissionProcessed = false;

  /// \brief Points granted when the color sequence is correct.
  public: double colorBonusPoints = 10.0;

  /// \brief Expected color sequence.
  public: std::vector<std::string> expectedSequence;

  /// \brief A mutex.
  private: std::mutex mutex;
};

//////////////////////////////////////////////////
bool ScanDockScoringPlugin::Implementation::ParseSDF(sdf::ElementPtr _sdf)
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

  auto sdf = _sdf->Clone();
  if (!this->dataPtr->ParseSDF(sdf))
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
}

//////////////////////////////////////////////////
void ScanDockScoringPlugin::OnRunning()
{
  if (this->dataPtr->enableColorChecker)
    this->dataPtr->colorChecker->Enable();
}

GZ_ADD_PLUGIN(ScanDockScoringPlugin,
              sim::System,
              ScanDockScoringPlugin::ISystemConfigure,
              ScanDockScoringPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::ScanDockScoringPlugin,
                    "vrx::ScanDockScoringPlugin")
