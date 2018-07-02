#include <robotx_gazebo/light_buoy_controller.hh>

// Register plugin with gazebo
GZ_REGISTER_MODEL_PLUGIN(LightBuoyController)

std_msgs::ColorRGBA LightBuoyController::CreateColor(double r, double g, double b, double a)
{
  static std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

const std_msgs::ColorRGBA LightBuoyController::RED    = CreateColor(1.0, 0.0, 0.0, 1.0);
const std_msgs::ColorRGBA LightBuoyController::GREEN  = CreateColor(0.0, 1.0, 0.0, 1.0);
const std_msgs::ColorRGBA LightBuoyController::BLUE   = CreateColor(0.0, 0.0, 1.0, 1.0);
const std_msgs::ColorRGBA LightBuoyController::YELLOW = CreateColor(1.0, 1.0, 0.0, 1.0);
const std_msgs::ColorRGBA LightBuoyController::OFF    = CreateColor(0.0, 0.0, 0.0, 1.0);
const LightBuoyController::colors_t LightBuoyController::COLORS[5] = {LightBuoyController::colors_t(RED,    "RED"),
                                                                      LightBuoyController::colors_t(GREEN,  "GREEN"),
                                                                      LightBuoyController::colors_t(BLUE,   "BLUE"),
                                                                      LightBuoyController::colors_t(YELLOW, "YELLOW"),
                                                                      LightBuoyController::colors_t(OFF,    "OFF")};

void LightBuoyController::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");

  // Quit if ros plugin was not loaded
  if (!ros::isInitialized())
  {
    ROS_ERROR("ROS was not initialized.");
    return;
  }

  std::string ns = "";
  if (_sdf->HasElement("robotNamespace"))
    ns = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else 
    ROS_INFO_NAMED("light_bouy_controller", "missing <robotNamespace>, defaulting to %s", ns.c_str());

  this->nh = ros::NodeHandle(ns);

  // Create publisher to set color on each panel 
  this->panelPubs[0] = this->nh.advertise<std_msgs::ColorRGBA>("panel1", 1);
  this->panelPubs[1] = this->nh.advertise<std_msgs::ColorRGBA>("panel2", 1);
  this->panelPubs[2] = this->nh.advertise<std_msgs::ColorRGBA>("panel3", 1);

  // Generate random initial pattern
  std::string initial;
  ChangePattern(initial);

  this->changePatternServer = this->nh.advertiseService("new_pattern", &LightBuoyController::ChangePattern, this);

  this->timer = this->nh.createTimer(ros::Duration(1.0), &LightBuoyController::IncrementState, this);
}

void LightBuoyController::IncrementState(const ros::TimerEvent& _event)
{
  (void) _event;
  std::lock_guard<std::mutex> lock(this->mutex);
  // Start over if at end of pattern
  if (this->state > 3)
      this->state = 0;
  auto msg = COLORS[this->pattern[this->state]].first;
  // Publish current color to each panel
  for (size_t i = 0; i < 3; ++i)
    this->panelPubs[i].publish(msg);
  // Increment index for next timer callback
  ++this->state;
}

bool LightBuoyController::ChangePattern(std_srvs::Trigger::Request& _req, std_srvs::Trigger::Response& _res)
{
  ChangePattern(_res.message);
  _res.message = "New pattern: " + _res.message;
  _res.success = true;
  return true;
}

void LightBuoyController::ChangePattern(std::string& _message)
{
  pattern_t new_pattern;
  // Last phase in pattern is always off
  new_pattern[3] = 4;
  // Loop until random pattern is different from current one
  do {
    // Generate random sequence of 3 colors among RED, GREEN, BLUE, and YELLOW
    for (size_t i = 0; i < 3; ++i)
        new_pattern[i] = ignition::math::Rand::IntUniform(0, 3);
    // Ensure there is no CONSECUTIVE repeats
    while(new_pattern[0] == new_pattern[1] || new_pattern[1] == new_pattern[2])
        new_pattern[1] = ignition::math::Rand::IntUniform(0, 3);
  } while(new_pattern == this->pattern);

  std::lock_guard<std::mutex> lock(this->mutex);
  // Copy newly generated pattern to pattern
  this->pattern = new_pattern;
  // Start in OFF state so pattern restarts at beginning
  this->state = 3;
  // Generate string representing pattern, ex: "RGB"
  for (size_t i = 0; i < 3; ++i)
    _message += COLORS[new_pattern[i]].second[0];
  // Log the new pattern
  ROS_INFO_NAMED("light_bouy_controller", "Pattern is %s", _message.c_str());
}
