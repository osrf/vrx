#include <robotx_gazebo/light_buoy_controller.hh>

// Register plugin with gazebo
GZ_REGISTER_MODEL_PLUGIN(LightBuoyController)

std_msgs::ColorRGBA LightBuoyController::color(double r, double g, double b, double a)
{
  static std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

const std_msgs::ColorRGBA LightBuoyController::RED    = color(1.0, 0.0, 0.0, 1.0);
const std_msgs::ColorRGBA LightBuoyController::GREEN  = color(0.0, 1.0, 0.0, 1.0);
const std_msgs::ColorRGBA LightBuoyController::BLUE   = color(0.0, 0.0, 1.0, 1.0);
const std_msgs::ColorRGBA LightBuoyController::YELLOW = color(1.0, 1.0, 0.0, 1.0);
const std_msgs::ColorRGBA LightBuoyController::OFF    = color(0.0, 0.0, 0.0, 1.0);
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

  nh_ = ros::NodeHandle(ns);

  // Create publisher to set color on each panel 
  panel_pubs_[0] = nh_.advertise<std_msgs::ColorRGBA>("panel1", 1);
  panel_pubs_[1] = nh_.advertise<std_msgs::ColorRGBA>("panel2", 1);
  panel_pubs_[2] = nh_.advertise<std_msgs::ColorRGBA>("panel3", 1);

  // Generate random initial pattern
  std::string initial;
  changePattern(initial);

  change_pattern_server_ = nh_.advertiseService("new_pattern", &LightBuoyController::changePattern, this);

  timer_ = nh_.createTimer(ros::Duration(1.0), &LightBuoyController::incrementState, this);
}

void LightBuoyController::incrementState(const ros::TimerEvent& _event)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Start over if at end of pattern
  if (state_ > 3)
      state_ = 0;
  auto msg = COLORS[pattern_[state_]].first;
  // Publish current color to each panel
  for (size_t i = 0; i < 3; ++i)
    panel_pubs_[i].publish(msg);
  // Increment index for next timer callback
  ++state_;
}

bool LightBuoyController::changePattern(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  changePattern(res.message);
  res.message = "New pattern: " + res.message;
  res.success = true;
  return true;
}

void LightBuoyController::changePattern(std::string& message)
{
  pattern_t new_pattern;
  // Last phase in pattern is always off
  new_pattern[3] = 4;
  do {
    // Generate random sequence of 3 colors among RED, GREEN, BLUE, and YELLOW
    for (size_t i = 0; i < 3; ++i)
        new_pattern[i] = ignition::math::Rand::IntUniform(0, 3);
    // Ensure there is no CONSECUTIVE repeats
    while(new_pattern[0] == new_pattern[1] || new_pattern[1] == new_pattern[2])
        new_pattern[1] = ignition::math::Rand::IntUniform(0, 3);
  } while(new_pattern == pattern_);

  std::lock_guard<std::mutex> lock(mutex_);
  // Copy newly generated pattern to pattern
  pattern_ = new_pattern;
  // Start in OFF state so pattern restarts at beginning
  state_ = 3;
  // Generate string representing pattern, ex: "RGB"
  for (size_t i = 0; i < 3; ++i)
    message += COLORS[i].second[0];
  // Log the new pattern
  ROS_INFO_NAMED("light_bouy_controller", "Pattern is %s", message.c_str());
}
