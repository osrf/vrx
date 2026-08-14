// Native Gazebo Transport monitor. Build instructions are in README.
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>

int main(int argc, char **argv) {
  const std::string left_topic =
    argc > 1 ? argv[1] : "/wamv/thrusters/left/thrust/ang_vel";
  const std::string right_topic =
    argc > 2 ? argv[2] : "/wamv/thrusters/right/thrust/ang_vel";
  gz::transport::Node node;
  std::mutex output_mutex;
  const auto subscribe = [&node, &output_mutex](const std::string &topic, const char *side) {
    return node.Subscribe(topic, [&output_mutex, side](const gz::msgs::Double &msg) {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        const auto seconds = std::chrono::duration<double>(now).count();
        std::lock_guard<std::mutex> lock(output_mutex);
        std::cout << std::fixed << std::setprecision(9) << seconds << ',' << side << ',' << msg.data() << '\n';
      });
  };
  if (!subscribe(left_topic, "left") || !subscribe(right_topic, "right")) {
    std::cerr << "Unable to subscribe to one or both setpoint topics\n";
    return 1;
  }
  std::cerr << "Monitoring " << left_topic << " and " << right_topic
            << " (steady_seconds,side,omega_cmd_rad_s)\n";
  while (true) std::this_thread::sleep_for(std::chrono::seconds(1));
}
