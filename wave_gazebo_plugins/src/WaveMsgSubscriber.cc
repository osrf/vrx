// Copyright (C) 2019  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

using namespace gazebo;

///////////////////////////////////////////////////////////////////////////////
/// Ignition Transport Tutorial
/// Signal handler example
/// https://ignitionrobotics.org/api/transport/6.0/messages.html
///
static std::atomic<bool> g_terminatePub(false);

void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminatePub = true;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Callback for gztopic "~/wave".
///
void OnWaveMsg(ConstParam_VPtr &_msg)
{
  std::string topic("~/wave");
  std::cout << "Received message on topic [" << topic << "]" << std::endl;
  std::cout << _msg->DebugString() << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  try 
  {
    // Copyright notice.
    std::cout
      << "ASV Wave Simulator: wave parameter subscriber.\n"
      << "Copyright (C) 2019  Rhys Mainwaring.\n"
      << "Released under the GNU General Public License.\n\n";

    // Signal handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Transport
    transport::init();
    transport::run();
    transport::NodePtr node(new transport::Node());
    node->Init();

    std::string topic("~/wave");
    transport::SubscriberPtr waveSub =
      node->Subscribe(topic, &OnWaveMsg);

    // Listen until interrupt
    while (!g_terminatePub)
    {
      // Listen
    }

    // Tear down
    waveSub.reset();
    transport::fini();
  }
  catch(const gazebo::common::Exception &_e)
  {
    std::cout << _e.GetErrorStr() << std::endl;
    transport::fini();
    return -1;
  }
  catch(const std::exception &_e)
  {
    std::cout << _e.what() << std::endl;
    transport::fini();
    return -1;
  }
  catch(...)
  {
    std::cout << "Unknown Error" << std::endl;
    transport::fini();
    return -1;
  }

  return 0;
}
