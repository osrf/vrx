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

#include <boost/lexical_cast/try_lexical_convert.hpp>
#include <boost/program_options.hpp>

using namespace gazebo;
namespace po = boost::program_options;

///////////////////////////////////////////////////////////////////////////////
/// Stack Overflow
/// Accepting negative doubles with boost::program_options
/// Answer: Aleksey Vitebskiy 2016
/// https://stackoverflow.com/questions/4107087/accepting-negative-doubles-with-boostprogram-options
///
std::vector<po::option> ignore_numbers(std::vector<std::string>& args)
{
  std::vector<po::option> result;
  int pos = 0;
  while(!args.empty())
  {
    const auto& arg = args[0];
    double num;
    if (boost::conversion::try_lexical_convert(arg, num))
    {
      result.push_back(po::option());
      po::option& opt = result.back();

      opt.position_key = pos++;
      opt.value.push_back(arg);
      opt.original_tokens.push_back(arg);

      args.erase(args.begin());
    }
    else
    {
      break;
    }
  }

  return result;
}

///////////////////////////////////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  try 
  {
    // Copyright notice.
    std::cout
      << "ASV Wave Simulator: wave parameter publisher.\n"
      << "Copyright (C) 2019  Rhys Mainwaring.\n"
      << "Released under the GNU General Public License.\n\n";

    // Program options
    po::options_description options("Publish wave parameters to gztopic \"~/wave\"");

    options.add_options()
      ("help,h", 
        "Dispay this help screen.")
      ("number,n", po::value<int>(),
        "Set the number of component waves [1 <= n <= 3].")
      ("amplitude,a", po::value<double>(),
        "Set the mean wave amplitude.")
      ("period,p", po::value<double>(),
        "Set the mean wave period.")
      ("phase", po::value<double>(),
        "Set the mean wave phase.")
      ("direction,d", po::value<std::vector<int>>()->multitoken(),
        "Set the mean wave direction. Must be two numbers.")
      ("scale,s", po::value<double>(),
        "The scale determines the range of amplitudes and wavelengths about the mean.")
      ("angle", po::value<double>(),
        "The angle determines the range of directions about the mean.")
      ("steepness,q", po::value<double>(),
        "The steepness determines the sharpness of the wave peaks.");

    po::variables_map vm;
    // po::store(po::parse_command_line(_argc, _argv, options), vm);
    po::store(po::command_line_parser(_argc, _argv)
      .extra_style_parser(&ignore_numbers)
      .options(options)
      .run(), vm);

    po::notify(vm);

    if (vm.count("help") || vm.empty())
    { 
      std::cout << options << std::endl;
      return 0;
    }

    // Transport
    transport::init();
    transport::run();
    transport::NodePtr node(new transport::Node());
    node->Init();

    std::string topic("~/wave");
    transport::PublisherPtr wavePub =
      node->Advertise<gazebo::msgs::Param_V>(topic);

    // Message
    gazebo::msgs::Param_V waveMsg;
    if (vm.count("number"))
    {
      auto nextParam = waveMsg.add_param();
      nextParam->set_name("number");
      nextParam->mutable_value()->set_type(msgs::Any::INT32);
      nextParam->mutable_value()->set_int_value(vm["number"].as<int>());
    }
    if (vm.count("scale"))
    {
      auto nextParam = waveMsg.add_param();
      nextParam->set_name("scale");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["scale"].as<double>());
    }
    if (vm.count("angle"))
    {
      auto nextParam = waveMsg.add_param();
      nextParam->set_name("angle");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["angle"].as<double>());
    }
    if (vm.count("steepness"))
    {
      auto nextParam = waveMsg.add_param();
      nextParam->set_name("steepness");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["steepness"].as<double>());
    }
    if (vm.count("amplitude"))
    {
      auto nextParam = waveMsg.add_param();
      nextParam->set_name("amplitude");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["amplitude"].as<double>());
    }
    if (vm.count("period"))
    {
      auto nextParam = waveMsg.add_param();
      nextParam->set_name("period");
      nextParam->mutable_value()->set_type(msgs::Any::DOUBLE);
      nextParam->mutable_value()->set_double_value(vm["period"].as<double>());
    }
    if (vm.count("direction"))
    {
      std::vector<int> dir(vm["direction"].as<std::vector<int>>());
      if (dir.size() != 2)
      {
        std::cout << "The value for option '--direction' must be two numbers." << std::endl;
        std::cout << "For example: '--direction 1 1'" << std::endl;
        return -1;
      }

      auto nextParam = waveMsg.add_param();
      nextParam->set_name("direction");
      nextParam->mutable_value()->set_type(msgs::Any::VECTOR3D);
      nextParam->mutable_value()->mutable_vector3d_value()->set_x(dir[0]);
      nextParam->mutable_value()->mutable_vector3d_value()->set_y(dir[1]);
      nextParam->mutable_value()->mutable_vector3d_value()->set_z(0);
    }
    
    // Don't block forever...
    wavePub->WaitForConnection(common::Time(1, 0));

    // Publish message (block while message is written)
    wavePub->Publish(waveMsg, true);

    std::cout << "Publishing on topic [" << wavePub->GetTopic() << "]" << std::endl;
    std::cout << waveMsg.DebugString() << std::endl;

    // Tear down
    wavePub.reset();
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
