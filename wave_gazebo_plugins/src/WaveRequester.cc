/*
 * Copyright (C) 2019  Brian Bingham
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

/* Example of using a gazebo request message to get the wave parameters  
 * fromt the WavefieldModelPlugin.
 * Based on Gazebo's publisher example - https://bitbucket.org/osrf/gazebo/src/default/examples/stand_alone/publisher/publisher.cc
*/

#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>
#include <iostream>

// Global flags
static std::atomic<bool> g_terminatePub(false);
static std::atomic<bool> g_rx(false);

void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    g_terminatePub = true;
}

///////////////////////////////////////////////////////////////////////////////
/// \brief Callback for gztopic "~/response"
///
void OnResponse(ConstResponsePtr &_msg)
{
  std::cout << _msg->DebugString() << std::endl;
	g_rx = true;
}

void RequestResponseUtility()
{
	gazebo::transport::init();
	gazebo::transport::run();
	boost::shared_ptr<gazebo::msgs::Response> response = gazebo::transport::request("robotx_example_course","wave_param");
	std::cout << response->DebugString() << std::endl;
	gazebo::transport::fini();
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{	
	std::cout << "Attempt to get a response with utility function..."
						<< std::endl;
	RequestResponseUtility();

	// Interestingly I needed to put the utilitye function
	// (gazebo::transport::request) in a separate funtion for the
	// by-hand pub/sub example to work?
	std::cout << "Now try to do the same with publish/subscribe..." << std::endl;
	gazebo::transport::init();
	gazebo::transport::run();
	// Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

	// Subscribe
	std::string topic("~/response");
	gazebo::transport::SubscriberPtr sub =
		node->Subscribe(topic, &OnResponse);
	
	// Publish to a Gazebo topic
	gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Request>("~/request");

	// Wait for a subscriber to connect, but don't block forever...
	if ( !pub->WaitForConnection(gazebo::common::Time(1, 0)) )
	{
		std::cout << "Error - couldn't connect to subscriber!" << std::endl;
		//return 1;
	}
	
	// Create a request message
	gazebo::msgs::RequestPtr msg(gazebo::msgs::CreateRequest("wave_param", ""));
	
	// Publish it
	std::cout << "Publish request" << std::endl;
	pub->Publish(*msg);
	
	// Listen until interrupt
	while (!g_terminatePub)
	{
		if (g_rx)
		{
			std::cout << "Publish request" << std::endl;
			g_rx = false;
			//pub->Publish(*msg);
		}

	}

	std::cout << "Shutting down" << std::endl;
	sub.reset();
	gazebo::transport::fini();
	
}
