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

#include "asv_joy_gazebo_plugins/SDLJoyPlugin.hh"

#include <gazebo/common/common.hh>

#include <ignition/transport/Node.hh>

#include <iostream>
#include <string>
#include <thread>

using namespace gazebo;

namespace asv
{

  GZ_REGISTER_WORLD_PLUGIN(SDLJoyPlugin)

///////////////////////////////////////////////////////////////////////////////
// SDLJoyPluginPrivate

  /// \internal
  /// \brief Private data for the SDLJoyPlugin
  class SDLJoyPluginPrivate
  {
    // Publish joystick data.
    public: void Run();

    /// \brief joystick message.
    public: ignition::msgs::Joy joyMsg;

    /// \brief True to stop the plugin.
    public: bool stop = false;

    /// \brief Thread to run the joystick publisher.
    public: std::unique_ptr<std::thread> runThread;

    /// \brief Publication rate.
    public: double publicationRate = 50.0;

    /// \brief Data accumulation rate.
    public: double accumulationRate = 1000.0;

    /// \brief Ignition transport node for igntopic "/joy".
    public: ignition::transport::Node ignNode;

    /// \brief Ignition publisher to igntopic "/joy".
    public: ignition::transport::Node::Publisher ignPub;
  };

  void SDLJoyPluginPrivate::Run()
  {
    while (!this->stop)
    {
      // Set the time stamp
      common::Time time = common::Time::GetWallTime();
      this->joyMsg.mutable_header()->mutable_stamp()->set_sec(time.sec);
      this->joyMsg.mutable_header()->mutable_stamp()->set_nsec(time.nsec);
      


      this->ignPub.Publish(this->joyMsg);

    }
  }

///////////////////////////////////////////////////////////////////////////////
// SDLJoyPlugin

  SDLJoyPlugin::~SDLJoyPlugin()
  {
    // Stop the thread.
    this->data->stop = true;
    if (this->data->runThread != nullptr && this->data->runThread->joinable())
      this->data->runThread->join();

    // Close the joystick.
  }

  SDLJoyPlugin::SDLJoyPlugin() : 
    WorldPlugin(), 
    data(new SDLJoyPluginPrivate())
  {
  }

  void SDLJoyPlugin::Load(physics::WorldPtr /*_world */, sdf::ElementPtr _sdf)
  {
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    // Parameters

    // Topic
    std::string topic = "/joy";

    // Publishers
    this->data->ignPub 
      = this->data->ignNode.Advertise<ignition::msgs::Joy>(topic);

    // Thread
    this->data->runThread.reset(new std::thread(
      std::bind(&SDLJoyPluginPrivate::Run, this->data)));

  }

} // namespace gazebo
