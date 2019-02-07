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

#include "asv_joy_gazebo_plugins/JoySDLPlugin.hh"

#include <gazebo/common/common.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include <SDL.h>

#include <iostream>
#include <string>
#include <thread>

using namespace gazebo;

namespace asv
{

  GZ_REGISTER_WORLD_PLUGIN(JoySDLPlugin)

///////////////////////////////////////////////////////////////////////////////
// JoySDLPluginPrivate

  /// \internal
  /// \brief Private data for the JoySDLPlugin
  class JoySDLPluginPrivate
  {
    /// \brief Publish joystick data.
    public: void Run();

    /// \brief The joystick device id, based on <joy_id>.
    public: int joyId = 0;

    /// \brief The published topic based on <output_topic>.
    /// The default is '/joy'
    public: std::string outputTopic = "/joy";

    /// \brief Publication rate, based on <rate>.
    public: double rate = 50.0;

    /// \brief The unscaled deadzone, based on <dead_zone>.
    public: double deadzone = 0.1;

    /// \brief True to stop the plugin.
    public: bool stop = false;

    /// \brief SDL joystick object.
    public: SDL_Joystick* joystick;

    /// \brief The joystick message.
    public: ignition::msgs::Joy joyMsg;

    /// \brief Thread to run the joystick publisher.
    public: std::unique_ptr<std::thread> joyThread;

    /// \brief Ignition transport node for igntopic "/joy".
    public: ignition::transport::Node node;

    /// \brief Ignition publisher to igntopic "/joy".
    public: ignition::transport::Node::Publisher pub;
  };

  void JoySDLPluginPrivate::Run()
  {
    // Initialise SDL.
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0)
    {
      gzerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
      return;
    }

    // Open the joystick.
    this->joystick = SDL_JoystickOpen(this->joyId);    
    if(this->joystick == nullptr)
    {
      gzerr << "Couldn't connect to joystick[" << this->joyId << "]" << std::endl;
      return;
    }

    // Set the message size.
    int numAxes = SDL_JoystickNumAxes(this->joystick);
    int numButtons = SDL_JoystickNumButtons(this->joystick);
    this->joyMsg.mutable_axes()->Resize(numAxes, 0.0);
    this->joyMsg.mutable_buttons()->Resize(numButtons, 0.0);

    // Info
    gzmsg << "Joystick name:  " << SDL_JoystickName(this->joystick) << std::endl;
    // gzmsg << "  power:        " << SDL_JoystickCurrentPowerLevel(this->joystick) << std::endl;
    gzmsg << "  num axes:     " << SDL_JoystickNumAxes(this->joystick) << std::endl;
    gzmsg << "  num buttons:  " << SDL_JoystickNumButtons(this->joystick) << std::endl;
    gzmsg << "  num hats:     " << SDL_JoystickNumHats(this->joystick) << std::endl;
    gzmsg << "  num balls:    " << SDL_JoystickNumBalls(this->joystick) << std::endl;
    SDL_JoystickClose(joystick);

    // Poll joystick for events
    const double interval = 1.0/this->rate;
    common::Time prevTime = common::Time::GetWallTime();
    while (!this->stop)
    {
      // Set the time stamp
      common::Time time = common::Time::GetWallTime();

      if ((time - prevTime).Double() > interval)
      {
        prevTime = time;

        this->joyMsg.mutable_header()->mutable_stamp()->set_sec(time.sec);
        this->joyMsg.mutable_header()->mutable_stamp()->set_nsec(time.nsec);
        
        // Set the axes
        // From the documentation: http://www.libsdl.org/
        // the return value of SDL_JoystickGetAxis is a 16-bit 
        // signed integer representing the current position of the axis.
        // The state is a value ranging from -32768 to 32767. 
        // ignition::msgs::Joy.axes is normalised to -1, 1.
        const int a = -32768;
        const int b =  32767;
        const int d =  b * this->deadzone;
        const float div = b - a;        

        // Open the joystick to read the next event 
        this->joystick = SDL_JoystickOpen(this->joyId);
        if (this->joystick == nullptr)
        {
          continue;
        }
        for (int i=0; i<numAxes; ++i)
        {
          const int x = SDL_JoystickGetAxis(this->joystick, i);
          float y = 0.0;
          if (std::abs(x) > d)
          {
            const float div = (x < 0) ? (d - a) : (b - d);
            y = (x - d) / div;
          }
          this->joyMsg.set_axes(i, y);
        }

        // Set the buttons
        for (int i=0; i<numButtons; ++i)
        {
          const int buttonValue = SDL_JoystickGetButton(this->joystick, i);
          this->joyMsg.set_buttons(i, buttonValue);
        }
        SDL_JoystickClose(joystick);

        this->pub.Publish(this->joyMsg);
        // @DEBUG_INFO
        // std::cout << this->joyMsg.DebugString() << std::endl;

      }
    }

    // Quit SDL.
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
    SDL_Quit();
    return;
  }

///////////////////////////////////////////////////////////////////////////////
// JoySDLPlugin

  JoySDLPlugin::~JoySDLPlugin()
  {
    // Stop the thread.
    this->data->stop = true;
    if (this->data->joyThread != nullptr && this->data->joyThread->joinable())
      this->data->joyThread->join();
  }

  JoySDLPlugin::JoySDLPlugin() : 
    WorldPlugin(), 
    data(new JoySDLPluginPrivate())
  {
  }

  void JoySDLPlugin::Load(physics::WorldPtr /*_world */, sdf::ElementPtr _sdf)
  {
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    // Parameters
    this->LoadParam(_sdf, "joy_id", this->data->joyId, this->data->joyId);
    this->LoadParam(_sdf, "output_topic", this->data->outputTopic, this->data->outputTopic);
    this->LoadParam(_sdf, "rate", this->data->rate, this->data->rate);
    this->LoadParam(_sdf, "dead_zone", this->data->deadzone, this->data->deadzone);

    // Constraints
    this->data->deadzone = ignition::math::clamp(
      this->data->deadzone, 0.0, 0.9);

    // Publishers
    this->data->pub 
      = this->data->node.Advertise<ignition::msgs::Joy>(this->data->outputTopic);

    // Run thread
    this->data->joyThread.reset(new std::thread(
      std::bind(&JoySDLPluginPrivate::Run, this->data)));
  }

} // namespace asv
