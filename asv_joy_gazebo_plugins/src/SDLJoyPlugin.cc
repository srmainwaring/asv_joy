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

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include <SDL.h>

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
    /// \brief Publish joystick data.
    public: void Run();

    /// \brief The joystick device id, based on <joy_id>.
    public: int joyId = 0;

    /// \brief The published topic based on <topic>.
    /// The default is '/joy'
    public: std::string topic;

    /// \brief Publication rate, based on <rate>.
    public: double rate = 50.0;

    /// \brief Data accumulation rate, based on <accumulation_rate>.
    public: double accumulationRate = 1000.0;

    /// \brief The unscaled deadzone, based on <dead_zone>.
    public: double deadzone = 0.0;

    /// \brief True when the buttons should act like toggle buttons.
    /// Based on <sticky_buttons>.
    public: bool stickyButtons = false;

    /// \brief True to stop the plugin.
    public: bool stop = false;

    /// \brief SDL joystick object.
    public: SDL_Joystick* joystick;

    /// \brief The non-sticky button joystick message.
    public: ignition::msgs::Joy joyMsg;

    /// \brief Previous joystick message, used to help compute
    /// the sticky  buttons.
    public: ignition::msgs::Joy lastJoyMsg;

    /// \brief Sticky button joystick message.
    public: ignition::msgs::Joy stickyButtonsJoyMsg;

    /// \brief Thread to run the joystick publisher.
    public: std::unique_ptr<std::thread> joyThread;

    /// \brief Ignition transport node for igntopic "/joy".
    public: ignition::transport::Node ignNode;

    /// \brief Ignition publisher to igntopic "/joy".
    public: ignition::transport::Node::Publisher ignPub;
  };

  void SDLJoyPluginPrivate::Run()
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
        const float div = b - a;        

        // Open the joystick to read the next event 
        this->joystick = SDL_JoystickOpen(this->joyId);
        if (this->joystick == nullptr)
        {
          continue;
        }
        for (int i=0; i<numAxes; ++i)
        {
          float axesValue = 2 * (SDL_JoystickGetAxis(this->joystick, i) - a) / div - 1.0;
          this->joyMsg.set_axes(i, axesValue);
        }

        // Set the buttons
        for (int i=0; i<numButtons; ++i)
        {
          int buttonValue = SDL_JoystickGetButton(this->joystick, i);
          this->joyMsg.set_buttons(i, buttonValue);
        }
        SDL_JoystickClose(joystick);

        this->ignPub.Publish(this->joyMsg);
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
// SDLJoyPlugin

  SDLJoyPlugin::~SDLJoyPlugin()
  {
    // Stop the thread.
    this->data->stop = true;
    if (this->data->joyThread != nullptr && this->data->joyThread->joinable())
      this->data->joyThread->join();
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
    this->data->joyId = _sdf->Get<int>(
      "joy_id",
      this->data->joyId).first;

    this->data->topic = _sdf->Get<std::string>(
      "topic",
      "/joy").first;

    this->data->rate = _sdf->Get<double>(
      "rate",
      this->data->rate).first;

    this->data->accumulationRate = _sdf->Get<double>(
      "accumulation_rate",
      this->data->accumulationRate).first;

    this->data->deadzone = ignition::math::clamp(_sdf->Get<double>(
      "dead_zone",
      this->data->deadzone).first,
      0.0, 0.9);

    this->data->stickyButtons = _sdf->Get<bool>(
      "sticky_buttons",
      this->data->stickyButtons).first;
    
    // Publishers
    this->data->ignPub 
      = this->data->ignNode.Advertise<ignition::msgs::Joy>(this->data->topic);

    // Run thread
    this->data->joyThread.reset(new std::thread(
      std::bind(&SDLJoyPluginPrivate::Run, this->data)));
  }

} // namespace gazebo
