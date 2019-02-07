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

#include "asv_joy_gazebo_plugins/TeleopTwistJoyPlugin.hh"

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include <SDL.h>

#include <iostream>
#include <string>
#include <thread>

using namespace gazebo;

namespace asv
{

  GZ_REGISTER_WORLD_PLUGIN(TeleopTwistJoyPlugin)

///////////////////////////////////////////////////////////////////////////////
// TeleopTwistJoyPluginPrivate

  /// \internal
  /// \brief Private data for the TeleopTwistJoyPlugin
  class TeleopTwistJoyPluginPrivate
  {
    /// \brief Topic to subscribe, based on <input_topic>. Default value: '/joy'.
    public: std::string inputTopic = "/joy";

    /// \brief Topic to publish, based on <output_topic>. Default value: '/cmd_vel'.
    public: std::string outputTopic = "/cmd_vel"; 

    /// \brief Joystick button to enable movement.
    public: unsigned int enableButton = 0;

    /// \brief Joystick axis to control linear motion.
    public: unsigned int axisLinear = 0;

    /// \brief Scale to apply to the joystick linear axis (m/s).
    public: double scaleLinear = 0;

    /// \brief Joystick axis to control angular motion.
    public: unsigned int axisAngular = 0;

    /// \brief Scale to apply to the joystick angular axis (rad/s).
    public: double scaleAngular = 0;

    /// \brief True if a disable mesage has been sent.
    public: bool sentDisableMsg = false;

    /// \brief Pointer to the world.
    public: physics::WorldPtr world;

    /// \brief Ignition transport publisher.
    public: ignition::transport::Node::Publisher pub;

    /// \brief Ignition transport node.
    public: ignition::transport::Node node;
  };

///////////////////////////////////////////////////////////////////////////////
// TeleopTwistJoyPlugin

  TeleopTwistJoyPlugin::~TeleopTwistJoyPlugin()
  {
  }

  TeleopTwistJoyPlugin::TeleopTwistJoyPlugin() : 
    WorldPlugin(), 
    data(new TeleopTwistJoyPluginPrivate())
  {
  }

  void TeleopTwistJoyPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    GZ_ASSERT(_world != nullptr, "Invalid parameter _world");
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    // World
    this->data->world = _world;

    // Parameters
    this->LoadParam(_sdf, "input_topic", this->data->inputTopic, this->data->inputTopic);
    this->LoadParam(_sdf, "output_topic", this->data->outputTopic, this->data->outputTopic);
    this->LoadParam(_sdf, "enable_button", this->data->enableButton, this->data->enableButton);
    this->LoadParam(_sdf, "axis_linear", this->data->axisLinear, this->data->axisLinear);
    this->LoadParam(_sdf, "scale_linear", this->data->scaleLinear, this->data->scaleLinear);
    this->LoadParam(_sdf, "axis_angular", this->data->axisAngular, this->data->axisAngular);
    this->LoadParam(_sdf, "scale_angular", this->data->scaleAngular, this->data->scaleAngular);

    // Publishers
    this->data->pub = this->data->node.Advertise<ignition::msgs::CmdVel2D>(
      this->data->outputTopic);
    if (!this->data->pub)
    {
      gzerr << "Error advertising topic [" << this->data->outputTopic << "]" << std::endl;
    }
    
    // Subscribers
    bool success = this->data->node.Subscribe(
      this->data->inputTopic, &TeleopTwistJoyPlugin::OnJoyMsg, this);
  }

  void TeleopTwistJoyPlugin::Init()
  {
  }

  void TeleopTwistJoyPlugin::Reset()
  {
  }

  void TeleopTwistJoyPlugin::OnJoyMsg(const ignition::msgs::Joy& _msg)
  {
    // Create a commond message and populate the header.
    ignition::msgs::CmdVel2D cmdMsg;
    common::Time time = common::Time::GetWallTime();
    cmdMsg.mutable_header()->mutable_stamp()->set_sec(time.sec);
    cmdMsg.mutable_header()->mutable_stamp()->set_nsec(time.nsec);

    // State of the enable button
    if (this->data->enableButton < _msg.buttons_size()
      && _msg.buttons(this->data->enableButton))
    {
      // Linear
      if (this->data->axisLinear < _msg.axes_size())
      {
        const double x = - _msg.axes(this->data->axisLinear);
        cmdMsg.set_velocity(x * this->data->scaleLinear);
      }

      // Angular
      if (this->data->axisAngular < _msg.axes_size())
      {
        const double x = - _msg.axes(this->data->axisAngular);
        cmdMsg.set_theta(x * this->data->scaleAngular);
      }

      // Publish the Twist (=CmdVel2D) message
      if (this->data->pub) 
      {
        this->data->pub.Publish(cmdMsg);
        this->data->sentDisableMsg = false;
      }
    }
    else
    {
      if (!this->data->sentDisableMsg && this->data->pub)
      {
        cmdMsg.set_velocity(0.0);
        cmdMsg.set_theta(0.0);
        this->data->pub.Publish(cmdMsg);
        this->data->sentDisableMsg = true;
      }
    }
  }

} // namespace asv
