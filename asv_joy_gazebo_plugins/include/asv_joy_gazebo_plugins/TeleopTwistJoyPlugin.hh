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

/// \file TeleopTwistJoyPlugin.hh
/// \brief A plugin that subscribes to an ignition::msgs::Joy message
/// and publishes an ignition::msgs::CmdVel2D message (Twist).

#ifndef _ASV_JOY_GAZEBO_PLUGINS_TELEOP_TWIST_JOY_PLUGIN_HH_
#define _ASV_JOY_GAZEBO_PLUGINS_TELEOP_TWIST_JOY_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace ignition
{
  namespace msgs
  {
    class Joy;
  }
}

namespace asv
{
  
///////////////////////////////////////////////////////////////////////////////
// TeleopTwistJoyPlugin

  /// \internal
  /// \brief Class to hold private data for TeleopTwistJoyPlugin.
  class TeleopTwistJoyPluginPrivate;

  /// \brief This is a World plugin that maps joystick messages into
  /// 'twist'commands (ignition::msgs::CmdVel2D) and publishes them  
  /// to an Ignition transport topic.  It's behaviour and usage are
  /// similar to the ROS teleop_twist_joy package upon which it is based.
  ///
  /// # Usage
  ///
  /// The plugin is loaded via the World plugin interface using the SDF elements: 
  ///
  /// /code
  /// <plugin name="teleop_twist_joy" filename="libTeleopTwistJoyPlugin.so">
  ///   <input_topic>/joy</input_topic>
  ///   <output_topic>/cmd_vel</output_topic>
  ///   <enable_button>10</enable_button>
  ///   <axis_linear>3</axis_linear>
  ///   <scale_linear>1.0</scale_linear>
  ///   <axis_angular>0</axis_angular>
  ///   <scale_angular>1.0</scale_angular>
  /// </plugin>
  /// /endcode
  ///
  /// # Configuration
  ///
  /// 1. <input_topic>(string): The name of the topic to subscribe to
  /// ignition::msgs::Joy messages. The default is '/joy'.
  ///
  /// 2. <output_topic>(string): The name of the topic to which
  /// ignition::msgs::CmdVel2 messages are published. The default is '/cmd_vel'.
  ///
  /// 3. <enable_button>(int): The index of the button that must be held
  /// for command messages to be published (deadman switch). The default is 0.
  ///
  /// 4. <axis_linear>(int): The index of the joystick axis that sets the 
  /// value of ignition::msgs::CmdVel2D.velocity. The default is 3.
  ///
  /// 5. <scale_linear>(double): A scale factor to apply to the value of the 
  /// linear joystick axis (which is in the range -1 to 1). The default is 1.
  ///
  /// 6. <axis_angular>(int): The index of the joystick axis that sets the 
  /// value of ignition::msgs::CmdVel2D.theta. The default is 0.
  ///
  /// 7. <scale_angular>(double): A scale factor to apply to the value of the 
  /// angular joystick axis (which is in the range -1 to 1). The default is 1.
  ///
  class GAZEBO_VISIBLE TeleopTwistJoyPlugin : public gazebo::WorldPlugin
  {
    /// \brief Destructor.
    public: virtual ~TeleopTwistJoyPlugin() override;

    /// \brief Constructor.
    public: TeleopTwistJoyPlugin();

    // Documentation inherited.
    public: void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

    // Documentation inherited.
    public: void Init() override;

    // Documentation inherited.
    public: void Reset() override;

    /// \internal
    /// \brief Callback each time a joy message is received.
    /// \param[in] _msg Joy message.
    private: void OnJoyMsg(const ignition::msgs::Joy& _msg);

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<TeleopTwistJoyPluginPrivate> data;
  };
} // namespace asv

#endif // _ASV_JOY_GAZEBO_PLUGINS_TELEOP_TWIST_JOY_PLUGIN_HH_
