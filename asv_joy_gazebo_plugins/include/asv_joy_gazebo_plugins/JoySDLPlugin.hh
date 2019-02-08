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

/// \file JoySDLPlugin.hh
/// \brief A plugin used to publish ignition::msgs:Joy messages
/// from a joystick or game controller.

#ifndef _ASV_JOY_GAZEBO_PLUGINS_JOY_SDL_PLUGIN_HH_
#define _ASV_JOY_GAZEBO_PLUGINS_JOY_SDL_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace asv
{
  
///////////////////////////////////////////////////////////////////////////////
// JoySDLPlugin

  /// \internal
  /// \brief Class to hold private data for JoySDLPlugin.
  class JoySDLPluginPrivate;

  /// \brief This is a version of the Gazebo JoyPlugin for OSX.
  /// It registers button presses and joystick movement with a user 
  /// configurable deadzone region, but does not support the 'sticky_button'
  /// feature of the Linux version. 
  ///
  /// # Usage
  /// 
  /// Add the SDF for plugin to the <world> element of your world file:
  ///
  /// \code
  /// <plugin name="joy" filename="libJoySDLPlugin.so">
  ///   <joy_id>0</joy_id>
  ///   <output_topic>/joy</output_topic>
  ///   <rate>50</rate>
  ///   <dead_zone>0.1</dead_zone>
  /// </plugin>
  /// \endcode
  ///
  /// # Published Topics
  ///
  /// 1. /joy (ignition::msgs::Joy)
  ///
  /// # Parameters
  /// 
  /// 1. <joy_id> (int, default: 0)
  ///   The joystick identifier.
  ///
  /// 2. <output_topic> (string, default /joy)
  ///   The name of the topic to which ignition::msgs::Joy are published.
  ///
  /// 3. <rate> (double, default: 50)
  ///   The rate in HZ at which joystick messages are published.
  ///
  /// 4. <dead_zone> (double between 0 and 0.9, default: 0.1)
  ///   The amount by which the joystick has to move to register a value.
  ///
  class GAZEBO_VISIBLE JoySDLPlugin : public gazebo::WorldPlugin
  {
    /// \brief Destructor.
    public: virtual ~JoySDLPlugin() override;

    /// \brief Constructor.
    public: JoySDLPlugin();

    // Documentation inherited.
    public: void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<JoySDLPluginPrivate> data;
  };
} // namespace asv

#endif // _ASV_JOY_GAZEBO_PLUGINS_JOY_SDL_PLUGIN_HH_
