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

/// \file ThrusterTeleopPlugin.hh
/// \brief Control a joint in a Gazebo model using a joystick.

#ifndef _ASV_JOY_GAZEBO_PLUGINS_THRUSTER_TELEOP_PLUGIN_HH_
#define _ASV_JOY_GAZEBO_PLUGINS_THRUSTER_TELEOP_PLUGIN_HH_

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
// ThrusterTeleopPlugin

  /// \internal
  /// \brief Class to hold private data for ThrusterTeleopPlugin.
  class ThrusterTeleopPluginPrivate;

  /// \brief A Model plugin to convert joystick messages into forces.
  class GAZEBO_VISIBLE ThrusterTeleopPlugin : public gazebo::ModelPlugin
  {
    /// \brief Destructor.
    public: virtual ~ThrusterTeleopPlugin() override;

    /// \brief Constructor.
    public: ThrusterTeleopPlugin();

    // Documentation inherited.
    public: void Load(gazebo::physics::ModelPtr _world, sdf::ElementPtr _sdf) override;

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
    private: std::shared_ptr<ThrusterTeleopPluginPrivate> data;
  };
} // namespace asv

#endif // _ASV_JOY_GAZEBO_PLUGINS_THRUSTER_TELEOP_PLUGIN_HH_
