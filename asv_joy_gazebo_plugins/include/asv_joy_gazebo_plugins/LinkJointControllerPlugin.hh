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

/// \file LinkJointControllerPlugin.hh
/// \brief PID controllers for a link and joint based upon a CmdVel2D message ('Twist').

#ifndef _ASV_JOY_GAZEBO_PLUGINS_LINK_JOINT_CONTROLLER_PLUGIN_HH_
#define _ASV_JOY_GAZEBO_PLUGINS_LINK_JOINT_CONTROLLER_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace ignition
{
  namespace msgs
  {
    class CmdVel2D;
  }
}

namespace asv
{
  
///////////////////////////////////////////////////////////////////////////////
// LinkJointControllerPlugin

  /// \internal
  /// \brief Class to hold private data for LinkJointControllerPlugin.
  class LinkJointControllerPluginPrivate;

  /// \brief A Model plugin to control a link and joint from CmdVel2D messages.
  class GAZEBO_VISIBLE LinkJointControllerPlugin : public gazebo::ModelPlugin
  {
    /// \brief Destructor.
    public: virtual ~LinkJointControllerPlugin() override;

    /// \brief Constructor.
    public: LinkJointControllerPlugin();

    // Documentation inherited.
    public: void Load(gazebo::physics::ModelPtr _world, sdf::ElementPtr _sdf) override;

    // Documentation inherited.
    public: void Init() override;

    // Documentation inherited.
    public: void Reset() override;

    /// \internal
    /// \brief Callback each time a CmdVel2D message is received.
    /// \param[in] _msg CmdVel2D message.
    private: void OnCmdVelMsg(const ignition::msgs::CmdVel2D& _msg);

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<LinkJointControllerPluginPrivate> data;
  };
} // namespace asv

#endif // _ASV_JOY_GAZEBO_PLUGINS_LINK_JOINT_CONTROLLER_PLUGIN_HH_
