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
/// \brief A Gazebo model plugin to control the movement of a Link and Joint
/// based upon an ignition::msgs::CmdVel2D message.

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

  /// \brief This is a Gazebo model plugin that controls the movement of
  /// a Link and Joint based upon an ignition::msgs::CmdVel2D message.
  /// A pair of PID controllers are used to set either the velocity
  /// or force for a single link and either the position, velocity or force
  /// for a single joint. 
  ///
  /// # Usage
  ///
  /// Add the SDF for the plugin to the <model> element of your Gazebo model:
  ///
  /// /code
  /// <plugin name="link_joint_controller" filename="libLinkJointControllerPlugin.so">
  ///   <input_topic>/cmd_vel</input_topic>
  ///   <link_name>thruster_link</link_name>
  ///   <link_scale>2</link_scale>
  ///   <link_type>velocity</link_type>
  ///   <link_pid>1000 0 10</link_pid>
  ///   <joint_name>thruster_joint</joint_name>
  ///   <joint_scale>0.8</joint_scale>
  ///   <joint_type>position</joint_type>
  ///   <joint_pid>1000 0 10</joint_pid>
  /// </plugin>
  /// /endcode
  ///
  /// # Subscribed Topics
  ///
  /// 1. /cmd_vel (ignition::msgs::CmdVel2D)
  ///
  /// # Parameters
  ///
  /// 1. <input_topic> (string, default: /cmd_vel)
  ///   The name of the topic to subscribe to ignition::msgs::CmdVel2D messages.
  ///
  /// 2. <link_name> (string, default: link)
  ///   The name of the link to control.
  ///
  /// 3. <link_scale> (double, default: 1.0)
  ///   A scale factor to apply to the value of the ignition::mgs::CmdVel2D.velocity
  ///   message setting the target motion of the link.
  ///
  /// 4. <link_type> (string, default: velocity)
  ///   The type of motion or action applied to the link.
  ///   Options are 'velocity' or 'force'.
  ///
  /// 5. <link_pid> (Vector3D, default: (1000 0 10))
  ///   PID gains (kp, ki, kd) for the link PID controller.
  ///   Only applied when the motion type is 'velocity'.
  ///
  /// 6. <joint_name> (string, default: joint)
  ///   The name of the joint to control.
  ///
  /// 7. <joint_scale> (double, default: 1.0)
  ///   A scale factor to apply to the value of the ignition::mgs::CmdVel2D.theta
  ///   message setting the target motion of the joint.
  ///
  /// 8. <joint_type> (string, default: position)
  ///   The type of motion or action applied to the joint.
  ///   Options are 'position', velocity' or 'force'.
  ///
  /// 9. <joint_pid> (Vector3D, default: (1000 0 10))
  ///   PID gains (kp, ki, kd) for the joint PID controller.
  ///   Only applied when the motion type is 'position' or velocity'.
  ///
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
