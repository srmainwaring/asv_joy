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
/// \brief A Model plugin to control the movement of a Link and Joint
/// based upon an ignition::msgs::CmdVel2D message ('Twist').

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

  /// \brief This is a Model plugin that controls the movement of
  /// a Link and Joint based upon an ignition::msgs::CmdVel2D message.
  /// A pair of PID controllers are be used to set either the velocity
  /// or force for a single link and either the position, velocity or force
  /// for a single joint. 
  ///
  /// # Usage
  ///
  /// The plugin is loaded via the Model plugin interface using the SDF elements:
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
  /// 1. <input_topic>(string): The name of the topic to subscribe to
  /// ignition::msgs::CmdVel2D messages. The default is '/cmd_vel'.
  ///
  /// 2. <link_name>(string): The name of the link to control.
  /// The default is 'link'.
  ///
  /// 3. <link_scale>(double): A scale factor to apply to the value of the 
  /// ignition::mgs::CmdVel2D.velocity message setting the target motion
  /// of the link. The default is 1.
  ///
  /// 4. <link_type>(double): The type of motion or action applied 
  /// to the link. Options are 'velocity' or 'force'. The default is 'velocity'.
  ///
  /// 5. <link_pid>(Vector3D): PID gains (kp, ki, kd) for the link PID controller.
  /// Only applied when the motion type is 'velocity'. The default is (1000 0 1).
  ///
  /// 6. <joint_name>(string): The name of the joint to control.
  /// The default is 'joint'.
  ///
  /// 7. <joint_scale>(double): A scale factor to apply to the value of the 
  /// ignition::mgs::CmdVel2D.theta message setting the target motion
  /// of the joint. The default is 1.
  ///
  /// 8. <joint_type>(double): The type of motion or action applied 
  /// to the joint. Options are 'position', velocity' or 'force'.
  /// The default is 'position'.
  ///
  /// 9. <joint_pid>(Vector3D): PID gains (kp, ki, kd) for the joint PID controller.
  /// Only applied when the motion type is 'position' or velocity'.
  /// The default is (1000 0 1).
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
