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

#include "asv_joy_gazebo_plugins/JoyToForceTorquePlugin.hh"

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

  GZ_REGISTER_MODEL_PLUGIN(JoyToForceTorquePlugin)

///////////////////////////////////////////////////////////////////////////////
// JoyToForceTorquePluginPrivate

  /// \internal
  /// \brief Private data for the JoyToForceTorquePlugin
  class JoyToForceTorquePluginPrivate
  {
    /// \brief Topic to subscribe, based on <topic>. Default value: '/joy'.
    public: std::string topic;

    /// \brief The name of the link to which the force will be applied.
    public: std::string linkName;

    /// \brief The gain on the force.
    public: double forceGain = 1000;

    /// \brief The gain on the yaw.
    public: double yawGain = 1.570796;

    /// \brief The model.
    public: physics::ModelPtr model;

    /// \brief The link to which the force will be applied.
    public: physics::LinkPtr link;

    /// \brief Ignition transport node.
    public: ignition::transport::Node ignNode;
  };

///////////////////////////////////////////////////////////////////////////////
// JoyToForceTorquePlugin

  JoyToForceTorquePlugin::~JoyToForceTorquePlugin()
  {
  }

  JoyToForceTorquePlugin::JoyToForceTorquePlugin() : 
    ModelPlugin(), 
    data(new JoyToForceTorquePluginPrivate())
  {
  }

  void JoyToForceTorquePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    GZ_ASSERT(_model != nullptr, "Invalid parameter _model");
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    // Model
    this->data->model = _model;

    // Parameters
    this->LoadParam(_sdf, "topic", this->data->topic, "/joy");
    this->LoadParam(_sdf, "link_name", this->data->linkName, "");
    this->LoadParam(_sdf, "force_gain", this->data->forceGain, this->data->forceGain);
    this->LoadParam(_sdf, "yaw_gain", this->data->yawGain, this->data->yawGain);
    
  }

  void JoyToForceTorquePlugin::Init()
  {
    // Link
    this->data->link = this->data->model->GetLink(this->data->linkName);
    if (this->data->link == nullptr)
    {
      gzerr << "Link with name [" << this->data->linkName << "] not found. "
        << "No forces will be applied." << std::endl;
    }

    // Subscribers
    bool success = this->data->ignNode.Subscribe(
      this->data->topic, &JoyToForceTorquePlugin::OnJoyMsg, this);
  }

  void JoyToForceTorquePlugin::Reset()
  {
  }

  void JoyToForceTorquePlugin::OnJoyMsg(const ignition::msgs::Joy& _msg)
  {
    // Mappings
    int idxForce = 3;   // R stick, U-D
    int idxRot = 0;     // L stick, L-R

    // There are no forces if the link is not valid.
    if (this->data->link == nullptr)
    {
      return;
    }

    // Force
    ignition::math::Vector3d forceX(
      - _msg.axes(idxForce) * this->data->forceGain,
      0,
      0);

    // Rotation
    ignition::math::Quaterniond rot(
      0,
      0,
      _msg.axes(idxRot) * this->data->yawGain);

    ignition::math::Vector3d force = rot.RotateVector(forceX);
    ignition::math::Vector3d torque(0, 0, 0);
    this->data->link->AddRelativeForce(force);
    this->data->link->AddRelativeTorque(torque);

    // std::cout << _msg.DebugString() << std::endl;
    // std::cout << "force:    " << force << std::endl;
  }

} // namespace gazebo
