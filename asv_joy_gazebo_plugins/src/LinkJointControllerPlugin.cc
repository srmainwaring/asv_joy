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

#include "asv_joy_gazebo_plugins/LinkJointControllerPlugin.hh"

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include <iostream>
#include <string>

using namespace gazebo;

namespace asv
{

  GZ_REGISTER_MODEL_PLUGIN(LinkJointControllerPlugin)

///////////////////////////////////////////////////////////////////////////////
// LinkJointControllerPluginPrivate

  /// \internal
  /// \brief Private data for the LinkJointControllerPlugin
  class LinkJointControllerPluginPrivate
  {
    /// \brief The input topic containing the CmdVel2D message.
    public: std::string input_topic = "/cmd_vel";

    /// \brief The name of the link to control.
    public: std::string linkName = "link";

    /// \brief The action to control: 'velocity' or 'force'
    public: std::string linkType = "velocity";

    /// \brief Scaling to apply to the CmdVel2D.velocity component of the message. 
    public: double linkScale = 1.0;

    /// \brief PID gains for the link controller.
    public: ignition::math::Vector3d linkPID = ignition::math::Vector3d(1000, 0 , 10);

    /// \brief PID controller for the link.
    public: common::PID linkController;

    /// Pointer to the link.
    public: physics::LinkPtr link;

    /// \brief The name of the joint to control.
    public: std::string jointName = "joint";

    /// \brief The action to control: 'position', velocity' or 'force'
    public: std::string jointType = "position";

    /// \brief Scaling to apply to the CmdVel2D.theta component of the message. 
    public: double jointScale = 1.0;

    /// \brief PID gains for the link controller.
    public: ignition::math::Vector3d jointPID = ignition::math::Vector3d(1000, 0 , 10);

    /// \brief Pointer to the link.
    public: physics::JointPtr joint;

    /// \brief Previous update time for PID controllers.
    public: common::Time prevTime;

    /// \brief Pointer to the model.
    public: physics::ModelPtr model;

    /// \brief Ignition transport node.
    public: ignition::transport::Node ignNode;
  };

///////////////////////////////////////////////////////////////////////////////
// LinkJointControllerPlugin

  LinkJointControllerPlugin::~LinkJointControllerPlugin()
  {
  }

  LinkJointControllerPlugin::LinkJointControllerPlugin() : 
    ModelPlugin(), 
    data(new LinkJointControllerPluginPrivate())
  {
  }

  void LinkJointControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    GZ_ASSERT(_model != nullptr, "Invalid parameter _model");
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    // Model
    this->data->model = _model;

    // Parameters
    this->LoadParam(_sdf, "input_topic", this->data->input_topic, this->data->input_topic);

    // Link Parameters
    this->LoadParam(_sdf, "link_name", this->data->linkName, this->data->linkName);
    this->LoadParam(_sdf, "link_type", this->data->linkType, this->data->linkType);
    this->LoadParam(_sdf, "link_scale", this->data->linkScale, this->data->linkScale);
    this->LoadParam(_sdf, "link_pid", this->data->linkPID, this->data->linkPID);

    // Link and Link Controller
    this->data->link = this->data->model->GetLink(this->data->linkName);
    if (this->data->link == nullptr)
    {
      gzerr << "Link with name [" << this->data->linkName << "] not found. "
        << "No forces will be applied." << std::endl;
    }
    else
    {
      this->data->linkController = common::PID(
        this->data->linkPID.X(),
        this->data->linkPID.Y(),
        this->data->linkPID.Z());
    }

    // Joint Parameters
    this->LoadParam(_sdf, "joint_name", this->data->jointName, this->data->jointName);
    this->LoadParam(_sdf, "joint_type", this->data->jointType, this->data->jointType);
    this->LoadParam(_sdf, "joint_scale", this->data->jointScale, this->data->jointScale);
    this->LoadParam(_sdf, "joint_pid", this->data->jointPID, this->data->jointPID);

    // Joint and Joint Controller
    this->data->joint = this->data->model->GetJoint(this->data->jointName);
    if (this->data->joint == nullptr)
    {
      gzerr << "Joint with name [" << this->data->jointName << "] not found. "
        << "No joint control will be applied." << std::endl;
    }
    else
    {
      auto controller = this->data->model->GetJointController();
      common::PID pid(
        this->data->jointPID.X(),
        this->data->jointPID.Y(),
        this->data->jointPID.Z());
      if (this->data->jointType == "position")
      {
        controller->SetPositionPID(
          this->data->joint->GetScopedName(), pid);
      }
      else if (this->data->jointType == "velocity")
      {
        controller->SetVelocityPID(
          this->data->joint->GetScopedName(), pid);
      }
    }

    // Time
    this->data->prevTime = common::Time::GetWallTime();

    // Subscribers
    bool success = this->data->ignNode.Subscribe(
      this->data->input_topic, &LinkJointControllerPlugin::OnCmdVelMsg, this);

  }

  void LinkJointControllerPlugin::Init()
  {
  }

  void LinkJointControllerPlugin::Reset()
  {
  }

  void LinkJointControllerPlugin::OnCmdVelMsg(const ignition::msgs::CmdVel2D& _msg)
  {
    // Compute time step.
    common::Time currTime = common::Time::GetWallTime();
    common::Time dt = currTime - this->data->prevTime;
    this->data->prevTime = currTime;

    // Link Controller (x axis)
    if (this->data->link != nullptr)
    {
      auto controller = this->data->linkController;

      if (this->data->linkType == "velocity")
      {
        ignition::math::Vector3d currLinearVel = this->data->link->RelativeLinearVel();
        double targetVel = _msg.velocity() * this->data->linkScale;
        double linearError = currLinearVel.X() - targetVel;

        ignition::math::Vector3d force(
          controller.Update(linearError, dt), 0, 0);

        this->data->link->AddRelativeForce(force);
      }
      else if (this->data->linkType == "force")
      {
        ignition::math::Vector3d force(
          _msg.velocity() * this->data->linkScale, 0, 0);

        this->data->link->AddRelativeForce(force);
      }
    }

    // Joint Controller (first axis)
    if (this->data->joint != nullptr)
    {
      auto controller = this->data->model->GetJointController();

      if (this->data->jointType == "position")
      {
        double currPos = this->data->joint->Position(0);
        controller->SetPositionTarget(
          this->data->joint->GetScopedName(),
          currPos + _msg.theta() * this->data->jointScale);
      }
      else if (this->data->jointType == "velocity")
      {
        controller->SetVelocityTarget(
          this->data->joint->GetScopedName(),
          _msg.theta() * this->data->jointScale);
      } 
      else if (this->data->jointType == "force")
      {
        this->data->joint->SetForce(
          0, _msg.theta() * this->data->jointScale);          
      }
    }    

  }

} // namespace asv
