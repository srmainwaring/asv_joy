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

#include "asv_joy_gazebo_plugins/ThrusterTeleopPlugin.hh"

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

  GZ_REGISTER_MODEL_PLUGIN(ThrusterTeleopPlugin)

///////////////////////////////////////////////////////////////////////////////
// JoyInfo

  /// \brief Store information from SDF for each joystick axes
  struct JoyInfo
  {
    /// \brief Joystick axes value.
    int axes;

    /// \brief Pointer to the joint controlled by this axes.
    physics::JointPtr joint;

    /// \brief Possible target types: position, velocity, force.
    std::string type;

    /// \brief Increments for position, absolute values for velocity and force.
    double scale;
  };

///////////////////////////////////////////////////////////////////////////////
// ThrusterTeleopPluginPrivate

  /// \internal
  /// \brief Private data for the ThrusterTeleopPlugin
  class ThrusterTeleopPluginPrivate
  {
    /// \brief Topic to subscribe, based on <topic>. Default value: '/joy'.
    public: std::string topic;

    /// \brief The name of the link to which the force will be applied.
    public: std::string linkName;

    /// \brief The name of the joint which will be controlled.
    public: std::string jointName;

    /// \brief The upper limit for the link force.
    public: double maxLinkForce = 1000.0;

    /// \brief The upper limit for the link force.
    public: double maxJointForce = 1000.0;

    /// \brief The deadzone for the controller (0 <= d < 1).
    public: double deadzone = 0.1;

    /// \brief Store information about each tracked joystick axes.
    public: std::vector<JoyInfo> joyInfo;

    /// \brief The model.
    public: physics::ModelPtr model;

    /// \brief The link to which the force will be applied.
    public: physics::LinkPtr link;

    /// \brief The joint which will be controlled.
    public: physics::JointPtr joint;

    /// \brief Ignition transport node.
    public: ignition::transport::Node ignNode;
  };

///////////////////////////////////////////////////////////////////////////////
// ThrusterTeleopPlugin

  ThrusterTeleopPlugin::~ThrusterTeleopPlugin()
  {
  }

  ThrusterTeleopPlugin::ThrusterTeleopPlugin() : 
    ModelPlugin(), 
    data(new ThrusterTeleopPluginPrivate())
  {
  }

  void ThrusterTeleopPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    GZ_ASSERT(_model != nullptr, "Invalid parameter _model");
    GZ_ASSERT(_sdf != nullptr, "Invalid parameter _sdf");

    // Model
    this->data->model = _model;

    // Parameters
    this->LoadParam(_sdf, "topic", this->data->topic, "/joy");
    this->LoadParam(_sdf, "link_name", this->data->linkName, "");
    this->LoadParam(_sdf, "max_force", this->data->maxLinkForce, this->data->maxLinkForce);
    this->LoadParam(_sdf, "dead_zone", this->data->deadzone, this->data->deadzone);

    if (this->data->deadzone < 0.0 || this->data->deadzone > 0.9)
    {
      gzerr << "Parameter <dead_zone> must be in [0, 0.9]" << std::endl;
      this->data->deadzone = 0.1;
    }    

    // Link
    this->data->link = this->data->model->GetLink(this->data->linkName);
    if (this->data->link == nullptr)
    {
      gzerr << "Link with name [" << this->data->linkName << "] not found. "
        << "No forces will be applied." << std::endl;
    }

    // Joint
    // this->data->joint = this->data->model->GetJoint(this->data->jointName);
    // if (this->data->joint == nullptr)
    // {
    //   gzerr << "Joint with name [" << this->data->jointName << "] not found. "
    //     << "No joint control will be applied." << std::endl;
    // }

    // From KeysToJointsPlugin.cc
    // Load params from SDF
    auto controller = this->data->model->GetJointController();
    if (_sdf->HasElement("map"))
    {
      auto mapElem = _sdf->GetElement("map");
      while (mapElem)
      {
        auto jointName = mapElem->Get<std::string>("joint");        
        auto joint = this->data->model->GetJoint(jointName);
        if (!joint)
        {
          gzwarn << "Can't find joint [" << jointName << "]" << std::endl;
        }
        else
        {
          if (!mapElem->HasAttribute("axes") ||
              !mapElem->HasAttribute("scale") ||
              !mapElem->HasAttribute("type"))
          {
            gzwarn << "Missing [axes], [scale] or [type] attribute, skipping map."
                << std::endl;
            mapElem = mapElem->GetNextElement("map");
            continue;
          }
          JoyInfo info;
          info.axes = mapElem->Get<int>("axes");
          info.joint = joint;
          info.scale = mapElem->Get<double>("scale");
          info.type = mapElem->Get<std::string>("type");

          if (info.type != "force")
          {
            double kp = 0;
            double ki = 0;
            double kd = 0;
            if (mapElem->HasAttribute("kp"))
              kp = mapElem->Get<double>("kp");
            if (mapElem->HasAttribute("ki"))
              ki = mapElem->Get<double>("ki");
            if (mapElem->HasAttribute("kd"))
              kd = mapElem->Get<double>("kd");

            common::PID pid(kp, ki, kd);
            if (info.type == "position")
              controller->SetPositionPID(info.joint->GetScopedName(), pid);
            else if (info.type == "velocity")
              controller->SetVelocityPID(info.joint->GetScopedName(), pid);
          }

          this->data->joyInfo.push_back(info);
        }

        mapElem = mapElem->GetNextElement("map");
      }
    }

    // Subscribers
    bool success = this->data->ignNode.Subscribe(
      this->data->topic, &ThrusterTeleopPlugin::OnJoyMsg, this);

  }

  void ThrusterTeleopPlugin::Init()
  {
  }

  void ThrusterTeleopPlugin::Reset()
  {
  }

  void ThrusterTeleopPlugin::OnJoyMsg(const ignition::msgs::Joy& _msg)
  {
    // Jpystick index mappings
    int idxLinkForce = 3;   // R stick, U-D
    int idxJointForce = 0;  // L stick, L-R

    // Apply Force to Link: there is no force if the link is not valid.
    if (this->data->link != nullptr)
    {
      const double b = this->data->maxLinkForce;
      const double d = this->data->deadzone;
      const double x = - _msg.axes(idxLinkForce);
      double f = 0.0;
      if (std::fabs(x) > d)
      {
        f = b * (x - d) / (1.0 - d);
      }
      ignition::math::Vector3d force(f, 0, 0);
      this->data->link->AddRelativeForce(force);
    }

    // Apply Position to Joint: there is no movement if the joint is not valid.
    //
    // Note: by default this does not preserve the world velocity
    // after the position has been set. This results in the link 
    // stopping each time this is called (undesirable).
    //
    // if (this->data->joint != nullptr)
    // {
    //   const unsigned int idxAxis = 0;
    //   const double a = this->data->joint->LowerLimit(idxAxis);
    //   const double b = this->data->joint->UpperLimit(idxAxis);      
    //   const double x = _msg.axes(idxRot);
    //   double y = 0.5 * (x + 1.0) * (b - a) + a; 
    //   this->data->joint->SetPosition(idxAxis, y);
    // }
    
    // Apply Force to Joint: there is no force if the joint is not valid.
    // if (this->data->joint != nullptr)
    // {
    //   const unsigned int idxAxis = 0;
    //   const double b = this->data->maxJointForce;
    //   const double d = this->data->deadzone;
    //   const double x = - _msg.axes(idxJointForce);
    //   double f = 0.0;
    //   if (std::fabs(x) > d)
    //   {
    //     f = b * (x - d) / (1.0 - d);
    //   }
    //   this->data->joint->SetForce(idxAxis, f);
    // }

    // Joint controllers
    for (auto&& info : this->data->joyInfo)
    {
      if (info.axes < _msg.axes_size())
      {
        const double x =  - _msg.axes(info.axes);
        const double b = info.scale;
        const double d = this->data->deadzone;
        double y = 0.0;
        if (std::fabs(x) > d)
        {
          y = b * (x - d) / (1.0 - d);
        }

        auto controller = this->data->model->GetJointController();

        if (info.type == "position")
        {
          controller->SetPositionTarget(
            info.joint->GetScopedName(),
            y);
          // gzmsg << "Set joint [" << info.joint->GetScopedName() << "] position to: "
          //   << currPos + y << std::endl;
        }
        else if (info.type == "velocity")
        {
          controller->SetVelocityTarget(
            info.joint->GetScopedName(),
            y);
          // gzmsg << "Set joint [" << info.joint->GetScopedName() << "] velocity to: "
          //   << y << std::endl;
        } 
        else if (info.type == "force")
        {
          info.joint->SetForce(0, y);          
          // gzmsg << "Set joint [" << info.joint->GetScopedName() << "] force to: "
          //   << y << std::endl;
        }
      }
    }

  }

} // namespace gazebo
