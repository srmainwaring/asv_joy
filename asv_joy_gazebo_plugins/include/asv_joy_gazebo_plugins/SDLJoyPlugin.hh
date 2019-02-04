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

/// \file SDLJoyPlugin.hh
/// \brief This file defines a Gazebo WorldPlugin used to publish
/// messages from a joystick or game controller.

#ifndef _ASV_JOY_GAZEBO_PLUGINS_SDL_JOY_PLUGIN_HH_
#define _ASV_JOY_GAZEBO_PLUGINS_SDL_JOY_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace asv
{
  
///////////////////////////////////////////////////////////////////////////////
// SDLJoyPlugin

  /// \internal
  /// \brief Class to hold private data for SDLJoyPlugin.
  class SDLJoyPluginPrivate;

  /// \brief A World plugin to capture joystick commands and publish
  /// to a Gazebo topic.
  class GAZEBO_VISIBLE SDLJoyPlugin : public gazebo::WorldPlugin
  {
    /// \brief Destructor.
    public: virtual ~SDLJoyPlugin() override;

    /// \brief Constructor.
    public: SDLJoyPlugin();

    // Documentation inherited.
    public: void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

    /// \internal
    /// \brief Pointer to the class private data.
    private: std::shared_ptr<SDLJoyPluginPrivate> data;
  };
} // namespace asv

#endif // _ASV_JOY_GAZEBO_PLUGINS_SDL_JOY_PLUGIN_HH_
