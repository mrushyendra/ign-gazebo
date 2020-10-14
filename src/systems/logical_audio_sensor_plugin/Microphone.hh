/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef IGNITION_GAZEBO_SYSTEMS_LOGICAL_AUDIO_SENSOR_PLUGIN_MICROPHONE_HH_
#define IGNITION_GAZEBO_SYSTEMS_LOGICAL_AUDIO_SENSOR_PLUGIN_MICROPHONE_HH_

#include <ignition/gazebo/config.hh>
#include <ignition/math/Vector3.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
namespace logical_audio
{
  /// \brief Class that acts as a microphone (audio sink).
  ///   This class can be used with the Source class to listen
  ///   for audio that's emitted from a particular location.
  class Microphone
  {
    /// \brief Constructor.
    /// \param[in] _id The microphone's ID.
    ///   Must be >= 0.
    /// \param[in] _position The microphone's position (x,y,z).
    /// \param[in] _volumeDetectionThreshold The microphone's
    ///   minimum volume detection level.
    ///   Must be >= 0.0 and <= 1.0.
    public: Microphone(const unsigned int _id,
                       const ignition::math::Vector3d &_position,
                       const float _volumeDetectionThreshold);

    /// \brief The microphone's ID.
    private: unsigned int id;

    /// \brief The microphone's position, stored as (x,y,z).
    private: ignition::math::Vector3d position;

    /// \brief The microphone's minimum volume detection level.
    ///   Must be a value between 0.0 and 1.0.
    private: float volumeDetectionThreshold;
  };
}
}
}
}
}

#endif
