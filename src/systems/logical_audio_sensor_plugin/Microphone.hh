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
    /// \param[in] _position The microphone's position (x,y,z).
    /// \param[in] _volumeDetectionThreshold The microphone's
    ///   minimum volume detection level.
    ///   Must be >= 0.0 and <= 1.0.
    ///   If the value given is < 0.0 or > 1.0, it will be clipped to
    ///   0.0 (if < 0.0) or 1.0 (if > 1.0).
    public: Microphone(const unsigned int _id,
                       const ignition::math::Vector3d &_position,
                       const float _volumeDetectionThreshold);

    /// \brief Determines if the microphone can detect volume at a specific
    ///   level.
    /// \param[in] _volumeLevel The volume level that the microphone is
    ///   attempting to detect. This should be a value between
    ///   \c 0.0 (no volume) and \c 1.0 (maximum volume).
    /// \return \c True if the microphone can detect volume at \c _volumeLevel,
    ///   \c False otherwise.
    public: bool Detect(const float _volumeLevel) const;

    /// \brief Get the microphone's ID.
    /// \return The ID of the microphone.
    public: unsigned int GetID() const;

    /// \brief Get the microphone's position.
    /// \return The position of the microphone.
    public: ignition::math::Vector3d GetPosition() const;

    /// \brief Get the microphone's volume detection threshold.
    /// \return The volume detection threshold.
    public: float GetVolumeDetectionThreshold() const;

    /// \brief Set the microphone's position.
    /// \param[in] _position The microphone's new position.
    public: void SetPosition(const ignition::math::Vector3d &_position);

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
