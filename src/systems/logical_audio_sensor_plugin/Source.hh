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

#ifndef IGNITION_GAZEBO_SYSTEMS_LOGICAL_AUDIO_SENSOR_PLUGIN_SOURCE_HH_
#define IGNITION_GAZEBO_SYSTEMS_LOGICAL_AUDIO_SENSOR_PLUGIN_SOURCE_HH_

#include <string>
#include <unordered_map>

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
  /// \brief Audio source attenuation functions.
  /// AttenuationFunction::Undefined is used to indicate that an
  /// attenuation function has not been defined yet.
  enum class AttenuationFunction { Linear, Undefined };

  /// \brief Audio source attenuation shapes.
  /// AttenuationShape::Undefined is used to indicate that an
  /// attenuation shape has not been defined yet.
  enum class AttenuationShape { Sphere, Undefined };

  /// \brief Class that acts as an audio source.
  ///   This class does not play actual audio to a device,
  ///   but can be used to mimic sound emission in an environment.
  ///   This can be used with the Microphone class to see where
  ///   the source's emitted sound is detected in an environment.
  class Source
  {
    /// \brief Constructor.
    /// \param[in] _id The source's ID.
    /// \param[in] _position The source's position (x,y,z).
    /// \param[in] _attenuationFunc The attenuation function.
    /// \param[in] _attenuationShape The attenuation shape.
    /// \param[in] _innerRadius The inner radius.
    ///   Must be >= 0.0.
    ///   If a value < 0 is given, the inner radius will be set to 0.
    ///   The source's sound within inner radius distance away from
    ///   the source will have a volume of _volumeLevel.
    /// \param[in] _falloffDistance The falloff distance.
    ///   This value must be greater than the value of _innerRadius.
    ///   If a value <= _innerRadius is given, the falloff distance will be
    ///   set to (_innerRadius + 1).
    ///   If a location in the environment has a distance that
    ///   is >= _falloffDistance away from the source, then the volume
    ///   at this location is 0.
    /// \param[in] _volumeLevel The volume emitted from the source.
    ///   This must be a value between 0.0 and 1.0
    ///   (representing 0% to 100% volume).
    ///   If the value given is < 0.0 or > 1.0, it will be clipped to
    ///   0.0 (if < 0.0) or 1.0 (if > 1.0).
    /// \param[in] _playing Whether the source is playing by default.
    ///   Use true to start playing by default, and false if you'd like
    ///   to start playing at a later time.
    public: Source(const unsigned int _id,
                   const ignition::math::Vector3d &_position,
                   const AttenuationFunction _attenuationFunc,
                   const AttenuationShape _attenuationShape,
                   const float _innerRadius,
                   const float _falloffDistance,
                   const float _volumeLevel,
                   const bool _playing);

    /// \brief Constructor.
    /// \param[in] _id The source's ID.
    /// \param[in] _position The source's position (x,y,z).
    /// \param[in] _attenuationFunc The attenuation function.
    /// \param[in] _attenuationShape The attenuation shape.
    /// \param[in] _innerRadius The inner radius.
    /// \param[in] _falloffDistance The falloff distance.
    /// \param[in] _volumeLevel The volume emitted from the source.
    /// \param[in] _playing Whether the source is playing by default.
    /// \sa Source() for more information about valid constructor parameters.
    public: Source(const unsigned int _id,
                   const ignition::math::Vector3d &_position,
                   const std::string &_attenuationFunc,
                   const std::string &_attenuationShape,
                   const float _innerRadius,
                   const float _falloffDistance,
                   const float _volumeLevel,
                   const bool _playing);

    /// \brief Computes the volume level of the source at a specific location.
    /// \param[in] _location The location where the volume level should be
    ///   calculated.
    /// \return The volume level at this location.
    ///   If the attenuation function or shape is undefined, -1.0 is returned.
    ///   If the source is not playing, 0.0 is returned.
    public: float VolumeLevel(const ignition::math::Vector3d &_location) const;

    /// \brief Start playing audio from the source. If the source is already
    ///   playing audio, nothing will change (the source will keep playing).
    public: void StartPlaying();

    /// \brief Stop playing audio from the source. If the source has already
    ///   been stopped, nothing will change (the source will remain stopped).
    public: void StopPlaying();

    /// \brief Check to see if the audio source is playing or not.
    /// \return \c true if the source is playing, \c false otherwise.
    public: bool IsPlaying() const;

    /// \brief Set the source's position.
    /// \param[in] _position The new position of the source.
    public: void SetPosition(const ignition::math::Vector3d &_position);

    /// \brief Set the attenuation function that matches the defined string.
    ///   The string is case sensitive, and must match the spelling
    ///   of the values in AttenuationFunction. If the spelling does not match,
    ///   the attenuation function is set as Undefined.
    ///
    ///   \em Example: to set the attenuation function to
    ///     AttenuationFunction::Linear, the following
    ///     are examples of valid strings: "linear", "Linear", "LINEAR"
    /// \param[in] _attenuationFunc a string that should map to a value in
    ///   AttenuationFunction.
    private: void SetAttenuationFunction(std::string _attenuationFunc);

    /// \brief Set the attenuation shape that matches the defined string.
    ///   The string is case sensitive, and must match the spelling
    ///   of the values in AttenuationShape. If the spelling does not match,
    ///   the attenuation shape is set as Undefined.
    ///
    ///   \em Example: to set the attenuation shape to
    ///     AttenuationShape::Sphere, the following
    ///     are examples of valid strings: "sphere", "Sphere", "SPHERE"
    /// \param[in] _attenuationShape a string that should map to a value in
    ///   AttenuationShape.
    private: void SetAttenuationShape(std::string _attenuationShape);

    /// \brief Set the inner radius for the audio source.
    /// \param[in] _innerRadius The inner radius to set for the source.
    ///   This value must be > 0.
    ///   If the value of this parameter is < 0, the source's inner radius
    ///   will be set to 0.
    private: void SetInnerRadius(const float _innerRadius);

    /// \brief Set the falloff distance for the audio source.
    /// \param[in] _falloffDistance The falloff distance to set for the source.
    ///   This value must be greater than the source's inner radius.
    ///   If this value is <= the source's inner radius, the falloff
    ///   distance will be set to (inner radius + 1).
    private: void SetFalloffDistance(const float _falloffDistance);

    /// \brief Set the source's volume level.
    /// \param[in] _volumeLevel The volume the source should play at.
    ///   The value of this parameter must be >= 0.0 (0% volume)
    ///   and <= 1.0 (100% volume).
    ///   If this parameter is < 0.0, the source's volume will be set to 0.
    ///   If this parameter is > 1.0, the source's volume will be set to 1.
    public: void SetVolumeLevel(const float _volumeLevel);

    /// \brief Get the source's ID.
    /// \return The ID.
    public: unsigned int GetID() const;

    /// \brief Get the source's position.
    /// \return The position.
    public: ignition::math::Vector3d GetPosition() const;

    /// \brief Get the source's attenuation function.
    /// \return The attenuation function.
    public: AttenuationFunction GetAttenuationFunction() const;

    /// \brief Get the source's attenuation shape.
    /// \return The attenuation shape.
    public: AttenuationShape GetAttenuationShape() const;

    /// \brief Get the source's inner radius.
    /// \return The inner radius.
    public: float GetInnerRadius() const;

    /// \brief Get the source's falloff distance.
    /// \return The falloff distance.
    public: float GetFalloffDistance() const;

    /// \brief Get the source's volume level.
    /// \return The volume level.
    public: float GetVolumeLevel() const;

    /// \brief The source ID.
    private: unsigned int id;

    /// \brief The source position, stored as (x,y,z).
    private: ignition::math::Vector3d position;

    /// \brief The attenuation function.
    private: AttenuationFunction attenuationFunc;

    /// \brief The attenuation shape.
    private: AttenuationShape attenuationShape;

    /// \brief The inner radius.
    ///   Must be >= 0.0.
    ///   Volume emitted from this source will be volumeLevel
    ///   at locations in the environment that are of
    ///   distance <= innerRadius away from the source.
    private: float innerRadius;

    /// \brief The falloff distance.
    ///   The value of falloffDistance must be > innerRadius.
    ///   Volume emitted from this source will be 0 at locations
    ///   in the environment that are of distance >= falloffDistance
    ///   away from the source.
    private: float falloffDistance;

    /// \brief The volume level emitted from the source.
    ///   This must be a value between 0.0 and 1.0
    ///   (representing 0% and 100%).
    private: float volumeLevel;

    /// \brief Whether the source is playing audio or not.
    private: bool playing;
  };
}
}
}
}
}

#endif
