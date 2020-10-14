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
    ///   The source's sound within inner radius distance away from
    ///   the source will have a volume of _volumeLevel.
    /// \param[in] _falloffDistance The falloff distance.
    ///   This value must be greater than the value of _innerRadius.
    ///   If a location in the environment has a distance that
    ///   is >= _falloffDistance away from the source, then the volume
    ///   at this location is 0.
    /// \param[in] _volumeLevel The volume emitted from the source.
    ///   This must be a value between 0.0 and 1.0
    ///   (representing 0% to 100%).
    /// \param[in] _playing Whether the source is playing by default.
    ///   Use true to start playing by default, and false if you'd like
    ///   to start playing at a later time.
    /// \param[in] _playDuration How long (in seconds) the source will
    ///   play audio for. This value must be >= 0.
    ///   A value of 0 means that the source will play for an infinite
    ///   amount of time.
    public: Source(const unsigned int _id,
                   const ignition::math::Vector3d &_position,
                   const AttenuationFunction _attenuationFunc,
                   const AttenuationShape _attenuationShape,
                   const float _innerRadius,
                   const float _falloffDistance,
                   const float _volumeLevel,
                   const bool _playing,
                   const unsigned int _playDuration);

    /// \brief Constructor.
    /// \param[in] _id The source's ID.
    /// \param[in] _position The source's position (x,y,z).
    /// \param[in] _attenuationFunc The attenuation function.
    /// \param[in] _attenuationShape The attenuation shape.
    /// \param[in] _innerRadius The inner radius.
    /// \param[in] _falloffDistance The falloff distance.
    /// \param[in] _volumeLevel The volume emitted from the source.
    /// \param[in] _playing Whether the source is playing by default.
    /// \param[in] _playDuration How long (in seconds) the source will
    ///   play audio for.
    /// \sa Source() for more information about the constructor parameters.
    public: Source(const unsigned int _id,
                   const ignition::math::Vector3d &_position,
                   const std::string &_attenuationFunc,
                   const std::string &_attenuationShape,
                   const float _innerRadius,
                   const float _falloffDistance,
                   const float _volumeLevel,
                   const bool _playing,
                   const unsigned int _playDuration);

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
    public: void SetAttenuationFunction(const std::string &_attenuationFunc);

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
    public: void SetAttenuationShape(const std::string &_attenuationShape);

    /// \brief A map to help convert user-input strings to the proper
    ///   attenuation function.
    private: static const std::unordered_map<std::string, AttenuationFunction>
              kAttFuncMap;

    /// \brief A map to help convert user-input strings to the proper
    ///   attenuation shape.
    private: static const std::unordered_map<std::string, AttenuationShape>
              kAttShapeMap;

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

    /// \brief How long (in seconds) the source plays audio for.
    ///   This must be a value >= 0.
    ///   A value of 0 means infinite play time.
    private: unsigned int playDuration;
  };
}
}
}
}
}

#endif
