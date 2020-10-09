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

#include "Source.hh"

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

  const std::unordered_map<std::string, AttenuationFunction>
    Source::kAttFuncMap {
      {"linear", AttenuationFunction::Linear}};

  const std::unordered_map<std::string, AttenuationShape>
    Source::kAttShapeMap {
      {"sphere", AttenuationShape::Sphere}};

  //////////////////////////////////////////////////
  Source::Source(const unsigned int _id,
                 const ignition::math::Vector3f &_position,
                 const AttenuationFunction _attenuationFunc,
                 const AttenuationShape _attenuationShape,
                 const float _innerRadius,
                 const float _falloffDistance,
                 const float _volumeLevel,
                 const bool _playing,
                 const unsigned int _playDuration) :
    id(_id),
    position(_position),
    attenuationFunc(_attenuationFunc),
    attenuationShape(_attenuationShape),
    innerRadius(_innerRadius),
    falloffDistance(_falloffDistance),
    volumeLevel(_volumeLevel),
    playing(_playing),
    playDuration(_playDuration)
  {
  }

}
}
}
}
}
