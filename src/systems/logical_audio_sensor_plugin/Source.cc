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

#include <algorithm>
#include <cctype>
#include <string>
#include <unordered_map>

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
                 const ignition::math::Vector3d &_position,
                 const AttenuationFunction _attenuationFunc,
                 const AttenuationShape _attenuationShape,
                 const float _innerRadius,
                 const float _falloffDistance,
                 const float _volumeLevel,
                 const bool _playing) :
    id(_id),
    position(_position),
    attenuationFunc(_attenuationFunc),
    attenuationShape(_attenuationShape),
    playing(_playing)
  {
    this->SetInnerRadius(_innerRadius);
    this->SetFalloffDistance(_falloffDistance);
    this->SetVolumeLevel(_volumeLevel);
  }

  //////////////////////////////////////////////////
  Source::Source(const unsigned int _id,
                 const ignition::math::Vector3d &_position,
                 const std::string &_attenuationFunc,
                 const std::string &_attenuationShape,
                 const float _innerRadius,
                 const float _falloffDistance,
                 const float _volumeLevel,
                 const bool _playing) :
    id(_id),
    position(_position),
    playing(_playing)
  {
    this->SetAttenuationShape(_attenuationShape);
    this->SetAttenuationFunction(_attenuationFunc);
    this->SetInnerRadius(_innerRadius);
    this->SetFalloffDistance(_falloffDistance);
    this->SetVolumeLevel(_volumeLevel);
  }

  //////////////////////////////////////////////////
  float Source::VolumeLevel(const ignition::math::Vector3d &_location) const
  {
    if (!this->IsPlaying())
      return 0.0f;

    // make sure the source has a valid attenuation function and shape
    if ((this->attenuationFunc == AttenuationFunction::Undefined) ||
        (this->attenuationShape == AttenuationShape::Undefined))
      return -1.0f;
    // make sure the audio source has a playing volume that's > 0
    else if (this->volumeLevel < 0.00001f)
      return 0.0f;

    auto dist = this->position.Distance(_location);

    // Implementing AttenuationShape::Sphere for now since that's
    // the only attenuation shape that's available
    if (dist <= this->innerRadius)
      return this->volumeLevel;
    else if (dist >= this->falloffDistance)
      return 0.0f;

    // Implementing AttenuationFunction::Linear for now since that's
    // the only attenuation function that's available.
    //
    // The equation below was calculated as follows:
    // Point slope formula is y - y_1 = m * (x - x_1), rewritten as:
    // y = (m * (x - x_1)) + y_1
    // The variables in the equation above are defined as:
    //    y = output volume, m = slope, x = distance(source, _location),
    //    x_1 = inner radius, y_1 = output volume of the source
    // The slope (m) is defined as delta_y / delta_x, where y is volume and x
    //    is position. We have two (x,y) points - y is the source's output
    //    volume when x is the inner radius, and y is 0 when x is the falloff
    //    distance - so we can calculate the slope using these two points.
    float m = -this->volumeLevel / (this->falloffDistance - this->innerRadius);
    return (m * (dist - this->innerRadius)) + this->volumeLevel;
  }

  //////////////////////////////////////////////////
  void Source::StartPlaying()
  {
    this->playing = true;
  }

  //////////////////////////////////////////////////
  void Source::StopPlaying()
  {
    this->playing = false;
  }

  //////////////////////////////////////////////////
  bool Source::IsPlaying() const
  {
    return this->playing;
  }

  //////////////////////////////////////////////////
  void Source::SetPosition(const ignition::math::Vector3d &_position)
  {
    this->position = _position;
  }

  //////////////////////////////////////////////////
  void Source::SetAttenuationFunction(std::string _attenuationFunc)
  {
    std::transform(_attenuationFunc.begin(), _attenuationFunc.end(),
        _attenuationFunc.begin(), ::tolower);

    auto iter = this->kAttFuncMap.find(_attenuationFunc);
    if (iter != this->kAttFuncMap.end())
      this->attenuationFunc = iter->second;
    else
      this->attenuationFunc = AttenuationFunction::Undefined;
  }

  //////////////////////////////////////////////////
  void Source::SetAttenuationShape(std::string _attenuationShape)
  {
    std::transform(_attenuationShape.begin(), _attenuationShape.end(),
        _attenuationShape.begin(), ::tolower);

    auto iter = this->kAttShapeMap.find(_attenuationShape);
    if (iter != this->kAttShapeMap.end())
      this->attenuationShape = iter->second;
    else
      this->attenuationShape = AttenuationShape::Undefined;
  }

  //////////////////////////////////////////////////
  void Source::SetInnerRadius(const float _innerRadius)
  {
    if (_innerRadius < 0.0f)
    {
      this->innerRadius = 0.0f;
      return;
    }

    this->innerRadius = _innerRadius;
  }

  //////////////////////////////////////////////////
  void Source::SetFalloffDistance(const float _falloffDistance)
  {
    if (_falloffDistance <= this->innerRadius)
    {
      this->falloffDistance = this->innerRadius + 1.0f;
      return;
    }

    this->falloffDistance = _falloffDistance;
  }

  //////////////////////////////////////////////////
  void Source::SetVolumeLevel(const float _volumeLevel)
  {
    if (_volumeLevel < 0.0f)
      this->volumeLevel = 0.0f;
    else if (_volumeLevel > 1.0f)
      this->volumeLevel = 1.0f;
    else
      this->volumeLevel = _volumeLevel;
  }

  //////////////////////////////////////////////////
  unsigned int Source::GetID() const
  {
    return this->id;
  }

  //////////////////////////////////////////////////
  ignition::math::Vector3d Source::GetPosition() const
  {
    return this->position;
  }

  //////////////////////////////////////////////////
  AttenuationFunction Source::GetAttenuationFunction() const
  {
    return this->attenuationFunc;
  }

  //////////////////////////////////////////////////
  AttenuationShape Source::GetAttenuationShape() const
  {
    return this->attenuationShape;
  }

  //////////////////////////////////////////////////
  float Source::GetInnerRadius() const
  {
    return this->innerRadius;
  }

  //////////////////////////////////////////////////
  float Source::GetFalloffDistance() const
  {
    return this->falloffDistance;
  }

  //////////////////////////////////////////////////
  float Source::GetVolumeLevel() const
  {
    return this->volumeLevel;
  }
}
}
}
}
}
