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

#include "Microphone.hh"

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

  //////////////////////////////////////////////////
  Microphone::Microphone(const unsigned int _id,
                         const ignition::math::Vector3d &_position,
                         const float _volumeDetectionThreshold) :
    id(_id),
    position(_position)
  {
    if (_volumeDetectionThreshold < 0.0f)
      this->volumeDetectionThreshold = 0.0f;
    else if (_volumeDetectionThreshold > 1.0f)
      this->volumeDetectionThreshold = 1.0f;
    else
      this->volumeDetectionThreshold = _volumeDetectionThreshold;
  }

  //////////////////////////////////////////////////
  bool Microphone::Detect(const float _volumeLevel) const
  {
    // if the volume level is <= 0, this can't be detected
    // (not even if this->volumeDetectionThreshold is 0.0)
    if (_volumeLevel < 0.00001f)
      return false;

    return _volumeLevel >= this->volumeDetectionThreshold;
  }

  //////////////////////////////////////////////////
  unsigned int Microphone::GetID() const
  {
    return this->id;
  }

  //////////////////////////////////////////////////
  ignition::math::Vector3d Microphone::GetPosition() const
  {
    return this->position;
  }

  //////////////////////////////////////////////////
  float Microphone::GetVolumeDetectionThreshold() const
  {
    return this->volumeDetectionThreshold;
  }

  //////////////////////////////////////////////////
  void Microphone::SetPosition(const ignition::math::Vector3d &_position)
  {
    this->position = _position;
  }
}
}
}
}
}
