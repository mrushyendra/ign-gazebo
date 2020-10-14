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

#include "LogicalAudioSensorPlugin.hh"

#include <string>
#include <vector>

#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>

#include "Microphone.hh"
#include "Source.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::LogicalAudioSensorPluginPrivate
{
  /// \brief Creates an audio source with attributes specified in an SDF file.
  /// \param[in] _elem A pointer to the source element in the SDF file.
  public: void CreateAudioSource(const sdf::ElementPtr &_elem);

  /// \brief Creates a microphone with attributes specified in an SDF file.
  /// \param[in] _elem A pointer to the microphone element in the SDF file.
  public: void CreateMicrophone(const sdf::ElementPtr &_elem);

  /// \brief The audio sources in the environment
  public: std::vector<logical_audio::Source> sources;

  /// \brief The microphones in the environment
  public: std::vector<logical_audio::Microphone> microphones;
};

//////////////////////////////////////////////////
LogicalAudioSensorPlugin::LogicalAudioSensorPlugin()
  : System(), dataPtr(std::make_unique<LogicalAudioSensorPluginPrivate>())
{
}

//////////////////////////////////////////////////
LogicalAudioSensorPlugin::~LogicalAudioSensorPlugin()
{
  // TODO(adlarkin) fill this in
}

//////////////////////////////////////////////////
void LogicalAudioSensorPlugin::Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr)
{
  const std::string kSource = "source";
  const std::string kMicrophone = "microphone";

  const auto sdfClone = _sdf->Clone();

  if (sdfClone->HasElement(kSource))
  {
    for (auto sourceElem = sdfClone->GetElement(kSource); sourceElem;
          sourceElem = sourceElem->GetNextElement(kSource))
    {
      this->dataPtr->CreateAudioSource(sourceElem);
    }
  }

  if (sdfClone->HasElement(kMicrophone))
  {
    for (auto micElem = sdfClone->GetElement(kMicrophone); micElem;
          micElem = micElem->GetNextElement(kMicrophone))
    {
      this->dataPtr->CreateMicrophone(micElem);
    }
  }
}

//////////////////////////////////////////////////
void LogicalAudioSensorPlugin::PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm)
{
  // TODO(adlarkin) fill this in
}

//////////////////////////////////////////////////
void LogicalAudioSensorPluginPrivate::CreateAudioSource(
    const sdf::ElementPtr &_elem)
{
  static const std::string kSourceSkipMsg =
    "Skipping the creation of this source.\n";

  if (!_elem->HasElement("id"))
  {
    ignerr << "Audio source is missing an id. " << kSourceSkipMsg;
    return;
  }
  const auto id = _elem->Get<unsigned int>("id");

  if (!_elem->HasElement("position"))
  {
    ignerr << "Audio source is missing a position. " << kSourceSkipMsg;
    return;
  }
  const auto position = _elem->Get<ignition::math::Vector3d>("position");

  if (!_elem->HasElement("attenuationfunction"))
  {
    ignerr << "Audio source has no attenuation function. " << kSourceSkipMsg;
    return;
  }
  const auto attenuationFunc = _elem->Get<std::string>("attenuationfunction");

  if (!_elem->HasElement("attenuationshape"))
  {
    ignerr << "Audio source has no attenuation shape. " << kSourceSkipMsg;
    return;
  }
  const auto attenuationShape = _elem->Get<std::string>("attenuationshape");

  if (!_elem->HasElement("innerradius"))
  {
    ignerr << "Audio source has no inner radius. " << kSourceSkipMsg;
    return;
  }
  const auto innerRadius = _elem->Get<float>("innerRadius");

  if (!_elem->HasElement("falloffdistance"))
  {
    ignerr << "Audio source is missing a falloff distance. " << kSourceSkipMsg;
    return;
  }
  const auto falloffDistance = _elem->Get<float>("falloffdistance");

  if (!_elem->HasElement("volumelevel"))
  {
    ignerr << "Audio source is missing a volume level. " << kSourceSkipMsg;
    return;
  }
  const auto volumeLevel = _elem->Get<float>("volumelevel");

  if (!_elem->HasElement("playing"))
  {
    ignerr << "Audio source is missing the playing attribute. "
           << kSourceSkipMsg;
    return;
  }
  const auto playing = _elem->Get<bool>("playing");

  if (!_elem->HasElement("playduration"))
  {
    ignerr << "Audio source is missing the play duration. " << kSourceSkipMsg;
    return;
  }
  const auto playDuration = _elem->Get<unsigned int>("playduration");

  this->sources.push_back({id, position, attenuationFunc, attenuationShape,
      innerRadius, falloffDistance, volumeLevel, playing, playDuration});
}

//////////////////////////////////////////////////
void LogicalAudioSensorPluginPrivate::CreateMicrophone(
    const sdf::ElementPtr &_elem)
{
  static const std::string kMicSkipMsg =
    "Skipping the creation of this microphone.\n";

  if (!_elem->HasElement("id"))
  {
    ignerr << "Microphone is missing an id. " << kMicSkipMsg;
    return;
  }
  const auto id = _elem->Get<unsigned int>("id");

  if (!_elem->HasElement("position"))
  {
    ignerr << "Microphone is missing a position. " << kMicSkipMsg;
    return;
  }
  const auto position = _elem->Get<ignition::math::Vector3d>("position");

  if (!_elem->HasElement("volumethreshold"))
  {
    ignerr << "Microphone is missing a volume threshold. " << kMicSkipMsg;
    return;
  }
  const auto volumeDetectionThreshold = _elem->Get<float>("volumethreshold");

  this->microphones.push_back({id, position, volumeDetectionThreshold});
}

IGNITION_ADD_PLUGIN(LogicalAudioSensorPlugin,
                    ignition::gazebo::System,
                    LogicalAudioSensorPlugin::ISystemConfigure,
                    LogicalAudioSensorPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LogicalAudioSensorPlugin,
  "ignition::gazebo::systems::LogicalAudioSensorPlugin")
