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

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ignition/plugin/Register.hh>
#include <ignition/transport.hh>
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

  /// \brief Struct that contains information about a source's
  ///   current playing state.
  public: struct PlayInfo
          {
            /// \brief When the audio source most recently started
            ///   to play (in the context of sim time)
            std::chrono::steady_clock::duration startTime;

            /// \brief How long the audio source will play for
            ///   (seconds, or number of simulation steps)
            unsigned int playDuration;
          };

  /// \brief The audio sources in the environment.
  ///   The key is the ID of the source.
  public: std::unordered_map<unsigned int,
            std::pair<logical_audio::Source, PlayInfo>> sources;

  /// \brief The microphones in the environment
  public: std::vector<logical_audio::Microphone> microphones;

  /// \brief Whether the first simulation step has occurred or not.
  ///   This is needed in order to properly initialize the start time for
  ///   audio sources that begin playing at the start of simulation.
  public: bool firstStep{true};

  /// \brief Node used to handle the start source service
  public: ignition::transport::Node startSrvNode;

  /// \brief Node used to handle the stop source service
  public: ignition::transport::Node stopSrvNode;

  /// \brief A set of sources that should be started based on recent
  ///   service calls. The set contains source IDs.
  public: std::unordered_set<unsigned int> sourcesToStart;

  /// \brief Mutex to keep the sourcesToStart variable thread-safe
  public: std::mutex startSourcesMutex;

  /// \brief A set of sources that should be stopped based on recent
  ///   service calls. The set contains source IDs.
  public: std::unordered_set<unsigned int> sourcesToStop;

  /// \brief Mutex to keep the sourcesToStop variable thread-safe
  public: std::mutex stopSourcesMutex;
};

//////////////////////////////////////////////////
LogicalAudioSensorPlugin::LogicalAudioSensorPlugin()
  : System(), dataPtr(std::make_unique<LogicalAudioSensorPluginPrivate>())
{
  if (!this->dataPtr->startSrvNode.Advertise("/play_source",
        &LogicalAudioSensorPlugin::PlaySourceSrv, this))
    ignerr << "Error advertising the play source service\n";

  if (!this->dataPtr->stopSrvNode.Advertise("/stop_source",
        &LogicalAudioSensorPlugin::StopSourceSrv, this))
    ignerr << "Error advertising the stop source service\n";
}

//////////////////////////////////////////////////
LogicalAudioSensorPlugin::~LogicalAudioSensorPlugin()
{
}

//////////////////////////////////////////////////
void LogicalAudioSensorPlugin::Configure(const Entity &,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &,
                           EventManager &)
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
void LogicalAudioSensorPlugin::PreUpdate(const UpdateInfo &_info,
                EntityComponentManager &)
{
  auto startTime = _info.simTime - _info.dt;

  if (this->dataPtr->firstStep)
  {
    // save the initial playback start time for audio sources
    // as the initial time of the simulation
    for (auto & elem : this->dataPtr->sources)
    {
      auto& playInfo = elem.second.second;

      playInfo.startTime = startTime;

      // playInfo.playDuration is expressed as a number of simulation steps,
      // so we must multiply playInfo.playDuration by the simulation time step
      // in order to get an accurate duration comparison
      playInfo.playDuration *= _info.dt.count();
    }

    this->dataPtr->firstStep = false;
  }

  // were any sources requested to start playing through a service call?
  const std::lock_guard<std::mutex> lock(this->dataPtr->startSourcesMutex);
  for (const auto & id : this->dataPtr->sourcesToStart)
  {
    auto iter = this->dataPtr->sources.find(id);
    if (iter != this->dataPtr->sources.end())
    {
      iter->second.first.StartPlaying();
      iter->second.second.startTime = startTime;

      ignmsg << "started source " << id << " via service call\n";
    }
  }
  this->dataPtr->sourcesToStart.clear();
}

//////////////////////////////////////////////////
void LogicalAudioSensorPlugin::PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &)
{
  // see if any sources should stop playing here because of a service call
  std::unique_lock<std::mutex> lock(this->dataPtr->stopSourcesMutex);
  for (const auto & id : this->dataPtr->sourcesToStop)
  {
    auto iter = this->dataPtr->sources.find(id);
    if (iter != this->dataPtr->sources.end())
      iter->second.first.StopPlaying();
    ignmsg << "stopped source " << id << " via service call\n";
  }
  this->dataPtr->sourcesToStop.clear();
  lock.unlock();

  for (auto & elem : this->dataPtr->sources)
  {
    auto& source = elem.second.first;
    auto& playInfo = elem.second.second;

    // ignore this source if it isn't playing
    if (!source.IsPlaying())
      continue;
    // see if an audio source has played for its playing duration
    // (make sure the audio source doesn't have an infinite play duration)
    else if ((_info.simTime.count() - playInfo.startTime.count() >
             playInfo.playDuration) && (playInfo.playDuration > 0u))
    {
      ignmsg << "stopping source " << source.GetID() << "\n";
      source.StopPlaying();
    }
    // this audio source is still playing - can any microphones hear it?
    else
    {
      for (const auto & mic : this->dataPtr->microphones)
      {
        if (mic.Detect(source.VolumeLevel(mic.GetPosition())))
        {
          ignmsg << "microphone " << mic.GetID() <<
            " can hear source " << source.GetID() << "\n";
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void LogicalAudioSensorPlugin::PlaySourceSrv(
    const ignition::msgs::UInt64 &_req)
{
  const std::lock_guard<std::mutex> lock(this->dataPtr->startSourcesMutex);
  this->dataPtr->sourcesToStart.insert(_req.data());
}

//////////////////////////////////////////////////
void LogicalAudioSensorPlugin::StopSourceSrv(
    const ignition::msgs::UInt64 &_req)
{
  const std::lock_guard<std::mutex> lock(this->dataPtr->stopSourcesMutex);
  this->dataPtr->sourcesToStop.insert(_req.data());
}

//////////////////////////////////////////////////
void LogicalAudioSensorPluginPrivate::CreateAudioSource(
    const sdf::ElementPtr &_elem)
{
  static std::unordered_set<unsigned int> sourceIDs;

  static const std::string kSourceSkipMsg =
    "Skipping the creation of this source.\n";

  if (!_elem->HasElement("id"))
  {
    ignerr << "Audio source is missing an id. " << kSourceSkipMsg;
    return;
  }
  const auto id = _elem->Get<unsigned int>("id");

  // make sure no other sources exist with the same ID
  if (sourceIDs.find(id) != sourceIDs.end())
  {
    ignerr << "The specified source ID already exists for "
      << "another source. " << kSourceSkipMsg;
    return;
  }
  sourceIDs.insert(id);

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

  logical_audio::Source nextSource(id, position, attenuationFunc,
      attenuationShape, innerRadius, falloffDistance, volumeLevel, playing);
  PlayInfo playInfo;
  playInfo.playDuration = playDuration;

  this->sources.insert({id, {nextSource, playInfo}});
}

//////////////////////////////////////////////////
void LogicalAudioSensorPluginPrivate::CreateMicrophone(
    const sdf::ElementPtr &_elem)
{
  static std::unordered_set<unsigned int> microphoneIDs;

  static const std::string kMicSkipMsg =
    "Skipping the creation of this microphone.\n";

  if (!_elem->HasElement("id"))
  {
    ignerr << "Microphone is missing an id. " << kMicSkipMsg;
    return;
  }
  const auto id = _elem->Get<unsigned int>("id");

  // make sure no other microphones exist with the same ID
  if (microphoneIDs.find(id) != microphoneIDs.end())
  {
    ignerr << "The specified microphone ID already exists for "
      << "another microphone. " << kMicSkipMsg;
    return;
  }
  microphoneIDs.insert(id);

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
                    LogicalAudioSensorPlugin::ISystemPreUpdate,
                    LogicalAudioSensorPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LogicalAudioSensorPlugin,
  "ignition::gazebo::systems::LogicalAudioSensorPlugin")
