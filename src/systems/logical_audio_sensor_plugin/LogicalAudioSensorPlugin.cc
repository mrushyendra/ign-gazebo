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
#include <sstream>
#include <string>

#include <ignition/gazebo/components/LogicalAudio.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/Util.hh>
#include <sdf/Element.hh>
#include "LogicalAudio.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::LogicalAudioSensorPluginPrivate
{
  /// \brief Creates an audio source with attributes specified in an SDF file.
  /// \param[in] _elem A pointer to the source element in the SDF file.
  /// \param[in] _parent The source element's parent entity.
  /// \param[in] _ecm The simulation's EntityComponentManager.
  /// \param[in] _sdfEntityCreator An SdfEntityCreator.
  public: void CreateAudioSource(const sdf::ElementPtr &_elem,
              const Entity &_parent,
              EntityComponentManager &_ecm,
              SdfEntityCreator &_sdfEntityCreator);

  /// \brief Creates a microphone with attributes specified in an SDF file.
  /// \param[in] _elem A pointer to the microphone element in the SDF file.
  /// \param[in] _parent The microphone element's parent entity.
  /// \param[in] _ecm The simulation's EntityComponentManager.
  /// \param[in] _sdfEntityCreator An SdfEntityCreator.
  public: void CreateMicrophone(const sdf::ElementPtr &_elem,
              const Entity &_parent,
              EntityComponentManager &_ecm,
              SdfEntityCreator &_sdfEntityCreator);

  /// \brief Callback for a service call that can start playing an audio
  /// source. If the source specified in the service is already playing,
  /// nothing happens.
  /// \param[out] _resp The service response, which is unused.
  private: bool PlaySourceSrv(ignition::msgs::Empty &_resp);

  /// \brief Callback for a service call that can stop playing an audio
  /// source. If the source specified in the service is already stopped,
  /// nothing happens.
  /// \param[out] _resp The service response, which is unused.
  private: bool StopSourceSrv(ignition::msgs::Empty &_resp);

  /// \brief Checks if a source has exceeded its play duration.
  /// \param[in] _simTimeInfo Information about the current simulation time.
  /// \param[in] _sourcePlayInfo The source's playing information.
  /// \returns true if the source's play duration has been exceeded,
  /// false otherwise
  public: bool DurationExceeded(const UpdateInfo &_simTimeInfo,
               const logical_audio::SourcePlayInfo &_sourcePlayInfo);

  /// \brief Node used to create publishers and services
  public: ignition::transport::Node node;

  /// \brief Publishes microphone detection information
  public: ignition::transport::Node::Publisher micDetectionPub;

  /// \brief A flag used to initialize a source's playing information
  /// before starting simulation.
  public: bool firstTime{true};

  /// \brief A reference to the source entity that the plugin created.
  /// If no source was specified when configuring the plugin, the
  /// reference will be to a null entity.
  public: Entity sourceEntity{kNullEntity};

  /// \brief Whether or not the source (this->sourceEntity)
  /// should be played because of a service call.
  public: bool playSource{false};

  /// \brief A mutex used to ensure that the play source service call does
  /// not interfere with the source's state in the PreUpdate step.
  public: std::mutex playSourceMutex;

  /// \brief Whether or not the source (this->sourceEntity)
  /// should be stopped because of a service call.
  public: bool stopSource{false};

  /// \brief A mutex used to ensure that the stop source service call does
  /// not interfere with the source's state in the PreUpdate step.
  public: std::mutex stopSourceMutex;

  /// \brief A reference to the microphone entity that the plugin created.
  /// If no microphone was specified when configuring the plugin, the
  /// reference will be to a null entity.
  public: Entity micEntity{kNullEntity};
};

//////////////////////////////////////////////////
LogicalAudioSensorPlugin::LogicalAudioSensorPlugin()
  : System(), dataPtr(std::make_unique<LogicalAudioSensorPluginPrivate>())
{
}

//////////////////////////////////////////////////
LogicalAudioSensorPlugin::~LogicalAudioSensorPlugin()
{
}

//////////////////////////////////////////////////
void LogicalAudioSensorPlugin::Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr)
{
  const std::string kSource = "source";
  const std::string kMicrophone = "microphone";

  SdfEntityCreator sdfEntityCreator(_ecm, _eventMgr);

  const auto sdfClone = _sdf->Clone();

  if (sdfClone->HasElement(kSource))
  {
    auto sourceElem = sdfClone->GetElement(kSource);
    this->dataPtr->CreateAudioSource(sourceElem, _entity, _ecm,
        sdfEntityCreator);
  }
  else if (sdfClone->HasElement(kMicrophone))
  {
    auto micElem = sdfClone->GetElement(kMicrophone);
    this->dataPtr->CreateMicrophone(micElem, _entity, _ecm, sdfEntityCreator);
  }
}

//////////////////////////////////////////////////
void LogicalAudioSensorPlugin::PreUpdate(const UpdateInfo &_info,
                EntityComponentManager &_ecm)
{
  if (this->dataPtr->sourceEntity != kNullEntity)
  {
    auto startTime = _info.simTime - _info.dt;

    auto& playInfo = _ecm.Component<components::LogicalAudioSourcePlayInfo>(
        this->dataPtr->sourceEntity)->Data();

    // configure the source's play information before starting the simulation
    if (this->dataPtr->firstTime)
    {
      playInfo.startTime = startTime;
      this->dataPtr->firstTime = false;
    }

    // start playing a source if the play source service was called
    std::unique_lock<std::mutex> play_lock(this->dataPtr->playSourceMutex);
    if (this->dataPtr->playSource)
    {
      playInfo.playing = true;
      playInfo.startTime = startTime;
      this->dataPtr->playSource = false;
    }
    play_lock.unlock();

    // stop playing a source if the stop source service was called
    std::unique_lock<std::mutex> stop_lock(this->dataPtr->stopSourceMutex);
    if (this->dataPtr->stopSource)
    {
      playInfo.playing = false;
      this->dataPtr->stopSource = false;
    }
    stop_lock.unlock();

    // stop playing a source if the play duration has been exceeded
    if (this->dataPtr->DurationExceeded(_info, playInfo))
      playInfo.playing = false;
  }
}

//////////////////////////////////////////////////
void LogicalAudioSensorPlugin::PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm)
{
  // check to see which sources a microphone can hear
  if (this->dataPtr->micEntity != kNullEntity)
  {
    const auto micPose = worldPose(this->dataPtr->micEntity, _ecm);
    const auto micInfo = _ecm.Component<components::LogicalMicrophone>(
        this->dataPtr->micEntity)->Data();

    // get the current sim time so that it can be placed in the header
    // of microphone detection messages
    const auto simSeconds =
      std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
    const auto simNanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime);
    const auto nanosecondOffset = (simNanoseconds - simSeconds).count();

    _ecm.Each<components::LogicalAudioSource,
              components::LogicalAudioSourcePlayInfo>(
      [&](const Entity &_entity,
          const components::LogicalAudioSource *_source,
          const components::LogicalAudioSourcePlayInfo *_playInfo)
      {
        // skip this source if the playing duration has been exceeded
        // (this source will be stopped in the following PreUpdate call)
        if (this->dataPtr->DurationExceeded(_info, _playInfo->Data()))
          return true;

        const auto sourcePose = worldPose(_entity, _ecm);
        const auto vol = logical_audio::computeVolume(
            _playInfo->Data().playing,
            _source->Data().attFunc,
            _source->Data().attShape,
            _source->Data().emissionVolume,
            _source->Data().innerRadius,
            _source->Data().falloffDistance,
            sourcePose,
            micPose);

        if (logical_audio::detect(vol, micInfo.volumeDetectionThreshold))
        {
          // publish the source that the microphone heard, along with the
          // volume level the microphone detected. The detected source's
          // ID is embedded in the message's header
          ignition::msgs::Double msg;
          auto header = msg.mutable_header();
          auto timeStamp = header->mutable_stamp();
          timeStamp->set_sec(simSeconds.count());
          timeStamp->set_nsec(nanosecondOffset);
          auto headerData = header->add_data();
          headerData->set_key(std::to_string(_source->Data().id));
          msg.set_data(vol);

          this->dataPtr->micDetectionPub.Publish(msg);
        }

        return true;
      });
  }
}

//////////////////////////////////////////////////
void LogicalAudioSensorPluginPrivate::CreateAudioSource(
    const sdf::ElementPtr &_elem,
    const Entity &_parent,
    EntityComponentManager &_ecm,
    SdfEntityCreator &_sdfEntityCreator)
{
  static const std::string kSourceSkipMsg =
    "Skipping the creation of this source.\n";

  if (!_elem->HasElement("id"))
  {
    ignerr << "Audio source is missing an id. " << kSourceSkipMsg;
    return;
  }
  const auto id = _elem->Get<unsigned int>("id");

  // make sure no other sources exist with the same ID
  bool duplicate = false;
  _ecm.Each<components::LogicalAudioSource>(
    [&](const Entity &/*_entity*/,
        const components::LogicalAudioSource *_source)
    {
      if (_source->Data().id == id)
      {
        ignerr << "The specified source ID already exists for "
          << "another source. " << kSourceSkipMsg;
        duplicate = true;
        return false;
      }
      return true;
    });
  if (duplicate)
    return;

  if (!_elem->HasElement("pose"))
  {
    ignerr << "Audio source is missing a pose. " << kSourceSkipMsg;
    return;
  }
  const auto pose = _elem->Get<ignition::math::Pose3d>("pose");

  if (!_elem->HasElement("attenuation_function"))
  {
    ignerr << "Audio source has no attenuation function. " << kSourceSkipMsg;
    return;
  }
  const auto attenuationFunc = _elem->Get<std::string>("attenuation_function");

  if (!_elem->HasElement("attenuation_shape"))
  {
    ignerr << "Audio source has no attenuation shape. " << kSourceSkipMsg;
    return;
  }
  const auto attenuationShape = _elem->Get<std::string>("attenuation_shape");

  if (!_elem->HasElement("inner_radius"))
  {
    ignerr << "Audio source has no inner radius. " << kSourceSkipMsg;
    return;
  }
  const auto innerRadius = _elem->Get<double>("inner_radius");

  if (!_elem->HasElement("falloff_distance"))
  {
    ignerr << "Audio source is missing a falloff distance. " << kSourceSkipMsg;
    return;
  }
  const auto falloffDistance = _elem->Get<double>("falloff_distance");

  if (!_elem->HasElement("volume_level"))
  {
    ignerr << "Audio source is missing a volume level. " << kSourceSkipMsg;
    return;
  }
  const auto volumeLevel = _elem->Get<double>("volume_level");

  if (!_elem->HasElement("playing"))
  {
    ignerr << "Audio source is missing the playing attribute. "
           << kSourceSkipMsg;
    return;
  }
  const auto playing = _elem->Get<bool>("playing");

  if (!_elem->HasElement("play_duration"))
  {
    ignerr << "Audio source is missing the play duration. " << kSourceSkipMsg;
    return;
  }
  const auto playDuration = _elem->Get<unsigned int>("play_duration");

  // create services for this source
  std::stringstream ss;
  ss << "/audio_source_" << id << "/play";
  if (!this->node.Advertise(ss.str(),
        &LogicalAudioSensorPluginPrivate::PlaySourceSrv, this))
  {
    ignerr << "Error advertising the play source service for source "
      << id << ". " << kSourceSkipMsg;
    return;
  }
  ss.str("");
  ss.clear();
  ss << "/audio_source_" << id << "/stop";
  if (!this->node.Advertise(ss.str(),
        &LogicalAudioSensorPluginPrivate::StopSourceSrv, this))
  {
    ignerr << "Error advertising the stop source service for source "
      << id << ". " << kSourceSkipMsg;
    return;
  }

  // create an audio source entity
  auto entity = _ecm.CreateEntity();
  if (entity == kNullEntity)
  {
    ignerr << "Failed to create a logical audio source entity. "
      << kSourceSkipMsg;
    return;
  }
  _sdfEntityCreator.SetParent(entity, _parent);

  // save the audio source properties as a component
  logical_audio::Source source;
  source.id = id;
  logical_audio::setAttenuationFunction(source.attFunc, attenuationFunc);
  logical_audio::setAttenuationShape(source.attShape, attenuationShape);
  source.innerRadius = innerRadius;
  source.falloffDistance = falloffDistance;
  logical_audio::validateInnerRadiusAndFalloffDistance(
      source.innerRadius,
      source.falloffDistance);
  source.emissionVolume = volumeLevel;
  logical_audio::validateVolumeLevel(source.emissionVolume);
  _ecm.CreateComponent(entity, components::LogicalAudioSource(source));

  // save the source's pose as a component
  _ecm.CreateComponent(entity,
      components::Pose(pose));

  // save the source's playing information as a component
  logical_audio::SourcePlayInfo playInfo;
  playInfo.playing = playing;
  playInfo.playDuration = std::chrono::seconds(playDuration);
  _ecm.CreateComponent(entity,
      components::LogicalAudioSourcePlayInfo(playInfo));

  this->sourceEntity = entity;
}

//////////////////////////////////////////////////
void LogicalAudioSensorPluginPrivate::CreateMicrophone(
    const sdf::ElementPtr &_elem,
    const Entity &_parent,
    EntityComponentManager &_ecm,
    SdfEntityCreator &_sdfEntityCreator)
{
  static const std::string kMicSkipMsg =
    "Skipping the creation of this microphone.\n";

  if (!_elem->HasElement("id"))
  {
    ignerr << "Microphone is missing an id. " << kMicSkipMsg;
    return;
  }
  const auto id = _elem->Get<unsigned int>("id");

  // make sure no other microphones exist with the same ID
  bool duplicate = false;
  _ecm.Each<components::LogicalMicrophone>(
    [&](const Entity &/*_entity*/,
        const components::LogicalMicrophone *_mic)
    {
      if (_mic->Data().id == id)
      {
        ignerr << "The specified microphone ID already exists for "
          << "another microphone. " << kMicSkipMsg;
        duplicate = true;
        return false;
      }
      return true;
    });
  if (duplicate)
    return;

  if (!_elem->HasElement("pose"))
  {
    ignerr << "Microphone is missing a pose. " << kMicSkipMsg;
    return;
  }
  const auto pose = _elem->Get<ignition::math::Pose3d>("pose");

  if (!_elem->HasElement("volume_threshold"))
  {
    ignerr << "Microphone is missing a volume threshold. " << kMicSkipMsg;
    return;
  }
  const auto volumeDetectionThreshold = _elem->Get<double>("volume_threshold");

  // create the detection publisher for this microphone
  std::stringstream ss;
  ss << "/mic_" << id << "/detection";
  this->micDetectionPub =
    this->node.Advertise<ignition::msgs::Double>(ss.str());
  if (!this->micDetectionPub)
  {
    ignerr << "Error creating a detection publisher for microphone "
      << id << ". " << kMicSkipMsg;
    return;
  }

  // create a microphone entity
  auto entity = _ecm.CreateEntity();
  if (entity == kNullEntity)
  {
    ignerr << "Failed to create a logical audio microphone entity. "
      << kMicSkipMsg;
    return;
  }
  _sdfEntityCreator.SetParent(entity, _parent);

  // save the microphone properties as a component
  logical_audio::Microphone microphone;
  microphone.id = id;
  microphone.volumeDetectionThreshold = volumeDetectionThreshold;
  _ecm.CreateComponent(entity, components::LogicalMicrophone(microphone));

  // save the microphone's pose as a component
  _ecm.CreateComponent(entity,
      components::Pose(pose));

  this->micEntity = entity;
}

//////////////////////////////////////////////////
bool LogicalAudioSensorPluginPrivate::PlaySourceSrv(
    ignition::msgs::Empty &_resp)
{
  std::lock_guard<std::mutex> lock(this->playSourceMutex);
  this->playSource = true;
  _resp.set_unused(true);
  return true;
}

//////////////////////////////////////////////////
bool LogicalAudioSensorPluginPrivate::StopSourceSrv(
    ignition::msgs::Empty &_resp)
{
  std::lock_guard<std::mutex> lock(this->stopSourceMutex);
  this->stopSource = true;
  _resp.set_unused(true);
  return true;
}

//////////////////////////////////////////////////
bool LogicalAudioSensorPluginPrivate::DurationExceeded(
    const UpdateInfo &_simTimeInfo,
    const logical_audio::SourcePlayInfo &_sourcePlayInfo)
{
  auto currDuration = _simTimeInfo.simTime - _sourcePlayInfo.startTime;

  // make sure the source doesn't have an infinite play duration
  if ((_sourcePlayInfo.playDuration.count() > 0) &&
      (currDuration > _sourcePlayInfo.playDuration))
    return true;

  return false;
}

IGNITION_ADD_PLUGIN(LogicalAudioSensorPlugin,
                    ignition::gazebo::System,
                    LogicalAudioSensorPlugin::ISystemConfigure,
                    LogicalAudioSensorPlugin::ISystemPreUpdate,
                    LogicalAudioSensorPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LogicalAudioSensorPlugin,
  "ignition::gazebo::systems::LogicalAudioSensorPlugin")
