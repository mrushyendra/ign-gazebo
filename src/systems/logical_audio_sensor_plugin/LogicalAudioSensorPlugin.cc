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

#include <vector>

#include <ignition/plugin/Register.hh>

#include "Microphone.hh"
#include "Source.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace logical_audio;

class ignition::gazebo::systems::LogicalAudioSensorPluginPrivate
{
  /// \brief The audio sources in the environment
  public: std::vector<ignition::gazebo::systems::logical_audio::Source>
            sources;

  /// \brief The microphones in the environment
  public: std::vector<ignition::gazebo::systems::logical_audio::Microphone>
            microphones;
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
  // TODO(adlarkin) fill this in
}

//////////////////////////////////////////////////
void PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm)
{
  // TODO(adlarkin) fill this in
}

IGNITION_ADD_PLUGIN(LogicalAudioSensorPlugin,
                    ignition::gazebo::System,
                    LogicalAudioSensorPlugin::ISystemConfigure,
                    LogicalAudioSensorPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LogicalAudioSensorPlugin,
  "ignition::gazebo::systems::LogicalAudioSensorPlugin")
