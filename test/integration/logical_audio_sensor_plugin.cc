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

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <thread>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "ignition/gazebo/components/LogicalAudio.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/test_config.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"
#include "ignition/gazebo/Types.hh"

#include "../helpers/Relay.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Test LogicalAudio system plugin
class LogicalAudioTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

TEST_F(LogicalAudioTest, LogicalAudio)
{
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/logical_audio_sensor_plugin.sdf";
  serverConfig.SetSdfFile(sdfFile);

  // start server
  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // helper variables for checking the validity of the ECM
  const ignition::math::Pose3d sourcePose(0, 0, 0, 0, 0, 0);
  const auto zeroSeconds = std::chrono::seconds(0);
  const ignition::math::Pose3d micPose(0.5, 0, 0, 0, 0, 0);
  std::chrono::steady_clock::duration sourceStartTime;
  bool firstTime{true};

  // flags that verify the ECM was checked for a source and microphone
  bool checkedSource{false};
  bool checkedMic{false};

  // make a test system and check the ECM for a source and microphone
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const UpdateInfo &_info,
                             EntityComponentManager &/*_ecm*/)
      {
        if (firstTime)
          sourceStartTime = _info.simTime;
        firstTime = false;
      });
  testSystem.OnPostUpdate([&](const UpdateInfo &/*_info*/,
                              const EntityComponentManager &_ecm)
      {
        // make sure the source is stored correctly in the ECM
        _ecm.Each<components::LogicalAudioSource,
                  components::LogicalAudioSourcePlayInfo,
                  components::Pose>(
          [&](const Entity &/*_entity*/,
              const components::LogicalAudioSource *_source,
              const components::LogicalAudioSourcePlayInfo *_playInfo,
              const components::Pose *_pose)
          {
            EXPECT_EQ(_source->Data().id, 1u);
            EXPECT_EQ(_source->Data().attFunc,
                logical_audio::AttenuationFunction::LINEAR);
            EXPECT_EQ(_source->Data().attShape,
                logical_audio::AttenuationShape::SPHERE);
            EXPECT_DOUBLE_EQ(_source->Data().innerRadius, 3.0);
            EXPECT_DOUBLE_EQ(_source->Data().falloffDistance, 8.0);
            EXPECT_DOUBLE_EQ(_source->Data().emissionVolume, 0.9);

            EXPECT_TRUE(_playInfo->Data().playing);
            EXPECT_EQ(_playInfo->Data().playDuration, zeroSeconds);
            EXPECT_EQ(_playInfo->Data().startTime, sourceStartTime);

            EXPECT_EQ(_pose->Data(), sourcePose);

            checkedSource = true;
            return true;
          });

        // make sure the microphone is stored correctly in the ECM
        _ecm.Each<components::LogicalMicrophone,
                  components::Pose>(
          [&](const Entity &/*_entity*/,
              const components::LogicalMicrophone *_mic,
              const components::Pose *_pose)
          {
            EXPECT_EQ(_mic->Data().id, 2u);
            EXPECT_DOUBLE_EQ(_mic->Data().volumeDetectionThreshold, 0.1);

            EXPECT_EQ(_pose->Data(), micPose);

            checkedMic = true;
            return true;
          });
      });
  server.AddSystem(testSystem.systemPtr);

  // subscribe to the microphone detection topic
  bool received{false};
  msgs::Double msg;
  msg.Clear();
  std::function<void(const msgs::Double &)> cb =
      [&received, &msg](const msgs::Double &_msg)
      {
        // only need one message
        if (received)
          return;

        msg = _msg;
        received = true;
      };
  transport::Node node;
  auto subscribed = node.Subscribe(
      "/model/mic_model/sensor/mic_2/detection", cb);
  EXPECT_TRUE(subscribed);

  // make sure microphone detection occurred
  server.Run(true, 100, false);
  // (wait on ignition-transport for message to be received)
  for (auto sleep = 0; !received && sleep < 30; ++sleep)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(checkedSource);
  EXPECT_TRUE(checkedMic);
  EXPECT_TRUE(received);
  EXPECT_EQ(msg.header().data(0).key(),
      "world/logical_audio_sensor/model/source_model/sensor/source_1");

  // TODO(adlarkin) make sure microphone doesn't occur if source isn't playing
  checkedSource = false;
  checkedMic = false;
  received = false;
}
