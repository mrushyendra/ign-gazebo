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

#include "Microphone.hh"

using Microphone = ignition::gazebo::systems::logical_audio::Microphone;

//////////////////////////////////////////////////
TEST(MicrophoneTest, Constructor)
{
  ignition::math::Vector3d vec(0.0, 0.0, 0.0);

  // test a valid volume detection threshold value (>= 0.0 and <= 1.0)
  Microphone m(0u, {0.0, 0.0, 0.0}, 0.5f);
  EXPECT_EQ(0u, m.GetID());
  EXPECT_EQ(vec, m.GetPosition());
  EXPECT_FLOAT_EQ(0.5f, m.GetVolumeDetectionThreshold());

  // test a volume detection threshold that's < 0.0
  Microphone m2(1u, {0.0, 0.0, 0.0}, -1.0f);
  EXPECT_EQ(1u, m2.GetID());
  EXPECT_EQ(vec, m2.GetPosition());
  EXPECT_FLOAT_EQ(0.0f, m2.GetVolumeDetectionThreshold());

  // test a volume detection threshold that's > 0.0
  Microphone m3(2u, {0.0, 0.0, 0.0}, 2.0f);
  EXPECT_EQ(2u, m3.GetID());
  EXPECT_EQ(vec, m3.GetPosition());
  EXPECT_FLOAT_EQ(1.0f, m3.GetVolumeDetectionThreshold());

  ignition::math::Vector3d vec2(0.0, 1.0, 2.0);

  // test the minimum volume dectection threshold (0.0)
  Microphone m4(3u, {0.0, 1.0, 2.0}, 0.0f);
  EXPECT_EQ(3u, m4.GetID());
  EXPECT_EQ(vec2, m4.GetPosition());
  EXPECT_FLOAT_EQ(0.0f, m4.GetVolumeDetectionThreshold());

  // test the maximum volume detection threshold (1.0)
  Microphone m5(4u, {0.0, 1.0, 2.0}, 1.0f);
  EXPECT_EQ(4u, m5.GetID());
  EXPECT_EQ(vec2, m5.GetPosition());
  EXPECT_FLOAT_EQ(1.0f, m5.GetVolumeDetectionThreshold());
}

//////////////////////////////////////////////////
TEST(MicrophoneTest, Setters)
{
  Microphone m(1u, {0.0, 0.0, 0.0}, 0.5f);
  EXPECT_EQ(ignition::math::Vector3d(0.0, 0.0, 0.0), m.GetPosition());
  m.SetPosition({1.0, 1.0, 1.0});
  EXPECT_EQ(ignition::math::Vector3d(1.0, 1.0, 1.0), m.GetPosition());
}

//////////////////////////////////////////////////
TEST(MicrophoneTest, AudioDetection)
{
  // test detection for a threshold within the [0.0, 1.0] range
  Microphone m(1u, {0.0, 0.0, 0.0}, 0.5f);
  EXPECT_FLOAT_EQ(0.5f, m.GetVolumeDetectionThreshold());
  EXPECT_EQ(true, m.Detect(1.0f));
  EXPECT_EQ(true, m.Detect(2.0f));
  EXPECT_EQ(true, m.Detect(0.5f));
  EXPECT_EQ(false, m.Detect(0.4f));
  EXPECT_EQ(false, m.Detect(0.0f));
  EXPECT_EQ(false, m.Detect(-1.0f));

  // test detection for a minimum threshold (0.0)
  Microphone m2(1u, {0.0, 0.0, 0.0}, 0.0f);
  EXPECT_FLOAT_EQ(0.0f, m2.GetVolumeDetectionThreshold());
  EXPECT_EQ(true, m2.Detect(1.0f));
  EXPECT_EQ(true, m2.Detect(0.2f));
  EXPECT_EQ(false, m2.Detect(0.0f));
  EXPECT_EQ(false, m2.Detect(-0.5f));

  // test detection for a maximum threshold (1.0)
  Microphone m3(2u, {0.0, 0.0, 0.0}, 1.0f);
  EXPECT_FLOAT_EQ(1.0f, m3.GetVolumeDetectionThreshold());
  EXPECT_EQ(true, m3.Detect(1.0f));
  EXPECT_EQ(true, m3.Detect(2.0f));
  EXPECT_EQ(false, m3.Detect(0.75f));
  EXPECT_EQ(false, m3.Detect(0.0f));
}
