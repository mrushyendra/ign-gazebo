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

#include "Source.hh"

namespace logical_audio = ignition::gazebo::systems::logical_audio;

//////////////////////////////////////////////////
TEST(SourceTest, Constructor)
{
  // test the constructor that uses logical_audio::attenuationfunction
  // and logical_audio::AttenuationShape parameters
  logical_audio::Source s(1u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Linear,
      logical_audio::AttenuationShape::Sphere,
      1.0f,
      2.0f,
      0.5f,
      true);
  EXPECT_EQ(1u, s.GetID());
  EXPECT_EQ(ignition::math::Vector3d(0.0, 0.0, 0.0), s.GetPosition());
  EXPECT_EQ(logical_audio::AttenuationFunction::Linear,
      s.GetAttenuationFunction());
  EXPECT_EQ(logical_audio::AttenuationShape::Sphere, s.GetAttenuationShape());
  EXPECT_FLOAT_EQ(1.0f, s.GetInnerRadius());
  EXPECT_FLOAT_EQ(2.0f, s.GetFalloffDistance());
  EXPECT_FLOAT_EQ(0.5f, s.GetVolumeLevel());
  EXPECT_EQ(true, s.IsPlaying());

  // test the constructor that uses valid lowercase strings to define
  // the attenuation function and shape
  logical_audio::Source s2(2u,
      {0.0, 1.0, 0.0},
      "linear",
      "sphere",
      2.0f,
      4.0f,
      0.8f,
      false);
  EXPECT_EQ(2u, s2.GetID());
  EXPECT_EQ(ignition::math::Vector3d(0.0, 1.0, 0.0), s2.GetPosition());
  EXPECT_EQ(logical_audio::AttenuationFunction::Linear,
      s2.GetAttenuationFunction());
  EXPECT_EQ(logical_audio::AttenuationShape::Sphere, s2.GetAttenuationShape());
  EXPECT_FLOAT_EQ(2.0f, s2.GetInnerRadius());
  EXPECT_FLOAT_EQ(4.0f, s2.GetFalloffDistance());
  EXPECT_FLOAT_EQ(0.8f, s2.GetVolumeLevel());
  EXPECT_EQ(false, s2.IsPlaying());

  // test the constructor that uses valid uppercase strings to define
  // the attenuation function and shape
  logical_audio::Source s3(3u,
      {0.0, 1.0, 1.0},
      "LINEAR",
      "SPHERE",
      3.0f,
      4.0f,
      0.1f,
      false);
  EXPECT_EQ(logical_audio::AttenuationFunction::Linear,
      s3.GetAttenuationFunction());
  EXPECT_EQ(logical_audio::AttenuationShape::Sphere, s3.GetAttenuationShape());

  // test the constructor that uses valid mixed case strings to define
  // the attenuation function and shape
  logical_audio::Source s4(2u,
      {0.0, 1.0, 0.0},
      "LiNeaR",
      "Sphere",
      2.0f,
      4.0f,
      0.8f,
      false);
  EXPECT_EQ(logical_audio::AttenuationFunction::Linear,
      s4.GetAttenuationFunction());
  EXPECT_EQ(logical_audio::AttenuationShape::Sphere, s4.GetAttenuationShape());

  // test the constructor that uses strings for attenuation function and shape,
  // but provides invalid strings
  logical_audio::Source s5(2u,
      {0.0, 1.0, 0.0},
      "linear ",
      "somethingRandom",
      2.0f,
      4.0f,
      0.8f,
      false);
  EXPECT_EQ(logical_audio::AttenuationFunction::Undefined,
      s5.GetAttenuationFunction());
  EXPECT_EQ(logical_audio::AttenuationShape::Undefined,
      s5.GetAttenuationShape());

  // test an inner radius < 0
  logical_audio::Source s6(1u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Linear,
      logical_audio::AttenuationShape::Sphere,
      -1.0f,
      2.0f,
      0.5f,
      true);
  EXPECT_FLOAT_EQ(0.0f, s6.GetInnerRadius());
  logical_audio::Source s7(2u,
      {0.0, 1.0, 0.0},
      "linear",
      "sphere",
      -2.0f,
      4.0f,
      0.8f,
      false);
  EXPECT_FLOAT_EQ(0.0f, s7.GetInnerRadius());

  // test a falloff volume < inner radius
  logical_audio::Source s8(1u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Linear,
      logical_audio::AttenuationShape::Sphere,
      1.0f,
      0.0f,
      0.5f,
      true);
  EXPECT_FLOAT_EQ(2.0f, s8.GetFalloffDistance());
  logical_audio::Source s9(2u,
      {0.0, 1.0, 0.0},
      "linear",
      "sphere",
      2.0f,
      1.0f,
      0.8f,
      false);
  EXPECT_FLOAT_EQ(3.0f, s9.GetFalloffDistance());

  // test a falloff volume == inner radius
  logical_audio::Source s10(1u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Linear,
      logical_audio::AttenuationShape::Sphere,
      1.0f,
      1.0f,
      0.5f,
      true);
  EXPECT_FLOAT_EQ(2.0f, s10.GetFalloffDistance());
  logical_audio::Source s11(2u,
      {0.0, 1.0, 0.0},
      "linear",
      "sphere",
      2.0f,
      2.0f,
      0.8f,
      false);
  EXPECT_FLOAT_EQ(3.0f, s11.GetFalloffDistance());

  // test a volume level > 1.0
  logical_audio::Source s12(1u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Linear,
      logical_audio::AttenuationShape::Sphere,
      1.0f,
      1.0f,
      1.5f,
      true);
  EXPECT_FLOAT_EQ(1.0f, s12.GetVolumeLevel());
  logical_audio::Source s13(2u,
      {0.0, 1.0, 0.0},
      "linear",
      "sphere",
      2.0f,
      2.0f,
      1.8f,
      false);
  EXPECT_FLOAT_EQ(1.0f, s13.GetVolumeLevel());

  // test a volume level < 0.0
  logical_audio::Source s14(1u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Linear,
      logical_audio::AttenuationShape::Sphere,
      1.0f,
      1.0f,
      -1.5f,
      true);
  EXPECT_FLOAT_EQ(0.0f, s14.GetVolumeLevel());
  logical_audio::Source s15(2u,
      {0.0, 1.0, 0.0},
      "linear",
      "sphere",
      2.0f,
      2.0f,
      -1.8f,
      false);
  EXPECT_FLOAT_EQ(0.0f, s15.GetVolumeLevel());
}

//////////////////////////////////////////////////
TEST(SourceTest, Setters)
{
  logical_audio::Source s(0u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Linear,
      logical_audio::AttenuationShape::Sphere,
      1.0f,
      2.0f,
      0.5f,
      false);

  EXPECT_EQ(ignition::math::Vector3d(0.0, 0.0, 0.0), s.GetPosition());
  s.SetPosition({1.0, 1.0, 1.0});
  EXPECT_EQ(ignition::math::Vector3d(1.0, 1.0, 1.0), s.GetPosition());

  EXPECT_FLOAT_EQ(0.5f, s.GetVolumeLevel());
  // set the volume to a value in the [0.0, 1.0] range
  s.SetVolumeLevel(0.8f);
  EXPECT_FLOAT_EQ(0.8f, s.GetVolumeLevel());
  // set the volume to a value above the [0.0, 1.0] range
  s.SetVolumeLevel(1.5f);
  EXPECT_FLOAT_EQ(1.0f, s.GetVolumeLevel());
  // set the volume to a value below the [0.0, 1.0] range
  s.SetVolumeLevel(-1.0f);
  EXPECT_FLOAT_EQ(0.0f, s.GetVolumeLevel());

  EXPECT_EQ(false, s.IsPlaying());
  s.StartPlaying();
  EXPECT_EQ(true, s.IsPlaying());
  s.StopPlaying();
  EXPECT_EQ(false, s.IsPlaying());

  // play a source that is already playing
  logical_audio::Source s2(0u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Linear,
      logical_audio::AttenuationShape::Sphere,
      1.0f,
      2.0f,
      0.5f,
      true);
  EXPECT_EQ(true, s2.IsPlaying());
  s2.StartPlaying();
  EXPECT_EQ(true, s2.IsPlaying());

  // stop a source that is already stopped
  logical_audio::Source s3(0u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Linear,
      logical_audio::AttenuationShape::Sphere,
      1.0f,
      2.0f,
      0.5f,
      false);
  EXPECT_EQ(false, s3.IsPlaying());
  s3.StopPlaying();
  EXPECT_EQ(false, s3.IsPlaying());
}

//////////////////////////////////////////////////
TEST(SourceTest, VolumeCalculation)
{
  logical_audio::Source s(0u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Linear,
      logical_audio::AttenuationShape::Sphere,
      1.0f,
      11.0f,
      1.0f,
      false);

  // make sure no volume is emitted if the source isn't playing
  EXPECT_EQ(false, s.IsPlaying());
  EXPECT_FLOAT_EQ(0.0f, s.VolumeLevel({5.0, 5.0, 5.0}));

  s.StartPlaying();
  EXPECT_EQ(true, s.IsPlaying());
  // check volume inside of the inner radius
  EXPECT_FLOAT_EQ(s.GetVolumeLevel(), s.VolumeLevel({0.5, 0.5, 0.5}));
  EXPECT_FLOAT_EQ(s.GetVolumeLevel(), s.VolumeLevel({0.0, 0.0, 0.0}));
  // check volume at the inner radius
  EXPECT_FLOAT_EQ(s.GetVolumeLevel(), s.VolumeLevel({1.0, 0.0, 0.0}));
  // check volume at falloff distance
  EXPECT_FLOAT_EQ(0.0f, s.VolumeLevel({11.0, 0.0, 0.0}));
  // check volume past falloff distance
  EXPECT_FLOAT_EQ(0.0f, s.VolumeLevel({20.0f, 20.0f, 20.0f}));
  // check volume between inner radius and falloff distance
  EXPECT_FLOAT_EQ(0.9f, s.VolumeLevel({2.0f, 0.0f, 0.0f}));
  EXPECT_FLOAT_EQ(0.5f, s.VolumeLevel({6.0f, 0.0f, 0.0f}));
  EXPECT_FLOAT_EQ(0.1f, s.VolumeLevel({10.0f, 0.0f, 0.0f}));

  // make sure a source that is playing can be stopped
  EXPECT_EQ(true, s.IsPlaying());
  s.StopPlaying();
  EXPECT_FLOAT_EQ(0.0f, s.VolumeLevel({0.0f, 0.0f, 0.0f}));

  // test a source with an undefined attenuation function
  logical_audio::Source s2(0u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Undefined,
      logical_audio::AttenuationShape::Sphere,
      1.0f,
      11.0f,
      1.0f,
      false);
  EXPECT_EQ(false, s2.IsPlaying());
  EXPECT_FLOAT_EQ(0.0f, s2.VolumeLevel({1.0, 1.0, 1.0}));
  s2.StartPlaying();
  EXPECT_EQ(true, s2.IsPlaying());
  EXPECT_FLOAT_EQ(-1.0f, s2.VolumeLevel({1.0, 1.0, 1.0}));

  // test a source with an undefined attenuation shape
  logical_audio::Source s3(0u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Linear,
      logical_audio::AttenuationShape::Undefined,
      1.0f,
      11.0f,
      1.0f,
      false);
  EXPECT_EQ(false, s3.IsPlaying());
  EXPECT_FLOAT_EQ(0.0f, s3.VolumeLevel({1.0, 1.0, 1.0}));
  s3.StartPlaying();
  EXPECT_EQ(true, s3.IsPlaying());
  EXPECT_FLOAT_EQ(-1.0f, s3.VolumeLevel({1.0, 1.0, 1.0}));

  // test a source with an emission volume of 0
  logical_audio::Source s4(0u,
      {0.0, 0.0, 0.0},
      logical_audio::AttenuationFunction::Linear,
      logical_audio::AttenuationShape::Sphere,
      1.0f,
      11.0f,
      0.0f,
      true);
  EXPECT_EQ(true, s4.IsPlaying());
  EXPECT_FLOAT_EQ(0.0f, s4.VolumeLevel({1.0, 0.0, 0.0}));
  s4.StopPlaying();
  EXPECT_EQ(false, s4.IsPlaying());
  EXPECT_FLOAT_EQ(0.0f, s4.VolumeLevel({1.0, 0.0, 0.0}));
}
