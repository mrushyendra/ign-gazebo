/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_SYSTEMS_BREADCRUMBS_HH_
#define IGNITION_GAZEBO_SYSTEMS_BREADCRUMBS_HH_

#include <memory>
#include <vector>

#include <sdf/Element.hh>
#include <sdf/Root.hh>

#include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"
#include "ignition/gazebo/System.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  /// \brief A system for creating Breadcrumbs in the form of models that can
  /// get deployed/spawned at the location of the model to which this system is
  /// attached. Each breadcrumb is a complete sdf::Model. When deployed, the
  /// pose of the breadcrumb model is offset from the containing model by the
  /// pose specified in the <pose> element of the breadcrumb model. A name is
  /// generated for the breadcrumb by appending the current count of deployments
  /// to the name specified in the breadcrumb <model> element. The model
  /// specified in the <breadcrumb> parameter serves as a template for deploying
  /// multiple breadcrumbs of the same type. Including models from Fuel is
  /// accomplished by creating a <model> that includes the Fuel model using the
  /// <include> tag. See the example in examples/worlds/breadcrumbs.sdf.
  ///
  /// System Paramters
  ///
  /// `<topic>`: Custom topic to be used to deploy breadcrumbs. If topic is not
  /// set, the default topic with the following pattern would be used
  /// "/model/<model_name>/breadcrumbs/<breadcrumb_name>/deploy". The topic type
  /// is ignition.msgs.Empty
  /// `<max_deployments>`: The maximum number of times this breadcrumb can be
  /// deployed. Once this many are deployed, publishing on the deploy topic will
  /// have no effect. If a negative number is set, the maximun deployment will
  /// be unbounded.
  /// `<breadcrumb>`: This is the model used as a template for deploying
  /// breadcrumbs.
  class IGNITION_GAZEBO_VISIBLE Breadcrumbs
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: Breadcrumbs() = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Callback to deployment topic
    private: void OnDeploy(const msgs::Empty &_msg);

    /// \brief Set to true after initialization with valid parameters
    private: bool initialized{false};

    /// \brief Ignition communication node.
    private: transport::Node node;

    /// \brief Model interface
    private: Model model{kNullEntity};

    /// \brief World entity
    private: Entity worldEntity{kNullEntity};

    /// \brief Creator interface
    public: std::unique_ptr<SdfEntityCreator> creator{nullptr};

    /// \brief The number of deployments allowed for the model
    private: int maxDeployments{-1};

    /// \brief The current number of deployments
    private: int numDeployments{0};

    /// \brief sdf::Root of the model to be deployed
    private: sdf::Root modelRoot;

    /// \brief Pending commands
    private: std::vector<bool> pendingCmds;

    /// \brief Mutex to protect pending commands
    private: std::mutex pendingCmdsMutex;
  };
  }
}
}
}

#endif
