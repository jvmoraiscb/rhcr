// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "create_object_tool.hpp"

#include <memory>
#include <string>

#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/vector_property.hpp"

namespace rhcr_rviz_plugins {

CreateObjectTool::CreateObjectTool() : rviz_default_plugins::tools::PoseTool() {
  shortcut_key_ = 'g';

  // topic_property_ = new rviz_common::properties::StringProperty("Topic", "create_object", "DESCRIPTION TODO", getPropertyContainer(), SLOT(updateTopic()), this);
  scale_property_ = new rviz_common::properties::VectorProperty("Scale", Ogre::Vector3::ZERO, "DESCRIPTION TODO", getPropertyContainer(), SLOT(updateScale()), this);
}

CreateObjectTool::~CreateObjectTool() {
}

void CreateObjectTool::onInitialize() {
  PoseTool::onInitialize();
  setName("Create Object");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png"));
}

void CreateObjectTool::onPoseSet(double x, double y, double theta) {
  // Set goal pose on global object GoalUpdater to update nav2 Panel
  std::cout << x + y + theta;
}

void CreateObjectTool::updateScale() {
  std::cout << scale_property_->getVector();
}

} // namespace rhcr_rviz_plugins

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(rhcr_rviz_plugins::CreateObjectTool, rviz_common::Tool)