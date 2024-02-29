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

#ifndef RHCR_RVIZ_PLUGINS__CREATE_OBJECT_TOOL_HPP_
#define RHCR_RVIZ_PLUGINS__CREATE_OBJECT_TOOL_HPP_

#include <QObject>

#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <rviz_default_plugins/visibility_control.hpp>
#include <std_msgs/msg/string.hpp>

namespace rviz_common {
class DisplayContext;
namespace properties {
class StringProperty;
class VectorProperty;
class QosProfileProperty;
} // namespace properties
} // namespace rviz_common

namespace rhcr_rviz_plugins {

class RVIZ_DEFAULT_PLUGINS_PUBLIC CreateObjectTool : public rviz_default_plugins::tools::PoseTool {
  Q_OBJECT

public:
  CreateObjectTool();
  ~CreateObjectTool() override;

protected:
  void onInitialize() override;
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateScale();
  void updateTopic();

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rviz_common::properties::VectorProperty *scale_property_;
  rviz_common::properties::StringProperty *topic_property_;
  rviz_common::properties::QosProfileProperty *qos_profile_property_;
  Ogre::Vector3 scale_;

  rclcpp::QoS qos_profile_;
};

} // namespace rhcr_rviz_plugins

#endif