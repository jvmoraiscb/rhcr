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

#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>

namespace rhcr_rviz_plugins {

CreateObjectTool::CreateObjectTool() : rviz_default_plugins::tools::PoseTool(), qos_profile_(5) {
  scale_ = Ogre::Vector3(1, 1, 1);

  shortcut_key_ = 'g';

  topic_property_ = new rviz_common::properties::StringProperty("Topic", "/console", "DESCRIPTION TODO", getPropertyContainer(), SLOT(updateTopic()), this);

  scale_property_ = new rviz_common::properties::VectorProperty("Scale", scale_, "DESCRIPTION TODO", getPropertyContainer(), SLOT(updateScale()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(topic_property_, qos_profile_);
}

CreateObjectTool::~CreateObjectTool() {
}

void CreateObjectTool::onInitialize() {
  PoseTool::onInitialize();
  qos_profile_property_->initialize([this](rclcpp::QoS profile) { this->qos_profile_ = profile; });
  setName("Create Object");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/Marker.png"));
  updateTopic();
}

void CreateObjectTool::onPoseSet(double x, double y, double theta) {
  std::string command = std::string("virtual-map");
  std::string sub_command = std::string("create");
  std::string pos_x = std::to_string(x);
  std::string pos_y = std::to_string(y);
  std::string pos_z = std::to_string(0.0);
  std::string quat_x = std::to_string(orientationAroundZAxis(theta).x);
  std::string quat_y = std::to_string(orientationAroundZAxis(theta).y);
  std::string quat_z = std::to_string(orientationAroundZAxis(theta).z);
  std::string quat_w = std::to_string(orientationAroundZAxis(theta).w);
  std::string scale_x = std::to_string(scale_.x);
  std::string scale_y = std::to_string(scale_.y);
  std::string scale_z = std::to_string(scale_.z);

  std_msgs::msg::String msg;
  msg.data = command + " " + sub_command + " " + pos_x + " " + pos_y + " " + pos_z + " " + quat_x + " " + quat_y + " " + quat_z + " " + quat_w + " " + scale_x + " " + scale_y + " " + scale_z;
  publisher_->publish(msg);
}

void CreateObjectTool::updateTopic() {
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = raw_node->template create_publisher<std_msgs::msg::String>(topic_property_->getStdString(), qos_profile_);
}

void CreateObjectTool::updateScale() {
  scale_ = scale_property_->getVector();
}

} // namespace rhcr_rviz_plugins

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(rhcr_rviz_plugins::CreateObjectTool, rviz_common::Tool)