/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "delete_all_objects_tool.hpp"

#include <sstream>

#include <OgreVector.h>

#include "rclcpp/qos.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace rhcr_rviz_plugins {

DeleteAllObjectsTool::DeleteAllObjectsTool()
    : qos_profile_(5) {
  shortcut_key_ = 'u';

  topic_property_ = new rviz_common::properties::StringProperty(
      "Topic", "/console",
      "The topic on which to publish points.",
      getPropertyContainer(), SLOT(updateTopic()), this);

  auto_deactivate_property_ = new rviz_common::properties::BoolProperty(
      "Single click", true,
      "Switch away from this tool after one click.",
      getPropertyContainer(), SLOT(updateAutoDeactivate()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
      topic_property_, qos_profile_);
}

void DeleteAllObjectsTool::onInitialize() {
  hit_cursor_ = cursor_;
  std_cursor_ = rviz_common::getDefaultCursor();
  qos_profile_property_->initialize(
      [this](rclcpp::QoS profile) { this->qos_profile_ = profile; });
  setName("Delete All Objects");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/MarkerArray.png"));
  updateTopic();
}

void DeleteAllObjectsTool::activate() {}

void DeleteAllObjectsTool::deactivate() {}

void DeleteAllObjectsTool::updateTopic() {
  rclcpp::Node::SharedPtr raw_node =
      context_->getRosNodeAbstraction().lock()->get_raw_node();
  // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
  publisher_ = raw_node->template create_publisher<std_msgs::msg::String>(
      topic_property_->getStdString(), qos_profile_);
}

void DeleteAllObjectsTool::updateAutoDeactivate() {}

int DeleteAllObjectsTool::processMouseEvent(rviz_common::ViewportMouseEvent &event) {
  int flags = 0;

  Ogre::Vector3 position;
  bool success = context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, position);
  setCursor(success ? hit_cursor_ : std_cursor_);

  if (success) {
    setStatusForPosition(position);

    if (event.leftUp()) {
      publishPosition(position);

      if (auto_deactivate_property_->getBool()) {
        flags |= Finished;
      }
    }
  } else {
    setStatus("Move over an object to select the target point.");
  }

  return flags;
}

void DeleteAllObjectsTool::setStatusForPosition(const Ogre::Vector3 &position) {
  std::ostringstream s;
  s << "<b>Left-Click:</b> Select this point.";
  s.precision(3);
  s << " [" << position.x << "," << position.y << "," << position.z << "]";
  setStatus(s.str().c_str());
}

void DeleteAllObjectsTool::publishPosition(const Ogre::Vector3 &position) const {
  std::string command = std::string("virtual-map");
  std::string sub_command = std::string("delete-all");
  std::string pos_x = std::to_string(position.x);
  std::string pos_y = std::to_string(position.y);
  std::string pos_z = std::to_string(0.0);

  std_msgs::msg::String msg;
  msg.data = command + " " + sub_command;
  publisher_->publish(msg);
}

} // namespace rhcr_rviz_plugins

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(rhcr_rviz_plugins::DeleteAllObjectsTool, rviz_common::Tool)