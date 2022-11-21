/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name   flatland_viz.cpp
 * @brief  Manages the librviz viewport for flatland
 * @author Joseph Duchesne
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Avidbots Corp. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <OgreColourValue.h>

#include <QAction>
#include <QApplication>
#include <QCloseEvent>
#include <QDesktopServices>
#include <QDir>
#include <QDockWidget>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QShortcut>
#include <QStatusBar>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QToolButton>
#include <QUrl>

#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>

#include "flatland_viz/flatland_window.h"

#include "flatland_viz/flatland_viz.h"

// Constructor.
FlatlandViz::FlatlandViz(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, FlatlandWindow *parent)
    : QWidget((QWidget *) parent), parent_(parent), node_(rviz_ros_node) {}

void FlatlandViz::initialize() {
  auto manager = parent_->getManager();

  // Create interactive markers display
  interactive_markers_ =
      manager->createDisplay("rviz_default_plugins/InteractiveMarkers", "Move Objects", false);
  if (interactive_markers_ == nullptr) {
    RCLCPP_WARN(rclcpp::get_logger("flatland_viz"), "Interactive markers failed to instantiate");
    exit(1);
  }
  interactive_markers_->initialize(manager);
  // TODO fix this: Update Topic doesn't exist
  // Interactive Markers Namespace
  // Show Descriptions
  // Show Axes
  // Show Visual Aids
  // Enable Transparency
  //interactive_markers_->subProp("Update Topic")->setValue("/interactive_model_markers/update");

  // Subscribe to debug topics topic
  using std::placeholders::_1;
  debug_topic_subscriber_ =
      node_.lock()->get_raw_node()->create_subscription<flatland_msgs::msg::DebugTopicList>(
          "/flatland_server/debug/topics", 0, std::bind(&FlatlandViz::RecieveDebugTopics, this, _1));
}

void FlatlandViz::RecieveDebugTopics(const flatland_msgs::msg::DebugTopicList::SharedPtr msg) {
  std::vector<std::string> topics = msg->topics;

  // check for deleted topics
  for (auto &topic : debug_displays_) {
    if (std::count(topics.begin(), topics.end(), topic.first) == 0) {
      delete debug_displays_[topic.first];
      debug_displays_.erase(topic.first);
    }
  }

  // check for new topics
  for (const auto &topic : topics) {
    if (debug_displays_.count(topic) == 0) {
      // Create the marker display and set its topic
      debug_displays_[topic] = parent_->getManager()->createDisplay(
          "rviz_default_plugins/MarkerArray", QString::fromLocal8Bit(topic.c_str()), true);
      if (debug_displays_[topic] == nullptr) {
        RCLCPP_WARN(rclcpp::get_logger("flatland_viz"), "MarkerArray failed to instantiate");
        exit(1);
      }
      QString topic_qt = QString::fromLocal8Bit(
          (std::string("/flatland_server/debug/") + topic).c_str());
      debug_displays_[topic]->subProp("Marker Topic")->setValue(topic_qt);
    }
  }
}
