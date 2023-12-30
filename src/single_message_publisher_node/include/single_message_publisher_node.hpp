// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 M. Fatih Cırıt
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

#ifndef SINGLE_MESSAGE_PUBLISHER_SINGLE_MESSAGE_PUBLISHER_NODE_HPP_
#define SINGLE_MESSAGE_PUBLISHER_SINGLE_MESSAGE_PUBLISHER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace single_message_publisher
{
class SingleMessagePublisherNode : public rclcpp::Node
{
public:
  explicit SingleMessagePublisherNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_{};

  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_{};
};

}  // namespace single_message_publisher

#endif  // SINGLE_MESSAGE_PUBLISHER_SINGLE_MESSAGE_PUBLISHER_NODE_HPP_
