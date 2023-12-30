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

#include "include/single_message_publisher_node.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>


using namespace std::literals;
using namespace std::placeholders;

namespace single_message_publisher
{
SingleMessagePublisherNode::SingleMessagePublisherNode(const rclcpp::NodeOptions & node_options)
: Node("bench_point_cloud", node_options)
{
  pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("out/cloud", 1);
  double rate = this->declare_parameter<double>("rate");

  const auto update_period_ns = rclcpp::Rate(rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&SingleMessagePublisherNode::on_timer, this));
}

void SingleMessagePublisherNode::on_timer() {}

}  // namespace single_message_publisher

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(single_message_publisher::SingleMessagePublisherNode)
