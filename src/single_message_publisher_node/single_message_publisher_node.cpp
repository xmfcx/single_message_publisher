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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>


using namespace std::literals;
using namespace std::placeholders;

namespace single_message_publisher
{
SingleMessagePublisherNode::SingleMessagePublisherNode(const rclcpp::NodeOptions & node_options)
: Node("bench_point_cloud", node_options)
{
  const double rate = this->declare_parameter<double>("rate");
  const std::string path_bag = this->declare_parameter<std::string>("path_bag");
  const std::string topic_cloud = this->declare_parameter<std::string>("topic_cloud");


  rosbag2_cpp::Reader reader;
  try {
    reader.open(path_bag);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error opening bag file: " << e.what());
    rclcpp::shutdown();
    return;
  }


  const auto & topics = reader.get_metadata().topics_with_message_count;
  const auto iter_topic =
    std::find_if(topics.begin(), topics.end(), [&topic_cloud](const auto & topic) {
      return topic.topic_metadata.name == topic_cloud;
    });

  if (iter_topic == topics.end()) {
    throw std::runtime_error("Topic not found in the bag file.");
  }

  const size_t message_count = iter_topic->message_count;
  if (message_count < 1) {
    throw std::runtime_error("Bag doesn't contain any messages with the given topic.");
  }
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;

  while (reader.has_next()) {
    auto bag_message = reader.read_next();

    if (bag_message->topic_name == topic_cloud) {
      using rosbag2_cpp::converter_interfaces::SerializationFormatConverter;
      msg_cloud_ = std::make_unique<sensor_msgs::msg::PointCloud2>();

      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization.deserialize_message(&extracted_serialized_msg, &(*msg_cloud_));
      break;
    }
  }
  reader.close();

  pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_cloud, 1);


  const auto update_period_ns = rclcpp::Rate(rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&SingleMessagePublisherNode::on_timer, this));
}

void SingleMessagePublisherNode::on_timer()
{
  msg_cloud_->header.stamp = this->get_clock()->now();
  pub_cloud_->publish(*msg_cloud_);
}


}  // namespace single_message_publisher

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(single_message_publisher::SingleMessagePublisherNode)
