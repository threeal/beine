// Copyright (c) 2021 Alfi Maulana
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef BEINE_CPP__LEGS__LEGS_CONSUMER_HPP_
#define BEINE_CPP__LEGS__LEGS_CONSUMER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "../utility.hpp"

namespace beine_cpp
{

class LegsConsumer
{
public:
  inline LegsConsumer();
  inline explicit LegsConsumer(rclcpp::Node::SharedPtr node);

  inline void set_node(rclcpp::Node::SharedPtr node);

  inline rclcpp::Node::SharedPtr get_node();

  inline const Position & get_position();
  inline const Orientation & get_orientation();
  inline const Joints & get_joints();
  inline const std::string & get_command();

private:
  rclcpp::Node::SharedPtr node;

  rclcpp::Subscription<Position>::SharedPtr position_subscription;
  rclcpp::Subscription<Orientation>::SharedPtr orientation_subscription;
  rclcpp::Subscription<Joints>::SharedPtr joints_subscription;
  rclcpp::Subscription<StringMsg>::SharedPtr command_subscription;

  Position current_position;
  Orientation current_orientation;
  Joints current_joints;
  std::string current_command;
};

LegsConsumer::LegsConsumer()
{
}

LegsConsumer::LegsConsumer(rclcpp::Node::SharedPtr node)
: LegsConsumer()
{
  set_node(node);
}

void LegsConsumer::set_node(rclcpp::Node::SharedPtr node)
{
  // Initialize the node
  this->node = node;

  // Initialize the position subscription
  {
    position_subscription = get_node()->create_subscription<Position>(
      "/legs/position", 10,
      [this](const Position::SharedPtr position) {
        current_position = *position;
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Position subscription initialized on " <<
        position_subscription->get_topic_name() << "!");
  }

  // Initialize the orientation subscription
  {
    orientation_subscription = get_node()->create_subscription<Orientation>(
      "/legs/orientation", 10,
      [this](const Orientation::SharedPtr orientation) {
        current_orientation = *orientation;
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Orientation subscription initialized on " <<
        orientation_subscription->get_topic_name() << "!");
  }

  // Initialize the joints subscription
  {
    joints_subscription = get_node()->create_subscription<Joints>(
      "/legs/joints", 10,
      [this](const Joints::SharedPtr joints) {
        current_joints = *joints;
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Joints subscription initialized on " <<
        joints_subscription->get_topic_name() << "!");
  }

  // Initialize the command subscription
  {
    command_subscription = get_node()->create_subscription<StringMsg>(
      "/legs/command", 10,
      [this](const StringMsg::SharedPtr msg) {
        current_command = msg->data;
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Command subscription initialized on " <<
        command_subscription->get_topic_name() << "!");
  }
}

rclcpp::Node::SharedPtr LegsConsumer::get_node()
{
  return node;
}

const Position & LegsConsumer::get_position()
{
  return current_position;
}

const Orientation & LegsConsumer::get_orientation()
{
  return current_orientation;
}

const Joints & LegsConsumer::get_joints()
{
  return current_joints;
}

const std::string & LegsConsumer::get_command()
{
  return current_command;
}

}  // namespace beine_cpp

#endif  // BEINE_CPP__LEGS__LEGS_CONSUMER_HPP_
