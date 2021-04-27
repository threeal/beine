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

#ifndef BEINE_CPP__LEGS__LEGS_PROVIDER_HPP_
#define BEINE_CPP__LEGS__LEGS_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>

#include "../utility.hpp"

namespace beine_cpp
{

class LegsProvider
{
public:
  inline LegsProvider();
  inline explicit LegsProvider(rclcpp::Node::SharedPtr node);

  inline void set_node(rclcpp::Node::SharedPtr node);

  inline void set_position(const Position & position);
  inline void set_orientation(const Orientation & orientation);
  inline void set_joints(const Joints & joints);
  inline void set_command(const std::string & command);

  inline rclcpp::Node::SharedPtr get_node() const;

private:
  rclcpp::Node::SharedPtr node;

  rclcpp::Publisher<Position>::SharedPtr position_publisher;
  rclcpp::Publisher<Orientation>::SharedPtr orientation_publisher;
  rclcpp::Publisher<Joints>::SharedPtr joints_publisher;
  rclcpp::Publisher<StringMsg>::SharedPtr command_publisher;
};

LegsProvider::LegsProvider()
{
}

LegsProvider::LegsProvider(rclcpp::Node::SharedPtr node)
: LegsProvider()
{
  set_node(node);
}

void LegsProvider::set_node(rclcpp::Node::SharedPtr node)
{
  // Initialize the node
  this->node = node;

  // Initialize the position publisher
  {
    position_publisher = get_node()->create_publisher<Position>("/legs/position", 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Position publisher initialized on " <<
        position_publisher->get_topic_name() << "!");
  }

  // Initialize the orientation publisher
  {
    orientation_publisher = get_node()->create_publisher<Orientation>(
      "/legs/orientation", 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Orientation publisher initialized on " <<
        orientation_publisher->get_topic_name() << "!");
  }

  // Initialize the joints publisher
  {
    joints_publisher = get_node()->create_publisher<Joints>(
      "/legs/joints", 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Joints publisher initialized on " <<
        joints_publisher->get_topic_name() << "!");
  }
}

void LegsProvider::set_position(const Position & position)
{
  position_publisher->publish(position);
}

void LegsProvider::set_orientation(const Orientation & orientation)
{
  orientation_publisher->publish(orientation);
}

void LegsProvider::set_joints(const Joints & joints)
{
  joints_publisher->publish(joints);
}

void LegsProvider::set_command(const std::string & command)
{
  StringMsg msg;
  msg.data = command;

  command_publisher->publish(msg);
}

rclcpp::Node::SharedPtr LegsProvider::get_node() const
{
  return node;
}

}  // namespace beine_cpp

#endif  // BEINE_CPP__LEGS__LEGS_PROVIDER_HPP_
