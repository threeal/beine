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

#ifndef BEINE_CPP__PROVIDER__LEGS_PROVIDER_HPP_
#define BEINE_CPP__PROVIDER__LEGS_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>

#include "../node.hpp"
#include "../utility.hpp"

namespace beine_cpp
{

class LegsProvider : public LegsNode
{
public:
  struct Options : public virtual LegsNode::Options
  {
  };

  inline explicit LegsProvider(rclcpp::Node::SharedPtr node, const Options & options = Options());

  void set_position(const Position & position);
  void set_orientation(const Orientation & orientation);
  void set_joints(const Joints & joints);
  void set_command(const std::string & command);

  const Position & get_position() const;
  const Orientation & get_orientation() const;
  const Joints & get_joints() const;
  const std::string & get_command() const;

private:
  rclcpp::Publisher<Position>::SharedPtr position_publisher;
  rclcpp::Publisher<Orientation>::SharedPtr orientation_publisher;
  rclcpp::Publisher<Joints>::SharedPtr joints_publisher;
  rclcpp::Publisher<StringMsg>::SharedPtr command_publisher;

  Position current_position;
  Orientation current_orientation;
  Joints current_joints;
  std::string current_command;
};

LegsProvider::LegsProvider(rclcpp::Node::SharedPtr node, const LegsProvider::Options & options)
: LegsNode(node, options)
{
  // Initialize the joints value
  current_joints.left_knee = 180.0;
  current_joints.right_knee = 180.0;
  current_joints.left_ankle = 90.0;
  current_joints.right_ankle = 90.0;

  // Initialize the position publisher
  {
    position_publisher = get_node()->create_publisher<Position>(
      get_legs_prefix() + POSITION_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Position publisher initialized on " << position_publisher->get_topic_name() << "!");
  }

  // Initialize the orientation publisher
  {
    orientation_publisher = get_node()->create_publisher<Orientation>(
      get_legs_prefix() + ORIENTATION_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Orientation publisher initialized on " << orientation_publisher->get_topic_name() << "!");
  }

  // Initialize the joints publisher
  {
    joints_publisher = get_node()->create_publisher<Joints>(
      get_legs_prefix() + JOINTS_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Joints publisher initialized on " << joints_publisher->get_topic_name() << "!");
  }

  // Initialize the command publisher
  {
    command_publisher = get_node()->create_publisher<StringMsg>(
      get_legs_prefix() + COMMAND_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Command publisher initialized on " << command_publisher->get_topic_name() << "!");
  }

  // Initial data publish
  set_position(get_position());
  set_orientation(get_orientation());
  set_joints(get_joints());
  set_command(get_command());
}

}  // namespace beine_cpp

#endif  // BEINE_CPP__PROVIDER__LEGS_PROVIDER_HPP_
