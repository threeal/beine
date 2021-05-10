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

#ifndef BEINE_CPP__CONSUMER__LEGS_CONSUMER_HPP_
#define BEINE_CPP__CONSUMER__LEGS_CONSUMER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>

#include "../node.hpp"
#include "../utility.hpp"

namespace beine_cpp
{

class LegsConsumer : public LegsNode
{
public:
  using PositionCallback = std::function<void (const Position &)>;
  using OrientationCallback = std::function<void (const Orientation &)>;
  using StanceCallback = std::function<void (const Stance &)>;
  using CommandCallback = std::function<void (const std::string &)>;

  inline explicit LegsConsumer(rclcpp::Node::SharedPtr node, const Options & options = Options());

  inline void set_on_position_changed(const PositionCallback & callback);
  inline void set_on_orientation_changed(const OrientationCallback & callback);
  inline void set_on_stance_changed(const StanceCallback & callback);
  inline void set_on_command_changed(const CommandCallback & callback);

  inline const Position & get_position() const;
  inline const Orientation & get_orientation() const;
  inline const Stance & get_stance() const;
  inline const std::string & get_command() const;

private:
  rclcpp::Subscription<Position>::SharedPtr position_subscription;
  rclcpp::Subscription<Orientation>::SharedPtr orientation_subscription;
  rclcpp::Subscription<StanceMsg>::SharedPtr stance_subscription;
  rclcpp::Subscription<StringMsg>::SharedPtr command_subscription;

  PositionCallback on_position_changed;
  OrientationCallback on_orientation_changed;
  StanceCallback on_stance_changed;
  CommandCallback on_command_changed;

  Position current_position;
  Orientation current_orientation;
  Stance current_stance;
  std::string current_command;
};

LegsConsumer::LegsConsumer(rclcpp::Node::SharedPtr node, const LegsConsumer::Options & options)
: LegsNode(node, options)
{
  // Initialize the position subscription
  {
    position_subscription = get_node()->create_subscription<Position>(
      get_legs_prefix() + POSITION_SUFFIX, 10,
      [this](const Position::SharedPtr msg) {
        current_position = *msg;

        if (on_position_changed) {
          on_position_changed(get_position());
        }
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Position subscription initialized on " << position_subscription->get_topic_name() << "!");
  }

  // Initialize the orientation subscription
  {
    orientation_subscription = get_node()->create_subscription<Orientation>(
      get_legs_prefix() + ORIENTATION_SUFFIX, 10,
      [this](const Orientation::SharedPtr msg) {
        current_orientation = *msg;

        if (on_orientation_changed) {
          on_orientation_changed(get_orientation());
        }
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Orientation subscription initialized on " <<
        orientation_subscription->get_topic_name() << "!");
  }

  // Initialize the stance subscription
  {
    stance_subscription = get_node()->create_subscription<StanceMsg>(
      get_legs_prefix() + STANCE_SUFFIX, 10,
      [this](const StanceMsg::SharedPtr msg) {
        current_stance = Stance(*msg);

        if (on_stance_changed) {
          on_stance_changed(get_stance());
        }
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Stance subscription initialized on " << stance_subscription->get_topic_name() << "!");
  }

  // Initialize the command subscription
  {
    command_subscription = get_node()->create_subscription<StringMsg>(
      get_legs_prefix() + COMMAND_SUFFIX, 10,
      [this](const StringMsg::SharedPtr msg) {
        current_command = msg->data;

        if (on_command_changed) {
          on_command_changed(get_command());
        }
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Command subscription initialized on " << command_subscription->get_topic_name() << "!");
  }
}

void LegsConsumer::set_on_position_changed(const PositionCallback & callback)
{
  on_position_changed = callback;
}

void LegsConsumer::set_on_orientation_changed(const OrientationCallback & callback)
{
  on_orientation_changed = callback;
}

void LegsConsumer::set_on_stance_changed(const StanceCallback & callback)
{
  on_stance_changed = callback;
}

void LegsConsumer::set_on_command_changed(const CommandCallback & callback)
{
  on_command_changed = callback;
}

const Position & LegsConsumer::get_position() const
{
  return current_position;
}

const Orientation & LegsConsumer::get_orientation() const
{
  return current_orientation;
}

const Stance & LegsConsumer::get_stance() const
{
  return current_stance;
}

const std::string & LegsConsumer::get_command() const
{
  return current_command;
}

}  // namespace beine_cpp

#endif  // BEINE_CPP__CONSUMER__LEGS_CONSUMER_HPP_
