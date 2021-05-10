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

#ifndef BEINE_CPP__CONSUMER__JOINTS_CONSUMER_HPP_
#define BEINE_CPP__CONSUMER__JOINTS_CONSUMER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "../node.hpp"
#include "../utility.hpp"

namespace beine_cpp
{

class JointsConsumer : public LegsNode
{
public:
  using JointsCallback = std::function<void (const Joints & joints)>;

  struct Options : public virtual LegsNode::Options
  {
  };

  inline explicit JointsConsumer(rclcpp::Node::SharedPtr node, const Options & options = Options());

  inline void set_on_joints_changed(const JointsCallback & callback);

  inline const Joints & get_joints() const;

private:
  rclcpp::Subscription<Joints>::SharedPtr joints_subscription;

  JointsCallback on_joints_changed;

  Joints current_joints;
};

JointsConsumer::JointsConsumer(
  rclcpp::Node::SharedPtr node, const JointsConsumer::Options & options)
: LegsNode(node, options)
{
  // Initialize the joints subscription
  {
    joints_subscription = get_node()->create_subscription<Joints>(
      get_legs_prefix() + JOINTS_SUFFIX, 10,
      [this](const Joints::SharedPtr msg) {
        current_joints = *msg;

        if (on_joints_changed) {
          on_joints_changed(get_joints());
        }
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Joints subscription initialized on " << joints_subscription->get_topic_name() << "!");
  }
}

void JointsConsumer::set_on_joints_changed(const JointsCallback & callback)
{
  on_joints_changed = callback;
}

const Joints & JointsConsumer::get_joints() const
{
  return current_joints;
}

}  // namespace beine_cpp

#endif  // BEINE_CPP__CONSUMER__JOINTS_CONSUMER_HPP_
