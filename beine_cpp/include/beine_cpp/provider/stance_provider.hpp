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

#ifndef BEINE_CPP__PROVIDER__STANCE_PROVIDER_HPP_
#define BEINE_CPP__PROVIDER__STANCE_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "../node.hpp"
#include "../utility.hpp"

namespace beine_cpp
{

class StanceProvider : public LegsNode
{
public:
  struct Options : public virtual LegsNode::Options
  {
  };

  inline explicit StanceProvider(rclcpp::Node::SharedPtr node, const Options & options = Options());

  void set_stance(const Stance & stance);

  const Stance & get_stance() const;

private:
  rclcpp::Publisher<StanceMsg>::SharedPtr stance_publisher;

  Stance current_stance;
};

StanceProvider::StanceProvider(
  rclcpp::Node::SharedPtr node, const StanceProvider::Options & options)
: LegsNode(node, options)
{
  // Initialize the stance publisher
  {
    stance_publisher = get_node()->create_publisher<StanceMsg>(
      get_legs_prefix() + STANCE_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Stance publisher initialized on " << stance_publisher->get_topic_name() << "!");
  }

  // Initial data publish
  set_stance(get_stance());
}

}  // namespace beine_cpp

#endif  // BEINE_CPP__PROVIDER__STANCE_PROVIDER_HPP_
