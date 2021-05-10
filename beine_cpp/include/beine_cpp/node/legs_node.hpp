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

#ifndef BEINE_CPP__NODE__LEGS_NODE_HPP_
#define BEINE_CPP__NODE__LEGS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>

#include "../utility.hpp"

namespace beine_cpp
{

class LegsNode
{
public:
  struct Options
  {
    std::string legs_prefix;

    Options()
    : legs_prefix("/legs")
    {
    }
  };

  inline explicit LegsNode(rclcpp::Node::SharedPtr node, const Options & options = Options());

  inline rclcpp::Node::SharedPtr get_node() const;

  inline const std::string & get_legs_prefix() const;

private:
  rclcpp::Node::SharedPtr node;

  std::string legs_prefix;
};

LegsNode::LegsNode(rclcpp::Node::SharedPtr node, const LegsNode::Options & options)
: node(node),
  legs_prefix(options.legs_prefix)
{
}

rclcpp::Node::SharedPtr LegsNode::get_node() const
{
  return node;
}

const std::string & LegsNode::get_legs_prefix() const
{
  return legs_prefix;
}

}  // namespace beine_cpp

#endif  // BEINE_CPP__NODE__LEGS_NODE_HPP_