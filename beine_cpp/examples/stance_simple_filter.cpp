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

#include <beine_cpp/beine_cpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <iomanip>
#include <memory>
#include <string>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("stance_simple_filter");
  auto joints_consumer = std::make_shared<beine_cpp::JointsConsumer>(node);
  auto stance_provider = std::make_shared<beine_cpp::StanceProvider>(node);

  beine_cpp::Stance prev_stance;

  joints_consumer->set_on_joints_changed(
    [&](const beine_cpp::Joints & joints) {
      beine_cpp::Stance stance;

      if (joints.left_knee < 120.0 && joints.right_knee < 120.0 &&
      joints.left_ankle < 60.0 && joints.right_ankle < 60.0)
      {
        stance.make_sitting();
      } else {
        stance.make_standing();
      }

      if (stance.get_state() != prev_stance.get_state()) {
        prev_stance = stance;
        RCLCPP_INFO_STREAM(node->get_logger(), "Stance changed into " << stance << "!");
      }

      stance_provider->set_stance(stance);
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
