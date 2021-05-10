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

#include <argparse/argparse.hpp>
#include <beine_cpp/beine_cpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <iomanip>
#include <memory>
#include <string>

using namespace std::chrono_literals;

struct Options : public virtual beine_cpp::JointsConsumer::Options,
  public virtual beine_cpp::StanceProvider::Options
{
  double knee_angle;
  double ankle_angle;

  Options()
  : knee_angle(120.0),
    ankle_angle(60.0)
  {
  }
};

int main(int argc, char ** argv)
{
  auto program = argparse::ArgumentParser("stance_simple_filter", "0.1.0");

  Options options;

  program.add_argument("--legs-prefix")
  .help("prefix name for legs's topics and services")
  .action(
    [&](const std::string & value) {
      options.legs_prefix = value;
    });

  program.add_argument("--knee-angle")
  .help("knee angle breakpoint for sitting stance")
  .action(
    [&](const std::string & value) {
      options.knee_angle = stod(value);
    });

  program.add_argument("--ankle-angle")
  .help("ankle angle breakpoint for sitting stance")
  .action(
    [&](const std::string & value) {
      options.ankle_angle = stod(value);
    });

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error & err) {
    std::cout << err.what() << std::endl;
    std::cout << program;
    return 1;
  }

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("stance_simple_filter");
  auto joints_consumer = std::make_shared<beine_cpp::JointsConsumer>(node, options);
  auto stance_provider = std::make_shared<beine_cpp::StanceProvider>(node, options);

  joints_consumer->set_on_joints_changed(
    [&](const beine_cpp::Joints & joints) {
      beine_cpp::Stance stance;

      if (
        joints.left_knee < options.knee_angle && joints.right_knee < options.knee_angle &&
        joints.left_ankle < options.ankle_angle && joints.right_ankle < options.ankle_angle)
      {
        stance.make_sitting();
      } else {
        stance.make_standing();
      }

      auto prev_stance = stance_provider->get_stance();
      if (stance.get_state() != prev_stance.get_state()) {
        stance_provider->set_stance(stance);

        RCLCPP_INFO_STREAM(node->get_logger(), "Stance changed into " << stance << "!");
      }
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
