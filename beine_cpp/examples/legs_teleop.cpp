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
#include <keisan/keisan.hpp>
#include <rclcpp/rclcpp.hpp>

#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <iomanip>
#include <memory>
#include <string>

#define STDIN 0

namespace ksn = keisan;

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  auto program = argparse::ArgumentParser("legs_teleop", "0.1.0");

  beine_cpp::LegsProvider::Options options;

  program.add_argument("--legs-prefix")
  .help("prefix name for legs's topics and services")
  .action(
    [&](const std::string & value) {
      options.legs_prefix = value;
    });

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error & err) {
    std::cout << err.what() << std::endl;
    std::cout << program;
    return 1;
  }

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("legs_teleop");
  auto legs_provider = std::make_shared<beine_cpp::LegsProvider>(node, options);

  // Get the original terminal configuration
  struct termios original_term;
  tcgetattr(STDIN, &original_term);

  // create a non-blocking terminal configuration
  struct termios nonblock_term = original_term;
  nonblock_term.c_lflag &= ~ICANON;
  nonblock_term.c_lflag &= ~ECHO;
  nonblock_term.c_lflag &= ~ISIG;
  nonblock_term.c_cc[VMIN] = 0;
  nonblock_term.c_cc[VTIME] = 0;

  RCLCPP_INFO(node->get_logger(), "Press enter to continue");
  std::cin.get();

  auto update_timer = node->create_wall_timer(
    100ms, [&]() {
      auto position = legs_provider->get_position();
      auto orientation = legs_provider->get_orientation();
      auto joints = legs_provider->get_joints();
      auto command = legs_provider->get_command();

      // Modify the terminal configuration
      tcsetattr(STDIN, TCSANOW, &nonblock_term);

      // Handle keyboard input
      char input;
      while (read(STDIN, &input, 1) > 0) {
        auto velocity = ksn::Point2::zero();

        switch (toupper(input)) {
          case 'W':
            velocity.x += 0.25;
            break;

          case 'S':
            velocity.x -= 0.25;
            break;

          case 'A':
            velocity.y += 0.25;
            break;

          case 'D':
            velocity.y -= 0.25;
            break;

          case 'Q':
            orientation.z += 30.0;
            break;

          case 'E':
            orientation.z -= 30.0;
            break;

          case 'I':
            joints.left_knee = std::max(joints.left_knee - 10.0, 0.0);
            joints.right_knee = std::max(joints.right_knee - 10.0, 0.0);
            break;

          case 'O':
            joints.left_knee = std::min(joints.left_knee + 10.0, 180.0);
            joints.right_knee = std::min(joints.right_knee + 10.0, 180.0);
            break;

          case 'K':
            joints.left_ankle = std::max(joints.left_ankle - 10.0, 0.0);
            joints.right_ankle = std::max(joints.right_ankle - 10.0, 0.0);
            break;

          case 'L':
            joints.left_ankle = std::min(joints.left_ankle + 10.0, 90.0);
            joints.right_ankle = std::min(joints.right_ankle + 10.0, 90.0);
            break;

          case 'C':
            // Temporarely reset the terminal configuration
            tcsetattr(STDIN, TCSANOW, &original_term);

            RCLCPP_INFO(node->get_logger(), "Input a new command:");
            std::getline(std::cin, command);

            tcsetattr(STDIN, TCSANOW, &nonblock_term);
            break;
        }

        // Set position according to the velocity and current orientation
        {
          auto angle = ksn::make_degree(orientation.z);
          auto rotated_velocity = velocity.rotate(angle);

          position.x += rotated_velocity.x;
          position.y += rotated_velocity.y;
        }
      }

      // Restore the terminal configuration
      tcsetattr(STDIN, TCSANOW, &original_term);

      // Clear screen
      std::cout << "\033[2J\033[2H" << std::endl;

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        std::fixed << std::setprecision(1) <<
          "\n\nPosition\t: " << position <<
          "\nOrientation\t: " << orientation <<
          "\n\nAnkle Joints\t: " << joints.left_knee << " " << joints.right_knee <<
          "\nKnee Joints\t: " << joints.left_ankle << " " << joints.right_ankle <<
          "\n\nCommand\t: \"" << command << "\"" <<
          "\n\nW/S -> local position x\tA/D -> local position y\tQ/E -> orientation z" <<
          "\nI/O -> knee joints\tK/L -> ankle joints\tC -> enter command input");

      // Set new data
      legs_provider->set_position(position);
      legs_provider->set_orientation(orientation);
      legs_provider->set_joints(joints);
      legs_provider->set_command(command);
    });

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
