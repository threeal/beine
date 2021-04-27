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

#include <termios.h>
#include <unistd.h>

#include <iomanip>
#include <memory>
#include <string>

#define STDIN 0

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("legs_provider_dummy");
  auto legs_provider = std::make_shared<beine_cpp::LegsProvider>(node);

  beine_cpp::Position position;
  beine_cpp::Orientation orientation;
  beine_cpp::Joints joints;
  std::string command;

  bool input_command = false;

  // Get the original terminal configuration
  struct termios original_term;
  tcgetattr(STDIN, &original_term);

  struct termios nonblock_term = original_term;
  nonblock_term.c_lflag &= ~ICANON;
  nonblock_term.c_lflag &= ~ECHO;
  nonblock_term.c_lflag &= ~ISIG;
  nonblock_term.c_cc[VMIN] = 0;
  nonblock_term.c_cc[VTIME] = 0;

  auto update_timer = node->create_wall_timer(
    100ms, [&]() {
      char input;

      // Modify the terminal configuration
      tcsetattr(STDIN, TCSANOW, &nonblock_term);

      // Handle keyboard input
      while (read(STDIN, &input, 1) > 0) {
        if (input_command) {
          if (input == '\n') {
            input_command = false;
            legs_provider->set_command(command);
          } else {
            command += input;
          }
        } else {
          std::cout << input << std::endl;
          switch (toupper(input)) {
            case 'W':
              position.x += 0.1;
              break;

            case 'S':
              position.x -= 0.1;
              break;

            case 'A':
              position.y += 0.1;
              break;

            case 'D':
              position.y -= 0.1;
              break;

            case 'Q':
              orientation.z += 10.0;
              break;

            case 'E':
              orientation.z -= 10.0;
              break;

            case 'O':
              joints.left_knee -= 10.0;
              joints.right_knee -= 10.0;
              break;

            case 'P':
              joints.left_knee += 10.0;
              joints.right_knee += 10.0;
              break;

            case 'K':
              joints.left_ankle -= 10.0;
              joints.right_ankle -= 10.0;
              break;

            case 'L':
              joints.left_ankle += 10.0;
              joints.right_ankle += 10.0;
              break;

            case 'I':
              command = "";
              input_command = true;
              break;
          }

          // Set new data
          legs_provider->set_position(position);
          legs_provider->set_orientation(orientation);
          legs_provider->set_joints(joints);
        }
      }

      // Restore the terminal configuration
      tcsetattr(STDIN, TCSANOW, &original_term);

      // Clear screen
      std::cout << "\033[2J\033[2H" << std::endl;

      // Set number precision
      std::cout << std::fixed << std::setprecision(1);

      std::cout << "Position: " <<
        position.x << " " <<
        position.y << " " <<
        position.z << std::endl;

      std::cout << "Orientation: " <<
        orientation.x << " " <<
        orientation.y << " " <<
        orientation.z << std::endl;

      std::cout << std::endl;

      std::cout << "Left Joints: " <<
        joints.left_knee << " " <<
        joints.left_ankle << std::endl;

      std::cout << "Right Joints: " <<
        joints.right_knee << " " <<
        joints.right_ankle << std::endl;

      std::cout << std::endl;

      std::cout << "Command: " << command << std::endl;

      std::cout << std::endl;

      if (input_command) {
        std::cout << "Enter -> Admit command input" << std::endl;
      } else {
        std::cout << "W/S -> position x" << std::endl;
        std::cout << "A/D -> position y" << std::endl;
        std::cout << "Q/E -> orientation z" << std::endl;
        std::cout << "O/P -> knee joints" << std::endl;
        std::cout << "K/L -> ankle joints" << std::endl;
        std::cout << "I -> enter command input" << std::endl;
      }
    });

  update_timer->reset();

  rclcpp::spin(node);

  update_timer->cancel();

  rclcpp::shutdown();

  return 0;
}
