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

#include <memory>
#include <string>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("legs_consumer_log");
  auto leg_consumer = std::make_shared<beine_cpp::LegsConsumer>(node);

  auto update_timer = node->create_wall_timer(
    100ms, [leg_consumer]() {
      // Clear screen
      std::cout << "\033[2J\033[2H" << std::endl;

      auto position = leg_consumer->get_position();

      std::cout << "Position: " <<
        position.x << " " <<
        position.y << " " <<
        position.z << std::endl;

      auto orientation = leg_consumer->get_orientation();

      std::cout << "Orientation: " <<
        orientation.x << " " <<
        orientation.y << " " <<
        orientation.z << std::endl;

      std::cout << std::endl;

      auto stance = leg_consumer->get_stance();

      std::string stance_string;
      switch (stance.get_state()) {
        case beine_cpp::Stance::State::STANDING:
          stance_string = "Standing";
          break;

        case beine_cpp::Stance::State::SITTING:
          stance_string = "Sitting";
          break;
      }

      std::cout << "Stance: " << stance_string << std::endl;
      std::cout << "Command: " << leg_consumer->get_command() << std::endl;
    });

  update_timer->reset();

  rclcpp::spin(node);

  update_timer->cancel();

  rclcpp::shutdown();

  return 0;
}