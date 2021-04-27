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

  auto node = std::make_shared<rclcpp::Node>("legs_consumer_log");
  auto legs_consumer = std::make_shared<beine_cpp::LegsConsumer>(node);

  RCLCPP_INFO(node->get_logger(), "Press enter to continue");
  std::cin.get();

  auto update_timer = node->create_wall_timer(
    100ms, [&]() {
      auto position = legs_consumer->get_position();
      auto orientation = legs_consumer->get_orientation();

      std::string stance_name;
      switch (legs_consumer->get_stance().get_state()) {
        case beine_cpp::Stance::State::STANDING:
          stance_name = "Standing";
          break;

        case beine_cpp::Stance::State::SITTING:
          stance_name = "Sitting";
          break;
      }

      // Clear screen
      std::cout << "\033[2J\033[2H" << std::endl;

      RCLCPP_INFO_STREAM(
        node->get_logger(),
        std::fixed << std::setprecision(1) <<
          "\n\nPosition\t: " << position.x << " " << position.y << " " << position.z <<
          "\nOrientation\t: " << orientation.x << " " << orientation.y << " " << orientation.z <<
          "\n\nStance\t: " << stance_name <<
          "\nCommand\t: \"" << legs_consumer->get_command() << "\"");
    });

  update_timer->reset();

  rclcpp::spin(node);

  update_timer->cancel();

  rclcpp::shutdown();

  return 0;
}
