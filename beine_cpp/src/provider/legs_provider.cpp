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

#include <beine_cpp/provider/legs_provider.hpp>

#include <string>

namespace beine_cpp
{

void LegsProvider::set_position(const Position & position)
{
  current_position = position;
  position_publisher->publish(get_position());
}

void LegsProvider::set_orientation(const Orientation & orientation)
{
  current_orientation = orientation;
  orientation_publisher->publish(get_orientation());
}

void LegsProvider::set_joints(const Joints & joints)
{
  current_joints = joints;
  joints_publisher->publish(get_joints());
}

void LegsProvider::set_command(const std::string & command)
{
  current_command = command;

  StringMsg msg;
  msg.data = get_command();

  command_publisher->publish(msg);
}

const Position & LegsProvider::get_position() const
{
  return current_position;
}

const Orientation & LegsProvider::get_orientation() const
{
  return current_orientation;
}

const Joints & LegsProvider::get_joints() const
{
  return current_joints;
}

const std::string & LegsProvider::get_command() const
{
  return current_command;
}

}  // namespace beine_cpp
