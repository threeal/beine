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

#include <beine_cpp/consumer/legs_consumer.hpp>

#include <string>

namespace beine_cpp
{

void LegsConsumer::set_on_position_changed(const PositionCallback & callback)
{
  on_position_changed = callback;
}

void LegsConsumer::set_on_orientation_changed(const OrientationCallback & callback)
{
  on_orientation_changed = callback;
}

void LegsConsumer::set_on_stance_changed(const StanceCallback & callback)
{
  on_stance_changed = callback;
}

void LegsConsumer::set_on_command_changed(const CommandCallback & callback)
{
  on_command_changed = callback;
}

const Position & LegsConsumer::get_position() const
{
  return current_position;
}

const Orientation & LegsConsumer::get_orientation() const
{
  return current_orientation;
}

const Stance & LegsConsumer::get_stance() const
{
  return current_stance;
}

const std::string & LegsConsumer::get_command() const
{
  return current_command;
}

}  // namespace beine_cpp
