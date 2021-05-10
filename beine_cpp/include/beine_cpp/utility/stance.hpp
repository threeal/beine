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

#ifndef BEINE_CPP__UTILITY__STANCE_HPP_
#define BEINE_CPP__UTILITY__STANCE_HPP_

#include <beine_interfaces/msg/stance.hpp>

namespace beine_cpp
{

using StanceMsg = beine_interfaces::msg::Stance;

struct Stance
{
public:
  enum State
  {
    STANDING,
    SITTING
  };

  Stance();
  explicit Stance(const StanceMsg & msg);

  operator StanceMsg() const;

  const Stance & operator=(const StanceMsg & msg);

  void set_state(State state);

  void make_standing();
  void make_sitting();

  State get_state() const;

  bool is_standing() const;
  bool is_sitting() const;

private:
  State state;
};

}  // namespace beine_cpp

std::ostream & operator<<(std::ostream & out, const beine_cpp::Stance & stance);

#endif  // BEINE_CPP__UTILITY__STANCE_HPP_
