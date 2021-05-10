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

  inline Stance();
  inline explicit Stance(const StanceMsg & msg);

  inline operator StanceMsg() const;

  inline void set_state(State state);

  inline void make_standing();
  inline void make_sitting();

  inline State get_state() const;

  inline bool is_standing() const;
  inline bool is_sitting() const;

private:
  State state;
};

Stance::Stance()
: state(State::STANDING)
{
}

Stance::Stance(const StanceMsg & msg)
{
  if (msg.sitting) {
    set_state(State::SITTING);
  } else {
    set_state(State::STANDING);
  }
}

Stance::operator StanceMsg() const
{
  StanceMsg msg;

  switch (get_state()) {
    case State::STANDING:
      msg.sitting = false;
      break;

    case State::SITTING:
      msg.sitting = true;
      break;
  }

  return msg;
}

void Stance::set_state(Stance::State state)
{
  this->state = state;
}

void Stance::make_standing()
{
  set_state(State::STANDING);
}

void Stance::make_sitting()
{
  set_state(State::SITTING);
}

Stance::State Stance::get_state() const
{
  return state;
}

bool Stance::is_standing() const
{
  return get_state() == State::STANDING;
}

bool Stance::is_sitting() const
{
  return get_state() == State::SITTING;
}

}  // namespace beine_cpp

inline std::ostream & operator<<(std::ostream & out, const beine_cpp::Stance & stance)
{
  switch (stance.get_state()) {
    case beine_cpp::Stance::State::STANDING:
      out << "Standing";
      break;

    case beine_cpp::Stance::State::SITTING:
      out << "Sitting";
      break;
  }

  return out;
}

#endif  // BEINE_CPP__UTILITY__STANCE_HPP_
