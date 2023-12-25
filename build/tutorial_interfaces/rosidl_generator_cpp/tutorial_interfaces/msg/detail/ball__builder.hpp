// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tutorial_interfaces:msg/Ball.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__MSG__DETAIL__BALL__BUILDER_HPP_
#define TUTORIAL_INTERFACES__MSG__DETAIL__BALL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tutorial_interfaces/msg/detail/ball__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tutorial_interfaces
{

namespace msg
{

namespace builder
{

class Init_Ball_y
{
public:
  explicit Init_Ball_y(::tutorial_interfaces::msg::Ball & msg)
  : msg_(msg)
  {}
  ::tutorial_interfaces::msg::Ball y(::tutorial_interfaces::msg::Ball::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tutorial_interfaces::msg::Ball msg_;
};

class Init_Ball_x
{
public:
  Init_Ball_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Ball_y x(::tutorial_interfaces::msg::Ball::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Ball_y(msg_);
  }

private:
  ::tutorial_interfaces::msg::Ball msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tutorial_interfaces::msg::Ball>()
{
  return tutorial_interfaces::msg::builder::Init_Ball_x();
}

}  // namespace tutorial_interfaces

#endif  // TUTORIAL_INTERFACES__MSG__DETAIL__BALL__BUILDER_HPP_
