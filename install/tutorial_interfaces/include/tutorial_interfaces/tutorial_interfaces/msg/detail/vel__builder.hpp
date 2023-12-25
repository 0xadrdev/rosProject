// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tutorial_interfaces:msg/Vel.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__MSG__DETAIL__VEL__BUILDER_HPP_
#define TUTORIAL_INTERFACES__MSG__DETAIL__VEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tutorial_interfaces/msg/detail/vel__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tutorial_interfaces
{

namespace msg
{

namespace builder
{

class Init_Vel_y
{
public:
  explicit Init_Vel_y(::tutorial_interfaces::msg::Vel & msg)
  : msg_(msg)
  {}
  ::tutorial_interfaces::msg::Vel y(::tutorial_interfaces::msg::Vel::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tutorial_interfaces::msg::Vel msg_;
};

class Init_Vel_x
{
public:
  Init_Vel_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Vel_y x(::tutorial_interfaces::msg::Vel::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Vel_y(msg_);
  }

private:
  ::tutorial_interfaces::msg::Vel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tutorial_interfaces::msg::Vel>()
{
  return tutorial_interfaces::msg::builder::Init_Vel_x();
}

}  // namespace tutorial_interfaces

#endif  // TUTORIAL_INTERFACES__MSG__DETAIL__VEL__BUILDER_HPP_
